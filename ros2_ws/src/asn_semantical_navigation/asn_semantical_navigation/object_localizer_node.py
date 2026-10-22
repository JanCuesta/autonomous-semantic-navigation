"""RGB-D localization of a YOLO-selected object in the ROS map frame.

Input: detections_json, depth image, and registered depth camera CameraInfo.
Output: validated target pose, validity flag, status, and RViz marker.
This node NEVER commands navigation or /cmd_vel.
"""

import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class ObjectLocalizer(Node):
    def __init__(self):
        super().__init__('object_localizer_node')

        self.declare_parameters('', [
            ('depth_topic', '/camera/depth/image_raw'),
            ('camera_info_topic', '/camera_info'),
            ('detections_topic', '/semantic/search/detections_json'),
            ('map_frame', 'map'),
            ('max_sync_difference_s', 0.10),
            ('min_depth_m', 0.15),
            ('max_depth_m', 5.0),
            ('roi_fraction', 0.40),
            ('min_valid_pixels', 20),
            ('min_valid_fraction', 0.30),
            ('max_depth_spread_m', 1.0),
            ('confirmation_positions', 3),
            ('max_position_jitter_m', 0.45),
            ('target_timeout_s', 2.0),
        ])

        self.bridge = CvBridge()
        self.camera_info = None
        # A bounded buffer is needed because CPU YOLO inference may be delayed.
        self.depth_buffer = deque(maxlen=60)
        self.pending_detections = deque(maxlen=8)
        self.position_history = deque(
            maxlen=max(1, int(self.get_parameter('confirmation_positions').value))
        )
        self.current_target = None
        self.last_detection_ns = -1
        self.valid = False
        self.last_valid_wall_time = 0.0
        self.last_status = None
        self.last_status_log_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(
            PoseStamped, '/semantic/perception/target_pose', 10)
        self.valid_pub = self.create_publisher(
            Bool, '/semantic/perception/target_valid', 10)
        self.status_pub = self.create_publisher(
            String, '/semantic/perception/status', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic/perception/target_markers', 10)

        self.depth_sub = self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.camera_info_callback, qos_profile_sensor_data)
        self.detection_sub = self.create_subscription(
            String, self.get_parameter('detections_topic').value,
            self.detection_callback, 10)
        self.timer = self.create_timer(0.2, self.watchdog_callback)
        self.report('WAITING: depth, camera calibration, and YOLO detections')

    @staticmethod
    def timestamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def report(self, message):
        self.status_pub.publish(String(data=message))
        now = time.monotonic()
        if message != self.last_status or now - self.last_status_log_time > 5.0:
            self.get_logger().info(message)
            self.last_status = message
            self.last_status_log_time = now

    def clear_marker(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def invalidate(self, reason):
        was_valid = self.valid
        self.valid = False
        self.position_history.clear()
        self.valid_pub.publish(Bool(data=False))
        if was_valid:
            self.clear_marker()
        self.report(reason)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
        stamp = self.timestamp_ns(msg.header.stamp)
        if self.depth_buffer and stamp + 100_000_000 < self.timestamp_ns(
                self.depth_buffer[-1].header.stamp):
            # Handles resetting Isaac Sim's simulation clock.
            self.depth_buffer.clear()
            self.pending_detections.clear()
            self.last_detection_ns = -1
            self.invalidate('WAITING: simulation clock reset')
        self.depth_buffer.append(msg)
        self.process_pending()

    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
            target = str(data['target'])
            stamp_ns = (int(data['timestamp_sec']) * 1_000_000_000
                        + int(data['timestamp_nanosec']))
            detections = data['detections']
            if not isinstance(detections, list):
                raise ValueError('detections must be a list')
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.invalidate(f'REJECTED: bad detection JSON: {exc}')
            return

        if target != self.current_target:
            self.current_target = target
            self.last_detection_ns = -1
            self.pending_detections.clear()
            self.invalidate(f'TARGET_SELECTED: {target}')
        if stamp_ns <= self.last_detection_ns:
            return  # Ignore duplicate or out-of-order detections.
        self.last_detection_ns = stamp_ns

        matches = [d for d in detections
                   if isinstance(d, dict) and d.get('class') == target]
        if not data.get('confirmed') or not matches:
            self.pending_detections.clear()
            self.invalidate('WAITING: no confirmed YOLO detection')
            return

        self.pending_detections.append((data, stamp_ns, time.monotonic()))
        self.process_pending()

    def process_pending(self):
        if not self.depth_buffer:
            return
        while self.pending_detections:
            data, rgb_ns, received_at = self.pending_detections[0]
            latest_depth_ns = self.timestamp_ns(self.depth_buffer[-1].header.stamp)
            # If depth is behind RGB, wait briefly for the matching frame.
            if latest_depth_ns < rgb_ns and time.monotonic() - received_at < 0.75:
                return
            self.pending_detections.popleft()
            depth_msg = min(
                self.depth_buffer,
                key=lambda m: abs(self.timestamp_ns(m.header.stamp) - rgb_ns)
            )
            dt = abs(self.timestamp_ns(depth_msg.header.stamp) - rgb_ns) / 1e9
            if dt > float(self.get_parameter('max_sync_difference_s').value):
                self.invalidate(f'REJECTED: RGB/depth timestamp gap {dt:.3f}s')
                continue
            self.localize(data, depth_msg, dt)

    def localize(self, data, depth_msg, dt):
        info = self.camera_info
        if info is None:
            self.invalidate('WAITING: /camera_info not received')
            return
        if not data.get('frame_id') or data['frame_id'] != depth_msg.header.frame_id:
            self.invalidate('REJECTED: RGB/depth optical frames differ')
            return
        if info.header.frame_id and info.header.frame_id != depth_msg.header.frame_id:
            self.invalidate('REJECTED: CameraInfo/depth optical frames differ')
            return
        if info.width != depth_msg.width or info.height != depth_msg.height:
            self.invalidate('REJECTED: CameraInfo/depth resolutions differ')
            return
        if (data.get('image_width', depth_msg.width) != depth_msg.width or
                data.get('image_height', depth_msg.height) != depth_msg.height):
            self.invalidate('REJECTED: RGB/depth image resolutions differ')
            return

        matches = [d for d in data['detections'] if d.get('class') == data['target']]
        try:
            best = max(matches, key=lambda d: float(d['confidence']))
            x1, y1, x2, y2 = (float(v) for v in best['bbox_xyxy'])
            if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
                raise ValueError('non-finite box coordinates')
            if x2 <= x1 or y2 <= y1:
                raise ValueError('empty or inverted box')
        except (KeyError, TypeError, ValueError) as exc:
            self.invalidate(f'REJECTED: invalid bounding box: {exc}')
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
            if depth_msg.encoding == '32FC1':
                depth = np.asarray(depth, dtype=np.float32)  # meters
            elif depth_msg.encoding == '16UC1':
                depth = np.asarray(depth, dtype=np.float32) * 0.001  # mm -> m
            else:
                self.invalidate(f'REJECTED: unsupported depth encoding {depth_msg.encoding}')
                return
        except Exception as exc:
            self.invalidate(f'REJECTED: depth conversion failed: {exc}')
            return
        if depth.ndim != 2:
            self.invalidate('REJECTED: expected one-channel depth image')
            return
        height, width = depth.shape
        u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        if not (0 <= u < width and 0 <= v < height):
            self.invalidate('REJECTED: bounding-box center outside depth image')
            return

        # Sample a central crop; exclude box edges that commonly include background.
        fraction = float(self.get_parameter('roi_fraction').value)
        fraction = max(0.05, min(1.0, fraction))
        half_w = (x2 - x1) * fraction / 2.0
        half_h = (y2 - y1) * fraction / 2.0
        left = max(0, int(math.floor(u - half_w)))
        right = min(width, int(math.ceil(u + half_w)))
        top = max(0, int(math.floor(v - half_h)))
        bottom = min(height, int(math.ceil(v + half_h)))
        region = depth[top:bottom, left:right]
        if region.size == 0:
            self.invalidate('REJECTED: empty depth crop')
            return
        min_depth = float(self.get_parameter('min_depth_m').value)
        max_depth = float(self.get_parameter('max_depth_m').value)
        values = region[np.isfinite(region) & (region >= min_depth) &
                        (region <= max_depth)]
        if (values.size < int(self.get_parameter('min_valid_pixels').value) or
                values.size / region.size < float(
                    self.get_parameter('min_valid_fraction').value)):
            self.invalidate('REJECTED: insufficient valid depth pixels')
            return
        spread = float(np.quantile(values, 0.90) - np.quantile(values, 0.10))
        if spread > float(self.get_parameter('max_depth_spread_m').value):
            self.invalidate(f'REJECTED: mixed foreground/background depth ({spread:.2f}m)')
            return
        z = float(np.median(values))

        k = info.k
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        if not all(math.isfinite(val) for val in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
            self.invalidate('REJECTED: invalid camera intrinsic matrix')
            return
        point = PointStamped()
        point.header = depth_msg.header
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z

        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter('map_frame').value,
                point.header.frame_id,
                Time.from_msg(point.header.stamp),
                timeout=Duration(seconds=0.0)
            )
            map_point = do_transform_point(point, tf)
        except TransformException as exc:
            self.invalidate(f'WAITING: map TF at camera timestamp: {exc}')
            return

        xyz = np.array([map_point.point.x, map_point.point.y, map_point.point.z])
        if not np.all(np.isfinite(xyz)):
            self.invalidate('REJECTED: invalid map coordinates')
            return

        tolerance = float(self.get_parameter('max_position_jitter_m').value)
        if self.position_history and any(
            np.linalg.norm(xyz - previous) > tolerance
            for previous in self.position_history
        ):
            if self.valid:
                self.clear_marker()
            self.valid = False
            self.position_history.clear()
        self.position_history.append(xyz)
        required = self.position_history.maxlen
        if len(self.position_history) < required:
            self.valid = False
            self.valid_pub.publish(Bool(data=False))
            self.report(
                f'OBSERVING: {len(self.position_history)}/{required} map poses; '
                f'camera depth={z:.2f}m, dt={dt:.3f}s'
            )
            return

        stable_xyz = np.median(np.stack(self.position_history), axis=0)
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('map_frame').value
        pose.header.stamp = depth_msg.header.stamp
        pose.pose.position.x = float(stable_xyz[0])
        pose.pose.position.y = float(stable_xyz[1])
        pose.pose.position.z = float(stable_xyz[2])
        pose.pose.orientation.w = 1.0  # orientation NOT estimated

        self.pose_pub.publish(pose)
        self.valid_pub.publish(Bool(data=True))
        self.publish_marker(pose)
        self.valid = True
        self.last_valid_wall_time = time.monotonic()
        self.report(
            f'LOCALIZED {data["target"]}: map=({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})m; '
            f'depth={z:.2f}m; dt={dt:.3f}s'
        )

    def publish_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'localized_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.17
        marker.scale.y = 0.17
        marker.scale.z = 0.17
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=2.0).to_msg()
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def watchdog_callback(self):
        now = time.monotonic()
        while (self.pending_detections and
               now - self.pending_detections[0][2] > 1.5):
            self.pending_detections.popleft()
            self.invalidate('REJECTED: timed out waiting for matching depth')
        if self.valid and now - self.last_valid_wall_time > float(
                self.get_parameter('target_timeout_s').value):
            self.invalidate('STALE: no recent valid target localization')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
