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
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def select_foreground_depth(region, *, min_depth_m=0.15, max_depth_m=5.0,
                            min_valid_pixels=20, min_valid_fraction=0.30,
                            max_depth_spread_m=1.0,
                            histogram_bin_m=0.10,
                            histogram_min_peak_separation_m=0.35,
                            histogram_valley_ratio=0.50,
                            foreground_min_fraction=0.15,
                            foreground_max_spread_m=0.60):
    """Find a foreground depth mode inside a YOLO ROI (NumPy only).
    The nearest *prominent, separated* mode is chosen, never the most numerous
    mode, because a background wall often occupies more pixels than a chair.
    If a strong foreground mode cannot be distinguished from the background,
    reject rather than returning a potentially incorrect wall position.
    Returns (selection, reason). selection has depth, pixel_x/y (ROI-local),
    raw_spread, selected_spread, support_fraction and mode. Pixel coordinates
    are taken from the selected foreground mask, NOT the original box center.
    """
    roi = np.asarray(region, dtype=np.float32)
    if roi.ndim != 2 or not roi.size:
        return None, 'empty or malformed depth ROI'
    valid = np.isfinite(roi) & (roi >= min_depth_m) & (roi <= max_depth_m)
    values = roi[valid]
    if (values.size < int(min_valid_pixels) or
            values.size / roi.size < float(min_valid_fraction)):
        return None, (f'insufficient valid depth pixels: '
                      f'{values.size}/{roi.size}')
    raw_spread = float(np.quantile(values, .90) - np.quantile(values, .10))
    bin_width = max(0.02, float(histogram_bin_m))
    edges = np.arange(float(min_depth_m), float(max_depth_m) + bin_width * 1.01,
                      bin_width)
    if edges.size < 3:
        return None, 'invalid histogram bin configuration'
    counts, _ = np.histogram(values, bins=edges)
    smoothed = np.convolve(counts.astype(float), [0.25, 0.50, 0.25], 'same')
    highest = float(np.max(smoothed))
    peak_cutoff = max(3.0, highest * 0.12)
    # Detect local peaks, suppressing duplicates within the same depth mode.
    maxima = []
    for i in range(len(smoothed)):
        left = smoothed[i-1] if i > 0 else -1.0
        right = smoothed[i+1] if i+1 < len(smoothed) else -1.0
        if (smoothed[i] >= peak_cutoff and smoothed[i] >= left and
                smoothed[i] >= right and
                (smoothed[i] > left or smoothed[i] > right)):
            maxima.append(i)
    min_bins = max(1, int(math.ceil(
        histogram_min_peak_separation_m / bin_width)))
    peaks = []
    for candidate in sorted(maxima, key=lambda i: smoothed[i], reverse=True):
        if all(abs(candidate - chosen) >= min_bins for chosen in peaks):
            peaks.append(candidate)
    peaks.sort()
    # A sharp valley between two prominent peaks indicates two separate
    # surfaces. Split at the center of a valley, not at the background peak.
    separators = []
    for front, rear in zip(peaks, peaks[1:]):
        between = smoothed[front + 1:rear]
        if between.size == 0:
            continue
        valley_value = float(np.min(between))
        if valley_value > float(histogram_valley_ratio) * min(
                smoothed[front], smoothed[rear]):
            continue
        # Use the middle of the flattest valley run, not its first bin.
        valley_bins = np.where(between <= valley_value + 1e-8)[0]
        valley = front + 1 + int(np.median(valley_bins))
        separators.append(valley)
    if separators:
        # A tiny near-field outlier must not cause us to select a farther wall.
        # Choose the NEAREST cluster only if it has meaningful support.
        boundary_m = float(edges[separators[0] + 1])
        chosen = valid & (roi < boundary_m)
        chosen_values = roi[chosen]
        fraction = float(chosen_values.size / values.size)
        if (chosen_values.size < int(min_valid_pixels) or
                fraction < float(foreground_min_fraction)):
            return None, (f'foreground too sparse: {chosen_values.size} pixels, '
                          f'{fraction:.0%} of valid ROI; raw spread={raw_spread:.2f}m')
        mode = 'histogram_foreground'
    else:
        if raw_spread > float(max_depth_spread_m):
            return None, (f'mixed depth {raw_spread:.2f}m, '
                          'no separable foreground histogram peak')
        # A small near-field mode may be too weak to form a histogram peak.
        # Do not silently return the background median if such a group exists.
        central_depth = float(np.median(values))
        small_near_group = int(np.count_nonzero(
            values < central_depth - histogram_min_peak_separation_m))
        if small_near_group >= max(int(min_valid_pixels),
                                   int(math.ceil(0.01 * values.size))):
            return None, (f'near-field pixels insufficient for confident mode: '
                          f'{small_near_group}/{values.size}; '
                          f'raw spread={raw_spread:.2f}m')
        # A single coherent mode: preserve the original median-depth method.
        chosen = valid
        chosen_values = values
        fraction = 1.0
        mode = 'single_depth_mode'
    selected_spread = float(np.quantile(chosen_values, .90) -
                            np.quantile(chosen_values, .10))
    if mode == 'histogram_foreground' and selected_spread > float(
            foreground_max_spread_m):
        return None, (f'foreground itself has mixed depth '
                      f'({selected_spread:.2f}m)')
    if mode == 'histogram_foreground':
        rows, cols = np.nonzero(chosen)
        pixel_u = float(np.median(cols) + 0.5)
        pixel_v = float(np.median(rows) + 0.5)
    else:
        pixel_u = float(roi.shape[1] / 2.0)
        pixel_v = float(roi.shape[0] / 2.0)
    return {
        'depth_m': float(np.median(chosen_values)),
        'u_roi': pixel_u,
        'v_roi': pixel_v,
        'raw_spread_m': raw_spread,
        'selected_spread_m': selected_spread,
        'support_fraction': fraction,
        'selected_pixels': int(chosen_values.size),
        'mode': mode,
    }, ''


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
            # Conservative 1-D depth histogram splitting (meters).
            ('histogram_bin_m', 0.10),
            ('histogram_min_peak_separation_m', 0.35),
            ('histogram_valley_ratio', 0.50),
            ('foreground_min_fraction', 0.15),
            ('foreground_max_spread_m', 0.60),
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
        # Visualization cache independent of perception validity.
        self.mission_state = None
        self.locked_marker_pose = None
        self.last_marker_pose = None
        self.nav2_active = False
        self.clear_when_nav_quiet = False
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
        mission_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/semantic/mission/state',
            self.on_mission_state, mission_qos)
        self.create_subscription(
            PoseStamped, '/semantic/mission/locked_target_pose',
            self.on_locked_marker_pose, mission_qos)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.on_nav_status, mission_qos)
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
        self.last_marker_pose = None


    def invalidate(self, reason):
        was_valid = self.valid
        self.valid = False
        self.position_history.clear()
        self.valid_pub.publish(Bool(data=False))
        if was_valid and not self._hold_marker():
            self.clear_marker()
        self.report(reason)


    def _hold_marker(self):
        # A marker is only a display of a previously measured position.
        return self.mission_state in ('APPROACHING', 'VERIFYING', 'CANCELLING')

    def _finish_marker(self):
        self.clear_when_nav_quiet = False
        self.clear_marker()
        self.locked_marker_pose = None

    def on_mission_state(self, msg):
        previous = self.mission_state
        state = msg.data.strip()
        if state == previous:
            return
        self.mission_state = state

        if state == 'APPROACHING':
            self.clear_when_nav_quiet = False
            # The mission's frozen map-frame target is authoritative when
            # beginning approach, even if live RGB-D localization expired.
            if self.locked_marker_pose is not None:
                self.publish_marker(self.locked_marker_pose)
            elif self.last_marker_pose is not None:
                self.publish_marker(self.last_marker_pose)
        elif state in ('SUCCEEDED', 'FAILED', 'CANCELLED', 'IDLE'):
            # Do not erase an active Nav2 navigation marker prematurely.
            if self.nav2_active:
                self.clear_when_nav_quiet = True
            else:
                self._finish_marker()
        elif state == 'PREPARING':
            # A new mission cannot reuse the previous mission's object pose.
            self.clear_when_nav_quiet = False
            self.locked_marker_pose = None
            self.clear_marker()

    def on_locked_marker_pose(self, msg):
        self.locked_marker_pose = msg
        # Guard against cross-topic subscription ordering: the locked pose
        # may arrive after the APPROACHING state event.
        if self._hold_marker() and self.last_marker_pose is None:
            self.publish_marker(msg)

    def on_nav_status(self, msg):
        active_states = {
            GoalStatus.STATUS_UNKNOWN,
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        self.nav2_active = any(s.status in active_states for s in msg.status_list)
        if self.clear_when_nav_quiet and not self.nav2_active:
            self._finish_marker()

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
            # One missed YOLO frame must not immediately destroy a
            # previously confirmed 3D map position during NBV braking.
            # This is a bounded hold, NOT a new object observation.
            if (self.valid and self.last_valid_wall_time > 0 and
                    time.monotonic() - self.last_valid_wall_time <
                    float(self.get_parameter('target_timeout_s').value)):
                self.report('HOLDING: last validated map target; waiting '
                            'for fresh RGB-D confirmation')
                return
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
        selection, error = select_foreground_depth(
            region,
            min_depth_m=min_depth,
            max_depth_m=max_depth,
            min_valid_pixels=int(self.get_parameter('min_valid_pixels').value),
            min_valid_fraction=float(self.get_parameter('min_valid_fraction').value),
            max_depth_spread_m=float(self.get_parameter('max_depth_spread_m').value),
            histogram_bin_m=float(self.get_parameter('histogram_bin_m').value),
            histogram_min_peak_separation_m=float(
                self.get_parameter('histogram_min_peak_separation_m').value),
            histogram_valley_ratio=float(
                self.get_parameter('histogram_valley_ratio').value),
            foreground_min_fraction=float(
                self.get_parameter('foreground_min_fraction').value),
            foreground_max_spread_m=float(
                self.get_parameter('foreground_max_spread_m').value),
        )
        if selection is None:
            self.invalidate('REJECTED: ' + error)
            return
        z = selection['depth_m']
        # The median foreground pixel may lie away from the box center,
        # especially for chairs with gaps between the seat and backrest.
        u = left + selection['u_roi']
        v = top + selection['v_roi']
        depth_note = (f"method={selection['mode']}, "
                      f"support={selection['support_fraction']:.0%}, "
                      f"raw_spread={selection['raw_spread_m']:.2f}m, "
                      f"selected_spread={selection['selected_spread_m']:.2f}m")
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
            if self.valid and not self._hold_marker():
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
                f'camera depth={z:.2f}m, dt={dt:.3f}s; {depth_note}'
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
            f'depth={z:.2f}m; dt={dt:.3f}s; {depth_note}'
        )


    def publish_marker(self, pose):
        # After a completed mission, wait for another mission before
        # displaying new detections; preserve live validity messages.
        if self.mission_state in ('IDLE', 'SUCCEEDED', 'FAILED', 'CANCELLED'):
            return
        self.last_marker_pose = pose
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
        marker.lifetime = Duration(seconds=0.0).to_msg()
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
