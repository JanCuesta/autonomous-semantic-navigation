"""RViz-only preview of local confidence-guided Next-Best-View candidates.

Reads /map, /semantic/coverage_map, and map -> base_link TF.
Never sends a velocity command or NavigateToPose action.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .nbv_core import (GridSpec, evaluate_candidates, predicted_coverage)


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def set_quaternion(pose, yaw):
    pose.orientation.z = math.sin(yaw / 2)
    pose.orientation.w = math.cos(yaw / 2)


class NBVPreviewNode(Node):
    def __init__(self, node_name='nbv_preview_node', preview_only=True):
        super().__init__(node_name)
        self.declare_parameters('', [
            ('step_m', 0.6),
            ('clearance_m', 0.22),
            ('fov_deg', 70.0),  # Replace with verified camera intrinsics.
            ('max_range_m', 3.0),
            ('ray_count', 71),
            ('decay_length_m', 1.5),
            ('linear_speed_mps', 0.2),
            ('angular_speed_radps', 1.0),
            ('time_weight', 0.005),
            ('minimum_gain_m2', 0.002),
            ('preview_hz', 1.0),
            ('robot_frame', 'base_link'),
        ])
        qos_map = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.occupancy = None
        self.coverage = None
        self.map_msg = None
        self.coverage_msg = None
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos_map)
        self.create_subscription(OccupancyGrid, '/semantic/coverage_map',
                                 self.on_coverage, qos_map)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic/nbv/candidates', 10)
        self.best_pub = self.create_publisher(
            PoseStamped, '/semantic/nbv/best_pose', 10)
        self.predicted_pub = self.create_publisher(
            OccupancyGrid, '/semantic/nbv/predicted_coverage_map', 10)
        self.status_pub = self.create_publisher(
            String, '/semantic/nbv/status', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._previous_status = None
        hz = max(0.1, float(self.get_parameter('preview_hz').value))
        self.timer = self.create_timer(1.0 / hz, self.run_preview)
        if preview_only:
            self.get_logger().info('NBV PREVIEW ONLY — no navigation commands')
        else:
            self.get_logger().info('NBV EXECUTOR — requires explicit execute_once service')

    def on_map(self, msg):
        self.map_msg = msg
        self.occupancy = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)

    def on_coverage(self, msg):
        self.coverage_msg = msg
        self.coverage = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)

    def get_spec(self):
        origin = self.map_msg.info.origin
        return GridSpec(
            self.map_msg.info.resolution,
            origin.position.x, origin.position.y,
            yaw_from_quaternion(origin.orientation))

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))
        if text != self._previous_status:
            self.get_logger().info(text)
            self._previous_status = text

    def run_preview(self):
        if self.occupancy is None or self.coverage is None:
            return
        if self.occupancy.shape != self.coverage.shape:
            self.publish_status('WAITING: map and coverage shapes differ')
            return
        a, b = self.map_msg.info, self.coverage_msg.info
        if (a.resolution != b.resolution or
                a.origin.position.x != b.origin.position.x or
                a.origin.position.y != b.origin.position.y or
                self.map_msg.header.frame_id !=
                self.coverage_msg.header.frame_id):
            self.publish_status('WAITING: map and coverage metadata differ')
            return
        frame = self.map_msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                frame, self.get_parameter('robot_frame').value,
                Time(), timeout=Duration(seconds=0.0))
        except TransformException:
            self.publish_status('WAITING: map->robot TF unavailable')
            return

        tr = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        current_pose = (tr.x, tr.y, yaw)
        spec = self.get_spec()
        results = evaluate_candidates(
            self.occupancy, self.coverage, spec, current_pose,
            step_m=self.get_parameter('step_m').value,
            clearance_m=self.get_parameter('clearance_m').value,
            fov_deg=self.get_parameter('fov_deg').value,
            max_range_m=self.get_parameter('max_range_m').value,
            ray_count=self.get_parameter('ray_count').value,
            decay_length_m=self.get_parameter('decay_length_m').value,
            linear_speed=self.get_parameter('linear_speed_mps').value,
            angular_speed=self.get_parameter('angular_speed_radps').value,
            time_weight=self.get_parameter('time_weight').value)
        best = results[0]
        useful = (best['safe'] and best['score'] > 0 and
                  best['gain'] >= self.get_parameter('minimum_gain_m2').value)
        self.publish_markers(results, best if useful else None, frame)
        if not useful:
            self.publish_status(
                'LOCAL_EXHAUSTED: no positive-scoring viewpoint')
            return
        x, y, heading = best['pose']
        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        set_quaternion(goal.pose, heading)
        self.best_pub.publish(goal)
        predicted = OccupancyGrid()
        predicted.header = goal.header
        predicted.info = self.map_msg.info
        predicted.data = predicted_coverage(
            self.coverage, best['visibility']).ravel().astype(int).tolist()
        self.predicted_pub.publish(predicted)
        self.publish_status(
            f"BEST {best['action']}: gain={best['gain']:.4f} m2, "
            f"time~{best['seconds']:.2f}s, score={best['score']:.4f}")

    def publish_markers(self, results, best, frame):
        stamp = self.get_clock().now().to_msg()
        delete = Marker()
        delete.action = Marker.DELETEALL
        output = MarkerArray(markers=[delete])
        for index, item in enumerate(results):
            x, y, yaw = item['pose']
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = stamp
            marker.ns = 'nbv_viewpoints'
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.12
            set_quaternion(marker.pose, yaw)
            marker.scale.x = 0.24
            marker.scale.y = 0.055
            marker.scale.z = 0.055
            marker.color.a = 0.95
            if not item['safe']:
                marker.color.r, marker.color.g, marker.color.b = 0.5, 0.5, 0.5
            elif item is best:
                marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.2
                marker.pose.position.z = 0.19
                marker.scale.x = 0.34
            else:
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.5, 1.0
            output.markers.append(marker)
        self.marker_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = NBVPreviewNode()
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
