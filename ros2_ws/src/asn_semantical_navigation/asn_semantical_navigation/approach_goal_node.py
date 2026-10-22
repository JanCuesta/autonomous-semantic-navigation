"""Preview-only approach-goal generator for a localized semantic target.

Generates candidate poses around a target in map coordinates, checks robot
clearance and a straight-line path against /map, then publishes an RViz preview.
Does NOT send Nav2 goals or /cmd_vel commands.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .nbv_core import GridSpec, candidate_poses, safe_footprint, safe_straight_segment
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_yaw(pose, yaw):
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)


def segment_distance(px, py, ax, ay, bx, by):
    """Shortest 2D distance from a point to segment A-B."""
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq == 0.0 else max(
        0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq)
    )
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class ApproachGoalNode(Node):
    def __init__(self):
        super().__init__('approach_goal_node')
        self.declare_parameters('', [
            ('stand_off_m', 0.60),
            ('clearance_m', 0.22),
            ('target_path_clearance_m', 0.30),
            ('target_timeout_s', 2.5),
            ('robot_frame', 'base_link'),
        ])
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_msg = None
        self.map_grid = None
        self.target = None
        self.target_valid = False
        self.target_received_at = 0.0
        self.last_status = None
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose'
        )

        self.latest_goal = None
        self.goal_at = 0.0
        self.nav_busy = False

        self.create_service(
            Trigger,
            '/semantic/approach/send_goal',
            self.send_best_goal
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(PoseStamped, '/semantic/perception/target_pose',
                                 self.on_target, 10)
        self.create_subscription(Bool, '/semantic/perception/target_valid',
                                 self.on_valid, 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/semantic/approach/goal', 10)
        self.valid_pub = self.create_publisher(
            Bool, '/semantic/approach/goal_valid', 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, '/semantic/approach/candidates', 10)
        self.status_pub = self.create_publisher(
            String, '/semantic/approach/status', 10)
        self.create_timer(1.0, self.plan)
        self.status('WAITING: map and valid target localization')

    def status(self, message):
        self.status_pub.publish(String(data=message))
        if message != self.last_status:
            self.get_logger().info(message)
            self.last_status = message

    def on_map(self, msg):
        if msg.info.width == 0 or msg.info.height == 0 or msg.info.resolution <= 0:
            return
        self.map_msg = msg
        self.map_grid = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)

    def on_target(self, msg):
        self.target = msg
        self.target_received_at = time.monotonic()

    def on_valid(self, msg):
        self.target_valid = bool(msg.data)
        if not self.target_valid:
            self.target = None

    def clear_markers(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.markers_pub.publish(MarkerArray(markers=[marker]))

    def no_goal(self, reason):
        self.valid_pub.publish(Bool(data=False))
        self.clear_markers()
        self.status(reason)
        self.latest_goal = None

    def plan(self):
        if self.map_msg is None or self.map_grid is None:
            self.no_goal('WAITING: /map')
            return
        if (not self.target_valid or self.target is None or
                time.monotonic() - self.target_received_at >
                float(self.get_parameter('target_timeout_s').value)):
            self.latest_goal = None
            self.no_goal('WAITING: recent valid target pose')
            return
        frame = self.map_msg.header.frame_id
        if self.target.header.frame_id != frame:
            self.no_goal('REJECTED: target is not in the map frame')
            return
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                frame, self.get_parameter('robot_frame').value,
                Time(), timeout=Duration(seconds=0.0))
        except TransformException:
            self.no_goal('WAITING: map-to-robot TF')
            return

        rx = robot_tf.transform.translation.x
        ry = robot_tf.transform.translation.y
        tx = self.target.pose.position.x
        ty = self.target.pose.position.y
        if not all(math.isfinite(x) for x in (rx, ry, tx, ty)):
            self.no_goal('REJECTED: non-finite map coordinates')
            return
        origin = self.map_msg.info.origin
        spec = GridSpec(
            self.map_msg.info.resolution,
            origin.position.x, origin.position.y,
            yaw_from_quaternion(origin.orientation))
        clearance = float(self.get_parameter('clearance_m').value)
        target_clearance = float(
            self.get_parameter('target_path_clearance_m').value)
        spacing = float(self.get_parameter('stand_off_m').value)
        if min(clearance, target_clearance, spacing) <= 0:
            self.no_goal('REJECTED: spacing and clearances must be positive')
            return

        # candidate_poses also returns in-place turns at (tx,ty): ignore those.
        candidates = []
        for x, y, _, name in candidate_poses((tx, ty, 0.0), step_m=spacing):
            if math.hypot(x - tx, y - ty) < 0.01:
                continue
            free = safe_footprint(self.map_grid, spec, x, y, clearance)
            straight = free and safe_straight_segment(
                self.map_grid, spec, (rx, ry), (x, y), clearance)
            avoids_object = segment_distance(
                tx, ty, rx, ry, x, y) >= target_clearance
            valid = free and straight and avoids_object
            heading = math.atan2(ty - y, tx - x)
            candidates.append((x, y, heading, name, valid))

        feasible = [c for c in candidates if c[4]]
        best = min(feasible, key=lambda c: math.hypot(c[0] - rx, c[1] - ry)) \
            if feasible else None
        self.publish_markers(frame, candidates, best)
        if best is None:
            self.latest_goal = None
            self.valid_pub.publish(Bool(data=False))
            self.status('NO_GOAL: no clear straight-line approach on static map')
            return

        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(best[0])
        goal.pose.position.y = float(best[1])
        goal.pose.position.z = 0.0
        set_yaw(goal.pose, best[2])
        self.latest_goal = goal
        self.goal_at = time.monotonic()
        self.goal_pub.publish(goal)
        self.valid_pub.publish(Bool(data=True))
        self.status(
            f'PREVIEW: goal=({best[0]:.2f},{best[1]:.2f}), '
            f'yaw={math.degrees(best[2]):.0f}deg; '
            f'{len(feasible)}/{len(candidates)} straight-line candidates safe')

    def send_best_goal(self, request, response):
        response.success = False

        if (self.nav_busy or self.latest_goal is None
                or not self.target_valid
                or time.monotonic() - self.goal_at > 2.0):
            response.message = 'No fresh goal available, or Nav2 is busy'
            return response

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            response.message = 'Nav2 action server unavailable'
            return response

        goal = NavigateToPose.Goal()
        goal.pose = self.latest_goal
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.nav_busy = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

        response.success = True
        response.message = 'Approach goal submitted to Nav2'
        return response


    def on_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.nav_busy = False
                self.status('NAV2: Goal rejected')
                return

            self.status('NAV2: Goal accepted, navigating')
            handle.get_result_async().add_done_callback(self.on_nav_result)

        except Exception as exc:
            self.nav_busy = False
            self.status(f'NAV2: Goal submission failed: {exc}')


    def on_nav_result(self, future):
        try:
            status = future.result().status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.status('NAV2: Approach goal reached')
            else:
                self.status(f'NAV2: Navigation ended with status {status}')

        except Exception as exc:
            self.status(f'NAV2: Result error: {exc}')
        finally:
            self.nav_busy = False


    def publish_markers(self, frame, candidates, best):
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        for i, c in enumerate(candidates):
            x, y, heading, _, valid = c
            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = stamp
            marker.ns = 'approach_candidates'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.12
            set_yaw(marker.pose, heading)
            marker.scale.x = 0.20
            marker.scale.y = 0.055
            marker.scale.z = 0.055
            marker.color.a = 0.95
            if best is c:
                marker.color.g = 1.0  # green: selected
            elif valid:
                marker.color.b = 1.0  # blue: valid alternative
            else:
                marker.color.r = marker.color.g = marker.color.b = 0.5
            marker.lifetime = Duration(seconds=2.0).to_msg()
            markers.append(marker)
        self.markers_pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = ApproachGoalNode()
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
