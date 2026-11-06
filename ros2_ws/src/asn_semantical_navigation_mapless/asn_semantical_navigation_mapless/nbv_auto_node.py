"""Mapless NBV search retaining the original mission-manager interface.

Maintains only a bounded history of odom viewpoints, not an occupancy map.
Uses the reactive /mapless/navigate_to_pose action for short steps/turns.
"""
import json
import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .mapless_core import choose_view, valid_scan_values


def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y),
                      1-2*(q.y*q.y+q.z*q.z))


class MaplessNBV(Node):
    def __init__(self):
        super().__init__('nbv_auto_node')
        self.declare_parameters('', [
            ('scan_topic', '/scan'),
            ('robot_frame', 'base_link'),
            ('odom_frame', 'odom'),
            ('nav_action_name', '/mapless/navigate_to_pose'),
            ('require_target', True),
            ('require_live_detector', True),
            ('detector_stale_s', 5.0),
            ('scan_stale_s', 0.65),
            ('auto_max_goals', 25),
            ('auto_dwell_s', 1.0),
            ('step_m', 0.55),
            ('clearance_m', 0.34),
            ('max_visited', 100),
            ('stop_on_2d_confirmation', False),
        ])
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.scan = None
        self.scan_at = 0.0
        self.detector_at = 0.0
        self.target = ''
        self.confirmed = False
        self.localized = False
        self.target_found = False
        self.enabled = False
        self.busy = False
        self.pending = False
        self.cancel_on_accept = False
        self.goal_handle = None
        self.goals_sent = 0
        self.next_at = float('inf')
        self.visited = deque(maxlen=max(10, int(self.p('max_visited'))))
        self.nav = ActionClient(self, NavigateToPose,
                                self.p('nav_action_name'))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LaserScan, self.p('scan_topic'),
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/semantic/search/target_class',
                                 self.on_target, qos)
        self.create_subscription(String, '/semantic/search/detections_json',
                                 self.on_detector, 10)
        self.create_subscription(Bool, '/semantic/nbv/target_found',
                                 self.on_found, 10)
        self.create_subscription(Bool, '/semantic/perception/target_valid',
                                 self.on_valid, 10)
        self.create_service(SetBool, '/semantic/nbv/auto_enable',
                            self.on_enable)
        self.create_service(Trigger, '/semantic/nbv/idle_check',
                            self.on_idle)
        self.create_service(Trigger, '/semantic/nbv/reset_search',
                            self.on_reset)
        self.create_service(Trigger, '/semantic/nbv/cancel',
                            self.on_cancel)
        self.pub = self.create_publisher(
            String, '/semantic/nbv/execution_status', 10)
        self.best_pub = self.create_publisher(
            PoseStamped, '/semantic/nbv/best_pose', 10)
        self.create_timer(0.25, self.tick)
        self.status('AUTO_DISARMED: mapless search awaiting mission')

    def p(self, name):
        return self.get_parameter(name).value

    def status(self, s):
        self.pub.publish(String(data=s))
        self.get_logger().info(s)

    def on_scan(self, msg):
        self.scan, self.scan_at = msg, time.monotonic()

    def scan_ready(self):
        return (self.scan is not None and self.scan_at > 0 and
                time.monotonic()-self.scan_at < self.p('scan_stale_s') and
                len(self.scan.ranges) >= 8 and
                sum(1 for _ in valid_scan_values(
                    self.scan.ranges, self.scan.range_min,
                    self.scan.range_max)) >= 8)

    def pose(self):
        t = self.tf.lookup_transform(self.p('odom_frame'),
                                     self.p('robot_frame'), Time())
        p = t.transform.translation
        return p.x, p.y, yaw_of(t.transform.rotation)

    def laser_yaw(self):
        t = self.tf.lookup_transform(self.p('robot_frame'),
                                     self.scan.header.frame_id, Time())
        return yaw_of(t.transform.rotation)

    def on_target(self, msg):
        target = msg.data.strip()
        if target != self.target:
            self.disarm('TARGET_CHANGED: cancel prior search')
            self.target = target
            self.confirmed = self.localized = self.target_found = False
            self.detector_at = 0.0
            self.status(f'TARGET_SELECTED: {target}')

    def on_detector(self, msg):
        try:
            record = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(record, dict) or record.get('target') != self.target:
            return
        self.detector_at = time.monotonic()
        self.confirmed = record.get('confirmed') is True
        if self.confirmed and (self.p('stop_on_2d_confirmation') or
                               self.localized):
            self.found()

    def detector_ready(self):
        return (self.target and self.detector_at > 0 and
                time.monotonic()-self.detector_at < self.p('detector_stale_s'))

    def on_found(self, msg):
        if msg.data and self.confirmed and self.detector_ready() and (
                self.p('stop_on_2d_confirmation') or self.localized):
            self.found()

    def on_valid(self, msg):
        self.localized = bool(msg.data)
        if self.localized and self.detector_ready():
            self.found()

    def found(self):
        self.target_found = True
        if self.enabled:
            self.disarm('TARGET_DETECTED: mapless NBV cancellation requested')

    def idle(self):
        return not self.enabled and not self.busy and not self.pending

    def on_idle(self, request, response):
        response.success = self.idle()
        response.message = 'NBV_IDLE' if response.success else 'NBV_BUSY'
        return response

    def on_reset(self, request, response):
        if not self.idle():
            response.success = False
            response.message = 'Cannot reset until action cancellation finishes'
            return response
        self.confirmed = self.localized = self.target_found = False
        # Preserve a still-fresh same-target detector heartbeat so the
        # supervisor cannot race the next camera frame when re-arming.
        self.visited.clear()
        self.goals_sent = 0
        response.success = True
        response.message = 'NBV_SEARCH_RESET'
        return response

    def disarm(self, reason):
        self.enabled = False
        self.next_at = float('inf')
        self.status(reason)
        if self.busy or self.pending:
            self.cancel_on_accept = True
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()

    def on_enable(self, request, response):
        if not request.data:
            self.disarm('AUTO_STOP_REQUESTED')
            response.success = True
            response.message = 'Disarmed; poll idle_check until terminal action'
            return response
        if not self.idle():
            response.success = False
            response.message = 'NBV action already active'
            return response
        if self.target_found or (self.p('require_target') and not self.target):
            response.success = False
            response.message = 'Select/reset a target before arming'
            return response
        if self.p('require_live_detector') and not self.detector_ready():
            response.success = False
            response.message = 'YOLO stream stale or missing'
            return response
        if not self.scan_ready():
            response.success = False
            response.message = 'LiDAR scan stale or missing'
            return response
        try:
            self.pose()
            self.laser_yaw()
        except TransformException:
            response.success = False
            response.message = 'Odom/laser TF unavailable'
            return response
        if not self.nav.wait_for_server(timeout_sec=0.2):
            response.success = False
            response.message = 'Mapless motion action unavailable'
            return response
        self.enabled = True
        self.cancel_on_accept = False
        self.next_at = time.monotonic() + 0.3
        response.success = True
        response.message = 'AUTO_ARMED: mapless NBV'
        self.status(response.message)
        return response

    def on_cancel(self, request, response):
        self.disarm('AUTO_CANCEL_REQUESTED')
        response.success = True
        response.message = 'Cancellation requested, check idle_check'
        return response

    def tick(self):
        if not self.enabled:
            return
        # Sensor failures cancel even an action already in flight.
        if self.p('require_live_detector') and not self.detector_ready():
            self.disarm('AUTO_PAUSED: stale YOLO stream')
            return
        if not self.scan_ready():
            self.disarm('AUTO_PAUSED: stale LiDAR')
            return
        if self.busy or self.pending or time.monotonic() < self.next_at:
            return
        if self.target_found:
            self.disarm('TARGET_ALREADY_FOUND')
            return
        if self.goals_sent >= self.p('auto_max_goals'):
            self.disarm('AUTO_BUDGET_REACHED: exploration budget exhausted')
            return
        try:
            pose = self.pose()
            scan = self.scan
            view = choose_view(
                scan.ranges, scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max, pose, tuple(self.visited),
                self.p('step_m'), self.p('clearance_m'), self.laser_yaw())
        except (TransformException, ValueError) as exc:
            self.disarm(f'AUTO_PAUSED: pose/scan unavailable: {exc}')
            return
        if view is None:
            self.disarm('AUTO_PAUSED: no observable safe candidate')
            return
        if not self.visited:
            self.visited.append(pose)
        self.visited.append((view.x, view.y, view.yaw))
        msg = PoseStamped()
        msg.header.frame_id = self.p('odom_frame')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = view.x
        msg.pose.position.y = view.y
        msg.pose.orientation.z = math.sin(view.yaw / 2)
        msg.pose.orientation.w = math.cos(view.yaw / 2)
        self.best_pub.publish(msg)
        request = NavigateToPose.Goal()
        request.pose = msg
        self.pending = True
        self.busy = True
        self.goals_sent += 1
        self.next_at = float('inf')
        self.status(f'REQUESTED {view.kind}: odom=({view.x:.2f},'
                    f'{view.y:.2f},{math.degrees(view.yaw):.0f}deg), '
                    f'novelty score={view.score:.2f}')
        try:
            future = self.nav.send_goal_async(request)
            future.add_done_callback(self.on_goal_response)
        except Exception as exc:
            self.pending = self.busy = False
            self.disarm(f'AUTO_PAUSED: action dispatch failed: {exc}')

    def on_goal_response(self, future):
        self.pending = False
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.complete(f'NAV2_SEND_ERROR: {exc}', False)
            return
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.complete('NAV2_REJECTED: local motion safety gate', False)
            return
        self.status('NAV2_ACCEPTED: local reactive goal')
        self.goal_handle.get_result_async().add_done_callback(self.on_result)
        if self.cancel_on_accept:
            self.goal_handle.cancel_goal_async()

    def on_result(self, future):
        try:
            status = future.result().status
            from action_msgs.msg import GoalStatus
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.complete('NAV2_SUCCEEDED | mapless local movement', True)
            elif status == GoalStatus.STATUS_CANCELED:
                self.complete('NAV2_CANCELED | mapless local movement', False)
            else:
                self.complete(f'NAV2_ABORTED | mapless status {status}', False)
        except Exception as exc:
            self.complete(f'NAV2_RESULT_ERROR: {exc}', False)

    def complete(self, message, success):
        self.goal_handle = None
        self.busy = self.pending = False
        self.cancel_on_accept = False
        self.status(message)
        if self.enabled:
            if not success:
                self.disarm('AUTO_PAUSED: mapless motion unsuccessful')
            else:
                self.next_at = time.monotonic() + self.p('auto_dwell_s')


def main(args=None):
    rclpy.init(args=args)
    node = MaplessNBV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.disarm('SHUTDOWN')
        rclpy.spin_once(node, timeout_sec=0.3)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
