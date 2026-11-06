"""Short-range reactive replacement for Nav2's NavigateToPose action.

Uses odom only as a drifting local coordinate frame; never consults /map,
SLAM, a global planner, or a costmap. NOT a substitute for hardware E-stop.
Default output is isolated /mapless/cmd_vel and motion is DISABLED.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from .mapless_core import (relative_scan_points, sector_clearance,
                           segment_clear, valid_scan_values, wrap)


def get_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class MaplessMotion(Node):
    def __init__(self):
        super().__init__('mapless_motion_node')
        self.declare_parameters('', [
            ('enable_motion', False),
            ('scan_topic', '/scan'),
            ('cmd_vel_topic', '/mapless/cmd_vel'),
            ('twist_stamped', True),
            ('robot_frame', 'base_link'),
            ('odom_frame', 'odom'),
            ('max_goal_distance_m', 3.0),
            ('max_linear_mps', 0.12),
            ('max_angular_rps', 0.28),
            ('robot_clearance_m', 0.34),
            ('max_scan_age_s', 0.65),
            ('max_tf_age_s', 0.85),
            ('goal_timeout_s', 90.0),
            ('progress_timeout_s', 12.0),
            ('xy_tolerance_m', 0.12),
            ('yaw_tolerance_rad', 0.16),
        ])
        self.group = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self, spin_thread=False)
        self.scan = None
        self.scan_at = 0.0
        self.active = False
        self.inspection_slowdown = False
        self.stamped = bool(self.p('twist_stamped'))
        message_type = TwistStamped if self.stamped else Twist
        self.cmd = self.create_publisher(
            message_type, self.p('cmd_vel_topic'), 10)
        self.create_subscription(
            LaserScan, self.p('scan_topic'), self.on_scan,
            qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(
            Bool, '/semantic/mission/inspection_slowdown',
            self.on_slowdown, 10, callback_group=self.group)
        self.create_timer(0.10, self.idle_stop, callback_group=self.group)
        self.server = ActionServer(
            self, NavigateToPose, '/mapless/navigate_to_pose',
            execute_callback=self.execute, goal_callback=self.accept,
            cancel_callback=self.on_cancel, callback_group=self.group)
        self.get_logger().warning(
            'Experimental reactive motion: ENABLED only after explicit '
            'enable_motion:=true. Output is isolated by default. '
            'Requires independent stop/watchdog and supervised simulation.')

    def p(self, name):
        return self.get_parameter(name).value

    def on_scan(self, scan):
        self.scan, self.scan_at = scan, time.monotonic()

    def on_slowdown(self, msg):
        self.inspection_slowdown = bool(msg.data)

    def fresh_scan(self):
        return (self.scan is not None and self.scan_at > 0 and
                time.monotonic() - self.scan_at <= self.p('max_scan_age_s') and
                len(self.scan.ranges) >= 8 and
                self.scan.angle_increment > 0.0 and
                sum(1 for _ in valid_scan_values(
                    self.scan.ranges, self.scan.range_min,
                    self.scan.range_max)) >= 8)

    def pose(self):
        tf = self.tf.lookup_transform(self.p('odom_frame'),
                                      self.p('robot_frame'), Time())
        stamp_ns = tf.header.stamp.sec * 1000000000 + tf.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns > 0 and now_ns > 0 and (
                now_ns - stamp_ns > self.p('max_tf_age_s') * 1e9 or
                stamp_ns - now_ns > 2.0e9):
            raise TransformException('Odometry TF stale or in the future')
        pos = tf.transform.translation
        return pos.x, pos.y, get_yaw(tf.transform.rotation)

    def laser_relative_geometry(self):
        scan = self.scan
        laser = self.tf.lookup_transform(
            self.p('robot_frame'), scan.header.frame_id, Time())
        trans = laser.transform.translation
        laser_yaw = get_yaw(laser.transform.rotation)
        points = list(relative_scan_points(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
            trans.x, trans.y, laser_yaw))
        return scan, laser_yaw, points

    def velocity(self, linear=0.0, angular=0.0):
        linear = max(-self.p('max_linear_mps'), min(
            self.p('max_linear_mps'), float(linear)))
        angular = max(-self.p('max_angular_rps'), min(
            self.p('max_angular_rps'), float(angular)))
        if self.inspection_slowdown:
            linear *= 0.5
            angular *= 0.5
        msg = TwistStamped() if self.stamped else Twist()
        twist = msg.twist if self.stamped else msg
        if self.stamped:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.p('robot_frame')
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd.publish(msg)

    def stop(self):
        self.velocity()

    def idle_stop(self):
        if not self.active:
            self.stop()

    def accept(self, request):
        if self.active or not self.p('enable_motion') or not self.fresh_scan():
            return GoalResponse.REJECT
        goal = request.pose
        if goal.header.frame_id != self.p('odom_frame'):
            return GoalResponse.REJECT
        try:
            x, y, _ = self.pose()
            self.laser_relative_geometry()
        except TransformException:
            return GoalResponse.REJECT
        gx, gy = goal.pose.position.x, goal.pose.position.y
        q = goal.pose.orientation
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if (not all(map(math.isfinite, (gx, gy, q.x, q.y, q.z, q.w))) or
                not 0.95 <= norm <= 1.05 or
                math.hypot(gx - x, gy - y) > self.p('max_goal_distance_m')):
            return GoalResponse.REJECT
        self.active = True
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, handle):
        result = NavigateToPose.Result()
        started = time.monotonic()
        last_progress_at = started
        best_distance = float('inf')

        def abort(reason):
            result.error_code = 9000
            result.error_msg = reason
            self.get_logger().warning('Mapless motion aborted: ' + reason)
            self.stop()
            if handle.is_active:
                handle.abort()
            return result

        try:
            goal = handle.request.pose.pose
            gx, gy = goal.position.x, goal.position.y
            target_yaw = get_yaw(goal.orientation)
            while rclpy.ok():
                if handle.is_cancel_requested:
                    self.stop()
                    handle.canceled()
                    return result
                if (time.monotonic() - started > self.p('goal_timeout_s') or
                        not self.fresh_scan()):
                    return abort('Action timeout or stale LiDAR')
                try:
                    x, y, heading = self.pose()
                    scan, laser_yaw, points = self.laser_relative_geometry()
                except TransformException:
                    return abort('Required odom/base/laser transform unavailable')
                distance = math.hypot(gx - x, gy - y)
                if distance + 0.025 < best_distance:
                    best_distance = distance
                    last_progress_at = time.monotonic()
                if (distance > self.p('xy_tolerance_m') and
                        time.monotonic() - last_progress_at >
                        self.p('progress_timeout_s')):
                    return abort('No odom distance progress within watchdog')
                yaw_error = wrap(target_yaw - heading)
                feedback = NavigateToPose.Feedback()
                feedback.distance_remaining = distance
                feedback.navigation_time.sec = int(time.monotonic() - started)
                feedback.current_pose.header.frame_id = self.p('odom_frame')
                feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
                feedback.current_pose.pose.position.x = x
                feedback.current_pose.pose.position.y = y
                feedback.current_pose.pose.orientation.z = math.sin(heading / 2)
                feedback.current_pose.pose.orientation.w = math.cos(heading / 2)
                handle.publish_feedback(feedback)
                if distance <= self.p('xy_tolerance_m'):
                    if abs(yaw_error) <= self.p('yaw_tolerance_rad'):
                        self.stop()
                        handle.succeed()
                        return result
                    # In-place turning requires a clear circular footprint.
                    if not segment_clear(points, 0.0, 0.0,
                                         self.p('robot_clearance_m')):
                        return abort('Unsafe clearance for final rotation')
                    self.velocity(angular=max(-0.20, min(0.20, 0.8*yaw_error)))
                else:
                    target_bearing = math.atan2(gy-y, gx-x)
                    delta = wrap(target_bearing - heading)
                    if abs(delta) > 0.25:
                        if not segment_clear(points, 0.0, 0.0,
                                             self.p('robot_clearance_m')):
                            return abort('Unsafe clearance for heading turn')
                        self.velocity(angular=max(-0.24, min(0.24, 0.8*delta)))
                    else:
                        # Never infer that unseen space is safe. Check the
                        # current travel direction AND nearby point returns.
                        clear = sector_clearance(
                            scan.ranges, scan.angle_min, scan.angle_increment,
                            scan.range_min, scan.range_max,
                            -laser_yaw, half_width=0.26)
                        lookahead = min(distance, 0.60)
                        if (clear < lookahead + self.p('robot_clearance_m') or
                                not segment_clear(points, lookahead, 0.0,
                                                  self.p('robot_clearance_m'))):
                            return abort('Obstacle or unobserved segment ahead')
                        self.velocity(linear=min(
                            self.p('max_linear_mps'), max(0.035, 0.55*distance)),
                            angular=max(-0.10, min(0.10, 0.65*delta)))
                time.sleep(0.05)
            return abort('ROS shutdown during motion')
        except Exception as exc:
            return abort(f'Unexpected controller exception: {exc}')
        finally:
            self.stop()
            self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = MaplessMotion()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
