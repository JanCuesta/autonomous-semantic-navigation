"""Single-action, operator-triggered Nav2 executor for the V3 NBV preview.

Safety/validation scope:
- Never navigates on startup; must receive a Trigger service request.
- Reuses the exact Step 1 geometric scorer and its configured parameters.
- Sends at most ONE NavigateToPose action per Trigger request.
- No automatic next goal and NO confidence-frontier fallback yet.
- Reports Nav2 outcome and change in measured (not predicted) coverage.
- Preserves the independent RGB-D coverage node and navigation stack.
"""

import csv
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from .nbv_core import evaluate_candidates
from .nbv_execution_math import (best_executable_candidate,
                                 goal_pose_error, measured_coverage_gain)
from .nbv_preview_node import NBVPreviewNode, set_quaternion, yaw_from_quaternion


class NBVExecuteOnceNode(NBVPreviewNode):
    """Manual one-goal experiment; inherits Step 1 diagnostics."""

    def __init__(self, node_name='nbv_execute_once_node'):
        super().__init__(node_name, preview_only=False)
        self.declare_parameters('', [
            ('nav_action_name', '/navigate_to_pose'),
            ('goal_timeout_s', 45.0),
            # Use simulated navigation time in Isaac while retaining bounded
            # wall-time protection if /clock pauses or the simulator stalls.
            # At the measured Isaac RTF (~0.25), a 45 s simulated budget
            # already consumes ~180 wall seconds. Leave explicit margin for
            # callback/settling overhead while still detecting a frozen sim.
            ('simulation_wall_guard_s', 300.0),
            ('simulation_clock_stale_s', 15.0),
            ('result_settle_s', 2.0),
            ('goal_xy_tolerance_m', 0.15),
            ('goal_yaw_tolerance_deg', 15.0),
            # A retained, append-only record makes terminal outcomes
            # inspectable after the action callback has completed.
            ('execution_log_path', '~/.ros/asn_nbv_execution.csv'),
        ])
        self.nav_client = ActionClient(
            self, NavigateToPose,
            self.get_parameter('nav_action_name').value)
        self.execute_service = self.create_service(
            Trigger, '/semantic/nbv/execute_once', self.on_execute_once)
        self.cancel_service = self.create_service(
            Trigger, '/semantic/nbv/cancel', self.on_cancel)
        self.last_result_service = self.create_service(
            Trigger, '/semantic/nbv/last_result', self.on_last_result)
        self.execution_pub = self.create_publisher(
            String, '/semantic/nbv/execution_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # The watchdog uses steady wall time, even with paused /clock.
        self.watchdog = self.create_timer(
            0.25, self.check_action,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.busy = False
        self.goal_handle = None
        self.response_pending = False
        self.cancel_requested = False
        self.cancel_sent = False
        self.timeout_reported = False
        self.settle_until = None
        self.started_at = 0.0
        self.goal_started_at = 0.0
        self.goal_started_clock_ns = None
        self.last_clock_ns = None
        self.last_clock_advanced_at = 0.0
        self.selected = None
        self.selected_from = None
        self.coverage_before = None
        self.final_outcome = None
        self.goal_uuid = ''
        self.last_terminal_result = 'NO_TERMINAL_RESULT_RECORDED'
        self.log_path = Path(
            self.get_parameter('execution_log_path').value).expanduser()
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._ensure_log_header()
        self.execution_status(
            'READY: call /semantic/nbv/execute_once to move once')

    def _ensure_log_header(self):
        if self.log_path.exists() and self.log_path.stat().st_size:
            return
        with self.log_path.open('w', newline='') as stream:
            csv.writer(stream).writerow([
                'event', 'wall_monotonic_s', 'sim_time_s', 'goal_uuid',
                'detail', 'map_x', 'map_y', 'yaw_deg',
                'distance_remaining_m', 'navigation_time_s',
                'recoveries', 'wall_elapsed_s', 'sim_elapsed_s'])

    def log_event(self, event, detail='', feedback=None):
        """Append lifecycle evidence without relying on a transient topic."""
        map_x = map_y = yaw_deg = distance = nav_time = recoveries = ''
        if feedback is not None:
            pose = feedback.current_pose.pose
            map_x, map_y = pose.position.x, pose.position.y
            yaw_deg = math.degrees(math.atan2(
                2.0 * (pose.orientation.w * pose.orientation.z),
                1.0 - 2.0 * pose.orientation.z ** 2))
            distance = feedback.distance_remaining
            nav_time = (feedback.navigation_time.sec +
                        feedback.navigation_time.nanosec * 1e-9)
            recoveries = feedback.number_of_recoveries
        now_wall = time.monotonic()
        now_sim = self.get_clock().now().nanoseconds / 1e9
        wall_elapsed = (now_wall - self.goal_started_at
                        if self.goal_started_at else '')
        sim_elapsed = ((self.get_clock().now().nanoseconds -
                        self.goal_started_clock_ns) / 1e9
                       if self.goal_started_clock_ns is not None else '')
        with self.log_path.open('a', newline='') as stream:
            csv.writer(stream).writerow([
                event, now_wall, now_sim, self.goal_uuid, detail,
                map_x, map_y, yaw_deg, distance, nav_time, recoveries,
                wall_elapsed, sim_elapsed])

    def execution_status(self, message):
        self.execution_pub.publish(String(data=message))
        self.get_logger().info(message)
        self.log_event('STATUS', message)

    def robot_pose(self):
        frame = self.map_msg.header.frame_id
        tr = self.tf_buffer.lookup_transform(
            frame, self.get_parameter('robot_frame').value,
            Time(), timeout=Duration(seconds=0.0))
        t = tr.transform.translation
        return (t.x, t.y, yaw_from_quaternion(tr.transform.rotation))

    def check_maps(self):
        """Return diagnostic or None; never assume coverage matches map."""
        if self.map_msg is None or self.coverage_msg is None:
            return 'WAITING: /map or /semantic/coverage_map missing'
        if self.occupancy is None or self.coverage is None:
            return 'WAITING: map array missing'
        if self.occupancy.shape != self.coverage.shape:
            return 'MAP_MISMATCH: occupancy and coverage dimensions differ'
        a, b = self.map_msg.info, self.coverage_msg.info
        if (a.resolution != b.resolution or
                a.origin.position.x != b.origin.position.x or
                a.origin.position.y != b.origin.position.y or
                a.origin.orientation != b.origin.orientation or
                self.map_msg.header.frame_id !=
                self.coverage_msg.header.frame_id):
            return 'MAP_MISMATCH: resolution, origin, or frame differs'
        return None

    def score_current_viewpoints(self, pose):
        args = {
            'step_m': self.get_parameter('step_m').value,
            'clearance_m': self.get_parameter('clearance_m').value,
            'fov_deg': self.get_parameter('fov_deg').value,
            'max_range_m': self.get_parameter('max_range_m').value,
            'ray_count': self.get_parameter('ray_count').value,
            'decay_length_m': self.get_parameter('decay_length_m').value,
            'linear_speed': self.get_parameter('linear_speed_mps').value,
            'angular_speed': self.get_parameter('angular_speed_radps').value,
            'time_weight': self.get_parameter('time_weight').value,
        }
        return evaluate_candidates(
            self.occupancy, self.coverage, self.get_spec(), pose, **args)

    def on_execute_once(self, request, response):
        """Request ONE action; return acceptance, not navigation success."""
        if self.busy:
            response.success = False
            response.message = 'BUSY: another Nav2 action or settling is active'
            return response
        issue = self.check_maps()
        if issue is not None:
            response.success = False
            response.message = issue
            return response
        try:
            pose = self.robot_pose()
        except TransformException:
            response.success = False
            response.message = 'WAITING: map -> base_link transform unavailable'
            return response
        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            response.success = False
            response.message = (
                'WAITING: Nav2 action server unavailable')
            return response
        try:
            ranked = self.score_current_viewpoints(pose)
        except (ValueError, ZeroDivisionError) as exc:
            response.success = False
            response.message = f'PLANNER_ERROR: {exc}'
            return response
        minimum = float(self.get_parameter('minimum_gain_m2').value)
        best = best_executable_candidate(ranked, minimum)
        if best is None:
            response.success = False
            response.message = 'LOCAL_EXHAUSTED: no useful safe candidate'
            self.execution_status(response.message)
            return response

        # Snapshot measured coverage BEFORE goal dispatch for later comparison.
        self.selected = best
        self.selected_from = pose
        self.coverage_before = self.coverage.copy()
        self.started_at = time.monotonic()
        self.goal_started_at = 0.0
        self.goal_started_clock_ns = None
        self.last_clock_ns = None
        self.last_clock_advanced_at = time.monotonic()
        self.response_pending = True
        self.cancel_requested = False
        self.cancel_sent = False
        self.timeout_reported = False
        self.busy = True
        self.settle_until = None
        self.final_outcome = None
        self.goal_handle = None
        self.goal_uuid = ''

        x, y, yaw = best['pose']
        stamped = PoseStamped()
        stamped.header.frame_id = self.map_msg.header.frame_id
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose.position.x = float(x)
        stamped.pose.position.y = float(y)
        set_quaternion(stamped.pose, yaw)
        # One final snapshot: this is the precise viewpoint sent to Nav2.
        # Inherited preview timer is gated while busy/nav-active.
        self.publish_markers(ranked, best, stamped.header.frame_id)
        self.best_pub.publish(stamped)
        action_goal = NavigateToPose.Goal()
        action_goal.pose = stamped
        try:
            pending = self.nav_client.send_goal_async(
                action_goal, feedback_callback=self.on_navigation_feedback)
            pending.add_done_callback(self.on_goal_response)
        except Exception as exc:  # Log and allow a later manual retry.
            self.busy = False
            self.response_pending = False
            response.success = False
            response.message = f'SEND_FAILED: {exc}'
            self.execution_status(response.message)
            return response
        response.success = True
        response.message = (
            f"REQUESTED {best['action']} -> "
            f"({x:.2f},{y:.2f},{math.degrees(yaw):.1f} deg), "
            f"predicted gain={best['gain']:.3f} m2. "
            'Nav2 success is NOT confirmed yet.')
        self.execution_status(response.message)
        return response

    def on_navigation_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.goal_uuid = ''.join(
            f'{value:02x}' for value in feedback_message.goal_id.uuid)
        self.log_event('FEEDBACK', feedback=feedback)

    def on_goal_response(self, future):
        self.response_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.finish(f'NAV2_SEND_ERROR: {exc}')
            return
        if not handle.accepted:
            self.finish('NAV2_REJECTED: NavigateToPose goal not accepted')
            return
        self.goal_handle = handle
        if hasattr(handle, 'goal_id'):
            self.goal_uuid = ''.join(
                f'{value:02x}' for value in handle.goal_id.uuid)
        self.goal_started_at = time.monotonic()
        self.goal_started_clock_ns = self.get_clock().now().nanoseconds
        self.last_clock_ns = self.goal_started_clock_ns
        self.last_clock_advanced_at = self.goal_started_at
        self.execution_status('NAV2_ACCEPTED: executing one selected viewpoint')
        try:
            result_future = handle.get_result_async()
            result_future.add_done_callback(self.on_navigation_result)
        except Exception as exc:
            self.request_cancel(f'RESULT_ERROR: {exc}')
            return
        if self.cancel_requested:
            self.request_cancel('CANCEL: late goal acceptance')

    def on_last_result(self, request, response):
        response.success = self.last_terminal_result != 'NO_TERMINAL_RESULT_RECORDED'
        response.message = self.last_terminal_result
        return response

    def on_navigation_result(self, future):
        try:
            wrap = future.result()
        except Exception as exc:
            self.finish(f'NAV2_RESULT_ERROR: {exc}')
            return
        if wrap.status == GoalStatus.STATUS_SUCCEEDED:
            outcome = 'NAV2_SUCCEEDED'
        elif wrap.status == GoalStatus.STATUS_CANCELED:
            outcome = 'NAV2_CANCELED'
        elif wrap.status == GoalStatus.STATUS_ABORTED:
            outcome = 'NAV2_ABORTED'
        else:
            outcome = f'NAV2_STATUS_{wrap.status}'
        self.final_outcome = outcome
        self.settle_until = time.monotonic() + max(
            0.0, float(self.get_parameter('result_settle_s').value))
        self.execution_status(
            f'{outcome}: waiting briefly for actual camera coverage update')

    def request_cancel(self, reason):
        if not self.busy:
            return False
        if self.cancel_sent:
            return False
        self.cancel_requested = True
        self.execution_status(reason)
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
                self.cancel_sent = True
            except Exception as exc:
                self.execution_status(f'CANCEL_REQUEST_ERROR: {exc}')
        return True

    def on_cancel(self, request, response):
        if not self.busy or self.settle_until is not None:
            response.success = False
            response.message = 'No active navigation goal to cancel'
        else:
            response.success = self.request_cancel('USER_CANCEL_REQUESTED')
            response.message = ('Cancellation requested; wait for Nav2 result'
                                if response.success else 'Already canceling')
        return response

    def check_action(self):
        if not self.busy:
            return
        now = time.monotonic()
        if self.settle_until is not None:
            if now >= self.settle_until:
                self.summarize_observation()
            return
        if self.response_pending:
            # A late response will be immediately canceled when accepted.
            if now - self.started_at > 10.0 and not self.timeout_reported:
                self.cancel_requested = True
                self.timeout_reported = True
                self.execution_status(
                    'RESPONSE_TIMEOUT: awaiting Nav2 acceptance; will cancel '
                    'if it eventually accepts')
            return
        limit = float(self.get_parameter('goal_timeout_s').value)
        if self.goal_handle is None or self.timeout_reported:
            return
        if not self.get_parameter('use_sim_time').value:
            expired = now - self.goal_started_at > max(1.0, limit)
        else:
            clock_ns = self.get_clock().now().nanoseconds
            if clock_ns < self.last_clock_ns:
                self.request_cancel('CLOCK_RESET: canceling active goal')
                return
            if clock_ns > self.last_clock_ns:
                self.last_clock_ns = clock_ns
                self.last_clock_advanced_at = now
            if now - self.last_clock_advanced_at > float(
                    self.get_parameter('simulation_clock_stale_s').value):
                self.request_cancel('CLOCK_STALE: canceling active goal')
                return
            expired = ((clock_ns - self.goal_started_clock_ns) / 1e9 >
                       max(1.0, limit))
            if now - self.goal_started_at > float(
                    self.get_parameter('simulation_wall_guard_s').value):
                self.request_cancel('WALL_GUARD: canceling active goal')
                return
        if expired:
            self.timeout_reported = True
            self.request_cancel('NAV2_TIMEOUT: canceling this one goal')

    def summarize_observation(self):
        outcome = self.final_outcome or 'UNKNOWN_RESULT'
        actual_gain = None
        if self.coverage_before is not None and self.coverage is not None:
            try:
                actual_gain = measured_coverage_gain(
                    self.coverage_before, self.coverage,
                    self.map_msg.info.resolution)
            except ValueError:
                pass
        pose_message = 'final pose TF unavailable'
        try:
            x, y, yaw = self.robot_pose()
            gx, gy, gyaw = self.selected['pose']
            dist, angle = goal_pose_error((x, y, yaw),
                                          (gx, gy, gyaw))
            passed = (dist <= self.get_parameter('goal_xy_tolerance_m').value
                      and angle <=
                      self.get_parameter('goal_yaw_tolerance_deg').value)
            pose_message = (f'goal error={dist:.3f}m/{angle:.1f}deg; '
                            f'pose_check={"PASS" if passed else "FAIL"}')
        except TransformException:
            pass
        predicted = self.selected['gain'] if self.selected else 0.0
        if actual_gain is None:
            coverage_message = 'measured coverage unavailable'
        else:
            coverage_message = f'actual coverage gain={actual_gain:.4f}m2'
        self.finish(
            f'{outcome} | predicted gain={predicted:.4f}m2 | '
            f'{coverage_message} | {pose_message} | '
            'READY for another EXPLICIT manual trigger')

    def finish(self, message):
        self.log_event('TERMINAL', message)
        self.last_terminal_result = message
        self.busy = False
        self.response_pending = False
        self.goal_handle = None
        self.settle_until = None
        self.selected = None
        self.coverage_before = None
        self.execution_status(message)


def main(args=None):
    rclpy.init(args=args)
    node = NBVExecuteOnceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send a best-effort Nav2 cancellation before shutting down the client.
        if node.busy and node.goal_handle is not None:
            node.request_cancel('SHUTDOWN: canceling active Nav2 goal')
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
