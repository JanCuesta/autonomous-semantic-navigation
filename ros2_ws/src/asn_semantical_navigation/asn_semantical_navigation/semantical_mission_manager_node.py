"""Explicit-start semantic search -> RGB-D localization -> Nav2 approach.

This supervisor coordinates existing ROS nodes; it never publishes /cmd_vel.
A user must select a target class and call /semantic/mission/start.

Search motion belongs to nbv_auto_node. Approach motion belongs to THIS node,
which submits NavigateToPose after NBV confirms its action is terminal, and
after global Nav2 action status is quiescent. No external Nav2 goal client
(including the old /mission/goal manager, RViz and approach/send_goal service)
may run concurrently. This is supervised simulation code, not a hardware E-stop.

Requires nbv_auto_node's /semantic/nbv/reset_search and /idle_check services.
Integration via setup.py and launch files is intentionally deferred.
"""

import json
import math
import time
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class State(str, Enum):
    IDLE = 'IDLE'
    PREPARING = 'PREPARING'
    SEARCHING = 'SEARCHING'
    STOPPING_SEARCH = 'STOPPING_SEARCH'
    LOCALIZING = 'LOCALIZING'
    WAITING_APPROACH = 'WAITING_APPROACH'
    APPROACHING = 'APPROACHING'
    VERIFYING = 'VERIFYING'
    CANCELLING = 'CANCELLING'
    SUCCEEDED = 'SUCCEEDED'
    FAILED = 'FAILED'
    CANCELLED = 'CANCELLED'


TERMINAL = {State.IDLE, State.SUCCEEDED, State.FAILED, State.CANCELLED}
NAV_ACTIVE = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
              GoalStatus.STATUS_CANCELING, GoalStatus.STATUS_UNKNOWN}


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def valid_pose(msg):
    if msg is None or msg.header.frame_id != 'map':
        return False
    p, q = msg.pose.position, msg.pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
    return all(math.isfinite(v) for v in values) and 0.95 < norm < 1.05


class SemanticMissionManager(Node):
    """ROS mission state machine; one active semantic mission at a time."""

    def __init__(self):
        super().__init__('semantical_mission_manager_node')
        for name, default in [
            ('prepare_timeout_s', 30.0),
            ('detector_stale_s', 5.0),
            ('stop_timeout_s', 25.0),
            ('localization_timeout_s', 12.0),
            ('target_pose_stale_s', 2.5),
            ('target_snapshot_max_age_s', 12.0),
            ('approach_preview_timeout_s', 8.0),
            ('approach_goal_stale_s', 2.5),
            ('approach_timeout_s', 120.0),
            ('verify_timeout_s', 4.0),
            ('cancel_timeout_s', 25.0),
            ('goal_xy_tolerance_m', 0.20),
            ('goal_yaw_tolerance_deg', 15.0),
            ('minimum_target_distance_m', 0.35),
            ('quiet_period_s', 1.0),
        ]:
            self.declare_parameter(name, default)

        self.state = State.IDLE
        self.reason = 'Awaiting target and explicit mission start'
        self.since = time.monotonic()
        self.target = ''
        self.mission_id = 0
        self.mission_started = 0.0
        self.last_detection = 0.0
        self.detected = False
        self.pose = None
        self.pose_at = 0.0
        self.pose_valid = False
        self.last_valid_pose = None
        self.last_valid_pose_at = 0.0
        self.preview = None
        self.preview_at = 0.0
        self.preview_valid = False
        self.target_snapshot = None
        self.snapshot_at = 0.0
        self.goal = None
        self.goal_pending = False
        self.goal_handle = None
        self.approach_at = 0.0
        self.cancel_goal_on_accept = False
        self.cancel_sent = False
        self.stop_requested = False
        self.nbv_request = None
        self.nbv_reset_pending = False
        self.nbv_reset_done = False
        self.nbv_idle_pending = False
        self.nbv_idle_verified = False
        self.nbv_idle_at = 0.0
        self.stop_ack = False
        self.nbv_busy = False
        self.nav_status_seen = False
        self.nav_active = False
        self.nav_quiet_since = 0.0
        self.final_state = State.CANCELLED
        self.final_reason = ''
        self.unsafe_stop = False

        latch = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.states = self.create_publisher(
            String, '/semantic/mission/state', latch)
        self.status = self.create_publisher(
            String, '/semantic/mission/status_json', latch)
        self.locked_pose_pub = self.create_publisher(
            PoseStamped, '/semantic/mission/locked_target_pose', latch)
        self.locked_valid_pub = self.create_publisher(
            Bool, '/semantic/mission/locked_target_valid', latch)
        self.create_subscription(String, '/semantic/search/target_class',
                                 self.on_target, latch)
        self.create_subscription(String, '/semantic/search/detections_json',
                                 self.on_detection, 10)
        self.create_subscription(String, '/semantic/nbv/execution_status',
                                 self.on_nbv, 10)
        self.create_subscription(PoseStamped, '/semantic/perception/target_pose',
                                 self.on_pose, 10)
        self.create_subscription(Bool, '/semantic/perception/target_valid',
                                 self.on_pose_valid, 10)
        self.create_subscription(PoseStamped, '/semantic/approach/locked_goal',
                                 self.on_preview, 10)
        self.create_subscription(Bool, '/semantic/approach/goal_valid',
                                 self.on_preview_valid, 10)
        self.create_subscription(GoalStatusArray,
                                 '/navigate_to_pose/_action/status',
                                 self.on_nav_status, latch)
        self.create_service(Trigger, '/semantic/mission/start', self.start)
        self.create_service(Trigger, '/semantic/mission/cancel', self.cancel)
        self.nbv = self.create_client(SetBool, '/semantic/nbv/auto_enable')
        self.nbv_idle = self.create_client(
            Trigger, '/semantic/nbv/idle_check')
        self.nbv_reset = self.create_client(
            Trigger, '/semantic/nbv/reset_search')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_timer(0.2, self.tick)
        self.locked_valid_pub.publish(Bool(data=False))
        self.publish_state()

    def param(self, name):
        return self.get_parameter(name).value

    def age_ok(self, timestamp, limit):
        return timestamp > 0 and time.monotonic() - timestamp <= self.param(limit)

    def publish_state(self):
        self.states.publish(String(data=self.state.value))
        self.status.publish(String(data=json.dumps({
            'mission_id': self.mission_id, 'state': self.state.value,
            'target_class': self.target, 'reason': self.reason,
            'stop_unconfirmed': self.unsafe_stop})))
        self.get_logger().info(f'{self.state.value}: {self.reason}')

    def set_state(self, state, reason):
        if self.state != state or self.reason != reason:
            self.state, self.reason = state, reason
            self.since = time.monotonic()
            if state in TERMINAL or state == State.CANCELLING:
                # Immediately invalidate the locked waypoint if mission ends.
                self.locked_valid_pub.publish(Bool(data=False))
            self.publish_state()

    def on_target(self, msg):
        target = msg.data.strip()
        if target == self.target:
            return
        if self.state not in TERMINAL:
            self.abort(State.FAILED, 'Target class changed during mission')
        self.target = target
        self.last_detection = 0.0
        self.detected = False
        self.pose = self.preview = self.last_valid_pose = None
        self.pose_valid = self.preview_valid = False
        self.last_valid_pose_at = 0.0
        self.locked_valid_pub.publish(Bool(data=False))
        self.publish_state()

    def on_detection(self, msg):
        try:
            record = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(record, dict) or record.get('target') != self.target:
            return
        self.last_detection = time.monotonic()
        if (record.get('confirmed') is True and
                self.state in (State.PREPARING, State.SEARCHING)):
            self.detected = True
            self.stop_search()

    def on_nbv(self, msg):
        line = msg.data
        if line.startswith(('REQUESTED ', 'NAV2_ACCEPTED:')):
            self.nbv_busy = True
        elif line.startswith(('NAV2_SUCCEEDED |', 'NAV2_CANCELED |',
                              'NAV2_ABORTED |', 'NAV2_STATUS_',
                              'NAV2_REJECTED:', 'NAV2_SEND_ERROR:',
                              'NAV2_RESULT_ERROR:')):
            self.nbv_busy = False
        if (self.state == State.SEARCHING and line.startswith(
                ('AUTO_PAUSED:', 'AUTO_BUDGET_REACHED:'))):
            self.abort(State.FAILED, 'NBV stopped before target localization: ' + line)

    def on_nav_status(self, msg):
        self.nav_status_seen = True
        active = any(s.status in NAV_ACTIVE for s in msg.status_list)
        if active:
            self.nav_quiet_since = 0.0
        elif self.nav_active or self.nav_quiet_since == 0.0:
            self.nav_quiet_since = time.monotonic()
        self.nav_active = active

    def on_pose(self, msg):
        self.pose, self.pose_at = msg, time.monotonic()
        self.cache_valid_localization()

    def on_pose_valid(self, msg):
        self.pose_valid = bool(msg.data)
        if self.pose_valid:
            self.cache_valid_localization()
        else:
            # The live detector can flicker or lose the object while NBV
            # cancels. Preserve the last VALID pose for a bounded interval.
            self.pose = None

    def cache_valid_localization(self):
        if (self.state in (State.PREPARING, State.SEARCHING,
                           State.STOPPING_SEARCH, State.LOCALIZING) and
                self.pose_valid and valid_pose(self.pose) and
                self.pose_at >= self.mission_started and
                self.mission_started > 0):
            self.last_valid_pose = self.pose
            self.last_valid_pose_at = self.pose_at

    def on_preview(self, msg):
        self.preview, self.preview_at = msg, time.monotonic()

    def on_preview_valid(self, msg):
        self.preview_valid = bool(msg.data)
        if not msg.data:
            self.preview = None

    def start(self, request, response):
        if (self.state not in TERMINAL or self.nbv_request is not None or
                self.nbv_idle_pending or self.nbv_reset_pending or
                self.goal_pending or self.goal_handle is not None):
            response.success = False
            response.message = 'Previous mission/goal/cancellation is still active'
            return response
        if self.unsafe_stop:
            response.success = False
            response.message = ('Previous stop unconfirmed: manually inspect '
                                'Nav2 and restart supervisor when safe')
            return response
        if not self.target:
            response.success = False
            response.message = 'Select /semantic/search/target_class first'
            return response
        # Nav2's action status may be silent until the first goal exists.
        # Do not mistake 'no status received yet' for 'an active goal'.
        # An actually observed active goal still blocks mission start.
        # NBV reset/idle and TF/Nav2 checks guard the subsequent motion.
        if self.nav_active:
            response.success = False
            response.message = 'Nav2 reports an active navigation goal'
            return response
        self.mission_id += 1
        self.mission_started = time.monotonic()
        self.last_detection = 0.0
        self.detected = False
        self.pose = self.preview = self.goal = self.target_snapshot = None
        self.last_valid_pose = None
        self.last_valid_pose_at = 0.0
        self.pose_at = self.preview_at = self.snapshot_at = 0.0
        self.pose_valid = self.preview_valid = False
        self.nbv_reset_done = False
        self.nbv_idle_verified = False
        self.nbv_busy = False
        self.stop_ack = self.stop_requested = False
        self.cancel_goal_on_accept = self.cancel_sent = False
        self.locked_valid_pub.publish(Bool(data=False))
        self.set_state(State.PREPARING, 'Waiting for live YOLO, TF and Nav2')
        response.success = True
        response.message = 'Mission accepted for preparation (not completed)'
        return response

    def cancel(self, request, response):
        if self.state in TERMINAL:
            response.success = False
            response.message = 'No active mission'
        else:
            self.abort(State.CANCELLED, 'Operator requested cancellation')
            response.success = True
            response.message = 'Cancellation requested; await terminal state'
        return response

    def ready(self):
        if not self.age_ok(self.last_detection, 'detector_stale_s'):
            return False
        if not (self.nbv.service_is_ready() and
                self.nbv_reset.service_is_ready() and
                self.nbv_idle.service_is_ready() and
                self.nav.server_is_ready()):
            return False
        try:
            self.tf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return False
        return True

    def reset_nbv(self):
        if self.nbv_reset_pending or self.nbv_request is not None:
            return
        self.nbv_reset_pending = True
        token = self.mission_id
        try:
            future = self.nbv_reset.call_async(Trigger.Request())
            future.add_done_callback(lambda f: self.on_reset_done(f, token))
        except Exception as exc:
            self.nbv_reset_pending = False
            self.abort(State.FAILED, f'NBV reset request failed: {exc}')

    def on_reset_done(self, future, token):
        if token != self.mission_id:
            return
        self.nbv_reset_pending = False
        try:
            result = future.result()
            ok, message = result.success, result.message
        except Exception as exc:
            ok, message = False, str(exc)
        if self.state != State.PREPARING:
            return
        if ok:
            self.nbv_reset_done = True
        else:
            self.abort(State.FAILED, 'NBV reset failed: ' + message)

    def nbv_call(self, enable):
        if self.nbv_request is not None or not self.nbv.service_is_ready():
            return False
        token = self.mission_id
        operation = 'arm' if enable else 'stop'
        self.nbv_request = operation
        try:
            future = self.nbv.call_async(SetBool.Request(data=enable))
            future.add_done_callback(
                lambda f: self.on_nbv_response(f, token, operation))
        except Exception as exc:
            self.nbv_request = None
            self.get_logger().error(f'NBV {operation} call failed: {exc}')
            return False
        return True

    def on_nbv_response(self, future, token, operation):
        if token != self.mission_id:
            return
        self.nbv_request = None
        try:
            reply = future.result()
            ok, message = reply.success, reply.message
        except Exception as exc:
            ok, message = False, str(exc)
        if operation == 'arm':
            if self.state in (State.STOPPING_SEARCH, State.CANCELLING):
                self.nbv_call(False)
            elif not ok:
                self.abort(State.FAILED, 'NBV refused to arm: ' + message)
            elif self.state == State.PREPARING:
                self.set_state(State.SEARCHING, 'NBV searching for ' + self.target)
        else:
            self.stop_ack = bool(ok)
            if not ok:
                # Do not claim cancellation completed or allow another mission.
                self.unsafe_stop = True
                self.set_state(State.FAILED,
                               'NBV disarm unconfirmed: ' + message)

    def stop_search(self):
        self.stop_ack = False
        self.stop_requested = True
        self.nbv_idle_verified = False
        self.set_state(State.STOPPING_SEARCH,
                       'YOLO confirmed; disarm NBV and await settled Nav2 result')
        if self.nbv_request is None:
            self.nbv_call(False)

    def check_nbv_idle(self):
        if (self.nbv_idle_pending or not self.stop_ack or
                not self.nbv_idle.service_is_ready()):
            return
        token = self.mission_id
        self.nbv_idle_pending = True
        try:
            future = self.nbv_idle.call_async(Trigger.Request())
            future.add_done_callback(lambda f: self.on_idle_done(f, token))
        except Exception as exc:
            self.nbv_idle_pending = False
            self.get_logger().error(f'NBV idle check failed: {exc}')

    def on_idle_done(self, future, token):
        if token != self.mission_id:
            return
        self.nbv_idle_pending = False
        try:
            reply = future.result()
            self.nbv_idle_verified = bool(reply.success)
        except Exception:
            self.nbv_idle_verified = False
        if self.nbv_idle_verified:
            self.nbv_idle_at = time.monotonic()

    def nav_quiet(self):
        # The Nav2 status topic is often silent until the first action.
        # If status has been received, require an explicit inactive status.
        # Otherwise rely on the NBV action-owner's affirmative idle service
        # and a full quiet period following that response. External clients
        # must NOT submit goals during an integrated semantic mission.
        if self.nav_status_seen:
            status_clear = not self.nav_active and self.nav_quiet_since > 0
            quiet_start = max(self.nav_quiet_since, self.nbv_idle_at)
        else:
            status_clear = not self.nav_active and self.nbv_idle_at > 0
            quiet_start = self.nbv_idle_at
        return (status_clear and self.nbv_idle_verified and
                not self.nbv_busy and self.stop_ack and
                time.monotonic() - quiet_start >=
                self.param('quiet_period_s'))

    def abort(self, terminal_state, reason):
        if self.state in TERMINAL:
            return
        previous = self.state
        self.final_state, self.final_reason = terminal_state, reason
        self.set_state(State.CANCELLING, reason)
        if previous == State.APPROACHING or self.goal_pending or self.goal_handle:
            self.cancel_goal_on_accept = True
            self.request_approach_cancel()
        elif previous == State.VERIFYING:
            self.set_state(terminal_state, reason)
        else:
            self.stop_ack = False
            self.nbv_idle_verified = False
            self.stop_requested = True
            if self.nbv_request is None:
                self.nbv_call(False)

    def request_approach_cancel(self):
        if self.goal_handle is None or self.cancel_sent:
            return
        self.cancel_sent = True
        try:
            f = self.goal_handle.cancel_goal_async()
            f.add_done_callback(self.on_cancel_ack)
        except Exception as exc:
            self.cancel_sent = False
            self.get_logger().error(f'Nav2 cancel submission failed: {exc}')

    def on_cancel_ack(self, future):
        try:
            acknowledgement = future.result()
            if not acknowledgement.goals_canceling:
                self.get_logger().warning(
                    'Nav2 cancellation not acknowledged; awaiting action result')
        except Exception as exc:
            self.get_logger().error(f'Nav2 cancellation response error: {exc}')

    def localized_target_snapshot(self):
        # A recently confirmed RGB-D localization is allowed to survive
        # temporary loss of detection during the Nav2 stop handshake. The
        # sample must belong to THIS mission, have a map frame, and be recent.
        if (self.pose_valid and valid_pose(self.pose) and
                self.pose_at >= self.mission_started and
                self.age_ok(self.pose_at, 'target_pose_stale_s')):
            return self.pose
        if (self.last_valid_pose is not None and
                valid_pose(self.last_valid_pose) and
                self.last_valid_pose_at >= self.mission_started and
                self.age_ok(self.last_valid_pose_at,
                            'target_snapshot_max_age_s')):
            return self.last_valid_pose
        return None

    def lock_target(self, target_pose):
        locked = PoseStamped()
        locked.header = target_pose.header
        locked.pose = target_pose.pose
        locked.header.frame_id = 'map'
        locked.header.stamp = self.get_clock().now().to_msg()
        self.target_snapshot = locked
        # Use lock-publication time so approach preview MUST be recomputed.
        self.snapshot_at = time.monotonic()
        self.locked_pose_pub.publish(locked)
        self.locked_valid_pub.publish(Bool(data=True))

    def has_approach(self):
        if (self.target_snapshot is None or not self.preview_valid or
                not valid_pose(self.preview) or
                self.preview_at < self.snapshot_at or
                not self.age_ok(self.preview_at, 'approach_goal_stale_s')):
            return False
        p, t = self.preview.pose.position, self.target_snapshot.pose.position
        return math.hypot(p.x - t.x, p.y - t.y) >= self.param(
            'minimum_target_distance_m')

    def send_approach(self):
        if not self.nav_quiet():
            return
        self.goal = PoseStamped()
        self.goal.header = self.preview.header
        self.goal.pose = self.preview.pose
        self.goal.header.stamp = self.get_clock().now().to_msg()
        request = NavigateToPose.Goal()
        request.pose = self.goal
        self.goal_pending = True
        self.approach_at = time.monotonic()
        self.set_state(State.APPROACHING, 'Submitting map-frame approach goal')
        try:
            future = self.nav.send_goal_async(request)
            future.add_done_callback(self.on_approach_accepted)
        except Exception as exc:
            self.goal_pending = False
            self.abort(State.FAILED, f'Approach submission failed: {exc}')

    def on_approach_accepted(self, future):
        self.goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            if self.state == State.CANCELLING:
                self.set_state(self.final_state,
                               self.final_reason + '; no goal accepted')
            else:
                self.abort(State.FAILED, f'Nav2 response failed: {exc}')
            return
        if not handle.accepted:
            if self.state == State.CANCELLING:
                self.set_state(self.final_state, self.final_reason)
            else:
                self.set_state(State.FAILED, 'Approach rejected by Nav2')
            return
        self.goal_handle = handle
        try:
            handle.get_result_async().add_done_callback(self.on_approach_result)
        except Exception as exc:
            self.abort(State.FAILED, f'Unable to track approach result: {exc}')
            return
        if self.state == State.CANCELLING or self.cancel_goal_on_accept:
            self.request_approach_cancel()
        else:
            self.set_state(State.APPROACHING, 'Nav2 accepted approach goal')

    def on_approach_result(self, future):
        self.goal_handle = None
        self.cancel_sent = False
        try:
            status = future.result().status
        except Exception as exc:
            self.unsafe_stop = True
            self.set_state(State.FAILED,
                           f'Nav2 result unavailable: {exc}; inspect robot')
            return
        if self.state == State.CANCELLING:
            self.set_state(self.final_state,
                           self.final_reason + f'; Nav2 terminal status={status}')
        elif self.state in TERMINAL:
            return
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self.set_state(State.VERIFYING,
                           'Nav2 succeeded; verifying final map-frame pose')
        else:
            self.set_state(State.FAILED, f'Approach ended with Nav2 status={status}')

    def verify(self):
        if self.goal is None or self.target_snapshot is None:
            return False
        try:
            tf = self.tf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return False
        robot = tf.transform.translation
        goal = self.goal.pose
        distance = math.hypot(robot.x - goal.position.x,
                              robot.y - goal.position.y)
        heading = math.degrees(abs(math.atan2(
            math.sin(yaw(tf.transform.rotation) - yaw(goal.orientation)),
            math.cos(yaw(tf.transform.rotation) - yaw(goal.orientation)))))
        target = self.target_snapshot.pose.position
        stand_off = math.hypot(robot.x - target.x, robot.y - target.y)
        if (distance <= self.param('goal_xy_tolerance_m') and
                heading <= self.param('goal_yaw_tolerance_deg') and
                stand_off >= self.param('minimum_target_distance_m')):
            self.set_state(
                State.SUCCEEDED,
                f'Approach verified: {distance:.3f} m, {heading:.1f} deg, '
                f'object distance={stand_off:.3f} m')
            return True
        return False

    def tick(self):
        elapsed = time.monotonic() - self.since
        if self.state == State.PREPARING:
            if self.detected:
                self.stop_search()
            elif self.ready():
                if not self.nbv_reset_done and not self.nbv_reset_pending:
                    self.reset_nbv()
                elif self.nbv_reset_done and self.nbv_request is None:
                    self.nbv_call(True)
            if (self.state == State.PREPARING and
                    elapsed > self.param('prepare_timeout_s')):
                self.abort(State.FAILED, 'Preparation timed out')

        elif self.state == State.SEARCHING:
            if not self.age_ok(self.last_detection, 'detector_stale_s'):
                self.abort(State.FAILED, 'YOLO detector stream became stale')

        elif self.state == State.STOPPING_SEARCH:
            if self.nbv_request is None and not self.stop_ack:
                self.nbv_call(False)
            elif self.stop_ack and not self.nbv_idle_pending and not self.nbv_idle_verified:
                self.check_nbv_idle()
            if self.nav_quiet():
                self.set_state(State.LOCALIZING,
                               'NBV/Nav2 stopped; awaiting stable 3D target')
            elif elapsed > self.param('stop_timeout_s'):
                self.unsafe_stop = True
                self.set_state(State.FAILED,
                               'Search stop unconfirmed; do not send another goal')

        elif self.state == State.LOCALIZING:
            target = self.localized_target_snapshot()
            if target is not None:
                self.lock_target(target)
                self.set_state(State.WAITING_APPROACH,
                               'RGB-D target locked; requesting safe approach preview')
            elif elapsed > self.param('localization_timeout_s'):
                self.set_state(State.FAILED,
                               'No recent confirmed RGB-D target after search stopped')

        elif self.state == State.WAITING_APPROACH:
            if self.has_approach() and self.nav.server_is_ready() and self.nav_quiet():
                self.send_approach()
            elif elapsed > self.param('approach_preview_timeout_s'):
                self.set_state(State.FAILED, 'No fresh valid approach preview')

        elif self.state == State.APPROACHING:
            if time.monotonic() - self.approach_at > self.param('approach_timeout_s'):
                self.abort(State.FAILED, 'Approach action timed out')

        elif self.state == State.VERIFYING:
            if not self.verify() and elapsed > self.param('verify_timeout_s'):
                self.set_state(State.FAILED, 'Final map-frame pose check failed')

        elif self.state == State.CANCELLING:
            if self.goal_pending:
                self.cancel_goal_on_accept = True
            elif self.goal_handle is not None:
                self.request_approach_cancel()
            else:
                if self.nbv_request is None and self.stop_requested and not self.stop_ack:
                    self.nbv_call(False)
                if (self.stop_requested and self.stop_ack and
                        not self.nbv_idle_pending and not self.nbv_idle_verified):
                    self.check_nbv_idle()
                if not self.stop_requested or self.nav_quiet():
                    self.set_state(self.final_state, self.final_reason)
            if (self.state == State.CANCELLING and
                    elapsed > self.param('cancel_timeout_s')):
                self.unsafe_stop = True
                self.set_state(State.FAILED,
                               'Cancellation unconfirmed; inspect Nav2/controller')


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning(
            'Process exit is not a hardware safety stop; use controller watchdog')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
