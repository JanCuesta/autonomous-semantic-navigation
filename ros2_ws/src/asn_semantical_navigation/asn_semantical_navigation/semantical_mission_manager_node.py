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
    SLOWING_FOR_INSPECTION = 'SLOWING_FOR_INSPECTION'
    STOPPING_SEARCH = 'STOPPING_SEARCH'
    LOCALIZING = 'LOCALIZING'
    ADVANCING_FOR_INSPECTION = 'ADVANCING_FOR_INSPECTION'
    CHECKING_TARGET = 'CHECKING_TARGET'
    SCANNING_FOR_TARGET = 'SCANNING_FOR_TARGET'
    RESUMING_SEARCH = 'RESUMING_SEARCH'
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
            ('localization_timeout_s', 20.0),
            # Inspection timings use steady-clock (wall) seconds.
            ('inspection_slowdown_s', 1.5),
            ('inspection_cooldown_s', 12.0),
            # Recovery uses Nav2 NavigateToPose, not direct wheel commands.
            ('inspection_forward_m', 0.60),
            ('inspection_scan_deg', 45.0),
            ('inspection_observe_s', 6.0),
            ('inspection_goal_timeout_s', 120.0),
            ('inspection_max_cycles', 3),
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
        self.inspection_cooldown_until = 0.0
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
        # Recovery Nav2 action is separate from the final approach action.
        self.recovery_pending = False
        self.recovery_handle = None
        self.recovery_kind = None           # 'forward' | 'scan'
        self.recovery_started_at = 0.0
        self.recovery_motion_id = 0
        self.recovery_cancel_requested = False
        self.recovery_cancel_sent = False
        self.recovery_cancel_at = 0.0
        self.recovery_forward_sent = False
        self.recovery_scan_offsets = (1.0, -1.0, 0.0)
        self.recovery_scan_index = 0
        self.recovery_scan_origin_yaw = None
        self.recovery_view_since = 0.0
        self.recovery_observe_since = 0.0
        self.recovery_scan_detection = False
        self.inspection_cycles = 0
        self.last_yolo_confirmed = False
        self.last_yolo_confirmed_at = 0.0
        # True only when LOCALIZING was entered from a stationary recovery check.
        # A timeout then advances the scan instead of repeating the forward move.
        self.recovery_localizing = False
        self.recovery_check_detection_seen = False
        latch = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.states = self.create_publisher(
            String, '/semantic/mission/state', latch)
        self.status = self.create_publisher(
            String, '/semantic/mission/status_json', latch)
        # Mission-manager DECISION signal, not a velocity command.
        # NBV/Nav2 consumes this to enforce the inspection speed limit.
        self.inspection_slowdown_pub = self.create_publisher(
            Bool, '/semantic/mission/inspection_slowdown', latch)
        self.inspection_slowdown_pub.publish(Bool(data=False))
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
            # Maintain slowdown through braking; clear once idle or resumed.
            self.inspection_slowdown_pub.publish(Bool(
                data=(state in (State.SLOWING_FOR_INSPECTION,
                                State.STOPPING_SEARCH))))
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
            # A validated 3D pose takes priority over inspecting again.
            self.stop_if_localized()
            if (self.state in (State.PREPARING, State.SEARCHING) and
                    time.monotonic() >= self.inspection_cooldown_until and
                    self.inspection_cycles < int(self.param('inspection_max_cycles'))):
                self.inspection_cycles += 1
                self.set_state(State.SLOWING_FOR_INSPECTION,
                               'YOLO confirmed; slowing before stationary inspection')
        elif (record.get('confirmed') is True and
              self.state == State.SCANNING_FOR_TARGET and
              not self.last_yolo_confirmed):
            # Interrupt only on a NEW detection: persistent YOLO boxes must
            # not cancel every scan before the robot changes viewpoint.
            self.recovery_scan_detection = True
            self.request_recovery_cancel('New YOLO confirmation during scan')
        self.last_yolo_confirmed = record.get('confirmed') is True
        if self.last_yolo_confirmed:
            self.last_yolo_confirmed_at = time.monotonic()

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
        self.stop_if_localized()

    def on_pose_valid(self, msg):
        self.pose_valid = bool(msg.data)
        if self.pose_valid:
            self.cache_valid_localization()
            self.stop_if_localized()
            if self.state == State.SCANNING_FOR_TARGET:
                self.recovery_scan_detection = True
                self.request_recovery_cancel('RGB-D localized during inspection scan')
        else:
            # Preserve a previously validated pose for a bounded interval.
            self.pose = None

    def cache_valid_localization(self):
        if (self.state in (State.PREPARING, State.SEARCHING,
                           State.SLOWING_FOR_INSPECTION,
                          State.STOPPING_SEARCH, State.LOCALIZING,
                          State.RESUMING_SEARCH, State.CHECKING_TARGET,
                          State.SCANNING_FOR_TARGET) and
                self.pose_valid and valid_pose(self.pose) and
                self.pose_at >= self.mission_started and
                self.mission_started > 0):
            self.last_valid_pose = self.pose
            self.last_valid_pose_at = self.pose_at

    def stop_if_localized(self):
        """A valid RGB-D map pose supersedes the observation pause."""
        if (self.state in (State.PREPARING, State.SEARCHING,
                           State.SLOWING_FOR_INSPECTION,
                           State.RESUMING_SEARCH) and
                self.localized_target_snapshot() is not None):
            self.stop_search()

    def on_preview(self, msg):
        self.preview, self.preview_at = msg, time.monotonic()

    def on_preview_valid(self, msg):
        self.preview_valid = bool(msg.data)
        if not msg.data:
            self.preview = None

    def start(self, request, response):
        if (self.state not in TERMINAL or self.nbv_request is not None or
                self.nbv_idle_pending or self.nbv_reset_pending or
                self.goal_pending or self.goal_handle is not None or
                self.recovery_pending or self.recovery_handle is not None):
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
        self.inspection_cooldown_until = 0.0
        self.inspection_cycles = 0
        self.recovery_forward_sent = False
        self.recovery_scan_index = 0
        self.recovery_scan_origin_yaw = None
        self.recovery_view_since = 0.0
        self.recovery_observe_since = 0.0
        self.recovery_scan_detection = False
        self.last_yolo_confirmed = False
        self.last_yolo_confirmed_at = 0.0
        self.recovery_localizing = False
        self.recovery_check_detection_seen = False
        self.recovery_cancel_requested = False
        self.recovery_cancel_sent = False
        self.recovery_cancel_at = 0.0
        self.recovery_kind = None
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
        if self.state not in (State.PREPARING, State.RESUMING_SEARCH):
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
            elif self.state in (State.PREPARING, State.RESUMING_SEARCH):
                self.stop_requested = False
                self.stop_ack = False
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
                       'Pause requested; disarm NBV and await settled Nav2 result')
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
                not self.recovery_pending and self.recovery_handle is None and
                time.monotonic() - quiet_start >=
                self.param('quiet_period_s'))

    def abort(self, terminal_state, reason):
        if self.state in TERMINAL:
            return
        previous = self.state
        self.final_state, self.final_reason = terminal_state, reason
        self.set_state(State.CANCELLING, reason)
        if self.recovery_pending or self.recovery_handle is not None:
            self.request_recovery_cancel('Mission aborted during recovery')
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
    # ------------------ bounded inspection recovery ------------------

    def robot_map_pose(self):
        tf = self.tf.lookup_transform('map', 'base_link', Time())
        p = tf.transform.translation
        return p.x, p.y, yaw(tf.transform.rotation)


    @staticmethod

    def make_map_goal(x, y, heading, stamp):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = stamp
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.orientation.w = math.cos(heading / 2.0)
        return goal


    def recovery_active(self):
        return self.recovery_pending or self.recovery_handle is not None


    def send_recovery_goal(self, kind, x, y, heading):
        # The supervisor owns recovery ONLY after NBV and Nav2 are idle.
        if self.recovery_active() or not self.nav_quiet():
            return False
        if not self.nav.server_is_ready():
            return False
        request = NavigateToPose.Goal()
        request.pose = self.make_map_goal(
            x, y, heading, self.get_clock().now().to_msg())
        self.recovery_motion_id += 1
        motion_id = self.recovery_motion_id
        mission_id = self.mission_id
        self.recovery_pending = True
        self.recovery_kind = kind
        self.recovery_started_at = time.monotonic()
        self.recovery_cancel_requested = False
        self.recovery_cancel_sent = False
        self.recovery_cancel_at = 0.0
        try:
            future = self.nav.send_goal_async(request)
            future.add_done_callback(
                lambda f: self.on_recovery_accept(f, mission_id, motion_id))
        except Exception as exc:
            self.recovery_pending = False
            self.recovery_kind = None
            self.get_logger().error(f'Inspection goal submission failed: {exc}')
            return False
        self.get_logger().info(
            f'INSPECTION_GOAL {kind}: map=({x:.2f},{y:.2f},'
            f'{math.degrees(heading):.1f}deg)')
        return True


    def on_recovery_accept(self, future, mission_id, motion_id):
        try:
            handle = future.result()
        except Exception as exc:
            if mission_id == self.mission_id and motion_id == self.recovery_motion_id:
                self.recovery_pending = False
                self.recovery_kind = None
                if self.state == State.CANCELLING:
                    return
                self.abort(State.FAILED,
                           f'Inspection Nav2 goal acceptance unavailable: {exc}')
            return
        if mission_id != self.mission_id or motion_id != self.recovery_motion_id:
            # Late acceptance must never start a goal owned by an old mission.
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.recovery_pending = False
        if not handle.accepted:
            self.on_recovery_terminal(GoalStatus.STATUS_ABORTED,
                                      'goal rejected by Nav2')
            return
        self.recovery_handle = handle
        try:
            future = handle.get_result_async()
            future.add_done_callback(
                lambda f: self.on_recovery_result(f, mission_id, motion_id))
        except Exception as exc:
            self.get_logger().error(f'Inspection Nav2 result tracking failed: {exc}')
            self.unsafe_stop = True
            self.request_recovery_cancel('Inspection result tracking failed')
            self.abort(State.FAILED, 'Inspection result tracking unavailable')
            return
        if self.recovery_cancel_requested or self.state == State.CANCELLING:
            self.request_recovery_cancel('Late inspection acceptance: cancel')


    def request_recovery_cancel(self, reason):
        if not self.recovery_active() or self.recovery_cancel_sent:
            return
        if not self.recovery_cancel_requested:
            self.recovery_cancel_requested = True
            self.recovery_cancel_at = time.monotonic()
            self.get_logger().warning(reason)
        if self.recovery_handle is None:
            return  # Cancel immediately upon late acceptance.
        try:
            future = self.recovery_handle.cancel_goal_async()
            future.add_done_callback(self.on_recovery_cancel_ack)
            self.recovery_cancel_sent = True
        except Exception as exc:
            self.get_logger().error(f'Inspection cancellation request failed: {exc}')


    def on_recovery_cancel_ack(self, future):
        try:
            if not future.result().goals_canceling:
                self.get_logger().warning(
                    'Inspection cancellation not acknowledged; awaiting terminal result')
        except Exception as exc:
            self.get_logger().error(f'Inspection cancellation acknowledgment error: {exc}')


    def on_recovery_result(self, future, mission_id, motion_id):
        if mission_id != self.mission_id or motion_id != self.recovery_motion_id:
            return
        try:
            status = future.result().status
        except Exception as exc:
            self.unsafe_stop = True
            self.abort(State.FAILED, f'Inspection goal result unavailable: {exc}')
            return
        self.recovery_handle = None
        self.recovery_pending = False
        self.on_recovery_terminal(status, f'Nav2 status={status}')


    def on_recovery_terminal(self, status, detail):
        kind = self.recovery_kind
        self.recovery_kind = None
        self.recovery_cancel_requested = False
        self.recovery_cancel_sent = False
        self.recovery_cancel_at = 0.0
        if self.state == State.CANCELLING or self.state in TERMINAL:
            return
        if kind == 'forward' and self.state == State.ADVANCING_FOR_INSPECTION:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.begin_inspection_check('Forward inspection move completed')
            else:
                self.set_state(
                    State.SCANNING_FOR_TARGET,
                    f'Forward move could not finish ({detail}); scanning in place')
        elif kind == 'scan' and self.state == State.SCANNING_FOR_TARGET:
            self.begin_inspection_check(f'Inspection scan turn finished ({detail})')


    def recovery_watchdog(self):
        if not self.recovery_active():
            return
        now = time.monotonic()
        if (not self.recovery_cancel_requested and
                now - self.recovery_started_at >
                self.param('inspection_goal_timeout_s')):
            self.request_recovery_cancel('Inspection Nav2 goal timed out')
        if (self.recovery_cancel_requested and self.recovery_cancel_at and
                now - self.recovery_cancel_at > self.param('cancel_timeout_s')):
            # Never hand motion back to NBV without a terminal action result.
            self.unsafe_stop = True
            self.set_state(State.FAILED,
                           'Inspection cancellation unconfirmed: inspect Nav2/controller')


    def begin_inspection_check(self, reason):
        # Preserve a detection that stopped a turning maneuver. Start the
        # observation clock only once the preceding Nav2 action is fully idle.
        self.recovery_check_detection_seen = self.recovery_scan_detection
        self.recovery_observe_since = 0.0
        self.recovery_view_since = float('inf')
        self.recovery_scan_detection = False
        self.recovery_localizing = False
        self.set_state(State.CHECKING_TARGET, reason)


    def stationary_recovery_target(self):
        # Do not use a localization captured during the forward translation.
        target = self.localized_target_snapshot()
        if target is None:
            return None
        observed_at = max(
            self.pose_at if self.pose_valid and self.pose is not None else 0.0,
            self.last_valid_pose_at)
        if observed_at < self.recovery_view_since:
            return None
        return target


    def finish_inspection_recovery(self):
        # Use an observed goal result AND Nav2 idle before returning control.
        self.detected = False
        self.nbv_reset_done = False
        self.last_valid_pose = None
        self.last_valid_pose_at = 0.0
        self.pose = None
        self.pose_valid = False
        self.recovery_localizing = False
        self.recovery_check_detection_seen = False
        self.inspection_cooldown_until = (
            time.monotonic() + self.param('inspection_cooldown_s'))
        self.set_state(State.RESUMING_SEARCH,
                       'Inspection viewpoints exhausted; resuming NBV exploration')


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
            if self.localized_target_snapshot() is not None:
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
            if self.localized_target_snapshot() is not None:
                self.stop_search()
            elif not self.age_ok(self.last_detection, 'detector_stale_s'):
                self.abort(State.FAILED, 'YOLO detector stream became stale')
        elif self.state == State.SLOWING_FOR_INSPECTION:
        # This node requests a speed limit; actual limiting is owned by
            # the NBV/Nav2 integration, which we will edit next.
            if (self.localized_target_snapshot() is not None or
                    elapsed >= self.param('inspection_slowdown_s')):
                self.stop_search()
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
            # The first localization follows the NBV stop. Later attempts come
            # from CHECKING_TARGET, where a fresh confirmed YOLO frame was seen
            # at a stationary recovery viewpoint.
            target = (self.stationary_recovery_target()
                      if self.recovery_localizing
                      else self.localized_target_snapshot())
            if target is not None:
                self.recovery_localizing = False
                self.lock_target(target)
                self.set_state(State.WAITING_APPROACH,
                               'RGB-D target locked; requesting safe approach preview')
            elif elapsed >= self.param('localization_timeout_s'):
                if self.recovery_localizing:
                    # This viewpoint has already been tried. Move to the NEXT
                    # scan angle; never restart the forward move here.
                    self.recovery_localizing = False
                    if self.recovery_scan_index < len(self.recovery_scan_offsets):
                        self.set_state(State.SCANNING_FOR_TARGET,
                                       'RGB-D not validated; trying next scan viewpoint')
                    else:
                        self.finish_inspection_recovery()
                else:
                    # Initial stationary inspection failed; try one forward
                    # recovery move before the bounded angular scan.
                    self.recovery_forward_sent = False
                    self.recovery_scan_index = 0
                    self.recovery_scan_origin_yaw = None
                    self.recovery_scan_detection = False
                    self.recovery_view_since = 0.0
                    self.last_valid_pose = None
                    self.last_valid_pose_at = 0.0
                    self.set_state(State.ADVANCING_FOR_INSPECTION,
                                   'RGB-D inconclusive; move 0.6m ahead to improve view')
        elif self.state == State.ADVANCING_FOR_INSPECTION:
            if self.recovery_active():
                self.recovery_watchdog()
            elif not self.recovery_forward_sent and self.nav_quiet():
                try:
                    x, y, heading = self.robot_map_pose()
                except TransformException as exc:
                    if elapsed > self.param('stop_timeout_s'):
                        self.abort(State.FAILED, f'Cannot obtain map TF for recovery: {exc}')
                else:
                    distance = float(self.param('inspection_forward_m'))
                    gx = x + distance * math.cos(heading)
                    gy = y + distance * math.sin(heading)
                    if self.send_recovery_goal('forward', gx, gy, heading):
                        self.recovery_forward_sent = True
                        # Fresh localization must be acquired after the move.
                        self.recovery_view_since = float('inf')
            elif not self.recovery_forward_sent and elapsed > self.param('stop_timeout_s'):
                self.abort(State.FAILED, 'No idle Nav2 available for inspection move')
        elif self.state == State.CHECKING_TARGET:
            if self.recovery_active():
                self.recovery_watchdog()
            elif self.nav_quiet():
                if self.recovery_observe_since == 0.0:
                    self.recovery_observe_since = time.monotonic()
                    # Ignore poses captured while the robot was translating or
                    # rotating. Only use localizations at this stopped viewpoint.
                    self.recovery_view_since = self.recovery_observe_since
                confirmed_at_view = (
                    self.recovery_check_detection_seen or
                    (self.last_yolo_confirmed_at >= self.recovery_observe_since and
                     self.age_ok(self.last_yolo_confirmed_at, 'detector_stale_s')))
                if confirmed_at_view or self.stationary_recovery_target() is not None:
                    # YOLO confirmation means "attempt RGB-D localization", NOT
                    # "target 3D pose ready". The LOCALIZING branch enforces it.
                    self.recovery_check_detection_seen = False
                    self.recovery_localizing = True
                    self.set_state(State.LOCALIZING,
                                   'Chair detected at inspection viewpoint; validating RGB-D')
                elif (time.monotonic() - self.recovery_observe_since >=
                      self.param('inspection_observe_s')):
                    if self.recovery_scan_index < len(self.recovery_scan_offsets):
                        self.set_state(State.SCANNING_FOR_TARGET,
                                       'No confirmed detection; trying next ±45 degree view')
                    else:
                        self.finish_inspection_recovery()
            elif elapsed > self.param('stop_timeout_s'):
                self.unsafe_stop = True
                self.abort(State.FAILED,
                           'Inspection motion terminal but Nav2 is not quiet')
        elif self.state == State.SCANNING_FOR_TARGET:
            if self.recovery_active():
                if self.recovery_scan_detection:
                    self.request_recovery_cancel('Target detected during scan')
                self.recovery_watchdog()
            elif self.nav_quiet():
                if self.recovery_scan_detection:
                    self.begin_inspection_check(
                        'Detected during scan; stationary RGB-D verification')
                elif self.recovery_scan_index >= len(self.recovery_scan_offsets):
                    self.finish_inspection_recovery()
                else:
                    try:
                        x, y, heading = self.robot_map_pose()
                    except TransformException as exc:
                        if elapsed > self.param('stop_timeout_s'):
                            self.abort(State.FAILED, f'Cannot obtain scan TF: {exc}')
                    else:
                        if self.recovery_scan_origin_yaw is None:
                            self.recovery_scan_origin_yaw = heading
                        index = self.recovery_scan_index
                        offset_deg = (self.recovery_scan_offsets[index] *
                                      self.param('inspection_scan_deg'))
                        requested_yaw = (self.recovery_scan_origin_yaw +
                                         math.radians(offset_deg))
                        if self.send_recovery_goal('scan', x, y, requested_yaw):
                            self.recovery_scan_index += 1
                            self.recovery_scan_detection = False
            elif elapsed > self.param('stop_timeout_s'):
                self.unsafe_stop = True
                self.abort(State.FAILED, 'Nav2 not idle before inspection scan')
        elif self.state == State.RESUMING_SEARCH:
            # Entered only after NBV/Nav2 is confirmed idle from inspection.
            if self.localized_target_snapshot() is not None:
                self.stop_search()
            elif self.nav_quiet():
                if not self.nbv_reset_done and not self.nbv_reset_pending:
                    self.reset_nbv()
                elif self.nbv_reset_done and self.nbv_request is None:
                    self.nbv_call(True)
            if (self.state == State.RESUMING_SEARCH and
                    elapsed > self.param('prepare_timeout_s')):
                self.abort(State.FAILED, 'Unable to resume NBV after inspection')
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
            if self.recovery_active():
                self.request_recovery_cancel('Cancel active inspection recovery')
                self.recovery_watchdog()
            elif self.goal_pending:
                self.cancel_goal_on_accept = True
            elif self.goal_handle is not None:
                self.request_approach_cancel()
            else:
                if self.nbv_request is None and self.stop_requested and not self.stop_ack:
                    self.nbv_call(False)
                if (self.stop_requested and self.stop_ack and
                        not self.nbv_idle_pending and not self.nbv_idle_verified):
                    self.check_nbv_idle()
                if not self.recovery_active() and (
                        not self.stop_requested or self.nav_quiet()):
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
