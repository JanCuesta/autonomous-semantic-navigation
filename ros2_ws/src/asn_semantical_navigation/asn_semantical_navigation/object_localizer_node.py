"""RGB-D localization of a YOLO-selected object in the ROS map frame.
Input: detections_json, depth image, and registered depth camera CameraInfo.
Output: validated target pose, validity flag, status, and RViz marker.
This node NEVER commands navigation or /cmd_vel.
"""
import json
import math
import time
from collections import Counter, deque
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .core.rgbd_observation import (
    Observation3D, ObservationHistory, stationary_from_twist)


def select_unique_target(detections, target):
    """Return one target detection; reject ambiguity without a tracker ID."""
    matches = [
        item for item in detections
        if isinstance(item, dict) and item.get('class') == target
    ]
    if len(matches) != 1:
        return None, len(matches)
    return matches[0], 1


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

    default_map_frame = 'map'

    def __init__(self):
        super().__init__('object_localizer_node')
        self.declare_parameters('', [
            ('depth_topic', '/camera/depth/image_raw'),
            ('camera_info_topic', '/camera_info'),
            ('detections_topic', '/semantic/search/detections_json'),
            ('map_frame', self.default_map_frame),
            ('odom_topic', '/odom'),
            ('max_sync_difference_s', 0.10),  # Deprecated compatibility alias.
            ('stationary_sync_tolerance_s', 0.15),
            ('moving_sync_tolerance_s', 0.10),
            ('sync_wait_s', 1.0),
            ('max_capture_age_s', 2.0),
            ('max_future_stamp_s', 0.10),
            ('motion_state_freshness_s', 0.50),
            ('stationary_linear_speed_mps', 0.02),
            ('stationary_angular_speed_rps', 0.05),
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
            ('observation_window_s', 4.0),
            ('max_position_jitter_m', 0.45),
            ('consecutive_outliers_to_reset', 2),
            ('target_timeout_s', 2.0),
        ])
        self.bridge = CvBridge()
        self.camera_info = None
        # A bounded buffer is needed because CPU YOLO inference may be delayed.
        self.depth_buffer = deque(maxlen=60)
        self.pending_detections = deque(maxlen=8)
        self.sync_counts = Counter({key: 0 for key in (
            'depth_received', 'detections_received', 'exact_matches',
            'close_matches', 'matches_after_wait', 'timestamp_mismatches', 'expired_pending',
            'out_of_order_depth', 'depth_evictions', 'pending_evictions',
            'stale_or_future_depth', 'stale_or_future_detections', 'clock_resets',
            'localization_validation_failures', 'tf_rejections',
            'depth_quality_rejections', 'validated_poses',
            'rejected_observations', 'history_resets', 'expired_observations',
            'duplicate_rgb_exclusions', 'duplicate_depth_exclusions',
            'position_outliers', 'ambiguous_target_rejections',
            'stationary_matches', 'moving_matches', 'unknown_motion_matches')})
        self.in_localization = False
        self.max_depth_stamp_ns = None
        self.last_sync_report = time.monotonic()
        self.history = ObservationHistory(
            required=int(self.get_parameter('confirmation_positions').value),
            window_s=float(self.get_parameter('observation_window_s').value),
            max_jitter_m=float(self.get_parameter('max_position_jitter_m').value),
            outliers_to_reset=int(
                self.get_parameter('consecutive_outliers_to_reset').value))
        self.current_target = None
        self.last_detection_ns = -1
        self.valid = False
        self.last_valid_wall_time = 0.0
        self.last_valid_depth_ns = 0
        self.observation_sequence = 0
        self.last_odom = None
        self.last_odom_received_at = None
        self.last_match = None
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
        self.observation_pub = self.create_publisher(
            String, '/semantic/perception/target_observation_json', 10)
        self.status_pub = self.create_publisher(
            String, '/semantic/perception/status', 10)
        self.sync_status_pub = self.create_publisher(
            String, '/semantic/perception/sync_diagnostics_json', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic/perception/target_markers', 10)
        self.last_known_marker_pub = self.create_publisher(
            MarkerArray, '/semantic/perception/last_known_target_markers',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Clear any marker retained by an RViz instance across node restarts.
        self.clear_last_known_marker()
        self.depth_sub = self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.camera_info_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.odom_callback, qos_profile_sensor_data)
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
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.2, self.watchdog_callback, clock=self.watchdog_clock)
        # Clock notifications, not message order, define simulation epochs.
        self.clock_jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True),
            post_callback=self.on_clock_jump)
        self.report('WAITING: depth, camera calibration, and YOLO detections')


    @staticmethod
    def timestamp_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


    def report(self, message):
        self.status_pub.publish(String(data=message))
        now = time.monotonic()
        if message != self.last_status or now - self.last_status_log_time > 5.0:
            warning_categories = (
                'RGBD_TIMEOUT:', 'DEPTH_REJECTED:', 'TF_REJECTED:',
                'POSITION_OUTLIER:', 'HISTORY_RESET:',
                'LIVE_POSE_INVALIDATED:', 'OBSERVATION_REJECTED:')
            if message.startswith(warning_categories):
                self._log_warning(message)
            else:
                self._log_info(message)
            self.last_status = message
            self.last_status_log_time = now

    def _log_info(self, message):
        self.get_logger().info(message)

    def _log_warning(self, message):
        self.get_logger().warning(message)


    def clear_marker(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        self.last_marker_pose = None

    def clear_last_known_marker(self):
        marker = Marker()
        marker.ns = 'last_known_target'
        marker.id = 0
        marker.action = Marker.DELETE
        self.last_known_marker_pub.publish(MarkerArray(markers=[marker]))


    def history_count(self):
        return len(self.history.observations)

    def reject_observation(self, category, reason):
        """Reject one candidate without destroying independent prior evidence."""
        self.sync_counts['rejected_observations'] += 1
        if self.in_localization:
            self.sync_counts['localization_validation_failures'] += 1
        if category == 'TF_REJECTED':
            self.sync_counts['tf_rejections'] += 1
        elif category == 'DEPTH_REJECTED':
            self.sync_counts['depth_quality_rejections'] += 1
        self.report(
            f'{category}: {reason}; history retained '
            f'{self.history_count()}/{self.history.required}')

    def publish_associated_observation(self, valid, pose=None, *,
                                       rgb_ns=0, depth_ns=0, reason=''):
        """Publish one atomic authorization record alongside legacy topics."""
        self.observation_sequence += 1
        record = {
            'sequence': self.observation_sequence,
            'valid': bool(valid),
            'target': self.current_target or '',
            'rgb_timestamp_ns': int(rgb_ns),
            'depth_timestamp_ns': int(depth_ns),
            'reason': reason,
        }
        if pose is not None:
            record['pose'] = {
                'frame_id': pose.header.frame_id,
                'timestamp_sec': int(pose.header.stamp.sec),
                'timestamp_nanosec': int(pose.header.stamp.nanosec),
                'position': [
                    pose.pose.position.x, pose.pose.position.y,
                    pose.pose.position.z],
                'orientation': [
                    pose.pose.orientation.x, pose.pose.orientation.y,
                    pose.pose.orientation.z, pose.pose.orientation.w],
            }
        self.observation_pub.publish(String(data=json.dumps(record)))

    def invalidate_live(self, reason):
        """Withdraw navigation authorization while retaining history."""
        was_valid = self.valid
        self.valid = False
        self.valid_pub.publish(Bool(data=False))
        self.publish_associated_observation(False, reason=reason)
        if was_valid and not self._hold_marker():
            self.clear_marker()
        self.report(
            f'LIVE_POSE_INVALIDATED: reason={reason}; history retained '
            f'{self.history_count()}/{self.history.required}')

    def reset_history(self, reason):
        """Discard accumulated evidence only for an explicit reset event."""
        previous = self.history.clear()
        self.sync_counts['history_resets'] += 1
        self.valid = False
        self.valid_pub.publish(Bool(data=False))
        self.publish_associated_observation(False, reason=reason)
        if previous and not self._hold_marker():
            self.clear_marker()
        self.report(
            f'HISTORY_RESET: reason={reason} old_count={previous} new_count=0')

    # Compatibility for older callers; new code should choose reject/reset/live.
    def invalidate(self, reason):
        self.invalidate_live(reason)


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


    def on_clock_jump(self, jump):
        # A ROS time-source change or backward /clock jump starts a new epoch.
        self.depth_buffer.clear()
        self.pending_detections.clear()
        self.max_depth_stamp_ns = None
        self.last_detection_ns = -1
        self.last_odom = None
        self.last_odom_received_at = None
        self.sync_counts['clock_resets'] += 1
        self.clear_last_known_marker()
        self.reset_history('CLOCK_EPOCH_CHANGED')

    def _capture_now_ns(self):
        return self.get_clock().now().nanoseconds

    def _capture_fresh(self, stamp_ns):
        age_ns = self._capture_now_ns() - stamp_ns
        return (stamp_ns > 0 and
                age_ns <= float(self.get_parameter('max_capture_age_s').value) * 1e9 and
                age_ns >= -float(self.get_parameter('max_future_stamp_s').value) * 1e9)

    def odom_callback(self, msg):
        self.last_odom = msg
        self.last_odom_received_at = time.monotonic()

    def motion_state(self):
        """Return True stationary, False moving, or None when uncertain."""
        freshness = float(
            self.get_parameter('motion_state_freshness_s').value)
        if (self.last_odom is None or self.last_odom_received_at is None or
                time.monotonic() - self.last_odom_received_at > freshness):
            return None
        stamp_ns = self.timestamp_ns(self.last_odom.header.stamp)
        if not self._capture_fresh(stamp_ns):
            return None
        twist = self.last_odom.twist.twist
        return stationary_from_twist(
            (twist.linear.x, twist.linear.y, twist.linear.z),
            (twist.angular.x, twist.angular.y, twist.angular.z),
            float(self.get_parameter('stationary_linear_speed_mps').value),
            float(self.get_parameter('stationary_angular_speed_rps').value))

    def current_sync_tolerance(self):
        stationary = self.motion_state()
        parameter = ('stationary_sync_tolerance_s' if stationary is True
                     else 'moving_sync_tolerance_s')
        return float(self.get_parameter(parameter).value), stationary

    def expire_history(self):
        expired = self.history.expire(self._capture_now_ns())
        for observation in expired:
            self.sync_counts['expired_observations'] += 1
            age = (self._capture_now_ns() - observation.depth_stamp_ns) / 1e9
            self.report(
                f'POSITION_EXPIRED: age={age:.1f}s remaining_count='
                f'{self.history_count()}/{self.history.required}')
        if expired and self.valid and not self.history.confirmed:
            self.invalidate_live('INSUFFICIENT_UNEXPIRED_CONFIRMATIONS')
        return expired

    def depth_callback(self, msg):
        # Resolve expired entries before admitting this arrival, even if the
        # steady-clock timer callback was delayed by executor scheduling.
        self.process_pending()
        self.sync_counts['depth_received'] += 1
        stamp = self.timestamp_ns(msg.header.stamp)
        if not self._capture_fresh(stamp):
            self.sync_counts['stale_or_future_depth'] += 1
            return
        if self.max_depth_stamp_ns is not None and stamp < self.max_depth_stamp_ns:
            self.sync_counts['out_of_order_depth'] += 1
        self.max_depth_stamp_ns = max(stamp, self.max_depth_stamp_ns or stamp)
        if len(self.depth_buffer) == self.depth_buffer.maxlen:
            self.sync_counts['depth_evictions'] += 1
        self.depth_buffer.append(msg)
        self.process_pending()


    def detection_callback(self, msg):
        self.sync_counts['detections_received'] += 1
        try:
            data = json.loads(msg.data)
            target = str(data['target'])
            stamp_ns = (int(data['timestamp_sec']) * 1_000_000_000
                        + int(data['timestamp_nanosec']))
            detections = data['detections']
            if not isinstance(detections, list):
                raise ValueError('detections must be a list')
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.reject_observation(
                'OBSERVATION_REJECTED', f'bad detection JSON: {exc}')
            return
        if not self._capture_fresh(stamp_ns):
            self.sync_counts['stale_or_future_detections'] += 1
            self.reject_observation(
                'OBSERVATION_REJECTED',
                'stale or future RGB capture timestamp')
            return
        if target != self.current_target:
            self.clear_last_known_marker()
            old_target = self.current_target
            self.current_target = target
            self.last_detection_ns = -1
            self.pending_detections.clear()
            self.reset_history(
                f'TARGET_CHANGED({old_target or "none"}->{target})')
        if stamp_ns <= self.last_detection_ns:
            return  # Ignore duplicate or out-of-order detections.
        self.last_detection_ns = stamp_ns
        matches = [d for d in detections
                   if isinstance(d, dict) and d.get('class') == target]
        if not data.get('confirmed') or not matches:
            # One missed YOLO frame must not immediately destroy a
            # previously confirmed 3D map position during NBV braking.
            # This is a bounded hold, NOT a new object observation.
            if (self.valid and self.last_valid_wall_time > 0 and
                    time.monotonic() - self.last_valid_wall_time <
                    float(self.get_parameter('target_timeout_s').value)):
                self.report('HOLDING: last validated target; waiting '
                            'for fresh RGB-D confirmation')
                return
            self.reject_observation(
                'OBSERVATION_REJECTED', 'no confirmed YOLO detection')
            return
        if len(self.pending_detections) == self.pending_detections.maxlen:
            self.sync_counts['pending_evictions'] += 1
            self.pending_detections.popleft()
            self.reject_observation(
                'RGBD_TIMEOUT', 'pending detection buffer overflow')
        self.pending_detections.append((data, stamp_ns, time.monotonic()))
        self.process_pending()


    def process_pending(self):
        # FIFO prevents later detections from publishing before earlier ones.
        while self.pending_detections:
            data, rgb_ns, received_at = self.pending_detections[0]
            if not self._capture_fresh(rgb_ns):
                self.pending_detections.popleft()
                self.sync_counts['stale_or_future_detections'] += 1
                self.reject_observation(
                    'OBSERVATION_REJECTED',
                    'stale or future RGB capture timestamp')
                continue
            candidates = [m for m in self.depth_buffer
                          if self._capture_fresh(self.timestamp_ns(m.header.stamp))]
            # Earlier timestamp wins an equal-distance tie; identical stamps use
            # first arrival. Independence is enforced by ObservationHistory.
            best = min(candidates, key=lambda m: (
                abs(self.timestamp_ns(m.header.stamp) - rgb_ns),
                self.timestamp_ns(m.header.stamp)), default=None)
            gap_ns = (abs(self.timestamp_ns(best.header.stamp) - rgb_ns)
                      if best is not None else None)
            exact = gap_ns == 0
            # The hardware produces same-cycle stamps 1–2 ms apart.
            # Keep the full reordering window for ambiguous/missing pairs.
            close_pair = gap_ns is not None and gap_ns <= 2_000_000
            if not close_pair and time.monotonic() - received_at < float(
                    self.get_parameter('sync_wait_s').value):
                return
            self.pending_detections.popleft()
            tolerance_s, stationary = self.current_sync_tolerance()
            if best is None or gap_ns > round(tolerance_s * 1e9):
                self.sync_counts['expired_pending'] += 1
                if best is not None:
                    self.sync_counts['timestamp_mismatches'] += 1
                detail = ('no eligible depth'
                          if best is None else
                          f'nearest dt={gap_ns / 1e6:.0f}ms exceeds '
                          f'{tolerance_s * 1e3:.0f}ms')
                self.reject_observation(
                    'RGBD_TIMEOUT',
                    f'{detail} after '
                    f'{float(self.get_parameter("sync_wait_s").value) * 1e3:.0f}ms')
                continue
            depth_ns = self.timestamp_ns(best.header.stamp)
            match_kind = 'close_matches' if close_pair else 'matches_after_wait'
            self.sync_counts['exact_matches' if exact else match_kind] += 1
            motion_label = ('stationary' if stationary is True else
                            'moving' if stationary is False else 'unknown')
            self.sync_counts[f'{motion_label}_matches'] += 1
            self.last_match = {
                'rgb_timestamp_ns': rgb_ns,
                'depth_timestamp_ns': depth_ns,
                'difference_ms': gap_ns / 1e6,
                'tolerance_ms': tolerance_s * 1e3,
                'motion_state': motion_label,
                'decision': 'accepted',
            }
            self.report(
                f'RGBD_MATCH: rgb={rgb_ns / 1e9:.3f}s '
                f'depth={depth_ns / 1e9:.3f}s dt={gap_ns / 1e6:.0f}ms '
                f'tolerance={tolerance_s * 1e3:.0f}ms '
                f'stationary={str(stationary).lower()}')
            self.in_localization = True
            try:
                self.localize(data, best, gap_ns / 1e9)
            finally:
                self.in_localization = False


    def localize(self, data, depth_msg, dt):
        info = self.camera_info
        if info is None:
            self.reject_observation(
                'OBSERVATION_REJECTED', '/camera_info not received')
            return
        if not data.get('frame_id') or data['frame_id'] != depth_msg.header.frame_id:
            self.reject_observation(
                'OBSERVATION_REJECTED', 'RGB/depth optical frames differ')
            return
        if info.header.frame_id and info.header.frame_id != depth_msg.header.frame_id:
            self.reject_observation(
                'OBSERVATION_REJECTED',
                'CameraInfo/depth optical frames differ')
            return
        if info.width != depth_msg.width or info.height != depth_msg.height:
            self.reject_observation(
                'OBSERVATION_REJECTED',
                'CameraInfo/depth resolutions differ')
            return
        if (data.get('image_width', depth_msg.width) != depth_msg.width or
                data.get('image_height', depth_msg.height) != depth_msg.height):
            self.reject_observation(
                'OBSERVATION_REJECTED', 'RGB/depth image resolutions differ')
            return
        best, match_count = select_unique_target(
            data['detections'], data['target'])
        if best is None:
            self.sync_counts['ambiguous_target_rejections'] += 1
            self.reject_observation(
                'OBSERVATION_REJECTED',
                f'ambiguous same-class detections count={match_count}')
            return
        try:
            x1, y1, x2, y2 = (float(v) for v in best['bbox_xyxy'])
            if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
                raise ValueError('non-finite box coordinates')
            if x2 <= x1 or y2 <= y1:
                raise ValueError('empty or inverted box')
        except (KeyError, TypeError, ValueError) as exc:
            self.reject_observation(
                'OBSERVATION_REJECTED', f'invalid bounding box: {exc}')
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
            if depth_msg.encoding == '32FC1':
                depth = np.asarray(depth, dtype=np.float32)  # meters
            elif depth_msg.encoding == '16UC1':
                depth = np.asarray(depth, dtype=np.float32) * 0.001  # mm -> m
            else:
                self.reject_observation(
                    'DEPTH_REJECTED',
                    f'unsupported depth encoding {depth_msg.encoding}')
                return
        except Exception as exc:
            self.reject_observation(
                'DEPTH_REJECTED', f'depth conversion failed: {exc}')
            return
        if depth.ndim != 2:
            self.reject_observation(
                'DEPTH_REJECTED', 'expected one-channel depth image')
            return
        height, width = depth.shape
        u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        if not (0 <= u < width and 0 <= v < height):
            self.reject_observation(
                'OBSERVATION_REJECTED',
                'bounding-box center outside depth image')
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
            self.reject_observation('DEPTH_REJECTED', 'empty depth crop')
            return
        min_depth = float(self.get_parameter('min_depth_m').value)
        max_depth = float(self.get_parameter('max_depth_m').value)
        values = region[np.isfinite(region) & (region >= min_depth) &
                        (region <= max_depth)]
        if (values.size < int(self.get_parameter('min_valid_pixels').value) or
                values.size / region.size < float(
                    self.get_parameter('min_valid_fraction').value)):
            self.reject_observation(
                'DEPTH_REJECTED', 'insufficient valid depth pixels')
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
            self.reject_observation('DEPTH_REJECTED', error)
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
            self.reject_observation(
                'OBSERVATION_REJECTED', 'invalid camera intrinsic matrix')
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
            self.reject_observation(
                'TF_REJECTED',
                f'transform unavailable at capture timestamp: {exc}')
            return
        xyz = np.array([map_point.point.x, map_point.point.y, map_point.point.z])
        if not np.all(np.isfinite(xyz)):
            self.reject_observation(
                'OBSERVATION_REJECTED', 'invalid map coordinates')
            return
        self.expire_history()
        rgb_ns = (int(data['timestamp_sec']) * 1_000_000_000
                  + int(data['timestamp_nanosec']))
        depth_ns = self.timestamp_ns(depth_msg.header.stamp)
        identity = str(best.get('track_id', data.get('target_identity', '')))
        observation = Observation3D(
            target=str(data['target']), identity=identity,
            rgb_stamp_ns=rgb_ns, depth_stamp_ns=depth_ns,
            xyz=tuple(float(value) for value in xyz))
        decision, details = self.history.add(observation)
        if decision == 'duplicate_rgb':
            self.sync_counts['duplicate_rgb_exclusions'] += 1
            self.reject_observation(
                'OBSERVATION_REJECTED',
                'duplicate RGB capture timestamp is not independent evidence')
            return
        if decision == 'duplicate_depth':
            self.sync_counts['duplicate_depth_exclusions'] += 1
            self.reject_observation(
                'OBSERVATION_REJECTED',
                'reused depth capture timestamp is not independent evidence')
            return
        if decision == 'outlier':
            self.sync_counts['position_outliers'] += 1
            self.reject_observation(
                'POSITION_OUTLIER',
                f'distance={details["distance_m"]:.2f}m '
                f'limit={self.history.max_jitter_m:.2f}m '
                f'consecutive={details["consecutive"]}')
            return
        if decision == 'cluster_reset':
            self.sync_counts['position_outliers'] += 1
            self.sync_counts['history_resets'] += 1
            self.valid = False
            self.valid_pub.publish(Bool(data=False))
            self.publish_associated_observation(
                False, reason='CONTRADICTORY_SPATIAL_CLUSTER')
            self.report(
                'HISTORY_RESET: reason=CONTRADICTORY_SPATIAL_CLUSTER '
                f'old_count={details["previous_count"]} new_count=1 '
                f'distance={details["distance_m"]:.2f}m')
        elif decision in ('target_changed', 'identity_changed'):
            reason = ('TARGET_CHANGED' if decision == 'target_changed'
                      else 'TARGET_IDENTITY_CHANGED')
            self.reset_history(reason)
            self.history.add(observation)
        elif decision != 'accepted':
            self.reject_observation(
                'OBSERVATION_REJECTED', f'invalid 3D candidate ({decision})')
            return
        required = self.history.required
        count = self.history_count()
        self.report(
            f'POSITION_ACCEPTED: target={data["target"]} count={count}/{required} '
            f'window={float(self.get_parameter("observation_window_s").value):.1f}s')
        if count < required:
            if self.valid:
                self.invalidate_live('INSUFFICIENT_CONFIRMATIONS')
            else:
                self.valid_pub.publish(Bool(data=False))
            return
        stable_xyz = self.history.stable_xyz()
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('map_frame').value
        pose.header.stamp = depth_msg.header.stamp
        pose.pose.position.x = float(stable_xyz[0])
        pose.pose.position.y = float(stable_xyz[1])
        pose.pose.position.z = float(stable_xyz[2])
        pose.pose.orientation.w = 1.0  # orientation NOT estimated
        self.pose_pub.publish(pose)
        self.sync_counts['validated_poses'] += 1
        self.valid_pub.publish(Bool(data=True))
        self.publish_associated_observation(
            True, pose, rgb_ns=rgb_ns, depth_ns=depth_ns)
        self.publish_marker(pose)
        self.publish_last_known_marker(pose)
        self.valid = True
        self.last_valid_wall_time = time.monotonic()
        self.last_valid_depth_ns = depth_ns
        self.report(
            f'TARGET_LOCALIZED: {data["target"]} '
            f'{pose.header.frame_id}=({pose.pose.position.x:.2f}, '
            f'{pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})m; '
            f'confirmations={count}/{required} latest_age='
            f'{max(0.0, (self._capture_now_ns() - depth_ns) / 1e9):.3f}s; '
            f'depth={z:.2f}m; dt={dt:.3f}s; {depth_note}'
        )


    def publish_marker(self, pose):
        # Show live validated detections while idle. After a completed mission,
        # keep its cleanup behavior until the next mission is started.
        if self.mission_state in ('SUCCEEDED', 'FAILED', 'CANCELLED'):
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


    def publish_last_known_marker(self, pose):
        """Publish visualization only; never refresh live localization state."""
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'last_known_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        # A translucent, slightly larger shell leaves the green marker visible.
        marker.scale.x = marker.scale.y = marker.scale.z = 0.19
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.lifetime = Duration(seconds=0.0).to_msg()
        self.last_known_marker_pub.publish(MarkerArray(markers=[marker]))

    def watchdog_callback(self):
        now = time.monotonic()
        self.process_pending()
        self.expire_history()
        if now - self.last_sync_report >= 5.0:
            report = dict(self.sync_counts)
            matched = report['exact_matches'] + report['close_matches'] + report['matches_after_wait']
            completed = matched + report['expired_pending']
            report.update({
                'eligible_match_rate': (
                    matched / completed if completed else 0.0),
                'history_count': self.history_count(),
                'history_required': self.history.required,
                'live_valid': self.valid,
                'pending_count': len(self.pending_detections),
                'depth_buffer_count': len(self.depth_buffer),
                'last_match': self.last_match,
            })
            self.sync_status_pub.publish(
                String(data=json.dumps(report, sort_keys=True)))
            self.last_sync_report = now
        if self.valid and now - self.last_valid_wall_time > float(
                self.get_parameter('target_timeout_s').value):
            self.invalidate_live('NO_RECENT_VALID_3D_OBSERVATION')


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
