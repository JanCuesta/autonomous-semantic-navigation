"""Deterministic regression tests for RGB/depth synchronization."""
import json
from types import SimpleNamespace

import pytest
import rclpy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import asn_semantical_navigation.object_localizer_node as module
from asn_semantical_navigation.core.rgbd_observation import Observation3D


@pytest.fixture
def sync_node(monkeypatch):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.now)
    rclpy.init()
    node = module.ObjectLocalizer()
    node.current_target = 'chair'
    node._capture_now_ns = lambda: 10_500_000_000 + int(clock.now * 1e9)
    chosen = []
    node.localize = lambda data, depth, dt: chosen.append((data, depth, dt))
    yield node, clock, chosen
    node.destroy_node()
    rclpy.shutdown()


def depth(stamp_ns):
    msg = Image()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    return msg


def detection(stamp_ns, target='chair'):
    sec, nano = divmod(stamp_ns, 10**9)
    return String(data=json.dumps({'target': target, 'timestamp_sec': sec,
                                  'timestamp_nanosec': nano, 'confirmed': True,
                                  'detections': [{'class': target}]}))


def motion(stamp_ns, linear=0.0, angular=0.0):
    msg = Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
    msg.twist.twist.linear.x = linear
    msg.twist.twist.angular.z = angular
    return msg


def test_delayed_out_of_order_depth_does_not_clear_pending(sync_node):
    node, clock, chosen = sync_node
    node.depth_callback(depth(10_300_000_000))
    node.detection_callback(detection(10_000_000_000))
    node.depth_callback(depth(10_000_000_000))
    assert len(chosen) == 1 and chosen[0][2] == 0


def test_newer_depth_does_not_trigger_premature_rejection(sync_node):
    node, clock, chosen = sync_node
    node.depth_callback(depth(10_200_000_000))
    node.detection_callback(detection(10_000_000_000))
    clock.now = 0.1
    node.depth_callback(depth(10_000_000_000))
    assert len(chosen) == 1 and chosen[0][2] == 0


def test_watchdog_evaluates_buffered_candidate_at_deadline(sync_node):
    node, clock, chosen = sync_node
    node.depth_callback(depth(9_980_000_000))
    node.detection_callback(detection(10_000_000_000))
    clock.now = 1.0
    node.watchdog_callback()
    assert len(chosen) == 1 and chosen[0][2] == pytest.approx(0.02)


@pytest.mark.parametrize('offset,accepted', [(2_000_001, True), (100_000_000, True),
                                              (101_000_000, False), (200_000_000, False)])
def test_tolerance_and_deadline(sync_node, offset, accepted):
    node, clock, chosen = sync_node
    node.depth_callback(depth(10_000_000_000 + offset))
    node.detection_callback(detection(10_000_000_000))
    clock.now = 0.999
    node.watchdog_callback()
    assert not chosen and len(node.pending_detections) == 1
    clock.now = 1.0
    node.watchdog_callback()
    assert bool(chosen) is accepted
    assert not node.pending_detections
    if accepted:
        assert chosen[0][2] == pytest.approx(offset / 1e9)
    else:
        assert node.sync_counts['timestamp_mismatches'] == 1


def test_no_depth_expires(sync_node):
    node, clock, chosen = sync_node
    node.detection_callback(detection(10_000_000_000))
    clock.now = 1.0
    node.watchdog_callback()
    assert not chosen and not node.pending_detections
    assert node.sync_counts['expired_pending'] == 1


@pytest.mark.parametrize('offset_ns,linear,stationary,accepted', [
    (80_000_000, 0.0, True, True),
    (120_000_000, 0.0, True, True),
    (120_000_000, 0.10, False, False),
    (120_000_000, None, None, False),
    (151_000_000, 0.0, True, False),
])
def test_motion_dependent_tolerance(
        sync_node, offset_ns, linear, stationary, accepted):
    node, clock, chosen = sync_node
    node.depth_callback(depth(10_000_000_000 + offset_ns))
    node.detection_callback(detection(10_000_000_000))
    clock.now = 1.0
    if linear is not None:
        node.odom_callback(motion(11_500_000_000, linear=linear))
    assert node.motion_state() is stationary
    node.watchdog_callback()
    assert bool(chosen) is accepted


def test_motion_state_is_evaluated_at_matching_deadline(sync_node):
    node, clock, chosen = sync_node
    node.depth_callback(depth(10_120_000_000))
    node.detection_callback(detection(10_000_000_000))
    node.odom_callback(motion(10_500_000_000, linear=0.0))
    assert not chosen
    clock.now = 1.0
    node.odom_callback(motion(11_500_000_000, linear=0.1))
    node.watchdog_callback()
    assert not chosen
    assert node.sync_counts['timestamp_mismatches'] == 1


def test_exact_priority_and_fifo(sync_node):
    node, clock, chosen = sync_node
    node.depth_callback(depth(9_990_000_000))
    node.detection_callback(detection(10_000_000_000))
    node.detection_callback(detection(10_100_000_000))
    node.depth_callback(depth(10_100_000_000))
    assert not chosen  # The second exact match cannot overtake the first.
    node.depth_callback(depth(10_000_000_000))
    assert [entry[0]['timestamp_nanosec'] for entry in chosen] == [0, 100_000_000]
    assert [entry[2] for entry in chosen] == [0, 0]
    assert node.sync_counts['out_of_order_depth'] == 1


def test_tie_prefers_earlier_stamp_and_depth_can_be_reused(sync_node):
    node, clock, chosen = sync_node
    node.depth_callback(depth(10_050_000_000))
    node.depth_callback(depth(9_950_000_000))
    node.detection_callback(detection(10_000_000_000))
    clock.now = 1.0
    node.watchdog_callback()
    assert node.timestamp_ns(chosen[0][1].header.stamp) == 9_950_000_000
    node._capture_now_ns = lambda: 11_000_000_000
    node.detection_callback(detection(10_010_000_000))
    node.detection_callback(detection(10_020_000_000))
    clock.now = 2.0
    node.watchdog_callback()
    assert node.timestamp_ns(chosen[1][1].header.stamp) == 10_050_000_000
    assert chosen[1][1] is chosen[2][1]


def test_depth_and_pending_overflow_accounted(sync_node):
    node, clock, chosen = sync_node
    for i in range(61):
        node.depth_callback(depth(9_000_000_000 + i))
    assert len(node.depth_buffer) == 60 and node.sync_counts['depth_evictions'] == 1
    assert node.timestamp_ns(node.depth_buffer[0].header.stamp) == 9_000_000_001
    for i in range(9):
        node.detection_callback(detection(10_000_000_000 + i))
    assert len(node.pending_detections) == 8 and node.sync_counts['pending_evictions'] == 1
    assert node.pending_detections[0][1] == 10_000_000_001


def test_duplicate_target_change_and_unconfirmed(sync_node):
    node, clock, chosen = sync_node
    node.detection_callback(detection(10_000_000_000))
    node.detection_callback(detection(10_000_000_000))
    assert len(node.pending_detections) == 1
    node.detection_callback(detection(10_000_000_001, 'bottle'))
    assert len(node.pending_detections) == 1 and node.pending_detections[0][0]['target'] == 'bottle'
    msg = json.loads(detection(10_000_000_002, 'bottle').data)
    msg['confirmed'] = False
    node.detection_callback(String(data=json.dumps(msg)))
    # An unrelated RGB-only miss must not discard an already pending match.
    assert len(node.pending_detections) == 1


def test_stale_inference_future_timestamps_and_recovery(sync_node):
    node, clock, chosen = sync_node
    node.detection_callback(detection(8_000_000_000))
    node.detection_callback(detection(20_000_000_000))
    assert not node.pending_detections and node.last_detection_ns == -1
    node.depth_callback(depth(20_000_000_000))
    node.depth_callback(depth(10_000_000_000))
    node.detection_callback(detection(10_000_000_000))
    assert len(chosen) == 1 and chosen[0][0]['timestamp_sec'] == 10
    assert node.sync_counts['stale_or_future_detections'] == 2
    assert node.sync_counts['stale_or_future_depth'] == 1


def test_epoch_reset_clears_buffers_and_rejects_old_epoch(sync_node):
    node, clock, chosen = sync_node
    node.depth_callback(depth(10_000_000_000))
    node.detection_callback(detection(10_050_000_000))
    node.on_clock_jump(None)
    assert not node.pending_detections and not node.depth_buffer
    node._capture_now_ns = lambda: 1_000_000_000
    node.depth_callback(depth(10_050_000_000))
    node.detection_callback(detection(1_000_000_000))
    assert not chosen
    node.depth_callback(depth(1_000_000_000))
    assert len(chosen) == 1


def test_nonpositive_stamps_are_never_rewritten_or_accepted(sync_node):
    node, clock, chosen = sync_node
    node.depth_callback(depth(0))
    node.detection_callback(detection(0))
    assert not chosen and not node.depth_buffer and not node.pending_detections


def test_different_frequencies_missing_and_no_rewrite(sync_node):
    node, clock, chosen = sync_node
    for stamp in (9_800_000_000, 10_000_000_000, 10_200_000_000):
        node.depth_callback(depth(stamp))
    for stamp in (9_800_000_000, 9_900_000_000, 10_000_000_000, 10_300_000_001):
        node.detection_callback(detection(stamp))
    clock.now = 1.0
    node.watchdog_callback()
    assert len(chosen) == 3
    assert [d['timestamp_sec'] * 10**9 + d['timestamp_nanosec'] for d, _, _ in chosen] == [
        9_800_000_000, 9_900_000_000, 10_000_000_000]
    assert node.sync_counts['timestamp_mismatches'] == 1
    assert all(dt <= 0.1 for _, _, dt in chosen)


def test_real_ros_clock_jump_callback(sync_node):
    from rclpy.time import Time
    node, clock, chosen = sync_node
    ros_clock = node.get_clock()
    ros_clock._set_ros_time_is_active(True)
    ros_clock.set_ros_time_override(Time(seconds=10))
    node._capture_now_ns = lambda: ros_clock.now().nanoseconds
    node.depth_callback(depth(10_000_000_000))
    node.detection_callback(detection(10_050_000_000))
    assert node.pending_detections
    ros_clock.set_ros_time_override(Time(seconds=1))
    assert not node.depth_buffer and not node.pending_detections
    assert node.sync_counts['clock_resets'] >= 1
    node.depth_callback(depth(10_050_000_000))
    assert not node.depth_buffer
    ros_clock._set_ros_time_is_active(False)


def test_depth_arriving_at_deadline_cannot_rescue_expired_detection(sync_node):
    node, clock, chosen = sync_node
    node.detection_callback(detection(10_000_000_000))
    clock.now = 1.0
    node.depth_callback(depth(10_000_000_000))
    assert not chosen and not node.pending_detections
    assert node.sync_counts['expired_pending'] == 1


def test_watchdog_uses_steady_clock(sync_node):
    from rclpy.clock import ClockType
    node, _, _ = sync_node
    assert node.watchdog_clock.clock_type == ClockType.STEADY_TIME


def test_multiple_same_class_detections_are_ambiguous():
    one = {'class': 'chair', 'confidence': 0.9}
    chosen, count = module.select_unique_target(
        [one, {'class': 'bottle'}], 'chair')
    assert chosen is one and count == 1
    chosen, count = module.select_unique_target(
        [one, {'class': 'chair', 'confidence': 0.8}], 'chair')
    assert chosen is None and count == 2


def test_live_authorization_expires_without_clearing_history(sync_node):
    node, clock, _ = sync_node
    node.history.add(Observation3D(
        'chair', '', 10_000_000_001, 10_000_000_002, (1.0, 2.0, 0.5)))
    node.valid = True
    node.last_valid_wall_time = 0.0
    clock.now = 2.01
    node.watchdog_callback()
    assert not node.valid
    assert len(node.history.observations) == 1


def test_hardware_pair_processed_without_wait(sync_node):
    node, clock, chosen = sync_node
    node.detection_callback(detection(10_000_000_000))
    node.depth_callback(depth(10_002_000_000))
    assert len(chosen) == 1
    assert chosen[0][2] == pytest.approx(0.002)
