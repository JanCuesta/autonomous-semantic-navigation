"""Tests for the visualization-only last-known target marker."""
import json
from types import SimpleNamespace

import asn_semantical_navigation.object_localizer_node as module
from geometry_msgs.msg import PointStamped
import numpy as np
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def localizer(monkeypatch):
    rclpy.init()
    node = module.ObjectLocalizer()
    for name in ('pose_pub', 'valid_pub', 'observation_pub', 'status_pub', 'marker_pub',
                 'last_known_marker_pub'):
        setattr(node, name, Recorder())
    node.bridge = SimpleNamespace(
        imgmsg_to_cv2=lambda *a, **k: np.ones((40, 40), dtype=np.float32))
    node.tf_buffer = SimpleNamespace(lookup_transform=lambda *a, **k: None)
    point = PointStamped()
    point.point.x, point.point.y, point.point.z = 1., 2., 3.
    monkeypatch.setattr(module, 'do_transform_point', lambda *a: point)
    info = CameraInfo()
    info.width = info.height = 40
    info.header.frame_id = 'camera'
    info.k = [100., 0., 20., 0., 100., 20., 0., 0., 1.]
    node.camera_info = info
    node.current_target = 'backpack'
    node._capture_now_ns = lambda: 2_000_000_000
    yield node, point
    node.destroy_node()
    rclpy.shutdown()


def sample(node):
    node._test_sample_index = getattr(node, '_test_sample_index', 0) + 1
    stamp_ns = 1_000_000_000 + node._test_sample_index * 10_000_000
    image = Image()
    image.width = image.height = 40
    image.encoding = '32FC1'
    image.header.frame_id = 'camera'
    image.header.stamp.sec, image.header.stamp.nanosec = divmod(
        stamp_ns, 1_000_000_000)
    data = {'target': 'backpack', 'frame_id': 'camera',
            'timestamp_sec': image.header.stamp.sec,
            'timestamp_nanosec': image.header.stamp.nanosec, 'detections': [
        {'class': 'backpack', 'confidence': 0.9, 'bbox_xyxy': [0, 0, 40, 40]}]}
    node.localize(data, image, 0.)


def confirm(node):
    for _ in range(3):
        sample(node)


def test_confirm_and_update_marker_without_affecting_live_interfaces(localizer):
    node, point = localizer
    sample(node)
    assert not node.last_known_marker_pub.messages
    sample(node)
    sample(node)
    marker = node.last_known_marker_pub.messages[-1].markers[0]
    assert node.marker_pub.messages[-1].markers[0].action == Marker.ADD
    assert marker.header.frame_id == 'map'
    assert (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z) == (1., 2., 3.)
    assert marker.type == Marker.SPHERE and marker.action == Marker.ADD
    assert marker.color.r == 1. and marker.color.g == 0.5 and marker.color.b == 0.
    assert marker.lifetime.sec == 0 and marker.lifetime.nanosec == 0
    counts = (len(node.pose_pub.messages), len(node.valid_pub.messages))
    timestamp = node.last_valid_wall_time
    node.publish_last_known_marker(node.pose_pub.messages[-1])
    assert (len(node.pose_pub.messages), len(node.valid_pub.messages)) == counts
    assert node.last_valid_wall_time == timestamp
    point.point.x = 1.2
    confirm(node)
    confirm(node)
    assert node.last_known_marker_pub.messages[-1].markers[0].pose.position.x == pytest.approx(1.2)


def test_dropout_timeout_idle_and_target_change(localizer, monkeypatch):
    node, _ = localizer
    confirm(node)
    last = node.last_known_marker_pub.messages[-1]
    node.last_valid_wall_time = 0
    detection = {'target': 'backpack', 'timestamp_sec': 1, 'timestamp_nanosec': 0,
                 'detections': [], 'confirmed': False}
    node.detection_callback(String(data=json.dumps(detection)))
    assert node.valid
    assert node.last_known_marker_pub.messages[-1] is last
    confirm(node)
    last = node.last_known_marker_pub.messages[-1]
    monkeypatch.setattr(module.time, 'monotonic', lambda: node.last_valid_wall_time + 3.)
    node.watchdog_callback()
    assert not node.valid and node.last_known_marker_pub.messages[-1] is last
    node.on_mission_state(String(data='IDLE'))
    assert node.last_known_marker_pub.messages[-1] is last
    detection = {'target': 'chair', 'timestamp_sec': 2, 'timestamp_nanosec': 0,
                 'detections': [], 'confirmed': False}
    node.detection_callback(String(data=json.dumps(detection)))
    assert node.last_known_marker_pub.messages[-1].markers[0].action == Marker.DELETE


def test_clock_reset_clears_history(localizer):
    node, _ = localizer
    confirm(node)
    old = Image()
    old.header.stamp.sec = 10
    node.depth_buffer.append(old)
    node.on_clock_jump(None)
    assert node.last_known_marker_pub.messages[-1].markers[0].action == Marker.DELETE
    assert not node.valid


def test_publisher_qos():
    rclpy.init()
    node = module.ObjectLocalizer()
    try:
        qos = node.last_known_marker_pub.qos_profile
        assert qos.depth == 1
        assert qos.reliability == ReliabilityPolicy.RELIABLE
        assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_localization_rejects_invalid_calibration_depth_and_tf(localizer):
    from tf2_ros import TransformException
    node, _ = localizer
    node.camera_info.width = 39
    sample(node)
    assert not node.valid and 'resolutions differ' in node.status_pub.messages[-1].data
    node.camera_info.width = 40
    node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *a, **k: np.zeros((40, 40), dtype=np.float32))
    sample(node)
    assert not node.valid and 'insufficient valid depth' in node.status_pub.messages[-1].data
    node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *a, **k: np.ones((40, 40), dtype=np.float32))
    def missing_tf(*args, **kwargs):
        raise TransformException('missing historical transform')
    node.tf_buffer = SimpleNamespace(lookup_transform=missing_tf)
    sample(node)
    assert not node.valid and 'TF_REJECTED' in node.status_pub.messages[-1].data
