"""Offline tests for mission target freshness and preserved capture time."""
import json
from types import SimpleNamespace
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from asn_semantical_navigation.approach_goal_node import ApproachGoalNode
from asn_semantical_navigation.semantical_mission_manager_node import SemanticMissionManager


class Clock:
    def now(self):
        return SimpleNamespace(nanoseconds=100_000_000_000)


class FakeMission:
    mission_started = 10.0
    pose_at = 11.0
    pose_valid = True
    associated_observation = True
    target = "chair"
    def __init__(self):
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.header.stamp.sec = 99
        self.pose.pose.orientation.w = 1.0
    def age_ok(self, stamp, limit): return True
    def param(self, name): return 2.5
    def get_clock(self): return Clock()


def test_live_pose_must_be_valid_current_and_from_this_mission():
    mission = FakeMission()
    get = SemanticMissionManager.localized_target_snapshot
    assert get(mission) is mission.pose
    mission.pose_valid = False
    mission.last_valid_pose = mission.pose
    assert get(mission) is None
    mission.pose_valid = True
    mission.pose.header.stamp.sec = 90
    assert get(mission) is None
    mission.pose.header.stamp.sec = 101
    assert get(mission) is None
    mission.pose.header.stamp.sec = 99
    mission.pose_at = 9.0
    assert get(mission) is None


def test_lock_keeps_original_measurement_stamp():
    mission = FakeMission()
    output = []
    mission.locked_pose_pub = SimpleNamespace(publish=output.append)
    mission.locked_valid_pub = SimpleNamespace(publish=lambda msg: None)
    SemanticMissionManager.lock_target(mission, mission.pose)
    assert output[0].header.stamp.sec == 99


def test_associated_records_ignore_delayed_reordered_validity():
    warnings = []
    mission = SimpleNamespace(
        target='chair', last_observation_sequence=-1, pose=None,
        pose_valid=False, associated_observation=False,
        cache_valid_localization=lambda: None,
        stop_if_localized=lambda: None,
        state=SimpleNamespace(name='IDLE'),
        get_logger=lambda: SimpleNamespace(warning=warnings.append))
    valid = {
        'sequence': 10, 'valid': True, 'target': 'chair',
        'rgb_timestamp_ns': 99_900_000_000,
        'depth_timestamp_ns': 100_000_000_000,
        'pose': {
            'frame_id': 'map', 'timestamp_sec': 100,
            'timestamp_nanosec': 0, 'position': [1.0, 2.0, 0.5],
            'orientation': [0.0, 0.0, 0.0, 1.0],
        },
    }
    SemanticMissionManager.on_target_observation(
        mission, String(data=json.dumps(valid)))
    assert mission.pose_valid and mission.pose.pose.position.x == 1.0

    invalid = {
        'sequence': 11, 'valid': False, 'target': 'chair',
        'rgb_timestamp_ns': 0, 'depth_timestamp_ns': 0,
    }
    SemanticMissionManager.on_target_observation(
        mission, String(data=json.dumps(invalid)))
    assert not mission.pose_valid and mission.pose is None

    # A delayed older "true" cannot become associated with another pose.
    SemanticMissionManager.on_target_observation(
        mission, String(data=json.dumps(valid)))
    assert not mission.pose_valid and mission.pose is None


def test_approach_ignores_delayed_cross_topic_authorization():
    warnings = []
    approach = SimpleNamespace(
        last_target_observation_sequence=-1, target=None,
        target_valid=False, target_received_at=0.0,
        get_logger=lambda: SimpleNamespace(warning=warnings.append))
    valid = {
        'sequence': 20, 'valid': True, 'target': 'chair',
        'rgb_timestamp_ns': 99_900_000_000,
        'depth_timestamp_ns': 100_000_000_000,
        'pose': {
            'frame_id': 'map', 'timestamp_sec': 100,
            'timestamp_nanosec': 0, 'position': [1.0, 2.0, 0.5],
            'orientation': [0.0, 0.0, 0.0, 1.0],
        },
    }
    ApproachGoalNode.on_target_observation(
        approach, String(data=json.dumps(valid)))
    assert approach.target_valid and approach.target is not None
    invalid = {
        'sequence': 21, 'valid': False, 'target': 'chair',
        'rgb_timestamp_ns': 0, 'depth_timestamp_ns': 0,
    }
    ApproachGoalNode.on_target_observation(
        approach, String(data=json.dumps(invalid)))
    ApproachGoalNode.on_target_observation(
        approach, String(data=json.dumps(valid)))
    assert not approach.target_valid and approach.target is None
