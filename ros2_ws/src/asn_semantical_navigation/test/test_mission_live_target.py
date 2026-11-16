"""Offline tests for mission target freshness and preserved capture time."""
from types import SimpleNamespace
from geometry_msgs.msg import PoseStamped
from asn_semantical_navigation.semantical_mission_manager_node import SemanticMissionManager


class Clock:
    def now(self):
        return SimpleNamespace(nanoseconds=100_000_000_000)


class FakeMission:
    mission_started = 10.0
    pose_at = 11.0
    pose_valid = True
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
