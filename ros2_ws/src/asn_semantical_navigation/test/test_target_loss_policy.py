"""Offline action lifecycle checks; fake handles never connect to ROS."""
import time
from types import SimpleNamespace
import pytest

from asn_semantical_navigation import semantical_mission_manager_node as mapped
from asn_semantical_navigation_mapless import semantical_mission_manager_node as mapless


@pytest.mark.parametrize('module,frame', [(mapped, 'map'), (mapless, 'odom')])
def test_freshness_rejects_delayed_future_and_invalid_observations(module, frame):
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.orientation.w = 1.0
    pose.header.stamp.sec = 99
    m = SimpleNamespace(pose=pose, pose_at=11., mission_started=10., pose_valid=True,
                        age_ok=lambda *args: True, param=lambda name: 2.5,
                        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))
    get = module.SemanticMissionManager.localized_target_snapshot
    assert get(m) is pose
    pose.header.stamp.sec = 90
    assert get(m) is None
    pose.header.stamp.sec = 101
    assert get(m) is None
    pose.header.stamp.sec = 99
    m.pose_valid = False
    m.last_valid_pose = pose
    assert get(m) is None


@pytest.mark.parametrize('module', [mapped, mapless])
def test_lost_target_during_approach_requests_abort(module):
    calls = []
    m = SimpleNamespace(state=module.State.APPROACHING, since=time.monotonic(),
                        localized_target_snapshot=lambda: None,
                        abort=lambda state, reason: calls.append((state, reason)))
    module.SemanticMissionManager.tick(m)
    assert calls and calls[0][0] == module.State.FAILED


@pytest.mark.parametrize('module', [mapped, mapless])
def test_loss_while_goal_pending_cancels_after_acceptance(module):
    class Future:
        def __init__(self, value): self.value = value
        def result(self): return self.value
        def add_done_callback(self, callback): pass
    class Handle:
        accepted = True
        def __init__(self): self.cancelled = False
        def get_result_async(self): return Future(None)
        def cancel_goal_async(self):
            self.cancelled = True
            return Future(None)
    handle = Handle()
    class Mission:
        state = module.State.APPROACHING
        goal_pending = True
        goal_handle = None
        cancel_goal_on_accept = False
        cancel_sent = False
        recovery_pending = False
        recovery_handle = None
        localized_target_snapshot = lambda self: None
        on_approach_result = lambda self, future: None
        on_cancel_ack = lambda self, future: None
        request_approach_cancel = module.SemanticMissionManager.request_approach_cancel
        abort = module.SemanticMissionManager.abort
        def set_state(self, state, reason): self.state = state
    m = Mission()
    module.SemanticMissionManager.on_approach_accepted(m, Future(handle))
    assert m.state == module.State.CANCELLING
    assert handle.cancelled
