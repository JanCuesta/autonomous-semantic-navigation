"""Offline tests: Nav2 acceptance is never interpreted as execution success."""
from asn_semantical_navigation.auto_policy import parse_navigation_summary


def test_acceptance_is_not_success():
    assert parse_navigation_summary('NAV2_ACCEPTED: sending goal') == (False, None)


def test_only_nav2_success_with_pose_pass_permits_next_action():
    assert parse_navigation_summary(
        'NAV2_SUCCEEDED | actual coverage gain=0.0020m2 | '
        'goal error=0.02m/2deg; pose_check=PASS')[0]
    assert not parse_navigation_summary(
        'NAV2_SUCCEEDED | actual coverage gain=0.50m2 | '
        'pose_check=FAIL')[0]
    assert not parse_navigation_summary(
        'NAV2_CANCELED | actual coverage gain=0.50m2 | '
        'pose_check=PASS')[0]


def test_actual_gain_requires_real_measurement():
    assert parse_navigation_summary(
        'NAV2_SUCCEEDED | predicted gain=99m2 | '
        'pose_check=PASS') == (True, None)
