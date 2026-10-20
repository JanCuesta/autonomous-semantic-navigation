"""Tests for safe NBV action selection and post-action observation checks."""

import math

import numpy as np
import pytest

from asn_semantical_navigation.nbv_execution_math import (
    best_executable_candidate, goal_pose_error, measured_coverage_gain,
)


def test_selection_requires_positive_utility_and_gain():
    good = {'safe': True, 'score': 0.1, 'gain': 0.3}
    assert best_executable_candidate([good], 0.002) is good
    assert best_executable_candidate([dict(good, score=-0.1)], 0.002) is None
    assert best_executable_candidate([dict(good, safe=False)], 0.002) is None
    assert best_executable_candidate([dict(good, gain=0.001)], 0.002) is None
    assert best_executable_candidate([], 0.002) is None


def test_actual_gain_ignores_invalid_and_declines():
    before = np.array([[0, 90, -1], [30, 50, 100]], dtype=np.int16)
    after = np.array([[80, 80, -1], [30, 75, 100]], dtype=np.int16)
    # +80 and +25 confidence points, no double counting of declining cells.
    assert math.isclose(measured_coverage_gain(before, after, 0.05),
                        1.05 * 0.05 ** 2)


def test_actual_gain_mismatched_arrays():
    with pytest.raises(ValueError):
        measured_coverage_gain(np.zeros((2, 2)), np.zeros((4,)), 0.05)


def test_yaw_wrap_in_pose_error():
    m, deg = goal_pose_error((0, 0, math.radians(179)),
                             (0.06, 0.08, math.radians(-179)))
    assert math.isclose(m, 0.1)
    assert math.isclose(deg, 2.0)
