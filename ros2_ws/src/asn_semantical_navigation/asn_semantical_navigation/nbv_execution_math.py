"""Pure helpers for one-action NBV validation (no ROS dependencies)."""

import math

import numpy as np

from .nbv_core import wrap_angle


def best_executable_candidate(ranked, minimum_gain_m2):
    """Return a useful best candidate, or None. No automatic fallback."""
    if not ranked:
        return None
    best = ranked[0]
    if (not best['safe'] or best['score'] <= 0 or
            best['gain'] < minimum_gain_m2):
        return None
    return best


def measured_coverage_gain(previous, current, resolution):
    """Area-weighted improvement from ACTUAL map snapshots, in m^2."""
    before = np.asarray(previous, dtype=np.int16)
    after = np.asarray(current, dtype=np.int16)
    if before.shape != after.shape:
        raise ValueError('Measured coverage snapshots have different shapes')
    valid = (before >= 0) & (after >= 0)
    increments = np.maximum(0, after - before)
    return float(increments[valid].sum()) / 100.0 * resolution ** 2


def goal_pose_error(actual, target):
    """Return planar position error in meters and absolute heading error deg."""
    x, y, yaw = actual
    gx, gy, gyaw = target
    return (math.hypot(x - gx, y - gy),
            abs(math.degrees(wrap_angle(yaw - gyaw))))
