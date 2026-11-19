"""Conservative correspondence checks for class-labelled target observations.

This cannot prove object identity: PoseStamped has no object tracking ID.
"""
import math


def corresponds(locked_label, current_label, locked_frame, current_frame,
                locked_xyz, current_xyz, locked_stamp, current_stamp,
                tolerance_m):
    """Require same nonempty class/frame, nondecreasing capture time, and 3D proximity."""
    if not locked_label or locked_label != current_label or not locked_frame or locked_frame != current_frame:
        return False
    values = (*locked_xyz, *current_xyz, locked_stamp, current_stamp, tolerance_m)
    if not all(math.isfinite(v) for v in values):
        return False
    if tolerance_m <= 0 or locked_stamp <= 0 or current_stamp < locked_stamp:
        return False
    return math.dist(locked_xyz, current_xyz) <= tolerance_m
