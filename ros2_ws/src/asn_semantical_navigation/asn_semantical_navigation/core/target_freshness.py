"""Observation-time validation, independent of ROS message types."""
import math


def measurement_is_fresh(measurement_time, now, maximum_age):
    """Reject missing, future, non-finite or expired capture timestamps."""
    if not all(math.isfinite(value) for value in (measurement_time, now, maximum_age)):
        return False
    if measurement_time <= 0 or maximum_age < 0:
        return False
    age = now - measurement_time
    return 0 <= age <= maximum_age
