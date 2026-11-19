"""ROS-independent policies for RGB-D motion tolerance and 3D confirmation."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Observation3D:
    """One independently measured target position."""

    target: str
    identity: str
    rgb_stamp_ns: int
    depth_stamp_ns: int
    xyz: tuple


class ObservationHistory:
    """Rolling spatial cluster of independent RGB-D observations."""

    def __init__(self, required=3, window_s=4.0, max_jitter_m=0.45,
                 outliers_to_reset=2):
        self.required = max(1, int(required))
        self.window_ns = max(0, round(float(window_s) * 1e9))
        self.max_jitter_m = float(max_jitter_m)
        self.outliers_to_reset = max(1, int(outliers_to_reset))
        self.observations = []
        self.consecutive_outliers = 0

    def clear(self):
        previous = len(self.observations)
        self.observations.clear()
        self.consecutive_outliers = 0
        return previous

    def expire(self, now_ns):
        expired = [
            item for item in self.observations
            if now_ns - item.depth_stamp_ns > self.window_ns
        ]
        if expired:
            expired_ids = {id(item) for item in expired}
            self.observations = [
                item for item in self.observations
                if id(item) not in expired_ids
            ]
        return expired

    def add(self, candidate):
        """Return ``(decision, details)`` after evaluating one candidate."""
        if (not candidate.target or candidate.rgb_stamp_ns <= 0 or
                candidate.depth_stamp_ns <= 0 or len(candidate.xyz) != 3 or
                not all(math.isfinite(value) for value in candidate.xyz)):
            return 'invalid', {}
        if any(item.rgb_stamp_ns == candidate.rgb_stamp_ns
               for item in self.observations):
            return 'duplicate_rgb', {}
        if any(item.depth_stamp_ns == candidate.depth_stamp_ns
               for item in self.observations):
            return 'duplicate_depth', {}
        if self.observations:
            expected_target = self.observations[0].target
            expected_identity = self.observations[0].identity
            if candidate.target != expected_target:
                return 'target_changed', {'old_target': expected_target}
            if (expected_identity and candidate.identity and
                    candidate.identity != expected_identity):
                return 'identity_changed', {'old_identity': expected_identity}
            distances = [
                math.dist(item.xyz, candidate.xyz)
                for item in self.observations
            ]
            disagreement = max(distances)
            if disagreement > self.max_jitter_m:
                self.consecutive_outliers += 1
                details = {
                    'distance_m': disagreement,
                    'consecutive': self.consecutive_outliers,
                    'previous_count': len(self.observations),
                }
                if self.consecutive_outliers < self.outliers_to_reset:
                    return 'outlier', details
                self.clear()
                self.observations.append(candidate)
                details['new_count'] = 1
                return 'cluster_reset', details
        self.consecutive_outliers = 0
        self.observations.append(candidate)
        return 'accepted', {'count': len(self.observations)}

    @property
    def confirmed(self):
        return len(self.observations) >= self.required

    def stable_xyz(self):
        """Return a coordinate-wise median without requiring NumPy."""
        if not self.observations:
            return None
        result = []
        for axis in range(3):
            values = sorted(item.xyz[axis] for item in self.observations)
            middle = len(values) // 2
            if len(values) % 2:
                result.append(values[middle])
            else:
                result.append((values[middle - 1] + values[middle]) / 2.0)
        return tuple(result)


def stationary_from_twist(linear_xyz, angular_xyz, linear_limit,
                          angular_limit):
    """Conservatively classify a finite 3D twist as stationary."""
    values = (*linear_xyz, *angular_xyz, linear_limit, angular_limit)
    if not all(math.isfinite(value) for value in values):
        return False
    linear_speed = math.sqrt(sum(value * value for value in linear_xyz))
    angular_speed = math.sqrt(sum(value * value for value in angular_xyz))
    return linear_speed <= linear_limit and angular_speed <= angular_limit
