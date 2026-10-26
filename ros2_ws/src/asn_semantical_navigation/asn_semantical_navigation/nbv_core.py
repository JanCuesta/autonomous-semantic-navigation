"""Pure-Python geometry and greedy next-best-view scoring (no ROS dependency).

Scores a single hypothetical camera observation at each candidate pose; never
modifies the measured coverage map. Uses conservative 2D line-of-sight only.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GridSpec:
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def world_to_cell(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (math.floor((-s * dx + c * dy) / self.resolution),
                math.floor((c * dx + s * dy) / self.resolution))

    def cell_to_world(self, row, col):
        gx, gy = (col + 0.5) * self.resolution, (row + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return self.origin_x + c * gx - s * gy, self.origin_y + s * gx + c * gy


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def candidate_poses(pose, step_m=0.6):
    """Eight map-aligned translations and four in-place turns."""
    x, y, yaw = pose
    candidates = []

    for dx in (-step_m, 0.0, step_m):
        for dy in (-step_m, 0.0, step_m):
            if dx != 0.0 or dy != 0.0:
                goal_yaw = math.atan2(dy, dx)
                candidates.append((
                    x + dx,
                    y + dy,
                    goal_yaw,
                    f'move({dx:+.1f},{dy:+.1f})'
                ))
    for degrees in (-90, -45, 45, 90):
        candidates.append((
            x,
            y,
            wrap_angle(yaw + math.radians(degrees)),
            f'turn({degrees:+d})'
        ))
    return candidates


def _is_free(occupancy, spec, x, y):
    row, col = spec.world_to_cell(x, y)
    return (0 <= row < occupancy.shape[0] and
            0 <= col < occupancy.shape[1] and occupancy[row, col] == 0)


def safe_footprint(occupancy, spec, x, y, clearance_m=0.22):
    """Check circular robot footprint against the static occupancy map."""
    if not _is_free(occupancy, spec, x, y):
        return False
    for i in range(16):
        a = 2 * math.pi * i / 16
        if not _is_free(occupancy, spec,
                        x + clearance_m * math.cos(a),
                        y + clearance_m * math.sin(a)):
            return False
    return True


def predict_observation(occupancy, spec, pose, fov_deg=70.0,
                        max_range_m=3.0, ray_count=71,
                        decay_length_m=1.5):
    """Return {flat grid index: best predicted confidence [0,100]}.

    Planar camera model with static-map occlusion; actual RGB-D coverage is
    supplied independently by coverage_node and is never replaced by this.
    """
    x, y, yaw = pose[:3]
    height, width = occupancy.shape
    seen = {}
    for offset in np.linspace(-fov_deg / 2, fov_deg / 2, ray_count):
        a = yaw + math.radians(float(offset))
        ux, uy = math.cos(a), math.sin(a)
        last_cell = None
        for dist in np.arange(0, max_range_m + spec.resolution / 2,
                              spec.resolution / 2):
            r, c = spec.world_to_cell(x + float(dist) * ux,
                                      y + float(dist) * uy)
            if (r, c) == last_cell:
                continue
            last_cell = r, c
            if not (0 <= r < height and 0 <= c < width):
                break
            if occupancy[r, c] != 0:
                break
            cx, cy = spec.cell_to_world(r, c)
            distance = math.hypot(cx - x, cy - y)
            if distance > max_range_m:
                continue
            confidence = max(1, min(100, round(100 * math.exp(
                -distance / decay_length_m))))
            key = r * width + c
            seen[key] = max(seen.get(key, 0), confidence)
    return seen


def coverage_gain(coverage, visibility, resolution):
    """Compare a hypothetical final grid against measured coverage."""
    flat = coverage.ravel()
    return sum(max(0.0, (confidence - float(flat[index])) / 100.0)
               for index, confidence in visibility.items()
               if flat[index] >= 0) * resolution ** 2


def predicted_coverage(coverage, visibility):
    result = coverage.copy()
    flat = result.ravel()
    for index, confidence in visibility.items():
        if flat[index] >= 0:
            flat[index] = max(int(flat[index]), confidence)
    return result


def estimated_time(start, target, linear_speed=0.20, angular_speed=1.0):
    """Approximate differential-drive action time, allowing reverse driving."""
    dx, dy = target[0] - start[0], target[1] - start[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return abs(wrap_angle(target[2] - start[2])) / angular_speed
    travel_heading = math.atan2(dy, dx)
    options = (travel_heading,)
    turns = [abs(wrap_angle(h - start[2])) +
             abs(wrap_angle(target[2] - h)) for h in options]
    return dist / linear_speed + min(turns) / angular_speed


def evaluate_candidates(occupancy, coverage, spec, current_pose,
                        step_m=0.6, clearance_m=0.22, fov_deg=70.0,
                        max_range_m=3.0, ray_count=71, decay_length_m=1.5,
                        linear_speed=0.2, angular_speed=1.0,
                        time_weight=0.005):
    """List one-step candidates by score; no motor command is emitted."""
    if occupancy.shape != coverage.shape:
        raise ValueError('Occupancy and confidence grids must have same shape')
    results = []
    for x, y, yaw, name in candidate_poses(current_pose, step_m):
        pose = (x, y, yaw)
        if not safe_footprint(occupancy, spec, x, y, clearance_m):
            continue
        
        vis = predict_observation(occupancy, spec, pose, fov_deg,
                                  max_range_m, ray_count, decay_length_m)
        gain = coverage_gain(coverage, vis, spec.resolution)
        seconds = estimated_time(current_pose, pose,
                                 linear_speed, angular_speed)
        results.append({'pose': pose, 'action': name, 'safe': True,
                        'gain': gain, 'seconds': seconds,
                        'score': gain - time_weight * seconds,
                        'visibility': vis})
    return sorted(results, key=lambda r: r['score'], reverse=True)
