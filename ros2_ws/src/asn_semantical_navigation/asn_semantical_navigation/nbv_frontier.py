"""Simple global confidence-frontier fallback for a locally exhausted NBV.

No predefined waypoints. This is a deliberately limited global proposal method:
sample reachable free cells and choose a camera viewpoint that increases
coverage of the < threshold confidence region, not an object belief map.
Nav2 is responsible for dynamic obstacles and actual path execution.
"""

from collections import deque
import math

import numpy as np

from .nbv_core import (coverage_gain, estimated_time, predict_observation,
                       safe_footprint, wrap_angle)


def reachable_free(occupancy, spec, start_pose):
    """Conservative 4-connected known-free flood fill (no diagonal shortcuts)."""
    h, w = occupancy.shape
    sr, sc = spec.world_to_cell(*start_pose[:2])
    reach = np.zeros((h, w), dtype=bool)
    if not (0 <= sr < h and 0 <= sc < w and occupancy[sr, sc] == 0):
        return reach
    queue = deque([(sr, sc)])
    reach[sr, sc] = True
    while queue:
        r, c = queue.popleft()
        for nr, nc in ((r + 1, c), (r - 1, c),
                       (r, c + 1), (r, c - 1)):
            if (0 <= nr < h and 0 <= nc < w and not reach[nr, nc]
                    and occupancy[nr, nc] == 0):
                reach[nr, nc] = True
                queue.append((nr, nc))
    return reach


def confidence_frontier(occupancy, coverage, threshold=30):
    """Low-confidence cells adjacent to higher-confidence free cells."""
    free = occupancy == 0
    low = free & (coverage >= 0) & (coverage < threshold)
    high = free & (coverage >= threshold)
    near_high = np.zeros_like(free)
    near_high[1:, :] |= high[:-1, :]
    near_high[:-1, :] |= high[1:, :]
    near_high[:, 1:] |= high[:, :-1]
    near_high[:, :-1] |= high[:, 1:]
    return low & near_high


def global_frontier_view(occupancy, coverage, spec, robot_pose,
                         threshold=30, spacing_m=0.6,
                         clearance_m=0.22, max_range_m=3.0,
                         fov_deg=70.0, ray_count=31,
                         decay_length_m=1.5,
                         time_weight=0.005,
                         linear_speed=0.2, angular_speed=1.0,
                         max_positions=70, minimum_gain_m2=0.002):
    """Return one distant safe viewpoint (or None) for confidence recovery.

    Positions are generated from a reachable lattice on the live occupancy
    map. First consider near the confidence frontier, otherwise the remaining
    low-confidence region. No locations or room centers are hand-entered.
    """
    if occupancy.shape != coverage.shape:
        raise ValueError('Coverage and map shapes differ')
    if max_positions <= 0:
        raise ValueError('max_positions must be positive')
    reach = reachable_free(occupancy, spec, robot_pose)
    free_low = (occupancy == 0) & (coverage >= 0) & (coverage < threshold)
    targets = confidence_frontier(occupancy, coverage, threshold)
    if not np.any(targets):
        targets = free_low  # May be uniformly low confidence initially.
    targets &= reach
    if not np.any(targets):
        return None

    h, w = occupancy.shape
    stride = max(1, round(spacing_m / spec.resolution))
    target_coords = np.argwhere(targets)
    pool = []
    for r in range(stride // 2, h, stride):
        for c in range(stride // 2, w, stride):
            if not reach[r, c]:
                continue
            x, y = spec.cell_to_world(r, c)
            dist = math.hypot(x - robot_pose[0], y - robot_pose[1])
            if dist <= spacing_m * 1.1:
                continue  # Local planner already evaluated nearby moves.
            if not safe_footprint(occupancy, spec, x, y, clearance_m):
                continue
            # Lightweight pre-ranking: proximity to an inspection frontier
            # and low travel distance (actual occlusion scored below).
            delta = target_coords - np.array([r, c])
            square = np.sum(delta * delta, axis=1)
            proximity = math.sqrt(float(np.min(square))) * spec.resolution
            if proximity > max_range_m * 1.4:
                continue
            pre_score = -proximity - 0.12 * dist
            pool.append((pre_score, x, y, dist))
    pool.sort(reverse=True)
    pool = pool[:max_positions]
    if not pool:
        return None

    best = None
    for _, x, y, distance in pool:
        for deg in range(0, 360, 45):
            yaw = math.radians(deg)
            pose = (x, y, yaw)
            vis = predict_observation(occupancy, spec, pose, fov_deg,
                                      max_range_m, ray_count,
                                      decay_length_m)
            # Reward improvements specifically in low confidence cells.
            targets_visible = {idx: q for idx, q in vis.items()
                               if free_low.ravel()[idx]}
            gain = coverage_gain(coverage, targets_visible, spec.resolution)
            if gain < minimum_gain_m2:
                continue
            # Euclidean lower bound plus required heading changes. Nav2
            # must still check actual path reachability/cost.
            seconds = estimated_time(robot_pose, pose, linear_speed,
                                     angular_speed)
            score = gain - time_weight * seconds
            if score <= 0:
                continue
            if best is None or score > best['score']:
                best = {
                    'pose': pose,
                    'action': 'frontier(+{:.1f}m,{:.0f}deg)'.format(
                        distance, math.degrees(wrap_angle(yaw))),
                    'safe': True,
                    'gain': gain,
                    'seconds': seconds,
                    'score': score,
                    'visibility': vis,
                    'kind': 'frontier',
                }
    return best
