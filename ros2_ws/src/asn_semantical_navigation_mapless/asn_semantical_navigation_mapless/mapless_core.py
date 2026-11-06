"""Mapless, local geometry and next-best-view heuristics; no ROS or grids.

All locations live in the drifting odom frame for one active mission only.
No occupancy map, SLAM, or persistent place graph is constructed.
"""
from dataclasses import dataclass
import math


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class View:
    x: float
    y: float
    yaw: float
    kind: str
    score: float
    clearance: float


def valid_scan_values(ranges, range_min, range_max):
    """Use +inf as clear to range_max; ignore NaN and invalid returns."""
    for i, raw in enumerate(ranges):
        if math.isinf(raw) and raw > 0:
            yield i, float(range_max)
        elif math.isfinite(raw) and range_min <= raw <= range_max:
            yield i, float(raw)


def sector_clearance(ranges, angle_min, angle_increment, range_min,
                     range_max, center, half_width=0.22):
    """Minimum valid scan range in a direction; unknown sectors return 0."""
    distances = [distance for index, distance in valid_scan_values(
        ranges, range_min, range_max)
        if abs(wrap(angle_min + index * angle_increment - center))
        <= half_width]
    return min(distances) if len(distances) >= 2 else 0.0


def relative_scan_points(ranges, angle_min, angle_increment, range_min,
                         range_max, laser_x=0.0, laser_y=0.0,
                         laser_yaw=0.0):
    """Finite sensor returns expressed in base_link coordinates."""
    for i, raw in enumerate(ranges):
        if not math.isfinite(raw) or raw < range_min or raw > range_max:
            continue
        angle = angle_min + i * angle_increment + laser_yaw
        yield (laser_x + raw * math.cos(angle),
               laser_y + raw * math.sin(angle))


def segment_clear(points, goal_x, goal_y, radius=0.29):
    """Check a straight local segment for points within robot clearance.

    Only observed points are considered; the caller must also verify fresh
    sensor data and adequate directional range before motion.
    """
    length_sq = goal_x * goal_x + goal_y * goal_y
    if length_sq <= 1e-8:
        return all(math.hypot(x, y) >= radius for x, y in points)
    for x, y in points:
        projection = max(0.0, min(1.0, (x * goal_x + y * goal_y) / length_sq))
        if math.hypot(x - projection * goal_x,
                      y - projection * goal_y) < radius:
            return False
    return True


def novel_view(x, y, yaw, visited, position_scale=0.8):
    """Return 0..1 novelty from short-term pose-heading memory."""
    if not visited:
        return 1.0
    best_similarity = 0.0
    for vx, vy, v_yaw in visited:
        distance = math.hypot(x - vx, y - vy)
        heading_diff = abs(wrap(yaw - v_yaw))
        similarity = math.exp(-distance / position_scale) * max(
            0.0, 1.0 - heading_diff / math.pi)
        best_similarity = max(best_similarity, similarity)
    return max(0.0, 1.0 - best_similarity)


def choose_view(ranges, angle_min, angle_increment, range_min, range_max,
                robot_pose, visited, step_m=0.55, clearance_m=0.34,
                sensor_yaw=0.0):
    """Rank short local actions by sensor clearance, novelty, and turn cost.

    No raycasting into an occupancy grid. No claim of global coverage.
    """
    rx, ry, robot_yaw = robot_pose
    candidates = []
    for heading_offset in (0.0, -math.pi / 4, math.pi / 4,
                           -math.pi / 2, math.pi / 2, math.pi):
        clear = sector_clearance(
            ranges, angle_min, angle_increment, range_min, range_max,
            heading_offset - sensor_yaw)
        distance = min(step_m, clear - clearance_m)
        if distance < 0.20:
            continue
        yaw = wrap(robot_yaw + heading_offset)
        x, y = rx + distance * math.cos(yaw), ry + distance * math.sin(yaw)
        novelty = novel_view(x, y, yaw, visited)
        score = 2.0 * novelty + 0.25 * min(clear, 3.0) + distance - (
            0.45 * abs(heading_offset) / math.pi)
        candidates.append(View(x, y, yaw, 'move', score, clear))
    # A rotation is a valid mapless observation action; essential at dead ends.
    for delta in (-math.pi / 2, math.pi / 2, math.pi):
        yaw = wrap(robot_yaw + delta)
        novelty = novel_view(rx, ry, yaw, visited)
        candidates.append(View(rx, ry, yaw, 'turn',
                               1.25 * novelty - 0.28, float('inf')))
    return max(candidates, key=lambda c: c.score) if candidates else None


def approach_candidates(robot_x, robot_y, target_x, target_y,
                        stand_off=0.60):
    """Preserve the radial-standoff approach logic, in odom coordinates."""
    bearing = math.atan2(robot_y - target_y, robot_x - target_x)
    for offset in (0.0, -math.pi / 6, math.pi / 6,
                   -math.pi / 3, math.pi / 3):
        theta = bearing + offset
        x = target_x + stand_off * math.cos(theta)
        y = target_y + stand_off * math.sin(theta)
        yaw = math.atan2(target_y - y, target_x - x)
        yield x, y, yaw
