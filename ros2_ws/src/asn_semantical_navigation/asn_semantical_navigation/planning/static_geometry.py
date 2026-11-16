"""Conservative static-grid collision checks for a circular footprint.

The path is a continuous line segment. Each occupied grid cell is expanded
by the footprint radius in both grid axes and intersected with that segment.
The square expansion contains the true circular expansion, including cell
corners, so it can reject a geometrically clear path but cannot miss an
intersection with a blocked cell. This is not a dynamic-obstacle check.
"""
import math


def _segment_intersects_box(x0, y0, x1, y1, xmin, ymin, xmax, ymax):
    low, high = 0.0, 1.0
    for start, delta, lower, upper in ((x0, x1 - x0, xmin, xmax),
                                       (y0, y1 - y0, ymin, ymax)):
        if delta == 0.0:
            if start < lower or start > upper:
                return False
            continue
        a, b = (lower - start) / delta, (upper - start) / delta
        low, high = max(low, min(a, b)), min(high, max(a, b))
        if low > high:
            return False
    return True


def static_segment_clear(occupancy, spec, start, end, radius_m,
                         occupied_threshold=1):
    """Check an entire swept disk; unknown and outside-map space are blocked.

    Values at or above occupied_threshold are blocked; negative values are
    always unknown. The default treats every nonzero cell conservatively.
    """
    if (getattr(occupancy, 'ndim', None) != 2 or
            not math.isfinite(spec.resolution) or spec.resolution <= 0 or
            not math.isfinite(radius_m) or radius_m < 0 or
            not 1 <= occupied_threshold <= 100):
        return False
    values = (*start[:2], *end[:2], spec.origin_x, spec.origin_y,
              spec.origin_yaw)
    if not all(math.isfinite(v) for v in values):
        return False
    c, s = math.cos(spec.origin_yaw), math.sin(spec.origin_yaw)
    def local(point):
        dx, dy = point[0] - spec.origin_x, point[1] - spec.origin_y
        return (c * dx + s * dy, -s * dx + c * dy)
    x0, y0 = local(start)
    x1, y1 = local(end)
    res = spec.resolution
    height, width = occupancy.shape
    if (min(x0, x1) - radius_m <= 0 or min(y0, y1) - radius_m <= 0 or
            max(x0, x1) + radius_m >= width * res or
            max(y0, y1) + radius_m >= height * res):
        return False
    col0 = max(0, math.floor((min(x0, x1) - radius_m) / res) - 1)
    col1 = min(width - 1, math.floor((max(x0, x1) + radius_m) / res))
    row0 = max(0, math.floor((min(y0, y1) - radius_m) / res) - 1)
    row1 = min(height - 1, math.floor((max(y0, y1) + radius_m) / res))
    for row in range(row0, row1 + 1):
        for col in range(col0, col1 + 1):
            value = occupancy[row, col]
            if not math.isfinite(value) or value < 0 or value >= occupied_threshold:
                if _segment_intersects_box(
                        x0, y0, x1, y1, col * res - radius_m,
                        row * res - radius_m, (col + 1) * res + radius_m,
                        (row + 1) * res + radius_m):
                    return False
    return True
