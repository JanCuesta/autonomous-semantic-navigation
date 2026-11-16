"""Pure occupancy-grid helpers for preserving RGB-D coverage across SLAM updates."""

import numpy as np


def coverage_for_map(occupancy):
    """Create an eligible confidence grid for an occupancy map."""
    return np.where(occupancy == 0, 0, -1).astype(np.int8)


def reproject_coverage(previous_coverage, previous_spec, occupancy, spec):
    """Transfer measured confidence to a resized or shifted map.

    Only positive, already measured cells are transferred.  Newly free cells
    start unobserved and cells that are no longer free remain ineligible.
    """
    result = coverage_for_map(occupancy)
    rows, cols = np.nonzero(previous_coverage > 0)
    for row, col in zip(rows, cols):
        x, y = previous_spec.cell_to_world(int(row), int(col))
        new_row, new_col = spec.world_to_cell(x, y)
        if (0 <= new_row < result.shape[0] and
                0 <= new_col < result.shape[1] and
                result[new_row, new_col] >= 0):
            result[new_row, new_col] = max(
                result[new_row, new_col], previous_coverage[row, col])
    return result
