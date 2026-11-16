"""Pure static-map checks; no ROS graph or robot interaction."""
import math
import numpy as np

from asn_semantical_navigation.nbv_core import GridSpec, safe_footprint, safe_straight_segment
from asn_semantical_navigation.planning.static_geometry import static_segment_clear


def test_interior_obstacle_and_unknown_are_rejected():
    grid = np.zeros((50, 50), dtype=np.int8)
    spec = GridSpec(0.1, 0, 0)
    grid[21, 21] = 100
    assert not safe_footprint(grid, spec, 2.05, 2.05, 0.3)
    grid[21, 21] = -1
    assert not safe_footprint(grid, spec, 2.05, 2.05, 0.3)
    grid[21, 21] = 0
    assert safe_footprint(grid, spec, 2.05, 2.05, 0.3)


def test_continuous_sweep_and_endpoint_margin():
    grid = np.zeros((50, 50), dtype=np.int8)
    spec = GridSpec(0.1, 0, 0)
    grid[22, 25] = 100
    assert safe_straight_segment(grid, spec, (1, 2), (4, 2))
    assert not safe_straight_segment(grid, spec, (1, 2), (4, 2), 0.25)
    grid[22, 25] = 0
    grid[21, 40] = 100
    assert not safe_straight_segment(grid, spec, (1, 2), (4, 2), 0.2)


def test_rotated_map_and_threshold():
    grid = np.zeros((30, 30), dtype=np.int8)
    spec = GridSpec(0.1, 4, -2, math.pi / 2)
    start, end = spec.cell_to_world(15, 5), spec.cell_to_world(15, 25)
    grid[15, 14] = 30
    assert not static_segment_clear(grid, spec, start, end, 0.0)
    assert static_segment_clear(grid, spec, start, end, 0.0, occupied_threshold=50)
    grid[15, 14] = -1
    assert not static_segment_clear(grid, spec, start, end, 0.0, occupied_threshold=50)


def test_invalid_input_and_map_boundary_fail_closed():
    grid = np.zeros((10, 10), dtype=np.int8)
    spec = GridSpec(0.1, 0, 0)
    assert not safe_footprint(grid, spec, 0.05, 0.5, 0.1)
    assert not safe_footprint(grid, spec, float('nan'), 0.5, 0.1)
    assert not safe_footprint(grid, spec, 0.5, 0.5, -0.1)
