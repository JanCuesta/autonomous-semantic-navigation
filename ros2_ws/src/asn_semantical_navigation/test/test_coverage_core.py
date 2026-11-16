"""Unit tests for coverage synchronization with an expanding SLAM map."""

import math

import numpy as np

from asn_semantical_navigation.coverage_core import (
    coverage_for_map,
    reproject_coverage,
)
from asn_semantical_navigation.nbv_core import GridSpec


def test_coverage_marks_only_free_cells_eligible():
    occupancy = np.array([[0, -1], [100, 0]], dtype=np.int8)
    assert np.array_equal(
        coverage_for_map(occupancy),
        np.array([[0, -1], [-1, 0]], dtype=np.int8))


def test_reprojection_preserves_measurement_when_map_expands():
    old_map = np.zeros((3, 3), dtype=np.int8)
    old_spec = GridSpec(1.0, 0.0, 0.0)
    old_coverage = coverage_for_map(old_map)
    old_coverage[1, 1] = 80
    new_map = np.zeros((5, 5), dtype=np.int8)
    new_spec = GridSpec(1.0, -1.0, -1.0)

    actual = reproject_coverage(old_coverage, old_spec, new_map, new_spec)

    assert actual[2, 2] == 80
    assert actual[0, 0] == 0


def test_reprojection_respects_rotated_origin_and_new_obstacles():
    old_map = np.zeros((3, 3), dtype=np.int8)
    old_spec = GridSpec(1.0, 0.0, 0.0, math.pi / 2)
    old_coverage = coverage_for_map(old_map)
    old_coverage[1, 1] = 70
    new_map = np.zeros((3, 3), dtype=np.int8)
    new_map[1, 1] = 100

    actual = reproject_coverage(old_coverage, old_spec, new_map, old_spec)

    assert actual[1, 1] == -1
