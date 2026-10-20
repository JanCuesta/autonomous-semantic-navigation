"""Unit tests for geometry, confidence scoring, and candidate search."""
import math
import numpy as np

from asn_semantical_navigation.nbv_core import (
    GridSpec, candidate_poses, coverage_gain, estimated_time,
    evaluate_candidates, predict_observation, predicted_coverage,
    safe_straight_segment,
)


def test_candidates_3x3_and_rotations():
    poses = candidate_poses((1.0, 2.0, 0.0))
    assert len(poses) == 12
    assert len(set((round(p[0], 5), round(p[1], 5), round(p[2], 5))
                   for p in poses)) == 12
    assert any(abs(p[0] - 1.6) < 1e-6 and abs(p[1] - 2.6) < 1e-6
               for p in poses)
    assert sum(p[3].startswith('turn') for p in poses) == 4


def test_grid_rotation_round_trip():
    spec = GridSpec(0.05, -1.0, -2.0, math.pi / 2)
    for cell in ((0, 0), (17, 25), (99, 2)):
        assert spec.world_to_cell(*spec.cell_to_world(*cell)) == cell


def test_wall_blocks_camera_and_motion():
    grid = np.zeros((80, 80), dtype=np.int8)
    grid[:, 40] = 100
    spec = GridSpec(0.05, 0.0, 0.0)
    vis = predict_observation(grid, spec, (1.0, 2.0, 0), fov_deg=20)
    assert vis
    assert all(key % 80 < 40 for key in vis)
    assert not safe_straight_segment(grid, spec,
                                     (1.0, 2.0, 0), (3.0, 2.0, 0))


def test_gain_is_difference_of_maps_no_double_counting():
    coverage = np.zeros((2, 3), dtype=np.int8)
    coverage[0, 0] = 70
    visible = {0: 50, 1: 80, 2: 80}
    new = predicted_coverage(coverage, visible)
    assert new[0, 0] == 70
    assert new[0, 1] == 80
    expected = (0.8 + 0.8) * 0.05 ** 2
    assert math.isclose(coverage_gain(coverage, visible, 0.05), expected)


def test_turn_cost_and_no_good_action():
    assert estimated_time((0, 0, 0), (0, 0, math.pi / 2)) > 0
    grid = np.zeros((100, 100), dtype=np.int8)
    coverage = np.full((100, 100), 100, dtype=np.int8)
    result = evaluate_candidates(grid, coverage, GridSpec(0.05, -2.5, -2.5),
                                 (0, 0, 0))
    assert len(result) == 12
    assert result[0]['score'] <= 0
    assert all(r['gain'] == 0 for r in result if r['safe'])
