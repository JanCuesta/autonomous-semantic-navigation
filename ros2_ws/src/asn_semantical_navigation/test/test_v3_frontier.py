"""Offline tests of low-confidence frontier and generated safe global views."""
import numpy as np

from asn_semantical_navigation.nbv_core import GridSpec
from asn_semantical_navigation.nbv_frontier import (
    confidence_frontier, global_frontier_view, reachable_free)


def test_frontier_is_confidence_not_occupancy_boundary():
    occ = np.zeros((12, 12), dtype=np.int16)
    cf = np.ones_like(occ) * 90
    cf[:, 6:] = 10
    frontier = confidence_frontier(occ, cf, 30)
    assert frontier[:, 6].all()
    assert not frontier[:, 7].any()


def test_reachability_stops_at_wall():
    occ = np.zeros((20, 20), dtype=np.int16)
    occ[:, 10] = 100
    spec = GridSpec(0.1, 0.0, 0.0)
    reachable = reachable_free(occ, spec, (0.3, 0.3, 0.0))
    assert reachable[3, 3]
    assert not reachable[3, 16]


def test_distant_view_is_generated_not_fixed():
    occ = np.zeros((80, 80), dtype=np.int16)
    cf = np.zeros_like(occ)
    cf[:30, :30] = 90
    spec = GridSpec(0.1, 0.0, 0.0)
    result = global_frontier_view(
        occ, cf, spec, (0.8, 0.8, 0.0), spacing_m=0.6,
        max_positions=18, ray_count=11, time_weight=0.0001,
        max_range_m=2.5, minimum_gain_m2=0.0001)
    assert result is not None
    assert result['safe']
    assert result['gain'] > 0
    assert result['kind'] == 'frontier'
