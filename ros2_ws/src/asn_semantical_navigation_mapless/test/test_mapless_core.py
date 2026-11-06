import math
from asn_semantical_navigation_mapless.mapless_core import (approach_candidates, choose_view, novel_view,
                          sector_clearance, segment_clear)


def test_unknown_sector_is_not_safe():
    assert sector_clearance([1.0, 2.0, 1.0], -0.2, 0.2, 0.1, 8.0,
                            math.pi) == 0.0


def test_no_map_required_for_view():
    ranges = [3.0] * 360
    view = choose_view(ranges, -math.pi, 2 * math.pi / 360, 0.1, 8.0,
                       (0.0, 0.0, 0.0), [])
    assert view is not None
    assert view.kind in ('move', 'turn')


def test_novelty_decreases_on_repeat():
    assert novel_view(1, 1, 0, []) == 1.0
    assert novel_view(1, 1, 0, [(1, 1, 0)]) < 0.001
    assert novel_view(2, 2, 0, [(1, 1, 0)]) > 0.5


def test_obstacle_on_segment_blocks():
    assert not segment_clear([(0.5, 0.0)], 1.0, 0.0)
    assert segment_clear([(0.5, 0.65)], 1.0, 0.0)


def test_standoff_geometry():
    x, y, yaw = next(approach_candidates(0, 0, 2, 0, 0.6))
    assert abs(math.hypot(2 - x, 0 - y) - 0.6) < 1e-6
    assert abs(yaw) < 1e-6


def test_blocked_space_leaves_only_a_turn():
    # Obstacles too close for any translation; turning may inspect another view.
    view = choose_view([0.40] * 360, -math.pi, 2 * math.pi / 360,
                       0.1, 8.0, (0, 0, 0), [])
    assert view is not None
    assert view.kind == 'turn'


def test_invalid_laser_returns_are_not_free_space():
    assert sector_clearance([float('nan')] * 360, -math.pi,
                            2 * math.pi/360, 0.1, 8.0, 0) == 0.0
