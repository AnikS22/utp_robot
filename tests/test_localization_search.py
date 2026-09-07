"""Reject aliases instead of injecting a confident but arbitrary global pose."""
import math
import numpy as np
import pytest
from bringup.localization_search import ScanMatcher, acceptance, distinct


def test_ambiguous_high_scores_are_rejected():
    assert not acceptance([(90, 0, 0, 0), (88, 8, 0, 0)], 100)[0]


def test_low_fit_is_rejected_even_without_competitor():
    assert not acceptance([(40, 0, 0, 0)], 100)[0]


def test_clear_match_passes():
    assert acceptance([(80, 0, 0, 0), (60, 8, 0, 0)], 100)[0]


def test_wrapped_yaw_is_one_hypothesis():
    assert not distinct((1, 0, 0, .01), (1, 0, 0, 2*math.pi-.01))
    assert distinct((1, 0, 0, 0), (1, 0, 0, math.pi))


def test_missing_scan_or_free_space_refuses():
    with pytest.raises(ValueError):
        ScanMatcher(np.zeros((5, 5)), .1, (0, 0), [], [])
    m = ScanMatcher(np.full((5, 5), 100), .1, (0, 0), np.ones(30), np.zeros(30))
    with pytest.raises(ValueError, match='no free'):
        m.search()


def test_outside_negative_boundary_does_not_hit_first_cell():
    m = ScanMatcher(np.full((5, 5), 100), 1, (0, 0), np.full(30, .1), np.zeros(30))
    assert m.fit(-.2, .5, 0) == 0


def test_identical_rooms_produce_competing_locations():
    grid = np.full((50, 110), -1)
    # Two copies separated by unknown space: no evidence distinguishes them.
    for x in (5, 65):
        grid[5:36, x:x+31] = 0
        grid[5, x:x+31] = grid[35, x:x+31] = 100
        grid[5:36, x] = grid[5:36, x+30] = 100
    angles = np.linspace(-math.pi, math.pi, 120, endpoint=False)
    ranges = 1.5 / np.maximum(abs(np.cos(angles)), abs(np.sin(angles)))
    matcher = ScanMatcher(grid, .1, (0, 0), ranges, angles)
    hypotheses = matcher.search()
    assert len(hypotheses) > 1
    assert not acceptance(hypotheses, len(ranges))[0]
