"""Relocalising the odom frame from a lidar scan must recover a known displacement -- or refuse."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.scan_anchor import (accumulate, apply_transform, compose, frame_transform,
                                match_scans, relocalize, scan_to_points)

N = 360
AMIN = -math.pi
AINC = 2 * math.pi / N


def room_scan(x, y, yaw, *, keep=0.14, seed=0):
    """A synthetic A1M8 scan of an L-shaped wall + a doorway gap, from pose (x,y,yaw).

    Sparse on purpose: only `keep` of beams return, as measured (13%).
    """
    rng = np.random.default_rng(seed)
    walls = [((-3.0, 2.0), (4.0, 2.0)),      # north wall
             ((4.0, 2.0), (4.0, -3.0)),      # east wall
             ((-3.0, 2.0), (-3.0, -3.0))]    # west wall
    ranges = []
    for i in range(N):
        a = yaw + AMIN + i * AINC
        dx, dy = math.cos(a), math.sin(a)
        best = float("inf")
        for (x0, y0), (x1, y1) in walls:
            ex, ey = x1 - x0, y1 - y0
            den = dx * ey - dy * ex
            if abs(den) < 1e-9:
                continue
            t = ((x0 - x) * ey - (y0 - y) * ex) / den
            u = ((x0 - x) * dy - (y0 - y) * dx) / den
            if t > 0 and 0.0 <= u <= 1.0:
                # a 1.2 m doorway in the north wall between x=0.4 and 1.6 returns nothing
                px = x + t * dx
                if (x0, y0) == (-3.0, 2.0) and (x1, y1) == (4.0, 2.0) and 0.4 < px < 1.6:
                    continue
                best = min(best, t)
        if math.isfinite(best) and rng.random() < keep:
            ranges.append(best + rng.normal(0, 0.01))
        else:
            ranges.append(float("nan"))
    return ranges


def test_recovers_a_known_displacement():
    """Moved 0.4 m, 0.2 m and 15 deg from the anchor: the match must say so, to a few cm/deg."""
    ref = accumulate([room_scan(0.0, 0.0, 0.0, seed=k) for k in range(100, 120)], AMIN, AINC)
    # the robot now stands at (0.4, 0.2, 15 deg) in the anchor's frame
    live = scan_to_points(room_scan(0.4, 0.2, math.radians(15), seed=2), AMIN, AINC)
    m = match_scans(live, ref)
    assert m.ok, m.why_not()
    assert abs(m.dx - 0.4) < 0.06 and abs(m.dy - 0.2) < 0.06
    assert abs(math.degrees(m.dyaw) - 15.0) < 1.5


def test_stationary_matches_at_zero():
    ref = accumulate([room_scan(0.0, 0.0, 0.0, seed=k) for k in range(100, 120)], AMIN, AINC)
    live = scan_to_points(room_scan(0.0, 0.0, 0.0, seed=3), AMIN, AINC)
    m = match_scans(live, ref)
    assert m.ok and abs(m.dx) < 0.04 and abs(m.dy) < 0.04 and abs(m.dyaw) < math.radians(1.0)


def test_too_few_returns_is_refused_not_guessed():
    ref = accumulate([room_scan(0.0, 0.0, 0.0, keep=0.04, seed=k) for k in range(100, 103)], AMIN, AINC)
    live = scan_to_points(room_scan(0.1, 0.0, 0.0, keep=0.04, seed=2), AMIN, AINC)
    m = match_scans(live, ref)
    assert not m.ok and "too few" in m.why_not()


def test_a_person_in_the_scan_does_not_drag_the_fit():
    """A cluster the reference never saw costs a bounded amount, not the whole answer."""
    ref = accumulate([room_scan(0.0, 0.0, 0.0, seed=k) for k in range(100, 120)], AMIN, AINC)
    live = scan_to_points(room_scan(0.3, 0.0, 0.0, seed=2), AMIN, AINC)
    person = np.array([[1.0 + 0.02 * i, 0.5 + 0.03 * (i % 3)] for i in range(12)])
    live = np.vstack([live, person])
    m = match_scans(live, ref)
    assert m.ok, m.why_not()
    assert abs(m.dx - 0.3) < 0.08 and abs(m.dy) < 0.08


def test_frame_transform_round_trips():
    """The transform built from one pose correspondence must map that pose exactly."""
    rec = (2.0, 1.0, 0.3)
    cur = (5.0, -2.0, -1.1)
    T = frame_transform(rec, cur)
    got = apply_transform(T, rec)
    assert all(abs(a - b) < 1e-9 for a, b in zip(got, cur))


def test_relocalize_moves_every_waypoint_consistently():
    """Drift of (0.4, 0.2, 15 deg): a waypoint recorded 3 m ahead must land 3 m ahead in the
    corrected frame too -- the whole map moves rigidly with the robot's measured error."""
    anchor_pose = (10.0, 5.0, 0.5)                      # odom pose when 'start' was recorded
    ref = accumulate([room_scan(0.0, 0.0, 0.0, seed=k) for k in range(100, 120)], AMIN, AINC)
    live = scan_to_points(room_scan(0.4, 0.2, math.radians(15), seed=2), AMIN, AINC)
    # odom now claims the robot is back on the anchor exactly -- it is not; it is 0.4/0.2/15deg off
    odom_now = anchor_pose
    m, T = relocalize(anchor_pose, live, ref, odom_now)
    assert T is not None, m.why_not()
    truth_now_in_rec = compose(anchor_pose, (0.4, 0.2, math.radians(15)))
    # a waypoint recorded 3 m ahead of the anchor, in the recording frame
    wp_rec = compose(anchor_pose, (3.0, 0.0, 0.0))
    wp_cur = apply_transform(T, wp_rec)
    # in the current frame it must sit where a 3 m-ahead point would, relative to the corrected
    # robot pose -- i.e. the rigid map must carry robot and waypoint together
    expect = apply_transform(frame_transform(truth_now_in_rec, odom_now), wp_rec)
    assert abs(wp_cur[0] - expect[0]) < 0.08 and abs(wp_cur[1] - expect[1]) < 0.08
