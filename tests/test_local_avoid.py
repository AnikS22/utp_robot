"""Reactive avoidance: steer around what the scan sees, or refuse."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.local_avoid import choose_heading

N = 360
AMIN = -math.pi
AINC = 2 * math.pi / N


def scan(obstacles=(), default=float("inf")):
    """obstacles: (centre_deg, half_width_deg, range_m)."""
    r = [default] * N
    for (c, hw, rng) in obstacles:
        for i in range(N):
            a = math.degrees(AMIN + i * AINC)
            d = (a - c + 180) % 360 - 180
            if abs(d) <= hw:
                r[i] = rng
    return r


def deg(x):
    return None if x is None else math.degrees(x)


def test_clear_path_goes_straight_at_the_goal():
    c = choose_heading(scan(), AMIN, AINC, 0.0)
    assert c.bearing_rad is not None
    assert abs(deg(c.bearing_rad)) < 3.0
    assert "clear" in c.reason


def test_obstacle_dead_ahead_steers_around_it():
    """A box straight ahead: keep going, but not through it."""
    c = choose_heading(scan([(0, 8, 1.0)]), AMIN, AINC, 0.0)
    assert c.bearing_rad is not None, "should find a way round, not give up"
    assert abs(deg(c.bearing_rad)) > 15.0, "must not still be aiming at the obstacle"
    assert "steering" in c.reason


def test_it_picks_the_side_nearer_the_goal():
    """Obstacle ahead, goal off to the left -> go left, not right."""
    c = choose_heading(scan([(0, 8, 1.0)]), AMIN, AINC, math.radians(30))
    assert c.bearing_rad is not None and deg(c.bearing_rad) > 0


def test_a_wall_across_the_whole_arc_refuses():
    """Refusing is a real answer. Better a stop than a confident drive into a wall."""
    c = choose_heading(scan([(0, 120, 0.8)]), AMIN, AINC, 0.0)
    assert c.bearing_rad is None


def test_close_obstacle_blocks_every_direction():
    """Inside the robot's own half-width there is no way past at any angle."""
    c = choose_heading(scan([(0, 5, 0.25)]), AMIN, AINC, 0.0)
    assert c.bearing_rad is None


def test_returns_behind_the_robot_are_ignored():
    """The rear sector is the robot seeing ITSELF (measured 0.17 m astern). It must not veto."""
    c = choose_heading(scan([(180, 30, 0.17)]), AMIN, AINC, 0.0)
    assert c.bearing_rad is not None and abs(deg(c.bearing_rad)) < 3.0


def test_far_obstacles_do_not_constrain_the_next_move():
    c = choose_heading(scan([(0, 8, 5.0)]), AMIN, AINC, 0.0)
    assert c.bearing_rad is not None and abs(deg(c.bearing_rad)) < 3.0


def test_unseen_beams_are_free_not_walls():
    """This lidar returns on a minority of beams; NaN everywhere must not read as enclosed."""
    c = choose_heading([float("nan")] * N, AMIN, AINC, 0.0)
    assert c.bearing_rad is not None


def test_hysteresis_keeps_the_side_it_committed_to():
    """Two near-equal gaps must not be re-litigated at 20 Hz: each flip re-steers four wheels."""
    ranges = scan([(0, 8, 1.0)])
    left = choose_heading(ranges, AMIN, AINC, 0.0, prev_bearing_rad=math.radians(40))
    right = choose_heading(ranges, AMIN, AINC, 0.0, prev_bearing_rad=math.radians(-40))
    assert deg(left.bearing_rad) > 0 and deg(right.bearing_rad) < 0


def test_gap_between_two_obstacles_is_used_when_it_fits():
    """A doorway. 60 deg of free arc at >=1 m is comfortably wider than the robot needs."""
    c = choose_heading(scan([(-60, 25, 1.5), (60, 25, 1.5)]), AMIN, AINC, 0.0)
    assert c.bearing_rad is not None and abs(deg(c.bearing_rad)) < 20.0
