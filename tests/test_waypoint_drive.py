"""Each test names the real failure it prevents."""
import math
import pytest
from safety.waypoint_drive import (Limits, Twist3, ZERO, corridor_blocked, plan_step, to_goal, wrap)

L = Limits()


def test_far_and_misaimed_turns_in_place_without_driving():
    """Driving a large bearing error as an arc sweeps a wide curve through space the lidar
    never cleared. Point first, then go."""
    s = plan_step(3.0, math.radians(90), None, False)
    assert s.state == "turn_to_bearing"
    assert s.twist.vx == 0.0 and s.twist.wz > 0


def test_aimed_and_far_drives_forward():
    s = plan_step(3.0, math.radians(2), None, False)
    assert s.state == "drive"
    assert s.twist.vx == pytest.approx(L.v_max)


def test_never_emits_strafe():
    """The firmware drops angular.z the instant linear.y is non-zero, so a strafe+yaw twist is
    silently truncated. Twist3 has no y component at all -- this pins that."""
    assert not hasattr(plan_step(3.0, 0.0, None, False).twist, "vy")


def test_slows_down_on_approach():
    near = plan_step(0.30, 0.0, None, False).twist.vx
    far = plan_step(3.00, 0.0, None, False).twist.vx
    assert near < far and near >= L.v_min


def test_does_not_creep_below_stall_speed():
    """A command under ~0.06 m/s makes the chassis buzz and not move, which reads as a hang."""
    assert plan_step(L.pos_tol_m + 0.001, 0.0, None, False).twist.vx >= L.v_min


def test_arrival_stops():
    assert plan_step(0.05, 0.0, None, False).twist.is_zero()
    assert plan_step(0.05, 0.0, None, False).state == "arrived"


def test_final_heading_is_optional():
    assert plan_step(0.05, 0.0, None, False).state == "arrived"
    s = plan_step(0.05, 0.0, math.radians(40), False)
    assert s.state == "final_heading" and s.twist.vx == 0.0


def test_blocked_stops_forward_motion():
    """The veto zeros anything that would ADVANCE the footprint, however far the goal is."""
    for dist in (5.0, 0.3):
        st = plan_step(dist, 0.0, None, True)       # aimed at the goal -> would drive
        assert st.twist.is_zero() and st.state == "blocked"


def test_blocked_permits_turning_in_place():
    """An in-place turn does not advance the footprint, and the goal may be BEHIND the robot.
    Sim, 2026-08-27: parked 0.17 m past the waypoint facing a closed door, the robot could
    never turn around because the veto keyed on the front rays it was in the act of leaving."""
    st = plan_step(5.0, math.pi * 0.9, None, True)  # goal behind -> must be allowed to turn
    assert st.state == "turn_to_bearing"
    assert st.twist.vx == 0.0 and st.twist.wz != 0.0
    # arrived + settling onto the final heading is a turn too -- also not vetoed
    st = plan_step(0.05, 0.0, 1.0, True)
    assert st.twist.vx == 0.0


def test_nan_fails_closed():
    """`not (x <= t)` is used instead of `x > t` so NaN takes the STOP branch. If someone
    'simplifies' that comparison, this test dies."""
    assert plan_step(float("nan"), 0.0, None, False).twist.is_zero()
    assert plan_step(3.0, float("nan"), None, False).twist.is_zero()
    assert plan_step(0.05, 0.0, float("nan"), False).twist.is_zero()


def test_infinite_distance_fails_closed():
    assert plan_step(float("inf"), 0.0, None, False).twist.is_zero()


def test_speeds_are_clamped():
    s = plan_step(50.0, math.radians(179), None, False)
    assert abs(s.twist.wz) <= L.w_max and abs(s.twist.vx) <= L.v_max


def test_wrap_takes_the_short_way_round():
    assert wrap(math.radians(350)) == pytest.approx(math.radians(-10))
    assert math.isnan(wrap(float("nan")))


def test_to_goal_bearing_is_relative_to_heading():
    """Goal due north while facing east is 90 deg to the LEFT, not an absolute compass bearing."""
    d, b = to_goal(0, 0, math.radians(90), 0, 5)
    assert d == pytest.approx(5.0) and b == pytest.approx(0.0)
    d, b = to_goal(0, 0, 0.0, 0, 5)
    assert b == pytest.approx(math.radians(90))


def _scan(points):
    """points: list of (angle_deg, range_m) -> a 360-beam scan at 1 deg."""
    r = [float("nan")]*360
    for deg, rng in points:
        r[int(deg) % 360] = rng
    return r, math.radians(0), math.radians(1)


def test_corridor_blocked_by_object_ahead():
    r, a0, ai = _scan([(0, 0.5), (1, 0.5), (359, 0.5)])
    assert corridor_blocked(r, a0, ai)


def test_corridor_ignores_walls_beside_us():
    """A wall 0.6 m to the side is not in the path. A cone-shaped check would veto on it."""
    r, a0, ai = _scan([(90, 0.6), (91, 0.6), (89, 0.6), (270, 0.6)])
    assert not corridor_blocked(r, a0, ai)


def test_corridor_ignores_things_beyond_look_ahead():
    r, a0, ai = _scan([(0, 5.0), (1, 5.0), (2, 5.0)])
    assert not corridor_blocked(r, a0, ai)


def test_single_stray_return_does_not_latch_us():
    """This lidar returns on ~30-40% of beams and emits isolated spurious points. One speck must
    not stop the robot forever."""
    r, a0, ai = _scan([(0, 0.5)])
    assert not corridor_blocked(r, a0, ai)


def test_nan_ranges_are_not_obstacles():
    r, a0, ai = _scan([])
    assert not corridor_blocked(r, a0, ai)


def test_turn_hysteresis_finishes_what_it_starts():
    """At 20 Hz the controller re-plans constantly. Without a wider exit band it flips between
    turn and drive around the threshold, and on a 4WS chassis each flip is a MODE CHANGE the
    firmware answers by re-steering all four wheels -- so the wheels spin and the body never
    moves. Observed 2026-08-26."""
    just_inside = L.turn_tol_rad - 0.01          # inside the ENTER band
    # fresh: would drive
    assert plan_step(3.0, just_inside, None, False).state == "drive"
    # mid-turn: keeps turning, because it is outside the tighter EXIT band
    assert plan_step(3.0, just_inside, None, False,
                     prev_state="turn_to_bearing").state == "turn_to_bearing"


def test_turn_releases_once_well_inside():
    tiny = L.turn_exit_tol_rad / 2
    assert plan_step(3.0, tiny, None, False, prev_state="turn_to_bearing").state == "drive"


def test_exit_band_is_tighter_than_entry_band():
    assert L.turn_exit_tol_rad < L.turn_tol_rad


def test_hysteresis_never_drives_through_the_veto():
    """Leaving a turn via the wide exit band must still not DRIVE while blocked."""
    st = plan_step(3.0, L.turn_exit_tol_rad - 0.01, None, True, prev_state="turn_to_bearing")
    assert st.twist.vx == 0.0 and st.state == "blocked"


def test_final_heading_has_hysteresis_too():
    just_inside = L.turn_tol_rad - 0.01
    assert plan_step(0.05, 0.0, just_inside, False).state == "arrived"
    assert plan_step(0.05, 0.0, just_inside, False,
                     prev_state="final_heading").state == "final_heading"


def test_arrival_has_hysteresis():
    """Once settling at the waypoint, small drift past pos_tol must NOT re-enter the approach.
    Without this the controller chattered turn/drive/settle ~20 cycles at the goal edge
    (sim, 2026-08-27). Fresh approaches still use the strict tolerance."""
    d = L.pos_tol_m * 1.3                     # just past strict tol
    assert plan_step(d, -0.4, 0.0, False, prev_state="final_heading").state in (
        "final_heading", "arrived")
    assert plan_step(d, -0.4, None, False, prev_state="arrived").state == "arrived"
    assert plan_step(d, -0.4, None, False, prev_state="").state == "turn_to_bearing"
    assert plan_step(L.pos_tol_m * 1.7, -0.4, None, False,
                     prev_state="final_heading").state == "turn_to_bearing"
