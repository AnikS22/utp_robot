"""The arm must never be commanded past its envelope, and the base must know how close to get."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.reach_envelope import (ARM_REACH_M, check_before_reach, in_reach, lidar_stop_for,
                                   shortfall)


def test_the_real_2026_08_29_failure_is_refused():
    """1.23 m measured at press time, 0.88 m envelope -> ControllerError 21."""
    ok, why = check_before_reach(1.23)
    assert not ok
    assert "0.35 m short" in why and "21" in why


def test_a_target_inside_the_envelope_passes():
    ok, why = check_before_reach(0.60)
    assert ok and "in reach" in why


def test_the_boundary_is_inclusive():
    assert in_reach(ARM_REACH_M)
    assert not in_reach(ARM_REACH_M + 0.001)


def test_nonsense_ranges_fail_closed():
    assert not check_before_reach(0.0)[0]
    assert not check_before_reach(-1.0)[0]
    assert not check_before_reach(float("nan"))[0]


def test_shortfall_is_what_the_base_must_close():
    assert abs(shortfall(1.23) - 0.35) < 1e-9
    assert shortfall(0.5) == 0.0


def test_lidar_stop_accounts_for_the_sensor_being_forward_of_base_link():
    """The lidar is 0.318 m ahead, so it reads SHORTER than base_link is from the wall.

    Ignoring that is what left the base 1.21 m out while the veto believed 0.90.
    """
    assert abs(lidar_stop_for(0.55) - 0.232) < 1e-6


def test_lidar_stop_never_goes_below_the_hard_floor():
    """However tight the standoff, never command the base within 0.30 m of anything."""
    assert lidar_stop_for(0.10) == 0.20


def test_press_pose_ok_matches_the_sim_acceptance_band():
    """isaac_world._press_pose_ok: dist <= d_goal + 0.18 and yaw_err <= 0.25."""
    from safety.reach_envelope import press_pose_ok
    assert press_pose_ok(0.50, 0.0)
    # 0.50 + 0.18 is 0.6799999... in binary, so test just inside/outside rather than on it.
    assert press_pose_ok(0.679, 0.25)         # just inside the band
    assert not press_pose_ok(0.69, 0.0)       # too far
    assert not press_pose_ok(0.50, 0.26)      # too far off the press axis


def test_the_hardware_failure_pose_is_not_accepted():
    """1.23 m from the plate, which the arm then faulted trying to reach."""
    from safety.reach_envelope import press_pose_ok
    assert not press_pose_ok(1.23, 0.0)


def test_acceptance_band_stays_inside_the_arm_envelope():
    """The whole point: anything press_pose_ok accepts must be reachable."""
    from safety.reach_envelope import (ARM_REACH_M, PRESS_ACCEPT_SLACK_M, PRESS_STANDOFF_M,
                                       in_reach)
    assert in_reach(PRESS_STANDOFF_M + PRESS_ACCEPT_SLACK_M)
    assert PRESS_STANDOFF_M + PRESS_ACCEPT_SLACK_M < ARM_REACH_M


def test_stall_needs_a_full_window_of_no_progress():
    from safety.reach_envelope import stalled
    assert not stalled([(0.0, 1.0, 0.5), (1.0, 0.99, 0.49)], 1.0)     # window not elapsed
    assert stalled([(0.0, 1.0, 0.5), (5.0, 0.99, 0.49)], 5.0)          # 5 s, no improvement
    assert not stalled([(0.0, 1.0, 0.5), (5.0, 0.90, 0.49)], 5.0)      # radial improved 0.10


def test_stall_is_false_while_yaw_alone_is_still_improving():
    """Either error improving means it is still converging -- turning first is legitimate."""
    from safety.reach_envelope import stalled
    assert not stalled([(0.0, 1.0, 0.50), (5.0, 1.0, 0.40)], 5.0)
