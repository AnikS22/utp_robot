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
