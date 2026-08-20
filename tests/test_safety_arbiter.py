"""Headless tests for the base-motion safety arbiter. No ROS, no Isaac.

    python3 -m pytest real_world/tests/test_safety_arbiter.py -q

Each test names the real-world failure it stands for. The interlock is the reason this file
exists, so it gets the most cases — including the ones where a publisher dies rather than
publishing something wrong, because that is the failure the old (emergent, control-flow-based)
safety could not survive.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "safety"))
from arbiter import Limits, SafetyArbiter, SourceSpec, Twist3   # noqa: E402


TELEOP = SourceSpec("teleop", "/cmd_vel_teleop", 100, requires_enable=False,
                    allows_arm_override=True)
SERVO = SourceSpec("servo", "/cmd_vel_servo", 50)
NAV = SourceSpec("nav", "/cmd_vel_nav", 10)

# Slew caps high enough that most tests see the commanded value in one step; the slew behaviour
# has its own dedicated tests below.
FAST = Limits(max_accel_lin=1e6, max_decel_lin=1e6, max_accel_ang=1e6, max_decel_ang=1e6)


def make(**kw) -> SafetyArbiter:
    kw.setdefault("sources", [TELEOP, SERVO, NAV])
    kw.setdefault("limits", FAST)
    return SafetyArbiter(**kw)


def armed(arb: SafetyArbiter, t: float, *, stowed=True, enable=True, override=False) -> None:
    """Assert the gates needed for ordinary autonomous motion."""
    arb.set_gate("arm_stowed", stowed, t)
    arb.set_gate("enable", enable, t)
    arb.set_gate("override", override, t)


# --------------------------------------------------------------------------------------------------
# The interlock — the reason this module exists
# --------------------------------------------------------------------------------------------------
def test_arm_extended_blocks_base_motion():
    """The core rule: an extended arm sits ~0.88 m outside the Nav2 footprint, so the base must
    not move on a planner that cannot see it."""
    arb = make()
    armed(arb, 0.0, stowed=False)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 0.0)
    d = arb.step(0.0)
    assert d.twist.is_zero()
    assert d.blocked_by == "arm_not_stowed"


def test_arm_stowed_allows_base_motion():
    arb = make()
    armed(arb, 0.0)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 0.0)
    d = arb.step(0.0)
    assert d.twist.vx == pytest.approx(0.5)
    assert d.source == "nav" and d.blocked_by is None


def test_stale_arm_gate_blocks__fail_closed():
    """THE most important test. The arm monitor crashing looks exactly like the arm monitor being
    quiet, and the crash is what the interlock defends against. Silence must mean 'do not move'."""
    arb = make(gate_timeout_s=0.2)
    armed(arb, 0.0)                       # arm_stowed=True asserted once, then the publisher dies
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 1.0)
    arb.set_gate("enable", True, 1.0)     # deadman still live, so only staleness is under test
    d = arb.step(1.0)
    assert d.twist.is_zero()
    assert d.blocked_by == "arm_not_stowed"


def test_never_seen_arm_gate_blocks():
    """Boot order must not create a window where motion is permitted before the monitor is up."""
    arb = make()
    arb.set_gate("enable", True, 0.0)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 0.0)
    assert arb.step(0.0).blocked_by == "arm_not_stowed"


def test_arm_interlock_applies_to_teleop_too():
    """A human driving with the arm out is exactly as dangerous as the planner doing it. Teleop is
    exempt from the deadman, never from the interlock."""
    arb = make()
    armed(arb, 0.0, stowed=False, override=False)
    arb.submit("teleop", Twist3(0.3, 0.0, 0.0), 0.0)
    assert arb.step(0.0).blocked_by == "arm_not_stowed"


def test_override_lets_teleop_recover_a_stuck_arm_but_crawls():
    """The recovery path: a faulted arm stuck extended still has to be driven out of the doorway."""
    arb = make(override_speed_factor=0.25)
    armed(arb, 0.0, stowed=False, override=True)
    arb.submit("teleop", Twist3(10.0, 0.0, 0.0), 0.0)   # ask for far more than allowed
    d = arb.step(0.0)
    assert d.override_active
    assert d.twist.vx == pytest.approx(0.25 * FAST.max_vx)


def test_override_does_not_rescue_autonomous_sources():
    """Override is a human recovery tool. Nav2 must never benefit from it."""
    arb = make()
    armed(arb, 0.0, stowed=False, override=True)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 0.0)
    assert arb.step(0.0).blocked_by == "arm_not_stowed"


# --------------------------------------------------------------------------------------------------
# Priority arbitration
# --------------------------------------------------------------------------------------------------
def test_teleop_preempts_nav__the_takeover_path():
    arb = make()
    armed(arb, 0.0)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 0.0)
    arb.submit("teleop", Twist3(-0.2, 0.0, 0.0), 0.0)
    d = arb.step(0.0)
    assert d.source == "teleop" and d.twist.vx == pytest.approx(-0.2)


def test_servo_preempts_nav():
    arb = make()
    armed(arb, 0.0)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 0.0)
    arb.submit("servo", Twist3(0.1, 0.0, 0.0), 0.0)
    assert arb.step(0.0).source == "servo"


def test_stale_teleop_yields_back_to_nav():
    """Releasing manual control must hand authority back, not deadlock the robot."""
    arb = make(input_timeout_s=0.3)
    arb.submit("teleop", Twist3(-0.2, 0.0, 0.0), 0.0)
    armed(arb, 1.0)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 1.0)
    assert arb.step(1.0).source == "nav"


# --------------------------------------------------------------------------------------------------
# Deadman, staleness, E-stop
# --------------------------------------------------------------------------------------------------
def test_deadman_released_blocks_autonomy():
    arb = make()
    armed(arb, 0.0, enable=False)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 0.0)
    assert arb.step(0.0).blocked_by == "deadman"


def test_deadman_does_not_gate_teleop():
    """The manual path must not depend on a second live topic — it is what you reach for when the
    rest of the stack has already failed."""
    arb = make()
    arb.set_gate("arm_stowed", True, 0.0)
    arb.submit("teleop", Twist3(0.2, 0.0, 0.0), 0.0)
    assert arb.step(0.0).source == "teleop"


def test_pipeline_death_mid_press_stops_the_base():
    """The scenario that motivated the whole module: the process publishing twists dies, and
    nothing else notices."""
    arb = make(input_timeout_s=0.3)
    armed(arb, 0.0)
    arb.submit("servo", Twist3(0.4, 0.0, 0.0), 0.0)
    assert arb.step(0.0).twist.vx == pytest.approx(0.4)
    armed(arb, 0.5)                       # gates stay healthy; only the commander went away
    d = arb.step(0.5)
    assert d.twist.is_zero() and d.blocked_by == "no_source"


def test_estop_latches_until_explicitly_cleared():
    """An E-stop that un-latches because a topic flapped is not an E-stop."""
    arb = make()
    armed(arb, 0.0)
    arb.set_gate("estop", True, 0.0)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 0.0)
    assert arb.step(0.0).blocked_by == "estop"

    arb.set_gate("estop", False, 1.0)     # topic says all-clear — must be ignored
    armed(arb, 1.0)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 1.0)
    assert arb.step(1.0).blocked_by == "estop"

    arb.clear_estop()                     # deliberate human re-arm
    armed(arb, 2.0)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 2.0)
    assert arb.step(2.0).source == "nav"


def test_estop_stops_hard_without_ramping():
    arb = make(limits=Limits(max_decel_lin=0.1))
    armed(arb, 0.0)
    arb.submit("nav", Twist3(0.5, 0.0, 0.0), 0.0)
    arb.step(0.0)
    arb.set_gate("estop", True, 0.1)
    assert arb.step(0.1).twist.is_zero()


# --------------------------------------------------------------------------------------------------
# Limits and slew
# --------------------------------------------------------------------------------------------------
def test_speed_ceilings_clamp():
    arb = make(limits=Limits(max_vx=0.6, max_vy=0.4, max_wz=0.8, max_accel_lin=1e6,
                             max_decel_lin=1e6, max_accel_ang=1e6, max_decel_ang=1e6))
    armed(arb, 0.0)
    arb.submit("nav", Twist3(5.0, -5.0, 5.0), 0.0)
    d = arb.step(0.0)
    assert (d.twist.vx, d.twist.vy, d.twist.wz) == pytest.approx((0.6, -0.4, 0.8))


def test_acceleration_is_slew_limited__tipping_guard():
    """The riser raised the CG and high-CoM tip is a failure we have already seen once."""
    arb = make(limits=Limits(max_vx=1.0, max_accel_lin=0.5, max_decel_lin=1.5), nominal_dt_s=0.05)
    armed(arb, 0.0)
    arb.submit("nav", Twist3(1.0, 0.0, 0.0), 0.0)
    assert arb.step(0.0).twist.vx == pytest.approx(0.025)   # first tick uses nominal dt
    armed(arb, 0.1)
    arb.submit("nav", Twist3(1.0, 0.0, 0.0), 0.1)
    assert arb.step(0.1).twist.vx == pytest.approx(0.075)   # + 0.5 m/s^2 * 0.1 s


def test_first_tick_does_not_jump_to_full_speed():
    """Regression: a mux restarting while Nav2 is already commanding full speed must ramp, not
    slam. With dt=0 on the first tick the slew limiter used to pass the command straight through —
    precisely the hard-acceleration case the CG cannot take."""
    arb = make(limits=Limits(max_vx=1.0, max_accel_lin=0.5, max_decel_lin=1.5), nominal_dt_s=0.05)
    armed(arb, 100.0)                                     # node starts mid-mission
    arb.submit("nav", Twist3(1.0, 0.0, 0.0), 100.0)
    assert arb.step(100.0).twist.vx == pytest.approx(0.025)


def test_repeated_timestamp_holds_rather_than_jumps():
    arb = make(limits=Limits(max_vx=1.0, max_accel_lin=0.5, max_decel_lin=1.5), nominal_dt_s=0.05)
    armed(arb, 0.0)
    arb.submit("nav", Twist3(1.0, 0.0, 0.0), 0.0)
    v1 = arb.step(0.0).twist.vx
    assert arb.step(0.0).twist.vx == pytest.approx(v1)     # no elapsed time -> no change


def test_deceleration_is_faster_than_acceleration():
    """Gentle ramp-up protects the CG; refusing to stop promptly is never the safer trade."""
    lim = Limits(max_vx=1.0, max_accel_lin=0.5, max_decel_lin=1.5)
    arb = make(limits=lim)
    armed(arb, 0.0)
    arb.submit("nav", Twist3(1.0, 0.0, 0.0), 0.0)
    arb.step(0.0)
    for i in range(1, 21):                          # ramp up to the ceiling
        t = i * 0.1
        armed(arb, t)
        arb.submit("nav", Twist3(1.0, 0.0, 0.0), t)
        arb.step(t)
    v_before = arb.step(2.1).twist.vx
    armed(arb, 2.2)
    arb.submit("nav", Twist3(0.0, 0.0, 0.0), 2.2)
    v_after = arb.step(2.2).twist.vx
    assert (v_before - v_after) == pytest.approx(0.15)     # 1.5 m/s^2 * 0.1 s


# --------------------------------------------------------------------------------------------------
# Robustness — the mux must never be the thing that dies
# --------------------------------------------------------------------------------------------------
def test_unknown_source_is_ignored_not_raised():
    arb = make()
    armed(arb, 0.0)
    arb.submit("stray_publisher", Twist3(9.0, 0.0, 0.0), 0.0)
    assert arb.step(0.0).twist.is_zero()


def test_duplicate_source_names_rejected_at_construction():
    with pytest.raises(ValueError):
        SafetyArbiter(sources=[SERVO, SourceSpec("servo", "/other", 1)])


def test_no_commands_at_all_is_quiet_not_an_error():
    arb = make()
    armed(arb, 0.0)
    d = arb.step(0.0)
    assert d.twist.is_zero() and d.blocked_by == "no_source" and not d.estop_latched
