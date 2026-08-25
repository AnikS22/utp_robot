"""Guards for the keyboard teleop — the regression suite for a real runaway.

On 2026-08-20 the base kept driving after the operator's hands left the keyboard and only the
hardware E-stop stopped it. The heartbeat watchdog did not fire and could not have: the page was
alive, posting a stale belief that a key was held after a `keyup` was dropped while the JS thread
was saturated rendering the camera.

Every test below names the real failure it prevents. The distinction the whole module turns on:
a heartbeat proves the SENDER IS ALIVE; the hold lease proves A HUMAN IS STILL PRESSING. The
incident happened because only the first was checked.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "safety"))
from teleop_guard import (  # noqa: E402
    HOLD_LEASE_S, WATCHDOG_S, ZERO, Command, decide,
)

DRIVE = Command(vx=0.4, wz=-0.2)
FRESH = dict(heartbeat_age_s=0.05, key_age_s=0.05)


def d(cmd=DRIVE, *, override=False, stopped=False, **kw):
    return decide(cmd, override=override, stopped=stopped, **{**FRESH, **kw})


# ---- the incident ---------------------------------------------------------------------------
def test_lost_keyup_stops_the_base():
    """THE REGRESSION. Page alive and posting (heartbeat fine), still claiming a key is held,
    but no keydown/autorepeat within the lease -> the key is not physically down -> zero."""
    v = d(key_age_s=HOLD_LEASE_S + 0.01)
    assert v.command == ZERO
    assert not v.moving
    assert v.reason == "hold_lease_expired"


def test_healthy_heartbeat_alone_does_not_authorise_motion():
    """The exact wrong assumption that caused the runaway, stated as a test."""
    assert not d(heartbeat_age_s=0.0, key_age_s=5.0).moving


def test_genuine_continuous_hold_is_not_interrupted():
    """Autorepeat renews the lease, so a real sustained press must keep driving -- otherwise the
    fix would be useless and someone would turn it off."""
    for age in (0.0, 0.03, 0.05, HOLD_LEASE_S - 0.01):
        v = d(key_age_s=age)
        assert v.command == DRIVE and v.reason == "ok", age


def test_lease_boundary_is_inclusive():
    assert d(key_age_s=HOLD_LEASE_S).reason == "ok"
    assert d(key_age_s=math.nextafter(HOLD_LEASE_S, 1e9)).reason == "hold_lease_expired"


# ---- heartbeat guard ------------------------------------------------------------------------
def test_no_heartbeat_zeroes():
    v = d(heartbeat_age_s=WATCHDOG_S + 0.01)
    assert v.command == ZERO and v.reason == "no_heartbeat"


def test_never_seen_page_fails_closed():
    """inf == the page has never posted. Startup must not authorise motion."""
    assert decide(DRIVE, override=True, stopped=False,
                  heartbeat_age_s=math.inf, key_age_s=0.0).command == ZERO


def test_heartbeat_loss_also_drops_override():
    """override means 'a human is watching'. A page that is gone is asserting nothing, so the gate
    must fall too -- otherwise a dead tab keeps the arm interlock overridden."""
    v = decide(ZERO, override=True, stopped=False, heartbeat_age_s=9.0, key_age_s=0.0)
    assert v.override is False


def test_override_survives_a_live_page():
    assert d(Command(), override=True).override is True


def test_heartbeat_beats_hold_lease():
    """Both expired -> report the heartbeat, the more fundamental failure."""
    assert d(heartbeat_age_s=9.0, key_age_s=9.0).reason == "no_heartbeat"


# ---- SPACE latch ----------------------------------------------------------------------------
def test_stop_latch_zeroes_but_keeps_override():
    v = d(stopped=True)
    assert v.command == ZERO and v.reason == "stop_latched"
    v2 = d(stopped=True, override=True)
    assert v2.override is True      # the human is still there; only motion is refused


def test_stop_latch_beats_a_valid_command():
    assert not d(DRIVE, stopped=True).moving


# ---- malformed input fails closed ------------------------------------------------------------
@pytest.mark.parametrize("bad", [math.nan, math.inf])
def test_nan_or_inf_key_age_refuses_motion(bad):
    """A NaN age must not slip through a naive `age > lease` comparison -- every comparison with
    NaN is False, so the check is written as `not (age <= lease)` on purpose."""
    assert d(key_age_s=bad).command == ZERO


@pytest.mark.parametrize("bad", [math.nan, math.inf])
def test_nan_or_inf_heartbeat_refuses_motion(bad):
    assert d(heartbeat_age_s=bad).command == ZERO


def test_missing_key_age_is_no_evidence():
    """The node defaults an absent key_age_ms to a huge number rather than 0. Absence of evidence
    must never read as evidence of a held key."""
    assert d(key_age_s=1e9).reason == "hold_lease_expired"


# ---- zero commands ----------------------------------------------------------------------------
def test_zero_command_passes_regardless_of_key_age():
    """The lease gates MOTION, not publication. The node publishes every tick, zeros included --
    a teleop that goes quiet is indistinguishable downstream from one that died."""
    v = d(Command(), key_age_s=1e6)
    assert v.command == ZERO and v.reason == "ok"


@pytest.mark.parametrize("cmd", [Command(vx=1e-3), Command(vy=1e-3), Command(wz=1e-3)])
def test_any_nonzero_axis_needs_a_live_key(cmd):
    """Strafe and yaw are as dangerous as forward -- the lease must not be forward-only."""
    assert decide(cmd, override=False, stopped=False,
                  heartbeat_age_s=0.0, key_age_s=5.0).command == ZERO


def test_command_passes_through_unmodified_when_healthy():
    v = d()
    assert (v.command.vx, v.command.vy, v.command.wz) == (DRIVE.vx, DRIVE.vy, DRIVE.wz)


# ---- tunables -----------------------------------------------------------------------------------
def test_thresholds_are_injectable():
    assert decide(DRIVE, override=False, stopped=False, heartbeat_age_s=0.5, key_age_s=0.5,
                  watchdog_s=1.0, hold_lease_s=1.0).reason == "ok"


def test_lease_is_shorter_than_a_person_can_react():
    """A lost keyup costs at most HOLD_LEASE_S of travel. At the 0.15 m/s override ceiling that is
    ~15 cm. If anyone raises this, that distance is what they are trading away."""
    assert 0 < HOLD_LEASE_S <= 1.5
    assert 0 < WATCHDOG_S <= 0.5
