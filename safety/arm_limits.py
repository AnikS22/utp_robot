#!/usr/bin/env python3
"""Joint limits this build has that the xArm does not know about. PURE LOGIC, no SDK, no rclpy.

THE xARM'S OWN LIMITS ARE NOT THIS ROBOT'S LIMITS. The arm is mounted on a riser above a chassis
that carries the rover laptop. J2 more negative than -55 deg swings the upper arm down into the
laptop -- the controller has no idea it is there, so it will happily plan through it and the only
thing that stops the motion is contact, or an operator on the E-stop.

SET BY THE OPERATOR 2026-09-01 as a hard limit in UFACTORY Studio, after two faults during a
press attempt. -55 is NOT the contact angle -- it already contains the operator's margin before
the laptop, and the controller enforces it itself. So a pose sitting exactly at -55 is legal.

WHAT THIS MODULE IS FOR, GIVEN THE CONTROLLER ALREADY ENFORCES IT. A limit hit at the controller
presents as error 23 ("Large Motor Position Deviation") or a bare -9 from set_servo_angle, with
the arm stopped mid-move and no statement about which joint or why. That is indistinguishable
from a payload fault or a blocked tool -- I spent a press attempt chasing tcp_load on exactly
that evidence. Checking here turns a silent stop into a sentence naming the joint and the limit,
BEFORE anything is commanded.

WHY A SEPARATE MODULE. Three different callers command joint angles -- stow_arm.py (named poses),
approach_target.py (Cartesian, so IK can produce any joint solution) and anything reading a stored
pose from calib/. A limit enforced in one of them is a limit that does not exist.
"""
from __future__ import annotations

# Degrees. None means "no limit beyond the arm's own".
JOINT_LIMITS_DEG: dict[int, tuple[float | None, float | None]] = {
    # joint: (min, max)
    2: (-55.0, None),      # laptop on the chassis deck below the arm
}

# 1 degree, and the distinction matters: -55 is a legal place for the arm to SIT (the operator's
# limit already contains room before the laptop), but COMMANDING exactly -55 makes the
# controller's own software limit clamp the trajectory and report error 23, "Large Motor Position
# Deviation" -- observed twice on 2026-09-01, both times with the arm arriving exactly on target.
# So poses that will be commanded are held off the boundary; the boundary itself is not the fault.
POSE_MARGIN_DEG = 1.0


def violations(joints_deg, *, margin: float = 0.0) -> list[str]:
    """Which limits a joint vector breaks. Empty list means it is safe to command.

    `joints_deg` is 1-indexed by convention in the xArm docs but a plain list here: index 0 is J1.
    A short or malformed vector is a VIOLATION, not a pass -- refusing to check is not the same
    as checking and finding nothing.
    """
    if joints_deg is None:
        return ["no joint vector given"]
    try:
        vals = [float(v) for v in joints_deg]
    except (TypeError, ValueError):
        return [f"joint vector is not numeric: {joints_deg!r}"]
    bad = []
    for j, (lo, hi) in JOINT_LIMITS_DEG.items():
        if len(vals) < j:
            bad.append(f"J{j} missing from a {len(vals)}-joint vector")
            continue
        v = vals[j - 1]
        if lo is not None and v < lo + margin:
            bad.append(f"J{j}={v:+.2f} deg is past its limit of {lo:+.1f} "
                       f"(margin {margin:.1f}) -- this build puts the laptop there")
        if hi is not None and v > hi - margin:
            bad.append(f"J{j}={v:+.2f} deg is past its limit of {hi:+.1f} (margin {margin:.1f})")
    return bad


def check_pose(joints_deg) -> list[str]:
    """Stricter check for a pose being SAVED or used as a target, with margin for overshoot."""
    return violations(joints_deg, margin=POSE_MARGIN_DEG)
