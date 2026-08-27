#!/usr/bin/env python3
"""Fold the arm into the stow pose, so the base is allowed to move.

    python3 bringup/stow_arm.py            # dry run: print the joint deltas
    python3 bringup/stow_arm.py --go       # THE ARM MOVES

THE ARM MOVES. It folds toward the chassis, not toward the world, but it sweeps on the way.

WHY THIS IS NEEDED, EVERY TIME. config/safety.yaml gates base motion on /safety/arm_stowed, and
that gate is driven by MEASURED joint angles, not by any FSM's belief about itself. With the riser
fitted an extended arm sweeps ~0.88 m of space the costmap believes is empty, on a chassis whose
high CoM already tips -- so the mux refuses every motion source while the arm is out, and reports
blocked_by="arm_not_stowed".

THE TRAP THIS EXISTS FOR. bringup/approach_target.py retreats to its START pose, which is wherever
the arm happened to be before the approach -- NOT to stow. So a route that presses a button and
then drives on is gated off immediately after a SUCCESSFUL press, and the failure looks like a
navigation problem. Stow after acting, always.

JOINT SPACE, not Cartesian: a Cartesian goal near a singularity can swing the elbow through a
large arc to reach a nearby point. Bounded joint moves bound the motion of every link.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / ".venv-arm/lib/python3.12/site-packages"))

ARM_IP = "192.168.1.221"
READY_FILE = REPO / "calib" / "arm_ready.json"
STOW_FILE = REPO / "calib" / "arm_stow.json"
STOW_DEG = [0.0, -45.0, -45.0, 0.0, 90.0, 0.0]   # config/safety.yaml arm_monitor.xarm.stow_pose_deg
TOL_DEG = 5.0
SPEED_DEG_S = 20.0                                # a crawl; there is no benefit to going faster


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true", help="actually move the arm")
    ap.add_argument("--ip", default=ARM_IP)
    # THE PRESS-READY POSE. Stow and press are DIFFERENT orientations and cannot be the same one:
    # stow folds the wrist to J5=90 so the tool points up out of the way, while a press needs the
    # tool pointing AT the wall (J5 ~ 2.5, set by the operator on 2026-08-26). approach_target.py
    # holds whatever orientation the arm is in when it starts, so approaching straight out of stow
    # reaches at the stow angle and skids off a round button. Hence: stow -> ready -> approach ->
    # retreat -> stow, and the ready orientation is the operator's, captured rather than guessed.
    ap.add_argument("--save-ready", action="store_true",
                    help="record the arm's CURRENT joints as the press-ready pose")
    # THE STOW POSE IS THE OPERATOR'S TOO. The default [0,-45,-45,0,90,0] from config/safety.yaml
    # leaves the tool HORIZONTAL, 254 mm forward of the arm base -- measured 2026-08-26, rpy pitch
    # 0. That is a poor driving pose: it is the tool tip, on a 0.88 m radius, sticking out over a
    # chassis that already tips. Tool-UP would need J5 ~ 180, which is exactly its joint stop, so
    # this is captured from a human who can see the arm rather than solved for here.
    ap.add_argument("--save-stow", action="store_true",
                    help="record the arm's CURRENT joints as the stow pose")
    ap.add_argument("--ready", action="store_true",
                    help="move to the saved press-ready pose instead of stow")
    a = ap.parse_args()

    from xarm.wrapper import XArmAPI
    arm = XArmAPI(a.ip, is_radian=False)
    arm.connect()
    if not arm.connected:
        print(f"cannot reach the arm at {a.ip}", file=sys.stderr)
        return 1
    cur = arm.get_servo_angle()[1][:6]

    if a.save_stow:
        STOW_FILE.parent.mkdir(parents=True, exist_ok=True)
        STOW_FILE.write_text(json.dumps({"joints_deg": [round(v, 2) for v in cur]}, indent=2))
        print(f"stow pose saved: {[round(v,1) for v in cur]}")
        print(f"  -> {STOW_FILE}")
        print("NOTE: config/safety.yaml arm_monitor.xarm.stow_pose_deg must match this, or the")
        print("      gate will read not-stowed at the very pose you just chose.")
        arm.disconnect()
        return 0

    if a.save_ready:
        READY_FILE.parent.mkdir(parents=True, exist_ok=True)
        READY_FILE.write_text(json.dumps({"joints_deg": [round(v, 2) for v in cur]}, indent=2))
        print(f"press-ready pose saved: {[round(v,1) for v in cur]}")
        print(f"  -> {READY_FILE}")
        arm.disconnect()
        return 0

    target = STOW_DEG
    label = "stow"
    if STOW_FILE.exists() and not a.ready:
        target = json.loads(STOW_FILE.read_text())["joints_deg"]
    if a.ready:
        if not READY_FILE.exists():
            print(f"no press-ready pose saved. Put the arm where you want it to press from, "
                  f"then: python3 bringup/stow_arm.py --save-ready", file=sys.stderr)
            arm.disconnect()
            return 2
        target = json.loads(READY_FILE.read_text())["joints_deg"]
        label = "ready"

    err = [c - s for c, s in zip(cur, target)]
    print(f"{'joint':>6}{'now':>9}{label:>9}{'delta':>9}")
    for i, (c, s, e) in enumerate(zip(cur, target, err), 1):
        print(f"  J{i}  {c:8.1f} {s:8.1f} {e:+8.1f}")
    stowed = all(abs(e) <= TOL_DEG for e in err)
    print(f"\nat {label} now: {stowed}   (tolerance {TOL_DEG} deg)")
    if stowed:
        print(f"already at {label}; nothing to do.")
        arm.disconnect()
        return 0
    if not a.go:
        print(f"DRY RUN. Largest move is J{max(range(6), key=lambda i: abs(err[i]))+1} "
              f"by {max(abs(e) for e in err):.0f} deg. Add --go.")
        arm.disconnect()
        return 0

    if arm.error_code:
        # Never move an arm whose fault you have not looked at.
        print(f"arm error_code={arm.error_code}; clear it before stowing", file=sys.stderr)
        arm.disconnect()
        return 1
    arm.motion_enable(True); arm.set_mode(0); arm.set_state(0); time.sleep(0.5)
    code = arm.set_servo_angle(angle=target, speed=SPEED_DEG_S, wait=True)
    now = arm.get_servo_angle()[1][:6]
    ok = all(abs(c - s) <= TOL_DEG for c, s in zip(now, target))
    print(f"set_servo_angle -> {code} (0 = ok)")
    print(f"now: {[round(v,1) for v in now]}")
    print(f"at {label}: {ok}")
    arm.disconnect()
    return 0 if (code == 0 and ok) else 1


if __name__ == "__main__":
    sys.exit(main())
