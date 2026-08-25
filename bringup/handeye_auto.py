#!/usr/bin/env python3
"""Drive the arm through a small set of poses and capture a hand-eye pair at each.

    python3 bringup/handeye_auto.py --dry-run     # print the plan, move nothing
    python3 bringup/handeye_auto.py --go          # actually move

THE ARM MOVES. Read this before --go.

Deliberately conservative, because on 2026-08-21 the arm was working close to a wall:

  * JOINT space, never Cartesian. A Cartesian goal near a singularity can swing the elbow through
    a large arc to reach a nearby point. Bounded joint deltas bound the motion of every link, not
    just the tool.
  * Deltas are small (see PLAN) and measured from wherever the arm is NOW, so the working volume
    is the one you already chose by placing it. This script never picks an absolute pose.
  * Slow: SPEED_DEG_S is a crawl. There is no benefit to going faster and a real cost to it.
  * The TCP is checked after every move; if it has travelled further than MAX_TCP_STEP_MM from
    the start, everything stops. That catches a wrong sign or a bad delta before the second move
    compounds it.
  * Any arm error code aborts immediately and the arm is left stopped, not homed -- homing after
    a fault means moving a machine whose state you do not understand.
  * It returns to the starting pose on success, so repeated runs are repeatable.

COLLECT MORE THAN ONE BATCH, AT DIFFERENT DISTANCES
Deltas are relative to wherever the arm starts, so one run samples one region. That is not enough.
On 2026-08-21 all ten poses sat ~0.43 m from the camera while the ADA plate is at 0.84 m, and
propagating the leave-one-out spread out to the plate showed the error growing 2.8x -- 4.5 mm mean
in the calibrated volume, 10.3 mm mean and 35.1 mm worst at the plate, against a ~30 mm budget.
Excellent where measured, marginal where it matters.

So run this once with the arm near the camera and again with it extended toward the work, using a
different --prefix each time. handeye_solve_rw.py merges every pair in calib/pairs/:

    python3 bringup/handeye_auto.py --go --prefix near     # arm folded back
    ... reposition the arm out toward the plate ...
    python3 bringup/handeye_auto.py --go --prefix far      # arm at working distance
    python3 bringup/handeye_solve_rw.py

Vary DEPTH between batches especially. Coplanar point sets fit their own data beautifully and
extrapolate badly, and "extrapolate badly" is exactly what a press outside the calibrated volume is.

WHY THESE PARTICULAR DELTAS
The solve needs two things that pull in different directions. ROTATION spread separates the camera
pose from the marker offset -- without it the problem is degenerate and OpenCV refuses. DEPTH
spread stops the point set being coplanar, which otherwise yields a tiny residual that extrapolates
badly. So the wrist joints (4,5,6) carry most of the rotation and the shoulder/elbow (2,3) carry
the translation, and every pose changes both.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
sys.path.insert(0, str(REPO / ".venv-arm/lib/python3.12/site-packages"))

ARM_IP = "192.168.1.221"
SPEED_DEG_S = 12.0
SETTLE_S = 1.5
MAX_TCP_STEP_MM = 220.0          # abort if the tool wanders further than this from the start

# (j1, j2, j3, j4, j5, j6) degrees, RELATIVE to the pose the arm is in when this starts.
# Wrist-heavy for rotation spread; shoulder/elbow for depth spread. Kept small on j1 because
# that one swings the whole arm sideways.
PLAN = [
    (  0,   0,   0,   0,   0,   0),      # pose_01: where you put it
    (  0,  -5,   4,  12,  10,  15),
    (  3,   4,  -5, -12,  12, -15),
    ( -3,  -6,   6,  15,  -8,  20),
    (  2,   6,  -4, -15,  -6, -20),
    (  0,  -8,   7,  20,  14,  10),
    ( -2,   7,  -6, -20,   8, -10),
    (  3,  -4,   3,  10, -14,  25),
    ( -3,   5,  -3, -10,  16, -25),
    (  0,  -7,   6,  18,  -4,   0),
]


def joint_limits_deg(arm):
    """The arm's REAL joint limits, from the same table the SDK validates against.

    Not a hardcoded table: xarm/x3/xarm.py:_is_out_of_joint_range looks up
    XCONF.Robot.JOINT_LIMITS by axis count and device type, so this reads the same source. The
    limits are NOT symmetric and not what you would guess -- J5 on this arm runs -97..+180, and
    on 2026-08-21 a plain +/-8 deg delta walked it off the -97 end and aborted the collection
    three poses in. Guessing this table is how that happens twice.
    """
    import math
    from xarm.core.config.x_config import XCONF
    axis = arm.axis
    sn = arm.sn or ""
    dt = int(f"{axis}1305") if (len(sn) >= 6 and sn[2:6].isdigit()
                                and 1305 <= int(sn[2:6]) < 8500) else arm.device_type
    lim = XCONF.Robot.JOINT_LIMITS.get(axis, {}).get(dt, [])
    return [(math.degrees(lo), math.degrees(hi)) for lo, hi in lim[:6]]


def clamp_plan(start, plan, limits, margin_deg=3.0):
    """Clamp every planned joint to its limit, keeping a margin. Returns (targets, clamped).

    Clamping rather than skipping: a pose pushed slightly back inside the envelope is still a
    perfectly good observation, and dropping poses costs the rotation spread the solve needs.
    """
    targets, clamped = [], []
    for pi, delta in enumerate(plan, 1):
        tgt, hits = [], []
        for j, (s, d) in enumerate(zip(start, delta)):
            v = s + d
            if j < len(limits):
                lo, hi = limits[j]
                v2 = min(max(v, lo + margin_deg), hi - margin_deg)
                if abs(v2 - v) > 1e-6:
                    hits.append(f"J{j+1} {v:+.1f}->{v2:+.1f}")
                v = v2
            tgt.append(v)
        targets.append(tgt)
        if hits:
            clamped.append((pi, hits))
    return targets, clamped


def connect():
    from xarm.wrapper import XArmAPI
    arm = XArmAPI(ARM_IP, is_radian=False, do_not_open=False)
    if arm.error_code:
        raise SystemExit(f"arm reports error {arm.error_code}. Clear it before moving.")
    return arm


def tcp_mm(arm):
    code, pos = arm.get_position(is_radian=False)
    if code != 0:
        raise RuntimeError(f"get_position code={code}")
    return np.array(pos[:3])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", type=int, default=3, help="ArUco id on the gripper")
    ap.add_argument("--prefix", default="pose",
                    help="name prefix for the pairs. Use a DIFFERENT prefix for each batch so "
                         "batches accumulate instead of overwriting -- see the note below.")
    ap.add_argument("--speed", type=float, default=SPEED_DEG_S)
    a = ap.parse_args()
    if not (a.go or a.dry_run):
        print("pass --dry-run to see the plan, or --go to move the arm", file=sys.stderr)
        return 2

    arm = connect()
    code, start = arm.get_servo_angle(is_radian=False)
    if code != 0:
        raise SystemExit(f"get_servo_angle code={code}")
    start = list(start)[:6]
    start_tcp = tcp_mm(arm)
    print(f"start joints : {[round(v,2) for v in start]}")
    print(f"start TCP    : {start_tcp[0]:.1f} {start_tcp[1]:.1f} {start_tcp[2]:.1f} mm")
    print(f"state={arm.state} mode={arm.mode} err={arm.error_code}\n")

    limits = joint_limits_deg(arm)
    print("joint limits (deg) and headroom from here:")
    for j, (lo, hi) in enumerate(limits):
        print(f"  J{j+1}: {lo:+7.1f} .. {hi:+7.1f}   now {start[j]:+7.2f}   "
              f"room {start[j]-lo:+6.1f} / {hi-start[j]:+6.1f}")

    targets, clamped = clamp_plan(start, PLAN, limits)
    print(f"\n{len(targets)} poses planned, speed {a.speed:g} deg/s:")
    for i, d in enumerate(PLAN, 1):
        print(f"  pose_{i:02d}  delta {str(list(d)):>28s}")
    if clamped:
        print("\nclamped to stay inside the joint envelope:")
        for pi, hits in clamped:
            print(f"  pose_{pi:02d}: {', '.join(hits)}")
    if a.dry_run:
        print("\ndry run: nothing moved.")
        arm.disconnect()
        return 0

    print("\nenabling servos...")
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)
    if arm.error_code:
        arm.disconnect()
        raise SystemExit(f"arm error {arm.error_code} after enable; not moving")

    captured, failed = 0, []
    try:
        for i, tgt in enumerate(targets, 1):
            name = f"{a.prefix}_{i:02d}"
            print(f"\n[{i}/{len(targets)}] {name}: moving...")
            code = arm.set_servo_angle(angle=tgt, speed=a.speed, is_radian=False, wait=True)
            if code != 0 or arm.error_code:
                print(f"  MOVE FAILED code={code} err={arm.error_code} -- stopping here")
                break
            time.sleep(SETTLE_S)

            now = tcp_mm(arm)
            travel = float(np.linalg.norm(now - start_tcp))
            print(f"  TCP {now[0]:.1f} {now[1]:.1f} {now[2]:.1f} mm  "
                  f"({travel:.0f} mm from start)")
            if travel > MAX_TCP_STEP_MM:
                print(f"  ABORT: travelled {travel:.0f} mm > {MAX_TCP_STEP_MM:.0f} mm limit")
                break

            r = subprocess.run(
                [sys.executable, str(REPO / "bringup" / "handeye_capture.py"),
                 "--name", name, "--id", str(a.id)],
                capture_output=True, text=True, timeout=180)
            if r.returncode == 0:
                captured += 1
                for ln in r.stdout.strip().splitlines():
                    if ln.strip().startswith(("arm ", "cam ", "PnP", "marker id")):
                        print(f"    {ln.strip()}")
            else:
                failed.append(name)
                print(f"    no pair: {r.stdout.strip().splitlines()[-1] if r.stdout else ''}"
                      f"{r.stderr.strip().splitlines()[-1] if r.stderr else ''}")
    finally:
        if not arm.error_code:
            print(f"\nreturning to start pose...")
            arm.set_servo_angle(angle=start, speed=a.speed, is_radian=False, wait=True)
        else:
            print(f"\narm error {arm.error_code}: leaving it where it is, NOT homing.")
        arm.disconnect()

    print(f"\ncaptured {captured} pair(s); {len(failed)} pose(s) gave no usable sighting")
    if failed:
        print(f"  {', '.join(failed)}  (marker out of view or too oblique at those poses)")
    print("\nnow solve:  python3 bringup/handeye_solve_rw.py")
    return 0 if captured >= 5 else 1


if __name__ == "__main__":
    sys.exit(main())
