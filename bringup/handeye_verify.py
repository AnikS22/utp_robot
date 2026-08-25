#!/usr/bin/env python3
"""Measure how accurately the calibrated chain can PLACE the tool at a point it was told.

    python3 bringup/handeye_verify.py --dry-run
    python3 bringup/handeye_verify.py --go

THE ARM MOVES, but nothing is ever commanded to touch anything: every target is a point in free
space near where the arm already is.

WHAT THIS MEASURES, AND WHY THE RESIDUAL DOES NOT
handeye_solve_rw.py reports an RMS residual, and it was 3.0 mm. That number says the ten
observations are mutually consistent -- it does NOT say the transform is right. A calibration with
a systematic error fits its own data perfectly and is wrong everywhere. (The independent check
that it IS right was the solved camera x of -327.6 mm against -324.2 mm off the CAD.)

This closes the loop instead: choose a target point, compute where the flange must go to put the
MARKER there, command it, then measure where the marker actually ended up -- through the camera,
through the calibration. The error is end-to-end and includes everything: calibration error, arm
positioning error, marker detection error, depth error. It is the number that decides whether an
11-15 cm ADA plate is a comfortable target or a coin flip.

    t_flange = P_target - R_flange @ marker_offset

Orientation is held at whatever the arm starts with. Varying it would be a better test of the
rotation, but it also moves the elbow, and the arm is near a wall. Translation accuracy is what
governs whether a press lands.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
sys.path.insert(0, str(REPO / ".venv-arm/lib/python3.12/site-packages"))

from handeye_solve_rw import rpy_deg_to_R  # noqa: E402

ARM_IP = "192.168.1.221"
SPEED_MM_S = 40.0
SETTLE_S = 1.5
MAX_REACH_MM = 160.0        # refuse any target further than this from the starting tool position

# Offsets from the marker's CURRENT position, metres. Spread in all three axes so a systematic
# error in one shows up as a pattern rather than hiding in the average.
TARGET_OFFSETS = [
    ( 0.00,  0.00,  0.00),
    ( 0.05,  0.00,  0.00),
    (-0.05,  0.00,  0.00),
    ( 0.00,  0.05,  0.00),
    ( 0.00, -0.05,  0.00),
    ( 0.00,  0.00,  0.05),
    ( 0.00,  0.00, -0.05),
    ( 0.04,  0.04, -0.03),
]


def load_calib():
    f = REPO / "calib" / "handeye.json"
    if not f.exists():
        raise SystemExit("no calib/handeye.json -- run bringup/handeye_solve_rw.py first")
    d = json.loads(f.read_text())
    return (np.array(d["T_link_base_camera"]),
            np.array(d["marker_on_flange_mm"]) / 1000.0, d)


def observe_marker(name, want_id, T_arm_cam):
    """Capture a pair and return the marker's position in link_base, or None."""
    r = subprocess.run(
        [sys.executable, str(REPO / "bringup" / "handeye_capture.py"),
         "--name", name, "--id", str(want_id)],
        capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        return None
    d = json.loads((REPO / "calib" / "pairs" / f"{name}.json").read_text())
    p_cam = np.array(d["t_cam_marker"])          # PnP, not the depth centroid
    return (T_arm_cam[:3, :3] @ p_cam + T_arm_cam[:3, 3]), d


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", type=int, default=3)
    a = ap.parse_args()
    if not (a.go or a.dry_run):
        print("pass --dry-run or --go", file=sys.stderr)
        return 2

    T_arm_cam, marker_off, meta = load_calib()
    print(f"calibration: {meta['n_pairs']} pairs, rms {meta['rms_mm']:.1f} mm")
    print(f"marker on flange: {marker_off*1000} mm\n")

    from xarm.wrapper import XArmAPI
    arm = XArmAPI(ARM_IP, is_radian=False, do_not_open=False)
    if arm.error_code:
        raise SystemExit(f"arm error {arm.error_code}")
    code, pos = arm.get_position(is_radian=False)
    start_xyz = np.array(pos[:3]) / 1000.0
    start_rpy = np.array(pos[3:6])
    R_flange = rpy_deg_to_R(start_rpy)
    print(f"start flange : {start_xyz*1000} mm  rpy {start_rpy}")

    # where the marker is right now, per the calibration
    marker_now = R_flange @ marker_off + start_xyz
    print(f"marker now   : {marker_now*1000} mm (predicted from calibration)")

    targets = [marker_now + np.array(o) for o in TARGET_OFFSETS]
    print(f"\n{len(targets)} targets:")
    for i, (t, o) in enumerate(zip(targets, TARGET_OFFSETS), 1):
        flange = t - R_flange @ marker_off
        reach = np.linalg.norm(flange - start_xyz) * 1000
        flag = "  TOO FAR" if reach > MAX_REACH_MM else ""
        print(f"  t{i:02d} offset {np.array(o)*1000}  -> flange {flange*1000} mm "
              f"({reach:.0f} mm){flag}")

    if a.dry_run:
        arm.disconnect()
        print("\ndry run: nothing moved.")
        return 0

    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.5)

    errors = []
    try:
        for i, tgt in enumerate(targets, 1):
            flange = tgt - R_flange @ marker_off
            reach = np.linalg.norm(flange - start_xyz) * 1000
            if reach > MAX_REACH_MM:
                print(f"\n[t{i:02d}] skipped: {reach:.0f} mm exceeds the {MAX_REACH_MM:.0f} mm limit")
                continue
            print(f"\n[t{i:02d}] target {tgt*1000} mm -> commanding flange...")
            code = arm.set_position(x=flange[0]*1000, y=flange[1]*1000, z=flange[2]*1000,
                                    roll=start_rpy[0], pitch=start_rpy[1], yaw=start_rpy[2],
                                    speed=SPEED_MM_S, is_radian=False, wait=True)
            if code != 0 or arm.error_code:
                print(f"  move failed code={code} err={arm.error_code} -- stopping")
                break
            time.sleep(SETTLE_S)

            got = observe_marker(f"verify_{i:02d}", a.id, T_arm_cam)
            if got is None:
                print("  marker not seen at this pose")
                continue
            measured, d = got
            err = (measured - tgt) * 1000
            mag = float(np.linalg.norm(err))
            errors.append((i, mag, err))
            print(f"  measured {measured*1000} mm")
            print(f"  ERROR  dx={err[0]:+.1f} dy={err[1]:+.1f} dz={err[2]:+.1f} mm   "
                  f"|e|={mag:.1f} mm")
    finally:
        print("\nreturning to start...")
        if not arm.error_code:
            arm.set_position(x=start_xyz[0]*1000, y=start_xyz[1]*1000, z=start_xyz[2]*1000,
                             roll=start_rpy[0], pitch=start_rpy[1], yaw=start_rpy[2],
                             speed=SPEED_MM_S, is_radian=False, wait=True)
        arm.disconnect()

    if not errors:
        print("no measurements taken.")
        return 1
    mags = np.array([e[1] for e in errors])
    vecs = np.array([e[2] for e in errors])
    print("\n" + "=" * 66)
    print(f"PLACEMENT ACCURACY over {len(errors)} targets")
    print(f"  mean |error| {mags.mean():.1f} mm    worst {mags.max():.1f} mm")
    print(f"  bias   dx={vecs[:,0].mean():+.1f}  dy={vecs[:,1].mean():+.1f}  "
          f"dz={vecs[:,2].mean():+.1f} mm")
    print(f"  spread dx={vecs[:,0].std():5.1f}  dy={vecs[:,1].std():5.1f}  "
          f"dz={vecs[:,2].std():5.1f} mm")
    print("\n  An ADA push plate is 110-150 mm across, so a press needs roughly +/-30 mm.")
    ok = mags.max() < 30.0
    print(f"  -> {'COMFORTABLE' if ok else 'MARGINAL -- see the bias above'}")
    if abs(vecs.mean(axis=0)).max() > 10:
        print("\n  A consistent BIAS (rather than scatter) points at the marker offset or the")
        print("  riser, not the solve -- both enter as pure translations.")
    print("=" * 66)
    return 0


if __name__ == "__main__":
    sys.exit(main())
