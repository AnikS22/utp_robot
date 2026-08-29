#!/usr/bin/env python3
"""Step the arm toward a grounded target, checking joint headroom at every move.

    python3 bringup/approach_target.py --capture press_scene2 --dry-run
    python3 bringup/approach_target.py --capture press_scene2 --go --min-standoff 150

THE ARM MOVES, TOWARD A WALL. It stops at --min-standoff and never commands contact.

Approaching is where a calibration gets tested for real, and where the two ways this arm bites
both live:

  J5 HEADROOM. J5 runs -97..+180 deg and sits near -92 in the working pose. A Cartesian goal
  lower than the current pose can need J5 below the stop, and the IK -- not us -- picks the joint
  solution, so no amount of clamping the TARGET prevents it. Error 23 at -96.98 deg on 2026-08-21
  is what that looks like. So every step is checked BEFORE the next one is issued, and the
  approach stops while there is still headroom rather than discovering the limit at the wall.

  THE UNKNOWN TOOL TIP. The calibration solved where the MARKER sits on the flange. It says
  nothing about where the gripper's fingers end, and that is what would touch. So standoff here is
  measured to the MARKER, and the real gap between the gripper and the wall is MEASURED from depth
  at each step instead of assumed. Guessing a fingertip offset is how you drive a gripper into a
  wall while believing you are 100 mm short.

Approach direction is the camera's optical +z mapped into the arm frame -- i.e. straight at the
wall the plate is on, not straight down the arm's own axis.
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

ARM_IP = "192.168.1.221"
SPEED_MM_S = 25.0
SETTLE_S = 1.5
J5_MARGIN_DEG = 4.0        # stop while this much headroom remains on the binding joint
STEP_MM = 60.0


def joint_limits_deg(arm):
    import math
    from xarm.core.config.x_config import XCONF
    sn = arm.sn or ""
    dt = int(f"{arm.axis}1305") if (len(sn) >= 6 and sn[2:6].isdigit()
                                    and 1305 <= int(sn[2:6]) < 8500) else arm.device_type
    lim = XCONF.Robot.JOINT_LIMITS.get(arm.axis, {}).get(dt, [])
    return [(math.degrees(lo), math.degrees(hi)) for lo, hi in lim[:6]]


def headroom(angles, limits):
    """Smallest distance to any joint stop, and which joint. The approach's stopping criterion."""
    worst, which = 1e9, None
    for j, (a, (lo, hi)) in enumerate(zip(angles, limits), 1):
        for room in (a - lo, hi - a):
            if room < worst:
                worst, which = room, j
    return worst, which


DEPTH_MAX_M = 15.0        # beyond this the D435 is guessing; 65.535 is the uint16 "no return"
PLANE_MIN_M, PLANE_MAX_M = 0.3, 5.0


def gripper_gap_mm(capture_dir, bbox_px=None):
    """How far the nearest thing sticks out of the WALL PLANE, in mm. None if not measurable.

    REWRITTEN 2026-08-25. The previous version returned 14-16 METRES and printed it as a
    measurement. Two independent faults, and the second is the instructive one:

    1. DEPTH SATURATION WAS TREATED AS DISTANCE. grab_frame.py maps a 0 mm reading to NaN, but
       the D435 also reports 65535 mm for out-of-range, which is finite and sailed through
       np.isfinite. 3.8% of the frame read over 20 m and poisoned the least-squares fit.

    2. THE "WALL" MASK SELECTED A GLASS DOOR. It was (xx < W*0.28) | (xx > W*0.78) then
       & (xx > 380). At W=1280, W*0.28 = 358, so `xx > 380` deleted the ENTIRE left band and left
       only xx > 998 -- which in the real ADA scene is the glass door and the trees outside.
       The plane was fitted to a car park 40 m away. It produced plane@centre = -9.67 m: a
       NEGATIVE distance, printed to the operator as a millimetre measurement.

    So the wall is now taken from where we KNOW the wall is: an annulus around the detected
    control, which is by definition mounted on it. And the fit is sanity-checked against physics
    -- a wall 0.3-5 m away -- because the failure to defend against is not noise, it is a
    confident answer from the wrong surface.

    Returns None rather than a number whenever it cannot stand behind one. An arm is going to
    act on this.
    """
    d = np.load(Path(capture_dir) / "depth.npy").astype(float)
    d[~np.isfinite(d)] = np.nan
    d[d > DEPTH_MAX_M] = np.nan            # fault 1
    H, W = d.shape
    yy, xx = np.mgrid[0:H, 0:W]

    if bbox_px is not None:                # fault 2: wall = ring around the detected control
        x0, y0, x1, y1 = bbox_px
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        r = max(x1 - x0, y1 - y0)
        rad = np.hypot(xx - cx, yy - cy)
        m = np.isfinite(d) & (rad > 0.75 * r) & (rad < 2.0 * r)
    else:
        m = np.isfinite(d) & ((xx < W * 0.25) | (xx > W * 0.75))
    if m.sum() < 2000:
        return None

    A = np.c_[xx[m].ravel(), yy[m].ravel(), np.ones(m.sum())]
    coef, *_ = np.linalg.lstsq(A, d[m].ravel(), rcond=None)
    plane = coef[0] * xx + coef[1] * yy + coef[2]

    centre = plane[H // 2, W // 2]
    if not (PLANE_MIN_M <= centre <= PLANE_MAX_M):
        return None                        # the fit is not a wall in front of us
    resid = np.abs(d[m] - plane[m])
    if float(np.nanmedian(resid)) > 0.05:  # 5 cm: not a plane, do not report a plane measurement
        return None

    proud = np.isfinite(d) & ((plane - d) > 0.03)
    if proud.sum() < 200:
        return None
    gap = float(np.nanpercentile((plane - d)[proud], 99) * 1000)
    return gap if 0.0 < gap < 1000.0 else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", required=True, help="capture dir holding the grounded detection")
    ap.add_argument("--target-cam", nargs=3, type=float, default=None,
                    metavar=("X", "Y", "Z"), help="target in camera optical frame, metres")
    ap.add_argument("--min-standoff", type=float, default=150.0,
                    help="stop with the MARKER this far from the target, mm")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", type=int, default=3)
    # HOLD AT THE STOP instead of retreating. Added 2026-08-25: the mast camera cannot measure
    # the tool-to-wall gap at the approach pose -- the arm crosses the camera's sightline to the
    # plate long before the wall, so there is no wall left to fit a plane to (measured: median
    # plane residual 181 mm, and the button's own pixels read 0.294 m because they are looking at
    # the gripper). The gap is therefore measured BY HAND, which requires the arm to still be
    # there when you walk up with a ruler.
    ap.add_argument("--hold", action="store_true",
                    help="stay at the final standoff instead of retreating, so the gap can be "
                         "measured by hand. Retreat afterwards with --retreat-only.")
    ap.add_argument("--retreat-only", action="store_true",
                    help="do not approach; just go back to the stored start pose")
    a = ap.parse_args()
    if not (a.go or a.dry_run):
        print("pass --dry-run or --go", file=sys.stderr)
        return 2

    if a.retreat_only:
        home_f = REPO / "calib" / "arm_home.json"
        if not home_f.exists():
            print(f"NO HOME STORED: {home_f} does not exist.", file=sys.stderr)
            print("  It is written by --hold. Nothing to retreat to; move the arm from the "
                  "xArm UI instead.", file=sys.stderr)
            return 2
        h = json.loads(home_f.read_text())
        if not a.go:
            print(f"DRY RUN: would move to {h['xyz_mm']} rpy {h['rpy_deg']}. Add --go.")
            return 0
        from xarm.wrapper import XArmAPI
        arm = XArmAPI(ARM_IP, is_radian=False, do_not_open=False)
        arm.motion_enable(True); arm.set_mode(0); arm.set_state(0); time.sleep(0.5)
        code = arm.set_position(x=h["xyz_mm"][0], y=h["xyz_mm"][1], z=h["xyz_mm"][2],
                                roll=h["rpy_deg"][0], pitch=h["rpy_deg"][1],
                                yaw=h["rpy_deg"][2], speed=30, wait=True)
        print(f"retreat -> code {code} (0 = ok); now "
              f"{[round(v, 1) for v in arm.get_position()[1]]}")
        arm.disconnect()
        return 0 if code == 0 else 1

    c = json.loads((REPO / "calib" / "handeye.json").read_text())
    T = np.array(c["T_link_base_camera"])
    moff = np.array(c["marker_on_flange_mm"]) / 1000.0
    # The target comes from the DETECTION IN THIS CAPTURE, never from a default.
    #
    # This line used to read:  p_cam = a.target_cam or [0.019, 0.162, 0.839]
    # -- a constant from an earlier session. On 2026-08-25 that constant sat 222 mm from the
    # button actually detected in the capture being passed in, on a control 170 mm across: a
    # clean miss, aimed with total confidence, at a wall. --capture was accepted and used only
    # for the depth-based gripper gap. Nothing printed anything wrong.
    #
    # So: read detection.json, or refuse. A wrong target is worse than no target, because the
    # arm executes it either way.
    if a.target_cam:
        p_cam = np.array(a.target_cam)
        print("target from --target-cam (explicit override)")
    else:
        det_path = Path(a.capture) / "detection.json"
        if not det_path.exists():
            print(f"NO TARGET: {det_path} does not exist.", file=sys.stderr)
            print("  Run the grounder on this capture first:", file=sys.stderr)
            print(f"    ~/unlocking-the-path/env/.venv/bin/python bringup/detect_frame.py "
                  f"{a.capture}", file=sys.stderr)
            print("  or pass --target-cam X Y Z to aim somewhere deliberately.", file=sys.stderr)
            return 2
        det = json.loads(det_path.read_text())
        p_cam = np.array(det["point3d_cam_m"], dtype=float)
        print(f"target from {det_path.name}: '{det.get('query')}' "
              f"score {det.get('score', float('nan')):.3f} via {det.get('backend')}")
    p_arm = T[:3, :3] @ p_cam + T[:3, 3]
    approach = T[:3, :3] @ np.array([0.0, 0.0, 1.0])       # wall-ward, in arm coordinates
    approach /= np.linalg.norm(approach)

    print(f"target (camera) : {p_cam}")
    print(f"target (link_base): {p_arm.round(4)}   |from base| {np.linalg.norm(p_arm):.3f} m")
    print(f"approach dir     : {approach.round(3)}")

    # REFUSE A TARGET THE ARM CANNOT REACH, rather than discovering it at the joint stop.
    # Commanding a Cartesian goal outside the envelope does not produce a short reach: the IK
    # drives a joint into its limit and the controller faults. On 2026-08-29 the base stopped
    # 1.23 m from a plate with a 0.88 m arm, this was commanded anyway, and it faulted with
    # ControllerError 21 -- after which the tool exited 0 and the route logged "complete".
    # The fix for being out of reach is to move the BASE (bringup/face_target.py), never to ask
    # the arm for reach it does not have.
    sys.path.insert(0, str(REPO))
    from safety.reach_envelope import check_before_reach
    _ok, _why = check_before_reach(float(np.linalg.norm(p_arm)))
    if not _ok:
        print(f"\nNOT REACHING: {_why}", file=sys.stderr)
        return 1

    from xarm.wrapper import XArmAPI
    arm = XArmAPI(ARM_IP, is_radian=False, do_not_open=False)
    if arm.error_code:
        raise SystemExit(f"arm error {arm.error_code}; clear it first")
    limits = joint_limits_deg(arm)
    code, ang = arm.get_servo_angle(is_radian=False)
    code, pos = arm.get_position(is_radian=False)
    start_xyz, start_rpy = np.array(pos[:3]) / 1000.0, np.array(pos[3:6])
    from handeye_solve_rw import rpy_deg_to_R
    R_f = rpy_deg_to_R(start_rpy)
    marker_now = R_f @ moff + start_xyz
    room, j = headroom(ang[:6], limits)
    print(f"\nstart flange {start_xyz.round(4)}  marker {marker_now.round(4)}")
    print(f"start headroom {room:.1f} deg on J{j}")

    dist_now = float(np.dot(p_arm - marker_now, approach))
    print(f"marker is {dist_now*1000:.0f} mm from the target along the approach axis")
    stops = []
    d = dist_now
    while d - STEP_MM / 1000.0 > a.min_standoff / 1000.0:
        d -= STEP_MM / 1000.0
        stops.append(d)
    stops.append(a.min_standoff / 1000.0)
    print(f"\n{len(stops)} steps, stopping at {a.min_standoff:.0f} mm standoff:")
    for i, s in enumerate(stops, 1):
        tgt = p_arm - approach * s
        fl = tgt - R_f @ moff
        print(f"  step {i}: marker standoff {s*1000:6.0f} mm -> flange {fl.round(4)}")

    if a.dry_run:
        arm.disconnect()
        print("\ndry run: nothing moved.")
        return 0

    arm.motion_enable(enable=True); arm.set_mode(0); arm.set_state(0); time.sleep(0.5)
    failed = False
    try:
        for i, s in enumerate(stops, 1):
            tgt = p_arm - approach * s
            fl = tgt - R_f @ moff
            print(f"\n[step {i}/{len(stops)}] marker standoff {s*1000:.0f} mm ...")
            code = arm.set_position(x=fl[0]*1000, y=fl[1]*1000, z=fl[2]*1000,
                                    roll=start_rpy[0], pitch=start_rpy[1], yaw=start_rpy[2],
                                    speed=SPEED_MM_S, is_radian=False, wait=True)
            if code != 0 or arm.error_code:
                print(f"  STOPPED: code={code} err={arm.error_code}")
                # A FAULTED ARM IS A FAILED PRESS. This used to break, retreat, and return 0, so
                # press_run.sh's `set -e` saw success and route_run printed "complete (4/4)" over
                # a ControllerError 21 -- a failed trial recorded as a successful one, which is
                # the worst outcome available to a benchmark. Observed 2026-08-29 at the doors.
                failed = True
                if arm.error_code == 23:
                    print("  error 23 = joint limit. The IK needed a joint past its stop for this")
                    print("  Cartesian goal. Reposition the BASE rather than forcing the arm.")
                break
            time.sleep(SETTLE_S)
            code, ang = arm.get_servo_angle(is_radian=False)
            room, j = headroom(ang[:6], limits)
            print(f"  joints {[round(v,1) for v in ang[:6]]}")
            print(f"  headroom {room:.1f} deg on J{j}")
            if room < J5_MARGIN_DEG:
                print(f"  STOPPING: only {room:.1f} deg left on J{j} (margin {J5_MARGIN_DEG})")
                break
            r = subprocess.run([sys.executable, str(REPO/"bringup"/"grab_frame.py"),
                                "--name", f"approach_{i:02d}", "--settle", "8"],
                               capture_output=True, text=True, timeout=120)
            if r.returncode == 0:
                gap = gripper_gap_mm(REPO / "captures" / f"approach_{i:02d}")
                if gap is not None:
                    print(f"  MEASURED: nearest part of the gripper is {gap:.0f} mm proud of the "
                          f"wall  (wall-to-tool gap unknown until this exceeds the plate's 64 mm)")
    finally:
        if a.hold:
            # Persist where to come back to. Without this --retreat-only has no home: the script
            # reads "start" from the arm's CURRENT pose, so calling it while extended would treat
            # the extended pose as home and retreat nowhere -- or, worse, re-approach the wall.
            (REPO / "calib" / "arm_home.json").write_text(json.dumps(
                {"xyz_mm": [float(v) for v in start_xyz*1000],
                 "rpy_deg": [float(v) for v in start_rpy]}, indent=2))
            print("\nHOLDING at the final standoff. The arm is still extended.")
            print("  Measure the gap from the TOOL TIP to the wall now, with a ruler.")
            print("  That number -- not the marker standoff -- is what makes a press safe.")
            print("  When done:  python3 bringup/approach_target.py --capture "
                  f"{a.capture} --retreat-only --go")
            return 0
        print("\nretreating to start...")
        if not arm.error_code:
            arm.set_position(x=start_xyz[0]*1000, y=start_xyz[1]*1000, z=start_xyz[2]*1000,
                             roll=start_rpy[0], pitch=start_rpy[1], yaw=start_rpy[2],
                             speed=SPEED_MM_S, is_radian=False, wait=True)
        else:
            print(f"  arm error {arm.error_code}: NOT moving it. Clear the fault with eyes on it.")
        arm.disconnect()
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
