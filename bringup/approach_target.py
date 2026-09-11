#!/usr/bin/env python3
"""Step the arm toward a grounded target, checking joint headroom at every move.

    python3 bringup/approach_target.py --capture press_scene2 --dry-run
    python3 bringup/approach_target.py --capture press_scene2 --go --min-standoff -45

THE ARM MOVES, TOWARD A WALL. It drives the FINGERTIP to --min-standoff from the target, which
is NEGATIVE by default: a button has to be pushed, so the tip is commanded past the target plane
and contact stops it. See --tool-tip-mm for why the reference point is the tip and not the marker.

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
import os
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
    ap.add_argument("--step-mm", type=float, default=None,
                    help="approach increment. Large values (e.g. 1000) make it ONE move. The "
                         "default 60 mm exists so a fault stops at a known pose, each segment "
                         "stays near-straight in tool space instead of letting the IK route the "
                         "elbow, and joint headroom is re-checked before every commit.")
    ap.add_argument("--speed", type=float, default=None,
                    help="Cartesian speed mm/s (default 25). Raising it raises the current the "
                         "joints draw against contact, which is what error 31 reads.")
    # WHICH CORRECTION TO USE, and why this is a flag rather than one number in the file.
    # The default offset was measured against the elevator CALL button and independently improved
    # the ADA plate -- two unrelated targets, which is what makes it a property of the rig. The
    # 25 mm floor-select button inside the lift car needed a further correction that was never
    # re-checked against either of them, and for a while that in-car value was the default and was
    # therefore applied to every press this rig makes. Naming it keeps it available for the target
    # it was measured on without letting it silently follow the arm everywhere else.
    ap.add_argument("--offset-profile", default=None,
                    help="named offset from calib/handeye.json target_offset_profiles, "
                         "e.g. lift_car_select. Default: the validated global offset.")
    ap.add_argument("--min-standoff", type=float, default=-45.0,
                    help="stop with the REFERENCE POINT this far from the target, mm. May be "
                         "negative with --tool-tip-mm: the tip is then driven that far PAST the "
                         "target plane, which is what actually depresses a button.")
    # AIM THE THING THAT TOUCHES THE WALL.
    #
    # Every approach until now placed the CALIBRATION MARKER at the target. The marker is a point
    # on the flange used by the hand-eye solve; it sits 96 mm BEHIND the flange face. The gripper
    # fingertip sits ~172 mm IN FRONT of it. Along the approach axis they are 268 mm apart, so
    # "stop with the marker 30 mm from the button" commands the FINGERTIP to a point 238 mm inside
    # the wall.
    #
    # Measured 2026-09-07 on the floor-2 call button, from the log and the arm's own encoders:
    #   plan     drove the tip to 236 mm past the button (i.e. through the wall)
    #   contact  error 31 fired 293 mm short of that commanded pose, with the tip 26 mm past the
    #            wall plane, 71 mm lateral and 165 mm ABOVE the button
    # The press "made contact" and the route scored it a success. What it touched was the wall,
    # a handspan above the control. That is the whole of the "depth is way off" report -- the
    # detector was right (0.645, 28x32 px) and the depth was right (camera 0.583 m vs lidar
    # 0.626 m at the same bearing, a 4 cm difference that IS the plate). The arm was aiming the
    # wrong point at it.
    #
    # This is NOT the unresolved tcp_offset question in docs/CALIBRATION.md item 2. Nothing is
    # written to the controller and hand-eye stays flange-relative and untouched; the tool length
    # is used HERE, in this script, only to choose which point on the flange to drive at the
    # target. 172 mm need not be exact -- with a small standoff and error-31 contact detection an
    # error of a centimetre or two is absorbed. 268 mm is not.
    # DEFAULT 172, NOT 0. Aiming the marker is now known to be wrong, so it must not be what a
    # caller gets by forgetting a flag -- every route in this repo goes through here, and the fix
    # is only a fix if it propagates to all of them without each one opting in. Pass 0 to restore
    # the old marker aiming, which is useful only for reproducing pre-2026-09-07 runs.
    # DEFAULT COMES FROM THE CALIBRATION FILE, not from a literal in this script. 172 was the
    # catalogue figure for an xArm Gripper and was never verified on this build; measured against a
    # confirmed physical contact on 2026-09-11 the real distance is 392 mm, and the 220 mm
    # difference is why every press stopped short. calib/handeye.json carries the number and the
    # measurement behind it.
    def _tool_tip_default():
        env = os.environ.get("UTP_TOOL_TIP_MM")
        if env:
            return float(env)
        try:
            return float(json.loads((REPO / "calib" / "handeye.json").read_text())["tool_tip_mm"])
        except Exception:
            return 172.0
    ap.add_argument("--tool-tip-mm", type=float, default=_tool_tip_default(),
                    help="distance from the flange face to the fingertip along the tool +z axis "
                         "(default 172). The TIP is placed at the standoff. 0 = legacy marker aim.")
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
    # SYSTEMATIC TARGET CORRECTION, from calib/handeye.json. Applied to every grounded target
    # rather than nudged per button, because the bias is a property of the lift and not of any one
    # target -- and this rig presses buttons on many floors and many panels. See
    # target_offset_note in that file for how it was measured and what would invalidate it.
    _prof = (getattr(a, "offset_profile", None) or os.environ.get("UTP_OFFSET_PROFILE") or "").strip()
    _profiles = c.get("target_offset_profiles", {}) or {}
    if _prof:
        if _prof not in _profiles:
            print(f"unknown --offset-profile '{_prof}'. Known: {sorted(_profiles) or 'none'}",
                  file=sys.stderr)
            return 2
        _off = np.array(_profiles[_prof], dtype=float)
        _src = f"profile '{_prof}'"
    else:
        _off = np.array(c.get("target_offset_link_base_m", [0.0, 0.0, 0.0]), dtype=float)
        _src = "default"
    if np.any(_off):
        p_arm = p_arm + _off
        print(f"target offset    : {_off.round(4)} m from calib/handeye.json ({_src})")
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
    from safety.reach_envelope import check_before_reach, ARM_REACH_M
    # UTP_REACH_MARGIN_M: refuse a little EARLY. in_reach is `range <= 0.88` with no slack, and on
    # 2026-09-07 the ADA plate grounded at 0.8801 m: refused as "0.00 m short". A target that close
    # to the envelope edge is no better than one past it -- IK at the boundary faults just the
    # same -- so the caller can ask for a margin, and the SHORTFALL_M line tells it exactly how far
    # to move the base before trying again (mission.sh reads it).
    _rng = float(np.linalg.norm(p_arm))
    _margin = float(os.environ.get("UTP_REACH_MARGIN_M", "0") or 0)
    _ok, _why = check_before_reach(_rng + _margin)
    if not _ok:
        print(f"\nNOT REACHING: {_why}", file=sys.stderr)
        if _margin:
            print(f"  (with a {_margin*1000:.0f} mm margin; measured {_rng:.3f} m)", file=sys.stderr)
        print(f"SHORTFALL_M {max(0.0, _rng + _margin - ARM_REACH_M):.3f}")
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

    # THE REFERENCE POINT the standoff is measured from. See --tool-tip-mm.
    if a.tool_tip_mm:
        ref_off = np.array([0.0, 0.0, a.tool_tip_mm / 1000.0])
        ref_name = f"tool tip ({a.tool_tip_mm:.0f} mm past the flange)"
    else:
        ref_off = moff
        ref_name = "calibration marker"
    ref_now = R_f @ ref_off + start_xyz
    print(f"reference point : {ref_name} -> {ref_now.round(4)}")
    if a.tool_tip_mm:
        _sep = float(np.dot(R_f @ (ref_off - moff), approach))
        print(f"  the marker sits {_sep*1000:.0f} mm behind it along the approach axis; aiming the "
              f"marker instead would drive the tip that far past the target")
    step_mm = a.step_mm if a.step_mm else STEP_MM
    speed = a.speed if a.speed else SPEED_MM_S
    dist_now = float(np.dot(p_arm - ref_now, approach))
    print(f"reference point is {dist_now*1000:.0f} mm from the target along the approach axis")
    if dist_now <= a.min_standoff / 1000.0:
        print(f"\nNOT REACHING: the reference point is already {dist_now*1000:.0f} mm from the "
              f"target, at or inside the {a.min_standoff:.0f} mm standoff. Driving would push it "
              f"further in, not press. Move the base back, or re-ground.", file=sys.stderr)
        arm.disconnect()
        return 1
    # TWO MOVES: ALIGN, THEN PUSH. Never one diagonal.
    #
    # The old plan walked from wherever the wrist was straight to the final pose. When the ready
    # pose already sat near the target's plane that collapsed to a single move that was mostly
    # VERTICAL -- 31 cm down and 10 cm across -- dressed up as an approach. It reads to an operator
    # as the arm diving at the panel, and if the aim is off at all it arrives off, with no chance to
    # see it first. Worse, the along-axis distance the step count is built from goes to nearly zero
    # in that geometry, so the whole approach became one lunge.
    #
    # So the shape is fixed rather than derived: stop ONCE at UTP_ALIGN_MM in front of the target,
    # correct in height and across the plate but held back in depth, and then push STRAIGHT along
    # the approach axis to the standoff. Everything but the depth is settled before anything goes
    # near the panel. The operator asked for exactly this after a run drove into the wall beside
    # the button: "get everything but the depth done, not all one movement, 2 movements but still
    # fast".
    # UNCONDITIONAL. It was `if dist_now > align`, which skipped the alignment stop whenever the
    # wrist happened to start inside it -- and the ready pose usually does, so the very run this was
    # written for still went in one move. Skipping it is backwards: a wrist that is already near the
    # target's PLANE is exactly the case where it may be far off across the plate, because the
    # along-axis distance says nothing about height. Going to the align stop first then corrects
    # height and lateral while still 90 mm clear of the panel, which is the whole point.
    align = float(os.environ.get("UTP_ALIGN_MM", "90")) / 1000.0
    stops = [align, a.min_standoff / 1000.0]
    print(f"\n{len(stops)} steps, stopping at {a.min_standoff:.0f} mm standoff:")
    for i, s in enumerate(stops, 1):
        tgt = p_arm - approach * s
        fl = tgt - R_f @ ref_off
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
            fl = tgt - R_f @ ref_off
            print(f"\n[step {i}/{len(stops)}] marker standoff {s*1000:.0f} mm ...")
            # ASK THE IK FIRST, THEN CHECK THE ANSWER AGAINST THE OPERATOR'S LIMITS.
            #
            # safety/arm_limits.py holds J2 >= -55 deg, set in UFACTORY Studio to keep the arm out
            # of the laptop on the chassis deck. The CONTROLLER enforces it, and when it does the
            # move dies as a bare -9 (EMERGENCY_STOP) with the arm in state 4 and no statement of
            # which joint or why -- arm_limits.py's own header predicts exactly that signature.
            # It happened on 2026-09-07 20:43: the base had stopped 27 mm closer than on the runs
            # that worked, so the first Cartesian goal sat lower and nearer the body, the IK
            # answered with J2 past the limit, and the press aborted on step 1 of 4.
            #
            # The headroom check above cannot catch this: it reads the angles the arm is ALREADY
            # at, and a Cartesian goal's joint solution is not known until the IK is asked. So ask
            # it. get_inverse_kinematics moves nothing.
            _ikc, _ik = arm.get_inverse_kinematics(
                [fl[0]*1000, fl[1]*1000, fl[2]*1000,
                 start_rpy[0], start_rpy[1], start_rpy[2]], input_is_radian=False,
                return_is_radian=False)
            if _ikc == 0 and _ik is not None:
                from safety.arm_limits import violations as _limit_violations
                _bad = _limit_violations(list(_ik[:6]))
                if _bad:
                    print(f"  REFUSING step {i}: the IK solution for this pose breaks a limit set "
                          f"in UFACTORY Studio --")
                    for _b in _bad:
                        print(f"    {_b}")
                    print("  The CONTROLLER enforces that, and hitting it aborts the move as a "
                          "bare -9 with no explanation.")
                    print("  Move the BASE back a little and re-ground; do not ask the arm to fold "
                          "under itself.")
                    failed = True
                    break
            code = arm.set_position(x=fl[0]*1000, y=fl[1]*1000, z=fl[2]*1000,
                                    roll=start_rpy[0], pitch=start_rpy[1], yaw=start_rpy[2],
                                    speed=speed, is_radian=False, wait=True)
            if code != 0 or arm.error_code:
                print(f"  STOPPED: code={code} err={arm.error_code}")
                # A FAULTED ARM IS A FAILED PRESS. This used to break, retreat, and return 0, so
                # press_run.sh's `set -e` saw success and route_run printed "complete (4/4)" over
                # a ControllerError 21 -- a failed trial recorded as a successful one, which is
                # the worst outcome available to a benchmark. Observed 2026-08-29 at the doors.
                failed = True
                if code == -9:
                    print("  code -9 = EMERGENCY_STOP: the arm was stopped mid-move by the")
                    print("  CONTROLLER, not by this script. With collision detection off, that is")
                    print("  almost always a hard joint limit set in UFACTORY Studio -- see")
                    print("  safety/arm_limits.py. Move the base back and re-ground.")
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
            # MEASURING THE GAP COSTS ~8 s PER STEP, and a stepped approach has five of them.
            # That is 40 s of standing still, which is fine when a human is walking up with a
            # ruler and ruinous on a task timed by a lift door closer. Off unless asked for.
            if os.environ.get("UTP_MEASURE_GAP", "0") != "1":
                continue
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
            # RETURN THE OUTCOME, NOT 0. A `return` inside `finally` DISCARDS whatever the try
            # block was returning or raising, so this line used to report success for every held
            # press -- including 2026-09-07 20:43, where the arm emergency-stopped on step 1 of 4
            # and never came near the button, and mission.sh printed "pressed" over it. Holding is
            # about where the ARM ends up; it says nothing about whether the press worked.
            return 1 if failed else 0
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
