#!/usr/bin/env python3
"""Solve link_base <- camera_optical WITHOUT knowing where the marker sits on the gripper.

    python3 bringup/handeye_solve_rw.py                 # solve from calib/pairs/*.json
    python3 bringup/handeye_solve_rw.py --compare       # also run the Kabsch solve and diff them

WHY THIS EXISTS
bringup/handeye.py solves Kabsch on point pairs, which needs the arm to REPORT the marker's
position -- i.e. someone must physically measure flange -> marker centre and set it as the TCP
offset first. That measurement is fiddly, easy to get wrong by a centimetre, and lands directly
in the press error.

cv2.calibrateRobotWorldHandEye removes the need for it. Given the marker's full 6-DoF pose in the
camera at each capture, plus the arm's pose, it solves for BOTH unknowns at once: where the camera
is, and where the marker sits on the gripper. The second one is exactly the measurement we would
otherwise take by ruler -- so the solve hands it back, and it can be checked against a ruler
afterwards rather than depended on beforehand.

EYE-TO-HAND FALLS OUT WITHOUT ANY INVERSION -- AND THE OUTPUT NAMES LIE
Our identity is   base_T_flange @ flange_T_marker  =  base_T_cam @ cam_T_marker,   i.e. A X = Z B.
OpenCV's function fits that directly, with NO inversion of either input, but its output names are
written for the eye-in-hand case and mean something else here:

    inverse of R/t_gripper2cam  is  link_base <- camera_optical  (the transform we want)
    inverse of R/t_base2world   is  flange -> marker             (the offset we would otherwise
                                                                  measure with a ruler)

Note the INVERSES. Using the matrices as returned gives answers wrong by ~0.8 m that still look
like plausible robot geometry.

This mapping was NOT derived from the documentation -- the first attempt reasoned its way to
inverting the robot poses, and was wrong. It is established by tests/test_handeye_rw.py, which
builds a synthetic rig with known ground truth and asserts recovery to sub-millimetre. Any change
to this file must keep that test passing; the convention is too easy to talk yourself into
backwards, and a sign error here produces a confident, plausible, completely wrong calibration.

ROTATION IS MANDATORY
With every pose at the same wrist orientation the two unknowns are algebraically entangled and no
quantity of data separates them -- the solver still returns something, and it is wrong. Vary the
wrist between captures. This refuses to solve if the rotations are too similar, rather than
returning a confident answer built on a degeneracy.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
PAIRS_DIR = REPO / "calib" / "pairs"

MIN_PAIRS = 5
MIN_ROT_SPREAD_DEG = 15.0     # below this the orientation set is effectively constant


def rpy_deg_to_R(rpy):
    """xArm reports roll/pitch/yaw in degrees. SDK convention is RPY about fixed X, Y, Z."""
    r, p, y = np.radians(rpy)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def rotation_spread(Rs):
    """Largest pairwise rotation angle in the set. The degeneracy detector."""
    worst = 0.0
    for i in range(len(Rs)):
        for j in range(i + 1, len(Rs)):
            worst = max(worst, rot_angle_deg(Rs[i].T @ Rs[j]))
    return worst


def load():
    if not PAIRS_DIR.exists():
        return []
    out = []
    for f in sorted(PAIRS_DIR.glob("*.json")):
        # verify_*.json are HELD OUT. They are captured by handeye_verify.py to test the solved
        # transform, and they carry the same R_cam_marker field as calibration pairs -- so a plain
        # *.json glob silently absorbs them into the fit. The calibration then "verifies" against
        # points it was fitted to, which is circular and looks excellent while proving nothing.
        # Caught 2026-08-25: six verify poses were sitting in this directory ready to be absorbed.
        if f.stem.startswith("verify"):
            print(f"  holding out {f.name}: verification pose, never a fit input")
            continue
        d = json.loads(f.read_text())
        if "R_cam_marker" not in d:
            print(f"  skipping {f.name}: no 6-DoF pose (captured before PnP was added)")
            continue
        d["_src"] = f.name
        out.append(d)
    print(f"  using: {', '.join(d['_src'] for d in out)}")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--riser-m", type=float, default=0.740,
                    help="base_link -> link_base, CALIBRATION item 1 (measured 0.740 m)")
    ap.add_argument("--compare", action="store_true", help="also run the Kabsch position-only solve")
    a = ap.parse_args()

    import cv2

    pairs = load()
    print(f"{len(pairs)} usable pair(s) in {PAIRS_DIR}")
    if len(pairs) < MIN_PAIRS:
        print(f"\nneed at least {MIN_PAIRS}. Collect more with:")
        print("  python3 bringup/handeye_capture.py --name pose_NN --id 3")
        return 1

    R_cam_marker, t_cam_marker, R_base_grip, t_base_grip = [], [], [], []
    for d in pairs:
        R_cam_marker.append(np.array(d["R_cam_marker"]))
        t_cam_marker.append(np.array(d["t_cam_marker"]).reshape(3, 1))
        R_base_grip.append(rpy_deg_to_R(d["arm_rpy_deg"]))
        t_base_grip.append(np.array(d["arm_xyz_m"]).reshape(3, 1))

    spread_arm = rotation_spread(R_base_grip)
    spread_cam = rotation_spread(R_cam_marker)
    print(f"rotation spread: arm {spread_arm:.1f} deg, camera {spread_cam:.1f} deg")
    if spread_arm < MIN_ROT_SPREAD_DEG:
        print(f"\nREFUSING: the wrist barely rotated between poses ({spread_arm:.1f} deg).")
        print("  With near-constant orientation the camera pose and the marker offset are")
        print("  algebraically entangled -- the solver would return a confident wrong answer.")
        print(f"  Re-collect with the wrist rotated at least {MIN_ROT_SPREAD_DEG:.0f} deg between poses.")
        return 2

    # Inputs go in exactly as measured -- see the header. Inverting them is the natural-looking
    # mistake and it silently produces a wrong transform.
    R_bw, t_bw, R_gc, t_gc = cv2.calibrateRobotWorldHandEye(
        R_cam_marker, t_cam_marker, R_base_grip, t_base_grip,
        method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)

    #   inv(R_gc, t_gc) is  link_base <- camera_optical
    #   inv(R_bw, t_bw) is  flange -> marker
    def _H(R, t):
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(t).reshape(3); return T

    T_arm_cam = np.linalg.inv(_H(R_gc, t_gc))
    T_flange_marker = np.linalg.inv(_H(R_bw, t_bw))
    R_gc = T_arm_cam[:3, :3]
    t_gc = T_arm_cam[:3, 3].reshape(3, 1)
    t_bw = T_flange_marker[:3, 3].reshape(3, 1)
    print("\n" + "=" * 70)
    print("link_base <- camera_optical")
    print(f"  translation  x={t_gc[0,0]:+.4f}  y={t_gc[1,0]:+.4f}  z={t_gc[2,0]:+.4f}  m")
    from handeye import rotation_to_rpy
    rpy = rotation_to_rpy(R_gc)
    print(f"  rotation rpy {np.degrees(rpy[0]):+.2f} {np.degrees(rpy[1]):+.2f} "
          f"{np.degrees(rpy[2]):+.2f} deg")

    print("\nmarker on the gripper (SOLVED, not measured -- check against a ruler):")
    print(f"  x={t_bw[0,0]*1000:+.1f}  y={t_bw[1,0]*1000:+.1f}  z={t_bw[2,0]*1000:+.1f}  mm from the flange")

    # Residual: push each camera sighting through the transform and compare with the arm.
    errs = []
    for d, Rcm, tcm in zip(pairs, R_cam_marker, t_cam_marker):
        p_arm_pred = (R_gc @ tcm + t_gc).reshape(3)
        p_marker_arm = (rpy_deg_to_R(d["arm_rpy_deg"]) @ t_bw.reshape(3)
                        + np.array(d["arm_xyz_m"]))
        errs.append(np.linalg.norm(p_arm_pred - p_marker_arm))
    errs = np.array(errs)
    print(f"\nRESIDUAL  rms {errs.mean()*1000:.1f} mm   worst {errs.max()*1000:.1f} mm")
    print(f"  CALIBRATION item 8 accept: rms < 20 mm, no point > 40 mm")
    ok = errs.mean() < 0.020 and errs.max() < 0.040
    print(f"  -> {'PASS' if ok else 'FAIL'}")
    if not ok:
        print("  If it fails only at the workspace edges, suspect the riser (item 1) or the")
        print("  marker size, not the solve.")

    T_base_arm = np.eye(4); T_base_arm[2, 3] = a.riser_m
    T_base_cam = T_base_arm @ T_arm_cam
    print("\nbase_link <- camera_optical  (riser folded in, item 1 = "
          f"{a.riser_m:.3f} m):")
    print(f"  translation  x={T_base_cam[0,3]:+.4f}  y={T_base_cam[1,3]:+.4f}  "
          f"z={T_base_cam[2,3]:+.4f}  m")
    out = REPO / "calib" / "handeye.json"
    out.write_text(json.dumps({
        "T_link_base_camera": T_arm_cam.tolist(),
        "T_base_link_camera": T_base_cam.tolist(),
        "marker_on_flange_mm": (t_bw.reshape(3) * 1000).tolist(),
        "riser_m": a.riser_m, "n_pairs": len(pairs),
        "rms_mm": float(errs.mean() * 1000), "max_mm": float(errs.max() * 1000),
        "rotation_spread_deg": spread_arm,
    }, indent=2))
    print(f"\nwritten: {out}")
    print("=" * 70)

    if a.compare:
        from handeye import report, solve
        src = np.array([d["cam_xyz_m"] for d in pairs])
        dst = np.array([d["arm_xyz_m"] for d in pairs])
        print("\n--- Kabsch (position only; assumes arm reports the MARKER) ---")
        print(report(solve(src, dst)))
        print("\nThese two solve different problems: Kabsch assumes the arm's reported point IS")
        print("the marker, which is only true once the TCP offset is set. A large disagreement")
        print("here is expected while the TCP offset is still zero -- and its size should be")
        print("close to the marker offset printed above.")
    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())
