#!/usr/bin/env python3
"""Capture ONE hand-eye pair: arm pose and marker sighting, at the same instant.

    python3 bringup/handeye_capture.py --name pose_01
    python3 bringup/handeye_capture.py --list          # show what has been collected
    python3 bringup/handeye_capture.py --solve         # solve from everything collected

Replaces bringup/handeye_collect.py, which cannot run on this machine: it needs pyrealsense2
(installed nowhere) and cv2.aruco.ArucoDetector (OpenCV >= 4.7; the system OpenCV is 4.6.0). The
camera already works over ROS and the 4.6 functional ArUco API does the same job, so this uses
those instead of adding dependencies.

WHY SIMULTANEITY IS THE POINT
On 2026-08-21 a camera frame was compared against an arm position read minutes later, while the
arm was being moved between the two. The numbers disagreed by half a metre and the disagreement
was read as a geometry error -- the frames were fine, the two measurements simply never described
the same pose. A pair is only a pair if both halves are of the same instant, so this reads the arm
immediately before and immediately after the frame and REFUSES the pair if the arm moved in
between. It cannot be got wrong by being in a hurry.

READ-ONLY WITH RESPECT TO MOTION. This never enables servos and never commands a pose. Move the
arm by hand in manual mode, with the E-stop in reach. A script that drives the arm to ten poses
while somebody holds a marker near the tool tip is not one anybody should run.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
sys.path.insert(0, str(REPO / ".venv-arm/lib/python3.12/site-packages"))

PAIRS_DIR = REPO / "calib" / "pairs"
ARM_IP = os.environ.get("UTP_XARM_IP", "192.168.1.221")
# Arm must not have moved by more than this between the two reads bracketing the frame.
MOVE_TOL_MM = 2.0


def read_arm(arm):
    code, pos = arm.get_position(is_radian=False)
    if code != 0:
        raise RuntimeError(f"get_position failed, code={code}")
    return np.array(pos[:3], dtype=float), np.array(pos[3:6], dtype=float)


def marker_pose_pnp(corners, K, marker_m):
    """Full 6-DoF pose of the marker in the camera frame, from its four corners.

    Depth gives position only. calibrateRobotWorldHandEye needs ORIENTATION too, because
    rotation is what separates the two unknowns (where the camera is, and where the marker sits
    on the gripper). With position alone those two are entangled and no amount of data fixes it.

    solvePnP with IPPE_SQUARE is the planar-square special case: exact, no iteration, and it uses
    the marker's known physical size as the scale reference -- which is why getting `marker_m`
    right matters more than it looks. A marker declared 40 mm but printed 38 mm puts every
    translation out by 5%.
    """
    import cv2
    h = marker_m / 2.0
    # ArUco corner order is clockwise from top-left, in the marker's own plane (z out of it).
    obj = np.array([[-h, +h, 0], [+h, +h, 0], [+h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, corners.reshape(4, 2).astype(np.float64), K, None,
                                  flags=getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE))
    if not ok:
        return None, None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3)


def detect_marker(rgb, depth, K, want_id=None):
    """Marker centre in the camera optical frame, metres. Returns (id, xyz, info) or (None,...).

    Depth is a MEDIAN over a patch, not the single centre pixel: one dead pixel at the centre
    injects a metre-scale outlier into an otherwise clean set, and least squares has no defence
    against that.
    """
    import cv2
    from check_marker import detect as _detect

    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    corners, ids, dname = _detect(gray, cv2.aruco)
    if ids is None or not len(ids):
        return None, None, {}
    idx = 0
    if want_id is not None:
        m = [i for i, v in enumerate(ids.ravel()) if int(v) == want_id]
        if not m:
            return None, None, {"seen": ids.ravel().tolist()}
        idx = m[0]
    p = corners[idx].reshape(4, 2)
    u, v = float(p[:, 0].mean()), float(p[:, 1].mean())

    r = 5
    win = depth[max(0, int(v)-r):int(v)+r+1, max(0, int(u)-r):int(u)+r+1]
    win = win[np.isfinite(win)]
    if not win.size:
        return int(ids.ravel()[idx]), None, {"reason": "no valid depth at marker centre"}
    z = float(np.median(win))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    xyz = np.array([(u-cx)*z/fx, (v-cy)*z/fy, z])
    sides = [float(np.linalg.norm(p[k] - p[(k+1) % 4])) for k in range(4)]
    return int(ids.ravel()[idx]), xyz, {
        "dict": dname, "px": [u, v], "squareness": min(sides)/max(sides),
        "apparent_mm": float(np.mean(sides) * z / fx * 1000),
        "corners": p.tolist(),
    }


def capture(name: str, want_id, marker_mm):
    from xarm.wrapper import XArmAPI

    arm = XArmAPI(ARM_IP, is_radian=False, do_not_open=False)
    if arm.error_code:
        raise RuntimeError(f"arm reports error {arm.error_code}; clear it first")

    before_xyz, before_rpy = read_arm(arm)

    import subprocess
    cap_dir = REPO / "captures" / f"he_{name}"
    r = subprocess.run([sys.executable, str(REPO / "bringup" / "grab_frame.py"),
                        "--name", f"he_{name}", "--settle", "8"],
                       capture_output=True, text=True, timeout=120)
    if r.returncode != 0:
        arm.disconnect()
        raise RuntimeError(f"frame capture failed:\n{r.stderr.strip()}")

    after_xyz, _ = read_arm(arm)
    arm.disconnect()

    drift = float(np.linalg.norm(after_xyz - before_xyz))
    if drift > MOVE_TOL_MM:
        raise RuntimeError(
            f"the arm moved {drift:.1f} mm during the capture (tolerance {MOVE_TOL_MM} mm).\n"
            f"  The pair is discarded: an arm pose and a camera sighting of different instants\n"
            f"  are not a pair, and mixing them corrupts the solve invisibly.\n"
            f"  Let the arm settle and try again.")

    rgb_path = cap_dir / "rgb.png"
    import cv2
    rgb = cv2.imread(str(rgb_path))
    depth = np.load(cap_dir / "depth.npy")
    K = np.array(json.loads((cap_dir / "cam.json").read_text())["K"]).reshape(3, 3)

    mid, xyz, info = detect_marker(rgb, depth, K, want_id)
    if xyz is None:
        raise RuntimeError(f"marker not usable in this frame: {info or 'not detected'}")

    R_cm, t_cm = marker_pose_pnp(np.array(info["corners"]), K, marker_mm / 1000.0)
    if R_cm is None:
        raise RuntimeError("solvePnP failed on the marker corners")
    info["R_cam_marker"] = R_cm.tolist()
    info["t_cam_marker"] = t_cm.tolist()
    # PnP distance vs depth distance is a free consistency check on two independent sensors:
    # PnP scales from the marker's printed size, depth from stereo. They should agree closely.
    info["pnp_vs_depth_mm"] = float((np.linalg.norm(t_cm) - np.linalg.norm(xyz)) * 1000)

    arm_xyz_m = before_xyz / 1000.0          # SDK is MILLIMETRES; this stack is metres
    PAIRS_DIR.mkdir(parents=True, exist_ok=True)
    rec = {"name": name, "marker_id": mid,
           "arm_xyz_m": arm_xyz_m.tolist(), "arm_rpy_deg": before_rpy.tolist(),
           "cam_xyz_m": xyz.tolist(), "drift_mm": drift,
           "capture": str(cap_dir), **info}
    (PAIRS_DIR / f"{name}.json").write_text(json.dumps(rec, indent=2))

    print(f"pair '{name}' recorded  (arm drift {drift:.2f} mm during capture)")
    print(f"  arm  (flange, link_base) : {arm_xyz_m[0]:+.4f} {arm_xyz_m[1]:+.4f} {arm_xyz_m[2]:+.4f} m")
    print(f"  cam  (marker, optical)   : {xyz[0]:+.4f} {xyz[1]:+.4f} {xyz[2]:+.4f} m")
    print(f"  marker id {mid} via {info.get('dict')}, squareness {info.get('squareness', 0):.2f}")
    print(f"  PnP pose   (marker, optical)  : {t_cm[0]:+.4f} {t_cm[1]:+.4f} {t_cm[2]:+.4f} m")
    dv = info["pnp_vs_depth_mm"]
    print(f"  PnP vs depth range           : {dv:+.0f} mm"
          + ("   <- >30 mm apart; check the marker size and depth" if abs(dv) > 30 else "   (agree)"))
    app = info.get("apparent_mm")
    if app and marker_mm:
        err = 100 * (app - marker_mm) / marker_mm
        print(f"  apparent {app:.0f} mm vs printed {marker_mm:.0f} mm  ({err:+.0f}%)"
              + ("   <- check the print scale" if abs(err) > 15 else ""))
    if info.get("squareness", 1) < 0.75:
        print("  WARNING: very oblique. Corner estimates degrade; re-aim if you can.")
    return rec


def load_pairs():
    if not PAIRS_DIR.exists():
        return []
    return [json.loads(p.read_text()) for p in sorted(PAIRS_DIR.glob("*.json"))]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default=None)
    ap.add_argument("--id", type=int, default=None, help="only accept this marker id")
    ap.add_argument("--marker-mm", type=float, default=40.0)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--solve", action="store_true")
    a = ap.parse_args()

    if a.list or a.solve:
        pairs = load_pairs()
        if not pairs:
            print(f"no pairs in {PAIRS_DIR}")
            return 1
        print(f"{len(pairs)} pair(s) in {PAIRS_DIR}:")
        for p in pairs:
            arm, cam = p["arm_xyz_m"], p["cam_xyz_m"]
            print(f"  {p['name']:14s} arm({arm[0]:+.3f},{arm[1]:+.3f},{arm[2]:+.3f})  "
                  f"cam({cam[0]:+.3f},{cam[1]:+.3f},{cam[2]:+.3f})")
        if not a.solve:
            return 0
        from handeye import report, solve
        src = np.array([p["cam_xyz_m"] for p in pairs])
        dst = np.array([p["arm_xyz_m"] for p in pairs])
        print()
        print(report(solve(src, dst)))
        print("\nNOTE: this solves link_base <- camera_optical. The stack needs")
        print("      base_link <- camera_optical = (base_link <- link_base) @ this,")
        print("      where base_link <- link_base is the riser (CALIBRATION item 1).")
        print("      Do NOT fold the riser in here -- handeye.py's compose() keeps them apart.")
        return 0

    if not a.name:
        print("need --name (or --list / --solve)", file=sys.stderr)
        return 2
    try:
        capture(a.name, a.id, a.marker_mm)
    except Exception as e:
        print(f"\nNOT RECORDED: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
