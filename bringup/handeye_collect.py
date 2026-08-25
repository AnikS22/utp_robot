#!/usr/bin/env python3
"""SUPERSEDED 2026-08-21 -- use bringup/handeye_capture.py. This file cannot run here.

It needs `pyrealsense2` (the direct RealSense SDK) and `cv2.aruco.ArucoDetector`:

    system python3 : cv2 4.6.0 -- no ArucoDetector,  pyrealsense2 MISSING
    pipeline venv  : cv2 5.0.0 -- has ArucoDetector, pyrealsense2 MISSING

Neither interpreter satisfies both, and installing pyrealsense2 is unnecessary: the camera already
works over ROS, which is what bringup/grab_frame.py uses.

It also hardcodes DICT_4X4_50, while the marker fitted to the gripper is DICT_6X6 id 3. That
combination fails as a flat "no marker detected" on a frame where the marker is plainly visible,
which sends you off adjusting placement and lighting that were never wrong.

USE INSTEAD. The replacement needs no ruler measurement at all: cv2.calibrateRobotWorldHandEye
solves for the flange->marker offset alongside the camera pose.

    python3 bringup/handeye_auto.py --go        # drive the arm through poses, capture each
    python3 bringup/handeye_solve_rw.py         # solve
    python3 bringup/handeye_verify.py --go      # measure end-to-end placement accuracy
    python3 bringup/check_calib.py              # later: still valid? (30 s, one frame)

The code below is LEFT INTACT rather than deleted, because its frame algebra is correct and worth
reading -- especially on why the riser must be kept separate from the camera extrinsic. It simply
exits before reaching any of it. bringup/handeye.py (Kabsch) also remains correct, and
handeye_solve_rw.py --compare checks against it.
"""
import sys as _sys

print(__doc__, file=_sys.stderr)
_sys.exit(2)

# ------------------------------------------------------------------------------------------
# Original implementation below. Unreachable. Kept for its documentation of the frame algebra.
# ------------------------------------------------------------------------------------------
# (the original's `from __future__ import annotations` is dropped: it must precede all other
#  statements, and nothing here executes anyway)

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from handeye import compose, homogeneous, report, solve   # noqa: E402

REPO = Path(__file__).resolve().parent.parent
DEPTH_PATCH = 5     # half-width, px, of the median window used to read depth at the marker centre


def marker_point(color, depth_frame, intr, aruco_dict, detector_params):
    """Marker centre in the camera optical frame, metres. None if not seen or depth is invalid.

    Depth is a MEDIAN over a small patch rather than the single centre pixel: one bad pixel at the
    marker centre would otherwise inject a metre-scale outlier into an otherwise clean set, and
    least-squares has no defence against that.
    """
    import cv2
    import pyrealsense2 as rs

    corners, ids, _ = cv2.aruco.ArucoDetector(aruco_dict, detector_params).detectMarkers(
        cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
    if ids is None or not len(ids):
        return None, None
    c = corners[0].reshape(4, 2)
    u, v = float(c[:, 0].mean()), float(c[:, 1].mean())

    zs = []
    for du in range(-DEPTH_PATCH, DEPTH_PATCH + 1):
        for dv in range(-DEPTH_PATCH, DEPTH_PATCH + 1):
            z = depth_frame.get_distance(int(round(u)) + du, int(round(v)) + dv)
            if 0.1 < z < 6.0:
                zs.append(z)
    if len(zs) < 10:
        return None, (u, v)
    z = float(np.median(zs))
    return np.array(rs.rs2_deproject_pixel_to_point(intr, [u, v], z), float), (u, v)


def collect(ip: str, min_points: int) -> list[dict]:
    import cv2
    import pyrealsense2 as rs
    from xarm.wrapper import XArmAPI

    arm = XArmAPI(ip, is_radian=False, do_not_open=True)
    arm.connect()
    if not arm.connected:
        raise SystemExit(f"could not connect to the arm at {ip}")
    if arm.error_code:
        print(f"!! arm reports error {arm.error_code} — clear it at the control box before "
              f"trusting get_position()", file=sys.stderr)

    pipe, cfg = rs.pipeline(), rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aparams = cv2.aruco.DetectorParameters()

    pairs: list[dict] = []
    try:
        for _ in range(40):
            pipe.wait_for_frames()          # let auto-exposure settle before anything is measured
        print("\nMove the arm by hand to a new pose, then press ENTER to capture. 'q' to finish.")
        print("Vary DEPTH, not just image position — coplanar points fail the spread check.\n")
        while True:
            if input(f"[{len(pairs)} captured] ENTER=capture, q=done > ").strip().lower() == "q":
                break
            fs = align.process(pipe.wait_for_frames())
            dfr, cfr = fs.get_depth_frame(), fs.get_color_frame()
            intr = cfr.profile.as_video_stream_profile().intrinsics
            color = np.asanyarray(cfr.get_data())

            p_cam, uv = marker_point(color, dfr, intr, adict, aparams)
            if p_cam is None:
                print("   no marker" if uv is None else
                      f"   marker at {uv[0]:.0f},{uv[1]:.0f} but NO VALID DEPTH — move/relight")
                continue
            code, pos = arm.get_position(is_radian=False)
            if code != 0:
                print(f"   get_position failed, code={code}")
                continue
            p_arm = np.array(pos[:3], float) / 1000.0     # SDK is MILLIMETRES; we are metres

            pairs.append({"cam_m": p_cam.tolist(), "arm_m": p_arm.tolist(), "uv": list(uv)})
            print(f"   cam {p_cam[0]:+.3f} {p_cam[1]:+.3f} {p_cam[2]:+.3f} | "
                  f"arm {p_arm[0]:+.3f} {p_arm[1]:+.3f} {p_arm[2]:+.3f}")
    finally:
        pipe.stop()
        arm.disconnect()

    if len(pairs) < min_points:
        print(f"\nonly {len(pairs)} pairs; CALIBRATION.md asks for 8-10.", file=sys.stderr)
    return pairs


def emit(sol: dict, riser_m: float | None) -> None:
    print()
    print(report(sol))
    if riser_m is None:
        print("\n--riser not given: the transform above is link_base <- mast_cam_optical.")
        print("Measure the riser (CALIBRATION.md item 1) and re-run to get base_link <- mast_cam_optical.")
        return
    T = compose(homogeneous(np.eye(3), np.array([0.0, 0.0, riser_m])), sol["T_arm_cam"])
    x, y, z = T[:3, 3]
    from handeye import rotation_to_rpy
    r, p, yw = rotation_to_rpy(T[:3, :3])
    print(f"\nbase_link -> mast_cam_optical  (riser {riser_m:.3f} m folded in):")
    print(f"  xyz = [{x:.4f}, {y:.4f}, {z:.4f}]   rpy = [{r:.4f}, {p:.4f}, {yw:.4f}]  rad")
    print("\nros2 run tf2_ros static_transform_publisher \\")
    print(f"  --x {x:.4f} --y {y:.4f} --z {z:.4f} --roll {r:.4f} --pitch {p:.4f} --yaw {yw:.4f} \\")
    print("  --frame-id base_link --child-frame-id mast_cam_optical")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=os.environ.get("UTP_XARM_IP", "192.168.1.221"))
    ap.add_argument("--riser", type=float, default=None, help="base_link->link_base height, metres")
    ap.add_argument("--out", type=Path, default=REPO / "handeye_pairs.json")
    ap.add_argument("--solve", type=Path, default=None, help="re-solve a saved pairs file")
    ap.add_argument("--min-points", type=int, default=8)
    a = ap.parse_args()

    if a.solve:
        d = json.loads(a.solve.read_text())
        pairs = d["pairs"] if isinstance(d, dict) else d
    else:
        pairs = collect(a.ip, a.min_points)
        if not pairs:
            print("no pairs captured", file=sys.stderr)
            return 1
        a.out.write_text(json.dumps({"pairs": pairs}, indent=2))
        print(f"\nwrote {len(pairs)} pairs -> {a.out}")

    if len(pairs) < 3:
        print("need at least 3 pairs to solve", file=sys.stderr)
        return 1
    emit(solve([p["cam_m"] for p in pairs], [p["arm_m"] for p in pairs]), a.riser)
    return 0


if __name__ == "__main__":
    sys.exit(main())
