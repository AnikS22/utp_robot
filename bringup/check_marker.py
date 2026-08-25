#!/usr/bin/env python3
"""Can the camera see the ArUco marker on the arm? Run before measuring anything.

    python3 bringup/check_marker.py captures/marker_test

Detects DICT_4X4_50 markers in a captured frame, reports each one's centre pixel, its 3D point
from depth, and how square-on it is to the camera. Writes an annotated image.

Run this BEFORE measuring the flange-to-marker offsets. If the camera cannot see the marker where
the arm actually works, the offsets are measurements of a placement that has to change anyway.

Uses the OLD functional cv2.aruco API (detectMarkers(img, dict, parameters=...)) because the
system OpenCV is 4.6.0 and ArucoDetector arrived in 4.7. bringup/handeye_collect.py assumes the
new API AND pyrealsense2, neither of which exists on this machine -- that is why it cannot run.
"""
from __future__ import annotations
import argparse, json, os, sys
import numpy as np


def _params(aruco):
    return (aruco.DetectorParameters_create() if hasattr(aruco, "DetectorParameters_create")
            else aruco.DetectorParameters())


def detect(gray, aruco, dict_name=None):
    """Detect markers, SEARCHING dictionaries rather than assuming one.

    Written after a real failure: the marker fitted to the gripper is DICT_6X6 id 3, the tooling
    assumed DICT_4X4_50, and the result was a flat "NO MARKER DETECTED" on a frame where the
    marker is plainly visible and well placed. That sends you off adjusting placement, lighting
    and angle -- none of which was wrong. A detector that cannot see a marker and a detector
    looking for the wrong marker produce identical output, so the tool has to rule the second one
    out itself.

    Returns (corners, ids, dict_name).
    """
    names = [dict_name] if dict_name else [n for n in dir(aruco) if n.startswith("DICT_")]
    params = _params(aruco)
    for n in names:
        d = aruco.getPredefinedDictionary(getattr(aruco, n))
        try:
            if hasattr(aruco, "ArucoDetector"):             # OpenCV >= 4.7
                corners, ids = aruco.ArucoDetector(d, params).detectMarkers(gray)[:2]
            else:                                           # OpenCV 4.6 functional API
                corners, ids = aruco.detectMarkers(gray, d, parameters=params)[:2]
        except Exception:
            continue
        if ids is not None and len(ids):
            return corners, ids, n
    return None, None, None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture_dir")
    ap.add_argument("--marker-mm", type=float, default=None,
                    help="printed size of the black square, mm. Omit to have it ESTIMATED "
                         "from depth instead of checked against.")
    ap.add_argument("--dict", default=None,
                    help="e.g. DICT_6X6_250. Omit to search all dictionaries.")
    a = ap.parse_args()

    import cv2
    cap = os.path.abspath(a.capture_dir)
    rgb = cv2.imread(os.path.join(cap, "rgb.png"))
    if rgb is None:
        print(f"no rgb.png in {cap}", file=sys.stderr); return 1
    depth = np.load(os.path.join(cap, "depth.npy"))
    K = np.array(json.load(open(os.path.join(cap, "cam.json")))["K"]).reshape(3, 3)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    corners, ids, dname = detect(cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY), cv2.aruco, a.dict)
    if ids is None or not len(ids):
        print("NO MARKER DETECTED.\n")
        print("  Most likely, in order:")
        print("   * not in the camera's view -- close in it only sees above ~0.9 m")
        print("   * facing away: it must look BACK at the camera, not forward at the wall")
        print("   * too oblique -- past roughly 60 deg off square, detection gets unreliable")
        print("   * motion blur, or the marker is curled rather than flat")
        return 2

    print(f"{len(ids)} marker(s) detected using {dname}\n")
    for c, i in zip(corners, ids.ravel()):
        p = c.reshape(4, 2)
        u, v = float(p[:, 0].mean()), float(p[:, 1].mean())
        # median over a patch: one dead pixel at the centre would inject a metre-scale outlier
        r = 5
        win = depth[max(0, int(v)-r):int(v)+r+1, max(0, int(u)-r):int(u)+r+1]
        win = win[np.isfinite(win)]
        z = float(np.median(win)) if win.size else float("nan")
        # side lengths reveal foreshortening: a square seen square-on has four equal sides
        sides = [float(np.linalg.norm(p[k] - p[(k+1) % 4])) for k in range(4)]
        squareness = min(sides) / max(sides)
        print(f"  id {i}: centre px ({u:.0f}, {v:.0f})   depth {z:.3f} m")
        if np.isfinite(z):
            print(f"         3D  x={(u-cx)*z/fx:+.3f} y={(v-cy)*z/fy:+.3f} z={z:+.3f} m (camera frame)")
            apparent = np.mean(sides) * z / fx * 1000
            if a.marker_mm:
                print(f"         apparent size {apparent:.0f} mm  (printed {a.marker_mm:.0f} mm)")
                if abs(apparent - a.marker_mm) > 0.15 * a.marker_mm:
                    print("         WARNING: >15% off -- printed at the wrong scale, or depth "
                          "is unreliable here.")
            else:
                print(f"         MEASURED size {apparent:.0f} mm across the black square")
                print("         (from depth + intrinsics; check it against a ruler)")
        else:
            print("         depth INVALID at the centre -- no return; marker may be too shiny")
        print(f"         squareness {squareness:.2f}  "
              f"({'good' if squareness > 0.75 else 'very oblique -- re-aim'})")
        cv2.polylines(rgb, [p.astype(int)], True, (0, 255, 0), 3)
        cv2.circle(rgb, (int(u), int(v)), 5, (0, 0, 255), -1)

    out = os.path.join(cap, "marker.png")
    cv2.imwrite(out, rgb)
    print(f"\nannotated: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
