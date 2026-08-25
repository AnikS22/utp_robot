#!/usr/bin/env python3
"""Is the stored hand-eye calibration still valid? One capture, ~30 seconds.

    python3 bringup/check_calib.py

Run this FIRST, before assuming a recalibration is needed. Redoing hand-eye costs ten arm poses
and a solve; checking it costs one frame.

WHAT ACTUALLY INVALIDATES THE CALIBRATION
It is a fixed transform between the camera's mount and the arm's mount. It is voided by exactly
two things:

    * the camera moving on its column
    * the arm moving on its pedestal

It is NOT voided by: driving the base, moving the arm, power-cycling anything, unplugging USB,
re-enumerating devices, or restarting every node in the stack. Those all happened repeatedly on
2026-08-21 and none of them touch the geometry. "We probably have to recalibrate" is a reasonable
worry and usually wrong -- so measure instead of assuming.

HOW IT WORKS
The arm knows where its flange is. The calibration says where the marker sits on that flange, so
together they PREDICT where the marker should appear. The camera then MEASURES where it actually
is. If the mounts have not moved, those agree to within the accuracy the calibration was verified
at; if either mount has shifted, they do not, and the size of the disagreement is roughly how far
it shifted.

THRESHOLDS come from the verified end-to-end accuracy on 2026-08-21: 4.3 mm mean, 9.7 mm worst
over six targets. So agreement under 15 mm means nothing has changed that we could detect; over
25 mm means something moved and the calibration should be redone before it is trusted.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
sys.path.insert(0, str(REPO / ".venv-arm/lib/python3.12/site-packages"))

GOOD_MM = 15.0
BAD_MM = 25.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, default=3)
    ap.add_argument("--name", default="calibcheck")
    a = ap.parse_args()

    f = REPO / "calib" / "handeye.json"
    if not f.exists():
        print("no calib/handeye.json -- nothing to check. Run handeye_auto.py then "
              "handeye_solve_rw.py.", file=sys.stderr)
        return 1
    c = json.loads(f.read_text())
    T = np.array(c["T_link_base_camera"])
    moff = np.array(c["marker_on_flange_mm"]) / 1000.0
    print(f"stored calibration: {c['n_pairs']} pairs, rms {c['rms_mm']:.1f} mm, "
          f"rotation spread {c['rotation_spread_deg']:.0f} deg")
    print(f"  camera at {T[:3,3].round(4)} m in link_base")
    print(f"  marker on flange {(moff*1000).round(1)} mm\n")

    # One synchronised pair. handeye_capture refuses if the arm moved during the frame, so a
    # stale-pose comparison -- the mistake that cost an hour on 2026-08-21 -- cannot happen here.
    r = subprocess.run([sys.executable, str(REPO / "bringup" / "handeye_capture.py"),
                        "--name", a.name, "--id", str(a.id)],
                       capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print("could not capture a pair:", file=sys.stderr)
        print((r.stderr or r.stdout).strip(), file=sys.stderr)
        print("\n  The marker must be on the gripper and visible. This checks GEOMETRY, so it "
              "cannot run\n  without a sighting -- that is not itself evidence the calibration "
              "is bad.", file=sys.stderr)
        return 1

    d = json.loads((REPO / "calib" / "pairs" / f"{a.name}.json").read_text())
    from handeye_solve_rw import rpy_deg_to_R

    predicted = rpy_deg_to_R(d["arm_rpy_deg"]) @ moff + np.array(d["arm_xyz_m"])
    measured = T[:3, :3] @ np.array(d["t_cam_marker"]) + T[:3, 3]
    err = (measured - predicted) * 1000
    mag = float(np.linalg.norm(err))

    print(f"predicted marker (arm + calibration) : {(predicted*1000).round(1)} mm")
    print(f"measured  marker (camera + calibration): {(measured*1000).round(1)} mm")
    print(f"\nDISAGREEMENT  dx={err[0]:+.1f}  dy={err[1]:+.1f}  dz={err[2]:+.1f}   "
          f"|e| = {mag:.1f} mm")

    print("\n" + "=" * 64)
    if mag < GOOD_MM:
        print(f"VALID -- {mag:.1f} mm, within the {GOOD_MM:.0f} mm the calibration was verified at.")
        print("  Nothing detectable has moved. Do not recalibrate.")
        rc = 0
    elif mag < BAD_MM:
        print(f"MARGINAL -- {mag:.1f} mm. Larger than expected but not conclusive.")
        print("  Take a second reading at a different arm pose before deciding: a single oblique")
        print("  or badly-lit sighting can produce this on its own.")
        rc = 0
    else:
        print(f"INVALID -- {mag:.1f} mm. Something moved.")
        print("  The dominant axis above hints at what: a shift mostly in one direction usually")
        print("  means a mount slipped rather than the solve being wrong.")
        print("  Recalibrate: bringup/handeye_auto.py --go, then bringup/handeye_solve_rw.py")
        rc = 2
    print("=" * 64)

    # Housekeeping: this pair is a health check, not calibration data. Leaving it in calib/pairs
    # would silently join the next solve.
    (REPO / "calib" / "pairs" / f"{a.name}.json").unlink(missing_ok=True)
    return rc


if __name__ == "__main__":
    sys.exit(main())
