#!/usr/bin/env python3
"""CALIBRATION item 7: is depth actually aligned to colour?

    python3 bringup/check_depth_alignment.py captures/ada_probe_01 577 467 755 648

Give it a capture dir and the RGB bounding box of something that STANDS PROUD of a flat wall
(an ADA plate is ideal: 11-15 cm, a few cm deep, on a large flat surface). It fits the local wall
plane, isolates the protruding object in the DEPTH image, and compares that object's centre with
the box you gave it from the COLOUR image. Those two centres agreeing is what "aligned" means.

Why this gate exists: misalignment presents as "grounding is right but the 3D point is wrong",
which is very easily misdiagnosed as hand-eye error -- sending you back to redo item 8 for nothing.
Ruling it out costs one frame.

ACCEPT: centre offset < 2 cm in both axes at ~1 m.

Three traps, all of which produced wrong answers before this was written:
  * A flat depth threshold assumes the wall is fronto-parallel. It usually is not -- 0.92 m one
    side and 0.87 m the other here -- and the slant alone then marks half the image as "near".
    Hence the plane fit.
  * Signs, fire alarms and door jambs also stand proud. A bounding box over all protruding pixels
    measures whichever of them is widest, not your target. Hence connected components.
  * The ROBOT'S OWN ARM appears in frame at ~0.37 m, half a metre proud. Hence the upper bound on
    protrusion: a wall control sticks out centimetres, not half a metre.
"""
from __future__ import annotations
import argparse, json, os, sys
import numpy as np

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture_dir")
    ap.add_argument("bbox", nargs=4, type=int, metavar=("X0", "Y0", "X1", "Y1"))
    ap.add_argument("--tol-m", type=float, default=0.02)
    ap.add_argument("--min-proud-m", type=float, default=0.020)
    ap.add_argument("--max-proud-m", type=float, default=0.150,
                    help="above this it is not a wall control (the arm, a person, a doorway)")
    a = ap.parse_args()

    from scipy import ndimage
    cap = os.path.abspath(a.capture_dir)
    d = np.load(os.path.join(cap, "depth.npy"))
    K = np.array(json.load(open(os.path.join(cap, "cam.json")))["K"]).reshape(3, 3)
    fx, fy = K[0, 0], K[1, 1]
    H, W = d.shape
    x0, y0, x1, y1 = a.bbox
    cxb, cyb = (x0 + x1) // 2, (y0 + y1) // 2

    wx0, wx1 = max(0, x0 - 100), min(W, x1 + 100)
    wy0, wy1 = max(0, y0 - 100), min(H, y1 + 100)
    sub = d[wy0:wy1, wx0:wx1]
    yy, xx = np.mgrid[wy0:wy1, wx0:wx1]

    ring = np.isfinite(sub) & (((xx < x0-30) | (xx > x1+30)) | ((yy < y0-30) | (yy > y1+30)))
    if ring.sum() < 500:
        print("not enough wall around the box to fit a plane", file=sys.stderr); return 1
    A = np.c_[xx[ring].ravel(), yy[ring].ravel(), np.ones(ring.sum())]
    coef, *_ = np.linalg.lstsq(A, sub[ring].ravel(), rcond=None)
    plane = coef[0]*xx + coef[1]*yy + coef[2]
    wall_rms = float(np.sqrt(((sub[ring] - plane[ring])**2).mean()))
    print(f"wall plane : {ring.sum()} px, residual RMS {wall_rms*1000:.1f} mm")

    proud = (np.isfinite(sub) & ((plane - sub) > a.min_proud_m)
             & ((plane - sub) < a.max_proud_m))
    lab, n = ndimage.label(proud)
    if n == 0:
        print("nothing protrudes -- wrong bbox, or the target is flush", file=sys.stderr); return 1
    cid = lab[cyb - wy0, cxb - wx0]
    if cid == 0:
        m = np.zeros_like(lab, bool); m[y0-wy0:y1-wy0, x0-wx0:x1-wx0] = True
        ids, counts = np.unique(lab[m & (lab > 0)], return_counts=True)
        if not len(ids):
            print("no protruding region overlaps the bbox", file=sys.stderr); return 1
        cid = ids[counts.argmax()]
    comp = lab == cid
    ys, xs = np.where(comp)
    px0, px1 = xs.min()+wx0, xs.max()+wx0
    py0, py1 = ys.min()+wy0, ys.max()+wy0
    z = float(np.nanmedian(sub[comp]))

    dxc = (px0+px1)/2 - (x0+x1)/2
    dyc = (py0+py1)/2 - (y0+y1)/2
    ex, ey = dxc*z/fx, dyc*z/fy
    print(f"depth blob : x {px0}..{px1} ({px1-px0} px)  y {py0}..{py1} ({py1-py0} px), z={z:.3f} m")
    print(f"rgb box    : x {x0}..{x1} ({x1-x0} px)  y {y0}..{y1} ({y1-y0} px)")
    print(f"offset     : dx {dxc:+.1f} px  dy {dyc:+.1f} px  =  {ex*1000:+.1f} mm  {ey*1000:+.1f} mm")
    pr = (plane - sub)[comp]
    print(f"protrusion : median {np.median(pr)*1000:.0f} mm, max {pr.max()*1000:.0f} mm")
    print(f"size       : {(px1-px0)*z/fx*100:.1f} cm from depth, {(x1-x0)*z/fx*100:.1f} cm from rgb")
    ok = abs(ex) < a.tol_m and abs(ey) < a.tol_m
    print(f"\nCALIBRATION item 7 (tol +/-{a.tol_m*100:.0f} cm): {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 2

if __name__ == "__main__":
    sys.exit(main())
