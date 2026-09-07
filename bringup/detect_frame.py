#!/usr/bin/env python3
"""Run the pipeline's REAL grounder on a captured frame. No robot, no motion, no calibration.

    ~/unlocking-the-path/env/.venv/bin/python bringup/detect_frame.py captures/ada_probe_01
    ... bringup/detect_frame.py captures/ada_probe_01 --query "the fire alarm pull station"

Answers exactly one question: does the shipped detector find the control in a REAL photograph,
and how far away does depth say it is? It does not move anything and does not need hand-eye
calibration, because the 3D point it reports is in the CAMERA frame.

Deliberately imports `utp.pipeline.grounding.decoupled.DecoupledGrounder` from the simulation
repo rather than reimplementing detection here. A demo detector written for the demo proves
nothing about the system in the paper -- the small-box preference, the lowered GDINO threshold and
the near-cluster depth logic are all load-bearing, and a reimplementation would quietly differ.
The sim repo is READ, never modified (CLAUDE.md).

WHAT A GOOD RESULT LOOKS LIKE, and why the runner-up list is printed:
a single winning box cannot distinguish "the detector could not see the plate" from "the detector
saw it and ranked a decoy higher". Those are different failures with different fixes, and only the
ranking separates them. This scene has a genuine decoy -- a red FIRE pull station beside the ADA
plate -- so the ranking is the interesting output, not the winner.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM_REPO = os.environ.get("UTP_SIM_REPO", os.path.expanduser("~/unlocking-the-path"))
sys.path.insert(0, SIM_REPO)

from utp.pipeline.grounding.decoupled import DecoupledGrounder  # noqa: E402
from utp.pipeline.types import Observation  # noqa: E402

# The query is the VLM reasoner's OUTPUT in the real pipeline -- free-form text naming the control,
# never coordinates. Hardcoding a default here stands in for the reasoner so the detector can be
# tested on its own; --query overrides it to probe how sensitive the result is to wording.
DEFAULT_QUERY = "the accessible door push button"


def load(capture_dir: str):
    with open(os.path.join(capture_dir, "cam.json")) as f:
        cam = json.load(f)
    depth = np.load(os.path.join(capture_dir, "depth.npy"))
    rgb_png = os.path.join(capture_dir, "rgb.png")
    if os.path.exists(rgb_png):
        from PIL import Image
        rgb = np.array(Image.open(rgb_png).convert("RGB"))
    else:
        rgb = np.load(os.path.join(capture_dir, "rgb.npy"))
    return rgb, depth, cam


def annotate(rgb, det, out_path, ranked):
    """Draw the winner and its runners-up. Colour encodes rank, not confidence."""
    from PIL import Image, ImageDraw
    im = Image.fromarray(rgb.copy())
    d = ImageDraw.Draw(im)
    for i, cand in enumerate(reversed(ranked[:6])):
        idx = len(ranked[:6]) - 1 - i
        b = cand.get("bbox")
        if not b:
            continue
        colour = (0, 255, 0) if idx == 0 else (255, 140, 0)
        width = 5 if idx == 0 else 2
        d.rectangle([b[0], b[1], b[2], b[3]], outline=colour, width=width)
        d.text((b[0] + 4, max(0, b[1] - 14)),
               f"#{idx} {cand.get('score', 0):.2f}", fill=colour)
    im.save(out_path)
    return out_path


def _relift(det, cand, depth, cam):
    """Move ``det`` onto ``cand``'s box and recompute its 3D point from depth.

    The grounder lifted only the box IT chose. Once geometry picks a different one, the old
    point3d belongs to the old box -- and approach_target.py reads point3d, not the bbox, so
    leaving it would aim the arm at the button we deliberately did not choose. That is the same
    class of error as the hardcoded target found on 2026-08-25: a handoff that looks right and
    points somewhere else.

    Median over the middle of the box, ignoring non-finite and zero depth, because a button's box
    catches some of the panel behind it.
    """
    import numpy as _np
    x0, y0, x1, y1 = [float(v) for v in cand["bbox"]]
    det.bbox = (x0, y0, x1, y1)
    det.score = float(cand.get("score", det.score))
    h, w = depth.shape[:2]
    cx0, cy0 = int(max(0, x0 + 0.25 * (x1 - x0))), int(max(0, y0 + 0.25 * (y1 - y0)))
    cx1, cy1 = int(min(w, x1 - 0.25 * (x1 - x0))), int(min(h, y1 - 0.25 * (y1 - y0)))
    if cx1 <= cx0 or cy1 <= cy0:
        cx0, cy0, cx1, cy1 = int(x0), int(y0), int(min(w, x1)), int(min(h, y1))
    patch = _np.asarray(depth[cy0:cy1, cx0:cx1], dtype=float)
    if patch.dtype.kind in "iu" or patch.max(initial=0) > 100:
        patch = patch / 1000.0                      # 16UC1 millimetres
    good = patch[_np.isfinite(patch) & (patch > 0.05)]
    if good.size == 0:
        det.point3d = None
        return det
    z = float(_np.median(good))
    # CENTRE ON THE BUTTON, NOT ON THE BOX.
    #
    # Everything above measures the detector's BOX: the median depth of its middle half, and the
    # geometric centre of its corners. A control is not its box. It stands PROUD of the plate it is
    # mounted on -- that is what makes it pressable -- so within the box the button's own pixels are
    # the nearest ones, and the plate behind it drags both the median depth and, when the box sits
    # loose, the lateral centre.
    #
    # Measured 2026-09-07 across three consecutive frames of the in-car floor button, 27x33 px each:
    # the protruding pixels' centroid sat 11.3, 11.0 and 10.9 mm to one side of the box centre, and
    # 6 to 9 mm nearer. Three independent frames agreeing to 0.4 mm is a measurement, not noise --
    # and it is the same correction the operator had been dialling in by hand as "+y", "-y", "a
    # touch more". This measures it instead of asking.
    #
    # GUARDED, because the same arithmetic on a bad box is nonsense: three frames where the detector
    # had caught something that was not a button (boxes 32x17, 29x15, 16x18 px) produced depth
    # shifts of 308, 156 and 45 mm. A real button is a small bump on a flat plate, so a shift beyond
    # these bounds means the box is not on one, and the box centre is kept.
    if _np.isfinite(z) and z > 0.05:
        _bx = _np.asarray(depth[int(y0):int(y1), int(x0):int(x1)], dtype=float)
        if _bx.size and (_bx.max(initial=0) > 100):
            _bx = _bx / 1000.0
        _m = _np.isfinite(_bx) & (_bx > 0.05)
        if _m.sum() >= 20:
            _near = _bx[_m] <= _np.percentile(_bx[_m], 25)      # the quarter standing proud
            _yy, _xx = _np.nonzero(_m & (_bx <= _np.percentile(_bx[_m], 25)))
            if _yy.size >= 8:
                _u = float(_xx.mean()) + x0
                _v = float(_yy.mean()) + y0
                _z = float(_np.median(_bx[_m][_near]))
                _K0 = cam["K"]
                if _K0 and isinstance(_K0[0], (list, tuple)):
                    _K0 = [q for r in _K0 for q in r]
                _du = abs(_u - (x0 + x1) / 2.0) * _z / float(_K0[0])
                _dv = abs(_v - (y0 + y1) / 2.0) * _z / float(_K0[4])
                if _du <= 0.030 and _dv <= 0.030 and abs(_z - z) <= 0.050:
                    x0, x1 = _u - (x1 - x0) / 2.0, _u + (x1 - x0) / 2.0
                    y0, y1 = _v - (y1 - y0) / 2.0, _v + (y1 - y0) / 2.0
                    z = _z
                    lift_note = (f"depth-centred on the protruding button "
                                 f"({_du*1000:.0f} mm lateral, {_dv*1000:.0f} mm vertical)")
                else:
                    lift_note = (f"box centre kept: the protruding pixels are {_du*1000:.0f}/"
                                 f"{_dv*1000:.0f} mm and {abs(_z-z)*1000:.0f} mm off, too far for a "
                                 f"button on a plate")
                print(f"  {lift_note}")
    # K is stored either flat (9 values) or as a 3x3 nested list, depending on who wrote the
    # capture. Flatten before indexing rather than assuming, which is what crashed here first.
    K = cam["K"]
    if K and isinstance(K[0], (list, tuple)):
        K = [v for row in K for v in row]
    fx, fy = float(K[0]), float(K[4])
    ppx, ppy = float(K[2]), float(K[5])
    ux, uy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    det.point3d = ((ux - ppx) * z / fx, (uy - ppy) * z / fy, z)
    return det


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture_dir")
    ap.add_argument("--query", default=DEFAULT_QUERY)
    ap.add_argument("--backend", default="gdino", choices=["gdino", "owlv2"])
    # Geometry, for panels where language cannot separate identical buttons. See the block in
    # main() for the measurements that forced this.
    ap.add_argument("--pick-from-bottom", type=int, default=0, metavar="N",
                    help="ignore the language ranking and take the Nth button-sized candidate "
                         "from the BOTTOM of the panel column (1 = lowest). 0 disables.")
    ap.add_argument("--max-button-area", type=float, default=0.6, metavar="PCT",
                    help="a candidate larger than this %% of the image is not a button (default 0.6)")
    ap.add_argument("--column-px", type=float, default=60.0,
                    help="horizontal tolerance for treating candidates as one panel column")
    ap.add_argument("--device", default="cuda:0")
    a = ap.parse_args()

    cap = a.capture_dir if os.path.isabs(a.capture_dir) else os.path.abspath(a.capture_dir)
    rgb, depth, cam = load(cap)
    print(f"frame  : {cap}")
    print(f"  rgb {rgb.shape}  depth {depth.shape}  "
          f"{100*np.isfinite(depth).mean():.1f}% valid")

    cfg = ({"kind": "grounding_dino", "model": "IDEA-Research/grounding-dino-base",
            "box_threshold": 0.30, "text_threshold": 0.25, "device": a.device}
           if a.backend == "gdino" else
           {"kind": "owlv2", "model": "google/owlv2-base-patch16-ensemble",
            "score_threshold": 0.20, "device": a.device})

    obs = Observation(rgb=rgb, depth=depth,
                      cam_info={"K": cam["K"], "frame": cam.get("frame"),
                                "width": cam["width"], "height": cam["height"]})

    g = DecoupledGrounder(cfg)
    import time
    t0 = time.monotonic()
    g._ensure_loaded()
    t_load = time.monotonic() - t0
    t0 = time.monotonic()
    det = g.locate(obs, a.query)
    t_infer = time.monotonic() - t0

    print(f"backend: {a.backend} on {g.device}   load {t_load:.1f}s   infer {t_infer*1000:.0f}ms")
    print(f"query  : {a.query!r}\n")

    lift = "depth"
    if det is not None and not det.point3d:
        # LIDAR LIFT. Depth had no valid return inside the box -- glass on hardware, or the
        # Isaac depth topic on this laptop, which publishes 100% inf (measured 2026-08-29). The
        # lidar sees the wall the control is mounted on, so: take the lidar returns, move them
        # into the CAMERA frame through the transforms grab_frame saved, keep the ones whose
        # horizontal bearing matches the bbox-centre ray, and use their median depth along the
        # optical axis as the ray's depth. Exact up to the lidar's planar sampling; the camera is
        # 0.65 m behind and 1.1 m above the lidar here, which is why this goes through TF rather
        # than assuming the two sensors share an origin.
        scan_f = os.path.join(cap, "scan.json")
        if os.path.exists(scan_f) and "T_cam_base" in cam and "T_base_lidar" in cam:
            sys.path.insert(0, str(REPO))
            from safety.lidar_lift import lift_bbox, scan_to_cam
            sc = json.load(open(scan_f))
            T = np.array(cam["T_cam_base"]) @ np.array(cam["T_base_lidar"])
            pts = scan_to_cam(sc["ranges"], sc["angle_min"], sc["angle_increment"], T)
            lf = lift_bbox(det.bbox, cam["K"], pts)
            if lf is not None:
                det.point3d = lf.point3d
                lift = f"lidar ({lf.n_returns} returns within 1.5 deg, depth {lf.depth_m:.2f} m)"
                print(f"  depth had no return in the box; LIDAR LIFT -> {lift}")
            else:
                print("  depth had no return in the box and the lidar has too few returns at "
                      "that bearing -- no lift")
    if det is None:
        print("NO DETECTION above threshold.")
        print("  This is a real result, not an error: it says the detector proposed nothing it")
        print("  was confident enough about. Try --query with different wording, or --backend")
        print("  owlv2, before concluding the control is undetectable.")
        return 2

    x0, y0, x1, y1 = det.bbox
    frac = ((x1 - x0) * (y1 - y0)) / float(rgb.shape[0] * rgb.shape[1])
    print(f"WINNER  score {det.score:.3f}")
    print(f"  bbox   ({x0:.0f}, {y0:.0f}) -> ({x1:.0f}, {y1:.0f})   "
          f"{x1-x0:.0f}x{y1-y0:.0f}px, {100*frac:.2f}% of image")
    if det.point3d:
        px, py, pz = det.point3d
        print(f"  3D     x={px:+.3f} y={py:+.3f} z={pz:+.3f} m  "
              f"in {cam.get('frame')}")
        print(f"         (ROS optical frame: +x right, +y down, +z FORWARD, so z is the "
              f"distance to the control)")
    else:
        print("  3D     none -- depth had no valid return inside the box")

    ranked = det.candidates or [{"bbox": list(det.bbox), "score": det.score}]

    # SPATIAL SELECTION, for panels where language cannot separate the buttons.
    #
    # config/floors.yaml has said this was coming since 2026-09-05: "A panel has one button per
    # floor and they are all the same blue. This query cannot tell them apart, and the grounder
    # will return whichever it likes best... the honest fix is a spatial one -- the panel's buttons
    # are vertically ordered and their order is known -- and that is a geometry question for the
    # grounder's output, not a better sentence."
    #
    # Measured 2026-09-06 in the floor-2 car: the "1" and "2" buttons are 34 px apart, identical
    # blue, and SEVEN phrasings all scored 0.28-0.35 with the ranking reshuffling between frames.
    # Twice the grounder chose "2" -- the floor the robot was already on, so the press did nothing
    # and looked like a miss. No sentence fixes that, because the two buttons are not linguistically
    # different; they are only spatially different.
    #
    # So: keep the language for finding the PANEL, and use geometry to pick the BUTTON. Candidates
    # are filtered to button-sized boxes, clustered by x to one panel column, ordered bottom-up,
    # and the Nth taken. Floor order is a property of the building, and the caller passes it.
    if a.pick_from_bottom:
        H, W = rgb.shape[0], rgb.shape[1]
        btn = [c for c in ranked
               if 0.02 <= 100 * ((c["bbox"][2]-c["bbox"][0]) * (c["bbox"][3]-c["bbox"][1]))
                                / float(H*W) <= a.max_button_area]
        if len(btn) >= a.pick_from_bottom:
            xs = sorted((c["bbox"][0]+c["bbox"][2])/2 for c in btn)
            xmed = xs[len(xs)//2]
            col = [c for c in btn if abs((c["bbox"][0]+c["bbox"][2])/2 - xmed) <= a.column_px]
            col.sort(key=lambda c: (c["bbox"][1]+c["bbox"][3])/2, reverse=True)   # lowest first
            if len(col) >= a.pick_from_bottom:
                chosen = col[a.pick_from_bottom - 1]
                print(f"\n  SPATIAL PICK: {a.pick_from_bottom} from the bottom of a "
                      f"{len(col)}-button column "
                      f"-> center=({(chosen['bbox'][0]+chosen['bbox'][2])/2:.0f},"
                      f"{(chosen['bbox'][1]+chosen['bbox'][3])/2:.0f}) score {chosen['score']:.3f}")
                print("  (language found the panel; geometry chose the button -- they are the "
                      "same blue and 34 px apart)")
                det = _relift(det, chosen, depth, cam)
                ranked = [chosen] + [c for c in ranked if c is not chosen]
            else:
                print(f"\n  SPATIAL PICK SKIPPED: only {len(col)} button(s) in the column, "
                      f"needed {a.pick_from_bottom}. Falling back to the language winner.")
        else:
            print(f"\n  SPATIAL PICK SKIPPED: only {len(btn)} button-sized candidate(s). "
                  f"Falling back to the language winner.")
    print(f"\nRANKING ({len(ranked)} candidates, best first) -- this is the evidence that")
    print("separates 'could not see it' from 'saw it and preferred a decoy':")
    for i, c in enumerate(ranked[:6]):
        b = c.get("bbox", [0, 0, 0, 0])
        f = ((b[2]-b[0]) * (b[3]-b[1])) / float(rgb.shape[0]*rgb.shape[1])
        mark = "  <- chosen" if i == 0 else ""
        print(f"  #{i}  score {c.get('score', 0):.3f}  "
              f"{b[2]-b[0]:5.0f}x{b[3]-b[1]:<5.0f}px  {100*f:5.2f}% area  "
              f"center=({(b[0]+b[2])/2:.0f},{(b[1]+b[3])/2:.0f}){mark}")

    out = annotate(rgb, det, os.path.join(cap, "detection.png"), ranked)
    print(f"\nannotated: {out}   (green = chosen, orange = runners-up)")

    # Write the result MACHINE-READABLE, next to the frame it came from.
    #
    # Added 2026-08-25 after approach_target.py was found aiming at a HARDCODED target left over
    # from an earlier session -- 222 mm from the button actually detected, on a control 170 mm
    # across. It took --capture and used it only for depth. A picture a human has to read is not
    # a handoff; this file is.
    if det.point3d:
        res = {"query": a.query, "backend": a.backend, "frame": cam.get("frame"),
               "point3d_cam_m": [float(v) for v in det.point3d],
               "bbox_px": [float(v) for v in det.bbox], "score": float(det.score),
               "lift": lift,
               "capture": os.path.basename(os.path.normpath(cap))}
        with open(os.path.join(cap, "detection.json"), "w") as fh:
            json.dump(res, fh, indent=2)
        print(f"detection: {os.path.join(cap, 'detection.json')}   (this is what the arm aims at)")
    else:
        print("NO detection.json written: no 3D point, so nothing may aim at this.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
