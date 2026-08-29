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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture_dir")
    ap.add_argument("--query", default=DEFAULT_QUERY)
    ap.add_argument("--backend", default="gdino", choices=["gdino", "owlv2"])
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
