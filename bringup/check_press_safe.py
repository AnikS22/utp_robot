#!/usr/bin/env python3
"""Ask the detector whether the thing we are about to press is a fire alarm. Refuse if it is.

    ~/unlocking-the-path/env/.venv/bin/python bringup/check_press_safe.py captures/reach_1788028445

Exit 0 = safe to press. Non-zero = do not press. Runs under the PIPELINE VENV (needs the
grounder). Fails closed on every error: if this cannot answer, nothing is pressed.

See safety/press_veto.py for why this exists -- on 2026-08-29 the grounder returned the FAU
atrium fire alarm pull station as "the accessible door push button", at the highest confidence of
the session, and every downstream check would have passed it through to the arm.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
PIPELINE = Path(os.environ.get("UTP_PIPELINE_REPO", Path.home() / "unlocking-the-path"))
sys.path.insert(0, str(PIPELINE))
from safety.press_veto import FORBIDDEN, check  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", type=Path)
    a = ap.parse_args()

    det_file = a.capture / "detection.json"
    if not det_file.exists():
        print("no detection.json; refusing", file=sys.stderr)
        return 2
    det = json.loads(det_file.read_text())
    bbox = det.get("bbox_px") or det.get("bbox") or det.get("bbox_xyxy")
    if not bbox:
        print("detection has no bbox; refusing", file=sys.stderr)
        return 2

    # Same detector, same frame, same construction as detect_frame.py -- a guard built on a
    # DIFFERENT detector would be testing that detector, not the one that will be wrong.
    try:
        import numpy as np
        from PIL import Image
        from utp.pipeline.grounding.decoupled import DecoupledGrounder
        from utp.pipeline.types import Observation
        rgb = np.asarray(Image.open(a.capture / "rgb.png").convert("RGB"))
        depth = np.load(a.capture / "depth.npy")
        cam = json.loads((a.capture / "cam.json").read_text())
        obs = Observation(rgb=rgb, depth=depth,
                          cam_info={"K": cam["K"], "frame": cam.get("frame"),
                                    "width": cam["width"], "height": cam["height"]})
        g = DecoupledGrounder({"kind": "grounding_dino",
                               "model": "IDEA-Research/grounding-dino-base",
                               "box_threshold": 0.30, "text_threshold": 0.25,
                               "device": "cuda:0"})
        g._ensure_loaded()
    except Exception as e:
        print(f"could not load the grounder ({type(e).__name__}: {e}); refusing", file=sys.stderr)
        return 2

    hits = []
    for q in FORBIDDEN:
        try:
            d = g.locate(obs, q)
            b = getattr(d, "bbox", None) if d else None
            sc = float(getattr(d, "score", 0.0) or 0.0) if d else 0.0
            hits.append((q, tuple(b) if b else None, sc))
        except Exception as e:
            print(f"forbidden-query {q!r} failed ({type(e).__name__}); refusing", file=sys.stderr)
            return 2

    ok, why = check(tuple(bbox), hits)
    print(("SAFE: " if ok else "") + why)
    for q, b, s in hits:
        print(f"  {q!r}: {'no hit' if b is None else f'{b} score {s:.3f}'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
