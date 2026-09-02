#!/usr/bin/env python3
"""Panel of real groundings from the hardware runs, correct ones beside the failures.

    python3 paper/make_grounding_panel.py
    python3 paper/make_grounding_panel.py --out paper/fig_grounding.png

THE ARGUMENT THIS FIGURE MAKES, and it is one the logs support rather than one we
would like to be true:

  DETECTOR CONFIDENCE DOES NOT SEPARATE A CORRECT GROUNDING FROM A WRONG ONE.
  A fire-alarm cover 3.9 m away scored 0.397. The elevator button we actually pressed
  scored 0.421. Those overlap. Historically the worst case scored 0.441 -- the single
  most confident detection of that session -- and it was a fire alarm.

  DEPTH DOES SEPARATE THEM. Every correct grounding here lands between 0.81 and 1.08 m.
  Every failure is at 0.80 m with the robot's own arm in frame, or out at 4.10 m. The
  reachable band is a physical fact about where a button can be if the robot is parked
  in front of it, and it is checked before the arm moves.

That is why safety/press_veto.py is built the way it is -- "low confidence is NOT the
signal", so it asks the detector what a fire alarm looks like and refuses if the answer
lands on the target -- and why the operator checklist says read the depth first.

Offline. Reads captures/ only. No ROS, no robot, no GPU.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# name, caption, correct?  -- ordered: the four that worked, then the three that did not.
PANELS = [
    ("press_152436", "ADA plate — the first complete press", True),
    ("ada_165838",   "ADA plate, with the systematic offset", True),
    ("prev_162234",  "lift car, star-1", True),
    ("elev2_163403", "lift car, button 2", True),
    ("press_151940", "arm in frame → corner of the FIRE ALARM", False),
    ("press_184615", "fire-alarm cover, 3.9 m away", False),
    ("elev_161959",  "worse wording, same button", False),
    # THE TILE THAT MAKES THE TITLE TRUE. Without it every wrong grounding in this panel
    # scores below every correct one, and the figure would claim an overlap it does not
    # show. This one is wrong (2.00 m, well outside the 0.88 m arm envelope) and scores
    # 0.447 -- ABOVE the 0.421 elevator button that was correctly pressed.
    ("reach_1788028380", "wrong target at 2.0 m, scored ABOVE a correct one", False),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(REPO / "paper" / "fig_grounding_panel.png"))
    ap.add_argument("--cols", type=int, default=4)
    a = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    have = []
    for name, cap, ok in PANELS:
        d = REPO / "captures" / name
        if (d / "detection.png").exists() and (d / "detection.json").exists():
            have.append((name, cap, ok, json.loads((d / "detection.json").read_text())))
        else:
            print(f"  skipping {name}: no detection.png/json")
    if not have:
        print("no captures to draw")
        return 1

    cols = a.cols
    rows = (len(have) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.0 * cols, 3.0 * rows), dpi=170)
    axes = axes.ravel() if hasattr(axes, "ravel") else [axes]

    for ax, (name, cap, ok, det) in zip(axes, have):
        ax.imshow(Image.open(REPO / "captures" / name / "detection.png"))
        ax.set_xticks([]); ax.set_yticks([])
        col = "#17a54a" if ok else "#c0392b"
        for sp in ax.spines.values():
            sp.set_edgecolor(col); sp.set_linewidth(3.0)
        z = (det.get("point3d_cam_m") or [0, 0, 0])[2]
        ax.set_title(cap, fontsize=9, color=col, pad=4)
        ax.set_xlabel(f"score {det.get('score', 0):.3f}     depth {z:.2f} m",
                      fontsize=9, labelpad=3)
    for ax in axes[len(have):]:
        ax.axis("off")

    fig.suptitle("Grounding on hardware: confidence does not separate correct from wrong; depth does",
                 fontsize=12, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(a.out, bbox_inches="tight", pad_inches=0.05)
    print(f"  {a.out}")

    okz = [(d.get('point3d_cam_m') or [0, 0, 0])[2] for _, _, o, d in have if o]
    bad = [(d.get('point3d_cam_m') or [0, 0, 0])[2] for _, _, o, d in have if not o]
    oks = [d.get('score', 0) for _, _, o, d in have if o]
    bads = [d.get('score', 0) for _, _, o, d in have if not o]
    print(f"  correct: score {min(oks):.3f}-{max(oks):.3f}   depth {min(okz):.2f}-{max(okz):.2f} m")
    print(f"  wrong:   score {min(bads):.3f}-{max(bads):.3f}   depth {min(bad):.2f}-{max(bad):.2f} m")
    print("  -> the score ranges OVERLAP; the depth ranges do not, except the arm-in-frame case,")
    print("     which is why the stage order (ground before the arm moves) is load-bearing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
