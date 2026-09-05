#!/usr/bin/env python3
"""Create analysis-ready dwell and event heatmaps from a recorded hardware run."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read(path):
    if not path.exists(): return []
    return [json.loads(x) for x in path.read_text().splitlines() if x.strip()]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__); ap.add_argument("run", type=Path)
    ap.add_argument("--resolution", type=float, default=0.10, help="heatmap cell size in metres")
    a = ap.parse_args(); run = a.run.resolve(); poses = read(run / "poses.jsonl")
    if not poses: return 2
    x = np.asarray([p["map"]["x"] for p in poses]); y = np.asarray([p["map"]["y"] for p in poses])
    t = np.asarray([p["stamp"] for p in poses]); dt = np.diff(t, append=t[-1])
    if len(dt) > 1: dt[-1] = float(np.median(dt[:-1]))
    pad = a.resolution; xe = np.arange(x.min()-pad, x.max()+2*pad, a.resolution)
    ye = np.arange(y.min()-pad, y.max()+2*pad, a.resolution)
    dwell, _, _ = np.histogram2d(y, x, bins=(ye, xe), weights=np.maximum(dt, 0))
    count, _, _ = np.histogram2d(y, x, bins=(ye, xe))
    out = run / "heatmaps"; out.mkdir(exist_ok=True)
    with (out / "dwell_cells.csv").open("w", newline="") as f:
        w = csv.writer(f); w.writerow(["x_m", "y_m", "samples", "dwell_s"])
        for iy, ix in zip(*np.nonzero(count)):
            w.writerow([(xe[ix]+xe[ix+1])/2, (ye[iy]+ye[iy+1])/2,
                        int(count[iy, ix]), round(float(dwell[iy, ix]), 6)])
    np.save(out / "dwell_seconds.npy", dwell)
    try:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 6), dpi=180)
        im = ax.imshow(dwell, origin="lower", extent=(xe[0], xe[-1], ye[0], ye[-1]),
                       cmap="inferno", aspect="equal")
        ax.plot(x, y, color="cyan", linewidth=.7, alpha=.7)
        fig.colorbar(im, ax=ax, label="dwell time (s)"); ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)")
        fig.savefig(out / "dwell_heatmap.png", bbox_inches="tight"); plt.close(fig)
    except Exception as e:
        (out / "render_error.txt").write_text(f"{type(e).__name__}: {e}\n")
    return 0


if __name__ == "__main__": raise SystemExit(main())
