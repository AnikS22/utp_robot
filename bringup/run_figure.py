#!/usr/bin/env python3
"""Draw a run: the map, the path actually driven, and the moments that mattered.

    python3 bringup/run_figure.py runs/20260906T223150Z_ours_multifloor
    python3 bringup/run_figure.py <run> --map floor1 --out figure.png

WHAT IT PLOTS AND WHY EACH PIECE IS THERE. The occupancy grid is the ground the claim is about;
a trajectory floating on white space is not evidence of navigating a building. The path comes from
poses.jsonl -- map-frame, 10 Hz, written by the recorder rather than reconstructed afterwards. The
markers come from events.jsonl, which the route appends as it runs: run_event.sh's header is the
rule, "a decision with no event is a decision that cannot be drawn".

A RUN WITH NO EVENTS STILL DRAWS, and says so on the figure. Every multifloor run recorded before
2026-09-06 has an empty events.jsonl because mission.sh never sourced run_event.sh -- 2069 poses
and nothing to segment them by. Refusing to plot those would hide the gap; labelling them makes it
obvious which runs are usable as figures and which are only trajectories.

The map is read from maps/<name>.pgm + .yaml, NOT from the run's own snapshot, because the .pgm is
the map the waypoints are expressed in. If the run used a different map the figure would be a
category error, so --map must match what the run loaded; meta.json is consulted first.
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent


def rows(p: pathlib.Path):
    out = []
    if not p.exists():
        return out
    for ln in p.read_text().splitlines():
        ln = ln.strip()
        if ln:
            try:
                out.append(json.loads(ln))
            except json.JSONDecodeError:
                pass
    return out


def load_map(name: str):
    import numpy as np
    import yaml
    y = yaml.safe_load((REPO / "maps" / f"{name}.yaml").read_text())
    pgm = REPO / "maps" / y["image"] if "/" not in y["image"] else pathlib.Path(y["image"])
    if not pgm.is_absolute():
        pgm = REPO / "maps" / y["image"]
    with open(pgm, "rb") as f:
        assert f.readline().strip() == b"P5", "expected a binary PGM"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = (int(v) for v in line.split())
        int(f.readline())
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    # FLIP. A PGM stores row 0 at the TOP, while the map frame puts the origin at the BOTTOM-left
    # and imshow(origin="lower") draws row 0 at the bottom. Without this the grid is mirrored about
    # its horizontal centre -- and the tell is that the trajectory lands in grey unknown space
    # while the free space it actually drove sits somewhere else on the figure. It reads as a
    # localization problem in a picture that is simply upside down.
    return np.flipud(img), float(y["resolution"]), y["origin"][0], y["origin"][1]



# A RUN'S PATH STARTS WHEN IT KNOWS WHERE IT IS, NOT WHEN THE RECORDER STARTS.
#
# poses.jsonl is map -> base_link sampled from the moment recording begins, and that is BEFORE
# load_map and find_self have run. Until find_self agrees with itself those numbers are whatever
# the previous session left in the frame, or load_map's 0,0,0 seed -- not positions. On the
# 2026-09-07 floor-1 arrival, 601 of 1836 poses predate the lock: they begin at (6.87, 4.32),
# which is a floor-2 coordinate left over in the frame, and snap to (2.39, 0.56) the instant the
# search converges.
#
# Drawn without this filter, that snap becomes a straight diagonal line across the map -- a path
# the robot never drove, in a place it never was. The operator's words on seeing it: "that is not
# what it looked like". They were right.
def _first_lock(events):
    """Timestamp of the first `localized` event, or None if the run never got a lock."""
    for e in events:
        if (e.get("kind") if isinstance(e, dict) else e[1]) == "localized":
            return (e.get("stamp") if isinstance(e, dict) else e[0])
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--map", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    run = pathlib.Path(a.run)
    poses = rows(run / "poses.jsonl")
    events = rows(run / "events.jsonl")
    # Drop everything before the lock -- see _first_lock.
    _lock = _first_lock(events)
    if _lock is not None:
        _before = len(poses)
        poses = [q for q in poses if q.get("stamp", 0) >= _lock] or poses
        if len(poses) < _before:
            print(f"  dropped {_before - len(poses)} poses from before the robot localized")
    if not poses:
        print(f"no poses in {run}", file=sys.stderr)
        return 2

    name = a.map
    if name is None:
        try:
            name = json.loads((run / "meta.json").read_text()).get("map")
        except Exception:
            name = None
    name = name or "floor1"

    img, res, ox, oy = load_map(name)
    h, w = img.shape
    extent = [ox, ox + w * res, oy, oy + h * res]

    xs = np.array([p["map"]["x"] for p in poses])
    ys = np.array([p["map"]["y"] for p in poses])
    ts = np.array([p["stamp"] for p in poses])

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.imshow(img, cmap="gray", origin="lower", extent=extent, vmin=0, vmax=255, alpha=0.85)
    # Colour the path by elapsed time so the reader can see where it waited, not just where it went.
    sc = ax.scatter(xs, ys, c=ts - ts[0], cmap="viridis", s=6, zorder=3)
    ax.plot(xs, ys, lw=0.6, color="k", alpha=0.35, zorder=2)
    ax.plot(xs[0], ys[0], "o", ms=11, mfc="white", mec="k", zorder=5)
    ax.annotate("start", (xs[0], ys[0]), xytext=(8, 8), textcoords="offset points", zorder=6)
    ax.plot(xs[-1], ys[-1], "s", ms=11, mfc="white", mec="k", zorder=5)
    ax.annotate("end", (xs[-1], ys[-1]), xytext=(8, 8), textcoords="offset points", zorder=6)

    drawn = 0
    for e in events:
        # Place each event at the pose nearest its timestamp -- the two streams are independent,
        # so an event is located by WHEN it happened, never by assuming a shared index.
        k = int(np.argmin(np.abs(ts - e.get("stamp", 0))))
        if abs(ts[k] - e.get("stamp", 0)) > 5.0:
            continue
        drawn += 1
        ax.plot(xs[k], ys[k], "^", ms=9, color="crimson", zorder=7)
        ax.annotate(f"{drawn}. {e.get('kind','')}", (xs[k], ys[k]),
                    xytext=(6, -12), textcoords="offset points", fontsize=8,
                    color="crimson", zorder=8)

    pad = 1.5
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)")
    dur = ts[-1] - ts[0]
    title = f"{run.name}\nmap '{name}'  ·  {len(poses)} poses  ·  {dur:.0f} s"
    if not events:
        title += "\nNO EVENTS RECORDED -- trajectory only, legs cannot be labelled"
    ax.set_title(title, fontsize=10)
    fig.colorbar(sc, ax=ax, label="seconds since start", shrink=0.8)
    fig.tight_layout()
    out = a.out or str(run / "figure_trajectory.png")
    fig.savefig(out, dpi=150)
    print(f"  wrote {out}")
    print(f"  {len(poses)} poses over {dur:.0f} s, {len(events)} events ({drawn} placed on the path)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
