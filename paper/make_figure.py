#!/usr/bin/env python3
"""Turn a recorded run into the trajectory panel, plus the keyframes that go beside it.

    python3 paper/make_figure.py runs/20260902T1400Z_ours_elevator
    python3 paper/make_figure.py runs/... --map elevator --out paper/fig_elevator.png

Produces, next to the run:

    figure_trajectory.png   the map, our path in green, numbered decision markers,
                            a gold star at the goal, and a scale bar
    keyframes/NN_*.jpg      the ego-centric frame nearest each numbered marker,
                            named so the number matches the marker

WHAT IT IS FOR. The reference layout puts a trajectory on the left and, on the right,
the frames the robot actually saw at each numbered point with the VLM's decision printed
beside them. There is no baseline line in our version -- one green path showing how we
move through the SLAM environment. So the numbers are the whole argument: marker 2 has
to be the frame where the decision at marker 2 was made, and that correspondence comes
from events.jsonl, not from eyeballing timestamps afterwards.

Offline. Reads only what the run wrote plus maps/<name>.pgm. No ROS, no robot, no GPU.
Safe to run while a trial is going on.
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

# Which events get a numbered marker. Ordered as they occur; a leg END is the moment the
# robot is standing where the decision gets made, which is what the reference figure
# numbers -- not the moment it set off.
MARKER_EVENTS = ("leg_end", "press_start", "press_done", "route_complete")


def load_map(name: str):
    y = (REPO / "maps" / f"{name}.yaml").read_text()
    res = float(re.search(r"resolution:\s*([-\d.]+)", y).group(1))
    ox, oy = (float(v) for v in re.search(r"origin:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)", y).groups())
    img = re.search(r"image:\s*(\S+)", y).group(1)
    from PIL import Image
    return np.array(Image.open(REPO / "maps" / img)), res, ox, oy


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="a runs/<...> directory")
    ap.add_argument("--map", default=None, help="map name (default: from the run's meta.json)")
    ap.add_argument("--out", default=None, help="output png (default: <run>/figure_trajectory.png)")
    ap.add_argument("--pad", type=float, default=1.5, help="metres of margin around the path")
    a = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    run = Path(a.run)
    meta = json.loads((run / "meta.json").read_text())
    poses = [json.loads(l) for l in (run / "poses.jsonl").read_text().splitlines() if l.strip()]
    if not poses:
        print(f"{run}/poses.jsonl is empty -- there is no trajectory to draw. The recorder "
              f"could not see map->base_link during the run, and this cannot be recovered.")
        return 1
    evs = []
    ef = run / "events.jsonl"
    if ef.exists():
        evs = [json.loads(l) for l in ef.read_text().splitlines() if l.strip()]

    grid, res, ox, oy = load_map(a.map or meta.get("map") or "elevator")
    H, W = grid.shape
    xs = np.array([p["map"]["x"] for p in poses])
    ys = np.array([p["map"]["y"] for p in poses])
    ts = np.array([p["stamp"] for p in poses])

    # Crop the map to the path, so the figure is the scene and not the whole floor.
    x0, x1 = xs.min() - a.pad, xs.max() + a.pad
    y0, y1 = ys.min() - a.pad, ys.max() + a.pad
    extent = [ox, ox + W * res, oy, oy + H * res]

    fig, ax = plt.subplots(figsize=(7, 6), dpi=200)
    occ = np.ma.masked_where(grid != 0, grid)          # draw only the walls
    ax.imshow(np.flipud(grid), cmap="gray", extent=extent, origin="lower",
              vmin=0, vmax=255, alpha=0.85, interpolation="nearest")
    ax.plot(xs, ys, "-", color="#17a54a", lw=2.6, zorder=3, solid_capstyle="round")
    ax.plot(xs[0], ys[0], "o", color="#17a54a", ms=9, zorder=4)


    # Numbered markers. MERGE events that happen at the same place: arriving at
    # call_button and pressing the button there are two events and ONE point on the map,
    # and drawing them as two markers stacks them into an unreadable blob -- which is
    # what the first version of this did. The reference figure carries four markers for a
    # whole run, so the unit is a PLACE WHERE SOMETHING HAPPENED, not an event.
    marks = [e for e in evs if e.get("kind") in MARKER_EVENTS]
    MERGE_M = 0.35
    groups = []                     # [(pose_index, [events])]
    for e in marks:
        i = int(np.argmin(np.abs(ts - e["stamp"])))
        if groups and np.hypot(xs[i] - xs[groups[-1][0]], ys[i] - ys[groups[-1][0]]) < MERGE_M:
            groups[-1][1].append(e)
        else:
            groups.append((i, [e]))
    # NUDGE MARKERS THAT WOULD SIT ON TOP OF EACH OTHER, and draw a leader line back to
    # the true point. Not cosmetic: lift_door_reverse and lift_door are 0.16 m apart by
    # design -- one faces the doors, the other backs onto them -- so on a 1 m-scale figure
    # their markers are the same blob and the reader cannot tell which number is which.
    # The dot stays on the path; only the numbered disc moves.
    # Seed the collision set with the GOAL STAR. It is drawn last and large, so without
    # this a marker landing near the final pose disappears underneath it -- which is where
    # lift_door_reverse ends up, 0.16 m from lift_door.
    picks, placed = [], [(xs[-1], ys[-1])]
    span = max(x1 - x0, y1 - y0)
    NUDGE = 0.055 * span                     # keep the offset proportional to the crop
    for n, (i, es) in enumerate(groups, start=1):
        picks.append((n, es, i))
        px, py = xs[i], ys[i]
        for _ in range(8):
            if all(np.hypot(px - qx, py - qy) > NUDGE for qx, qy in placed):
                break
            # push along the local path NORMAL, so the label steps off the line, not along it
            j0, j1 = max(i - 3, 0), min(i + 3, len(xs) - 1)
            dx, dy = xs[j1] - xs[j0], ys[j1] - ys[j0]
            nrm = np.hypot(dx, dy) or 1.0
            px += -dy / nrm * NUDGE * 0.8
            py += dx / nrm * NUDGE * 0.8
        placed.append((px, py))
        if np.hypot(px - xs[i], py - ys[i]) > 1e-9:
            ax.plot([xs[i], px], [ys[i], py], "-", color="#17a54a", lw=1.0, zorder=5)
            ax.plot(xs[i], ys[i], "o", ms=4, color="#17a54a", zorder=5)
        ax.plot(px, py, "o", ms=17, color="#17a54a", zorder=6,
                markeredgecolor="white", markeredgewidth=1.4)
        ax.text(px, py, str(n), ha="center", va="center", color="white",
                fontsize=9, fontweight="bold", zorder=7)

    # Goal star last, so a marker landing on the final pose cannot bury it.
    ax.plot(xs[-1], ys[-1], "*", color="#f5a623", ms=24, zorder=8,
            markeredgecolor="k", markeredgewidth=0.4)

    # Scale bar, not axes -- the reference figure has no gridlines or tick labels.
    bar = 1.0
    bx, by = x0 + 0.10 * (x1 - x0), y0 + 0.07 * (y1 - y0)
    ax.plot([bx, bx + bar], [by, by], "-", color="k", lw=2.5)
    ax.text(bx + bar / 2, by + 0.02 * (y1 - y0), "1m", ha="center", va="bottom", fontsize=9)

    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)
    ax.set_aspect("equal"); ax.axis("off")
    out = Path(a.out) if a.out else run / "figure_trajectory.png"
    fig.savefig(out, bbox_inches="tight", pad_inches=0.02)
    print(f"  {out}   {len(poses)} poses, {len(picks)} markers")

    # The keyframe beside each marker: the recorded frame closest in time to that event.
    frames = sorted((run / "frames").glob("*.jpg")) if (run / "frames").exists() else []
    if frames:
        ftimes = np.array([float(f.stem) for f in frames])
        kd = run / "keyframes"; kd.mkdir(exist_ok=True)
        for n, es, _ in picks:
            e = es[-1]              # the last thing that happened here is what the frame shows
            j = int(np.argmin(np.abs(ftimes - e["stamp"])))
            dt = abs(ftimes[j] - e["stamp"])
            tag = re.sub(r"[^A-Za-z0-9]+", "_", f"{e['kind']}_{e.get('detail','')}")[:60].strip("_")
            dst = kd / f"{n:02d}_{tag}.jpg"
            shutil.copy(frames[j], dst)
            warn = "   <- NEAREST FRAME IS FAR OFF" if dt > 2.0 else ""
            print(f"  {dst.name}   {dt:.1f}s from the event{warn}")
    else:
        print("  no frames/ -- the keyframe column cannot be built for this run")

    print("\n  markers, in order (these are the numbers on the figure):")
    for n, es, _ in picks:
        what = "; ".join(f"{e['kind']}({e.get('detail','')})" for e in es)
        print(f"    {n}  {what}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
