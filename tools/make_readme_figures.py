#!/usr/bin/env python3
"""Rebuild the README figures in media/ from real data.

Simulation charts: numbers typed in from the manuscript's Tables I-III and Sec. V-B (the
manuscript is the source of truth; nothing here is estimated). Hardware charts: computed from
the run records committed under runs/ and the map under maps/.

    python3 tools/make_readme_figures.py                     # charts only
    python3 tools/make_readme_figures.py --pdf paper.pdf     # also re-crop the paper figures

Needs matplotlib, numpy, pyyaml, pillow; --pdf also needs pdftoppm (poppler).
"""
import argparse, bisect, glob, json, math, os, subprocess, tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MEDIA = os.path.join(ROOT, "media")

ACCENT, BASE1, BASE2, INK, MUTED = "#1f5aa6", "#7f7f7f", "#bfbfbf", "#222222", "#666666"
plt.rcParams.update({
    "font.size": 10, "axes.titlesize": 11, "axes.labelsize": 10, "axes.edgecolor": "#444444",
    "axes.spines.top": False, "axes.spines.right": False, "axes.titleweight": "bold",
    "axes.titlelocation": "left", "figure.facecolor": "white", "savefig.facecolor": "white",
    "legend.frameon": False, "xtick.color": INK, "ytick.color": INK, "text.color": INK,
})
METHODS = ["GAIN (ours)", "Direct Spatial VLM", "Direct VLM-2Call"]
MCOL = [ACCENT, BASE1, BASE2]


def save(fig, name, dpi=150):
    fig.savefig(os.path.join(MEDIA, name), dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print("wrote", name, os.path.getsize(os.path.join(MEDIA, name)) // 1024, "KB")


def wilson(k, n, z=1.96):
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


# ---------------------------------------------------------------- simulation (manuscript)

def fig_model_sensitivity():
    # Table II: 100 long-horizon missions per model, 23-call budget.
    models = ["GPT-5.4 mini", "Grok 4.3", "Gemma 4-family"]
    succ, guard, calls = [85, 60, 18], [14, 28, 75], [13.62, 16.63, 20.65]
    col = [ACCENT, BASE1, BASE2]
    fig, (a, b) = plt.subplots(1, 2, figsize=(6.4, 2.9), gridspec_kw={"wspace": 0.55})
    y = np.arange(3)[::-1]
    a.barh(y, succ, color=col, height=0.6)
    for yi, s in zip(y, succ):
        lo, hi = wilson(s, 100)
        a.plot([lo * 100, hi * 100], [yi, yi], color=INK, lw=1)
        a.text(hi * 100 + 2, yi, f"{s}/100", va="center", fontsize=9)
    a.set_yticks(y, models); a.set_xlim(0, 115); a.set_xticks([0, 25, 50, 75, 100])
    a.set_xlabel("missions completed (%)"); a.set_title("Success")
    b.barh(y, calls, color=col, height=0.6)
    b.axvline(23, color=MUTED, ls="--", lw=1)
    b.text(22.6, 2.45, "23-call\nbudget", ha="right", va="center", fontsize=8, color=MUTED)
    for yi, c, g in zip(y, calls, guard):
        b.text(c - 0.4, yi, f"{c:.2f}", va="center", ha="right", fontsize=9, color="white")
    b.set_yticks(y, [""] * 3); b.set_xlim(0, 25)
    b.set_xlabel("mean model calls per mission"); b.set_title("Cost")
    save(fig, "fig_model_sensitivity.png")


def fig_failure_breakdown():
    # Table III: primary failure cause of the 137 failed long-horizon missions.
    models = ["GPT-5.4 mini", "Grok 4.3", "Gemma 4-family"]
    cats = {"Semantic reasoning / action selection": [5, 3, 74],
            "Target acquisition / spatial grounding": [10, 23, 5],
            "Physical execution": [0, 14, 3]}
    ccol = ["#9db4d6", ACCENT, "#0d2c57"]
    fig, ax = plt.subplots(figsize=(6.4, 2.9))
    y = np.arange(3)[::-1]
    left = np.zeros(3)
    for (lab, v), c in zip(cats.items(), ccol):
        v = np.array(v)
        ax.barh(y, v, left=left, color=c, height=0.6, label=lab)
        for yi, l, n in zip(y, left, v):
            if n >= 4:
                ax.text(l + n / 2, yi, str(n), ha="center", va="center", fontsize=9,
                        color="white" if c != ccol[0] else INK)
        left += v
    for yi, t in zip(y, left):
        ax.text(t + 1.2, yi, f"{int(t)} failed", va="center", fontsize=9, color=MUTED)
    ax.set_yticks(y, models); ax.set_xlim(0, 100)
    ax.set_xlabel("failed missions (of 100 per model)")
    ax.set_title("Where the 137 failures happen")
    ax.legend(loc="upper right", fontsize=8.5, title="primary cause", title_fontsize=8.5)
    save(fig, "fig_failure_breakdown.png")


def fig_distractor():
    # Sec. V-B: the 120 Door-only trials split 90 ordinary / 30 with a look-alike distractor.
    data = {"Ordinary control": [(85, 90), (81, 90), (83, 90)],
            "Look-alike distractor": [(24, 30), (23, 30), (23, 30)]}
    fig, ax = plt.subplots(figsize=(6.4, 3.0))
    w = 0.26
    for gi, (grp, vals) in enumerate(data.items()):
        for mi, (k, n) in enumerate(vals):
            x = gi + (mi - 1) * w
            lo, hi = wilson(k, n)
            ax.bar(x, 100 * k / n, w * 0.92, color=MCOL[mi], label=METHODS[mi] if gi == 0 else None)
            ax.errorbar(x, 100 * k / n, yerr=[[100 * (k / n - lo)], [100 * (hi - k / n)]],
                        color=INK, lw=1, capsize=3)
            ax.text(x, 100 * hi + 1.5, f"{k}/{n}", ha="center", fontsize=8)
    ax.set_xticks([0, 1], list(data)); ax.set_ylim(50, 108); ax.set_yticks([50, 60, 70, 80, 90, 100])
    ax.set_ylabel("Door-only success (%)")
    ax.set_title("Distractor robustness (Wilson 95% CI)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=3, fontsize=8.5)
    save(fig, "fig_distractor.png")


def fig_calls_vs_success():
    # Table I: success and mean foundation-model calls per trial, 120 trials per cell.
    pts = {"Door": [(109, 4.24), (104, 7.59), (106, 11.69)],
           "Elevator": [(104, 8.59), (50, 15.74), (60, 16.99)]}
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for task, mk in (("Door", "o"), ("Elevator", "s")):
        xs = [c for _, c in pts[task]]; ys = [100 * k / 120 for k, _ in pts[task]]
        for mi, (x, yv) in enumerate(zip(xs, ys)):
            ax.scatter(x, yv, s=70, marker=mk, color=MCOL[mi], edgecolor=INK, lw=0.6, zorder=2)
    for mi, m in enumerate(METHODS):
        ax.scatter([], [], s=60, color=MCOL[mi], edgecolor=INK, lw=0.6, label=m)
    ax.scatter([], [], marker="o", color="white", edgecolor=INK, label="Door task")
    ax.scatter([], [], marker="s", color="white", edgecolor=INK, label="Elevator task")
    ax.text(8, 95, "Door", ha="center", fontsize=9, color=MUTED)
    ax.text(14, 55, "Elevator: the two direct baselines\ncost ~2x the calls and fail ~half", fontsize=8.5, color=MUTED)
    ax.set_xlim(0, 20); ax.set_ylim(30, 100)
    ax.set_xlabel("mean foundation-model calls per trial (lower is cheaper)")
    ax.set_ylabel("success (%), n = 120 per point")
    ax.set_title("Success vs. model calls")
    ax.legend(loc="lower left", fontsize=8, ncol=1)
    save(fig, "fig_calls_vs_success.png")


# ---------------------------------------------------------------- hardware (runs/, maps/)

def load_legs():
    """Every leg_end in runs/*/events.jsonl; arrival distance where the run kept its waypoints."""
    legs = []
    for run in sorted(glob.glob(os.path.join(ROOT, "runs", "*", "events.jsonl"))):
        d = os.path.dirname(run)
        wp = os.path.join(d, "provenance", "maps", "waypoints.yaml")
        W = yaml.safe_load(open(wp)) if os.path.exists(wp) else {}
        P = [json.loads(l) for l in open(os.path.join(d, "poses.jsonl"))] \
            if os.path.exists(os.path.join(d, "poses.jsonl")) else []
        P = [p for p in P if "map" in p]
        ts = [p["stamp"] for p in P]
        curmap = json.load(open(os.path.join(d, "meta.json"))).get("map")
        for l in open(run):
            e = json.loads(l)
            if e["kind"] == "map_load_done":
                curmap = e["detail"]
            if e["kind"] != "leg_end":
                continue
            det = e["detail"].replace(":", " ")
            name, outcome = det.split()[0], det.split()[1]
            err = None
            w = W.get(name)
            # ponytail: localized pose vs recorded waypoint, same map frame only; not ground truth
            if outcome == "arrived" and w and P and w.get("map_name") == curmap:
                p = P[max(bisect.bisect_right(ts, e["stamp"]) - 1, 0)]["map"]
                err = math.hypot(p["x"] - w["x"], p["y"] - w["y"])
            legs.append(dict(run=os.path.basename(d), name=name, outcome=outcome, err=err))
    return legs


def fig_hw_legs():
    legs = load_legs()
    order = ["arrived", "already-there", "blocked", "timeout", "none", "rejected"]
    counts = [sum(l["outcome"] == o for l in legs) for o in order]
    labels = ["arrived", "already there", "blocked", "timeout", "no result", "rejected"]
    runs = len({l["run"] for l in legs})
    errs = [l for l in legs if l["err"] is not None]
    fig, (a, b) = plt.subplots(1, 2, figsize=(7.6, 3.2), gridspec_kw={"width_ratios": [1, 1.5], "wspace": 0.45})
    y = np.arange(len(order))[::-1]
    a.barh(y, counts, color=[ACCENT] + [BASE2] + [BASE1] * 4, height=0.6)
    for yi, c in zip(y, counts):
        a.text(c + 1, yi, str(c), va="center", fontsize=9)
    a.set_yticks(y, labels); a.set_xlim(0, max(counts) * 1.2)
    a.set_xlabel("Nav2 legs"); a.set_title(f"Leg outcomes ({len(legs)} legs, {runs} runs)")

    names = sorted({l["name"] for l in errs}, key=lambda n: (n[:2] != "f2", n))
    rng = np.random.default_rng(0)
    for i, n in enumerate(names):
        v = np.array([l["err"] for l in errs if l["name"] == n])
        b.scatter(v * 100, i + rng.uniform(-0.15, 0.15, len(v)), s=16, color=ACCENT, alpha=0.75, lw=0)
        b.plot([np.median(v) * 100] * 2, [i - 0.3, i + 0.3], color=INK, lw=1.5)
        b.text(30.5, i, f"n={len(v)}", va="center", fontsize=8, color=MUTED)
    b.axvline(15, color=MUTED, ls="--", lw=1)
    b.set_yticks(range(len(names)), [n.replace("_", " ") for n in names], fontsize=8.5)
    b.invert_yaxis(); b.set_xlim(0, 30)
    b.set_xlabel("distance from waypoint at 'arrived' (cm)")
    b.set_title(f"Arrival distance ({len(errs)} arrivals)")
    save(fig, "fig_hw_legs.png")
    within = sum(l["err"] <= 0.15 for l in errs)
    print(f"  arrivals within 0.15 m: {within}/{len(errs)}; median {np.median([l['err'] for l in errs]):.3f} m")


def fig_hw_route():
    run = os.path.join(ROOT, "runs", "20260907T235743Z_ours_floor1_arrival")
    ev = [json.loads(l) for l in open(os.path.join(run, "events.jsonl"))]
    t0 = next(e["stamp"] for e in ev if e["kind"] == "localized")
    P = [json.loads(l) for l in open(os.path.join(run, "poses.jsonl"))]
    P = [p for p in P if "map" in p and p["stamp"] >= t0]
    xy = np.array([[p["map"]["x"], p["map"]["y"]] for p in P])
    t = np.array([p["stamp"] - t0 for p in P])
    W = yaml.safe_load(open(os.path.join(run, "provenance", "maps", "waypoints.yaml")))
    meta = yaml.safe_load(open(os.path.join(ROOT, "maps", "floor1.yaml")))
    img = np.array(Image.open(os.path.join(ROOT, "maps", meta["image"])))
    res, (ox, oy, _) = meta["resolution"], meta["origin"]
    h, w = img.shape
    grid = np.where(img < 100, 0.15, np.where(img > 230, 1.0, 0.88))  # occupied / free / unknown

    fig, ax = plt.subplots(figsize=(7.6, 4.0))
    ax.imshow(grid, cmap="gray", vmin=0, vmax=1, extent=[ox, ox + w * res, oy, oy + h * res],
              interpolation="nearest")
    sc = ax.scatter(xy[:, 0], xy[:, 1], c=t, cmap=matplotlib.colors.LinearSegmentedColormap.from_list("g", ["#a9c4e8", ACCENT, "#0d2c57"]), vmin=0, vmax=t.max(), s=4, lw=0)
    for e in ev:
        if e["stamp"] < t0:
            continue
        i = min(bisect.bisect_left(t, e["stamp"] - t0), len(xy) - 1)
        lab = {"localized": "localized (start)",
               "press_creep": "reach refused at 0.94 m,\nbase creeps 0.19 m, press lands",
               "mission_complete": "mission complete"}.get(e["kind"])
        if lab:
            ax.scatter(*xy[i], s=36, color="white", edgecolor=INK, zorder=3)
            off = {"localized": (6, -22), "press_creep": (-40, -42), "mission_complete": (-40, -26)}[e["kind"]]
            ax.annotate(lab, xy[i], xytext=off, textcoords="offset points", fontsize=8,
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85))
    for n in ("f1_ada_button", "f1_outside"):
        ax.scatter(W[n]["x"], W[n]["y"], marker="x", s=50, color="#c0392b", zorder=4)
        ax.annotate({"f1_ada_button": "ADA-plate waypoint", "f1_outside": "exit waypoint"}[n],
                    (W[n]["x"], W[n]["y"]), xytext=(-40, 9), textcoords="offset points",
                    fontsize=8, color="#c0392b",
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85))
    pad = 2.0
    ax.set_xlim(xy[:, 0].min() - pad, xy[:, 0].max() + pad)
    ax.set_ylim(xy[:, 1].min() - pad, xy[:, 1].max() + pad)
    ax.set_aspect("equal"); ax.set_xlabel("map x (m)"); ax.set_ylabel("map y (m)")
    ax.set_title(f"Floor-1 run after a lift ride, 2026-09-07 ({len(xy)} poses, {t.max():.0f} s)")
    fig.colorbar(sc, ax=ax, fraction=0.03, pad=0.02).set_label("s since localized")
    save(fig, "fig_hw_route_floor1.png")


def fig_hw_timeline():
    # Dated headings in EXPERIMENT_LOG.md and docs/PIPELINE_STATE.md.
    ms = [("2026-08-17", "safety stack, first code"),
          ("2026-08-18", "lidar publishing /scan"),
          ("2026-08-20", "base drives on /cmd_vel"),
          ("2026-08-21", "hand-eye verified, 4.3 mm mean placement"),
          ("2026-08-25", "arm reaches a real ADA plate under vision"),
          ("2026-08-29", "first autonomous runs at the atrium doors"),
          ("2026-08-30", "Ouster OS0-128, 3-D SLAM on the robot"),
          ("2026-09-01", "first press-and-exit through the ADA door"),
          ("2026-09-03", "elevator call pressed and confirmed"),
          ("2026-09-05", "floor-1 map saved"),
          ("2026-09-07", "floor-1 arrival after a lift ride, ADA press"),
          ("2026-09-11", "new building: call, ride, in-car press; stops at FIND SELF")]
    d = np.array([np.datetime64(a) for a, _ in ms])
    day = (d - d[0]).astype(int)
    key = {"2026-09-01", "2026-09-03", "2026-09-07"}
    fig, ax = plt.subplots(figsize=(7.6, 3.9))
    y = np.arange(len(ms))[::-1]
    ax.plot(day, y, color=BASE2, lw=1, zorder=0)
    for (ds, lab), x, yi in zip(ms, day, y):
        k = ds in key
        ax.scatter(x, yi, s=34, color=ACCENT if k else BASE1, zorder=2)
        ax.text(x + 0.7, yi, lab, va="center", fontsize=8.3, color=INK if k else MUTED,
                fontweight="bold" if k else "normal")
    ax.set_yticks(y, [ds[5:] for ds, _ in ms], fontsize=8)
    ax.set_xlim(-1, day[-1] + 30); ax.set_xticks([])
    ax.spines["bottom"].set_visible(False)
    ax.set_title("Hardware bring-up, from the experiment log")
    save(fig, "fig_hw_timeline.png")


# ---------------------------------------------------------------- paper figures

# (page, x0, y0, x1, y1) in pixels at 300 dpi; crops exclude the running header and footer.
CROPS = {"paper_sim_env.jpg": (5, 230, 210, 1222, 868),
         "paper_grounding.jpg": (5, 1290, 215, 2355, 1140),
         "paper_distractor.jpg": (6, 200, 632, 1255, 950),
         "paper_long_horizon.jpg": (7, 210, 212, 2340, 1052)}


def paper_crops(pdf):
    with tempfile.TemporaryDirectory() as tmp:
        for name, (pg, *box) in CROPS.items():
            out = os.path.join(tmp, f"p{pg}")
            if not os.path.exists(out + ".png"):
                subprocess.run(["pdftoppm", "-r", "300", "-png", "-singlefile", "-f", str(pg),
                                "-l", str(pg), pdf, out], check=True)
            im = Image.open(out + ".png").convert("RGB").crop(box)
            if im.width > 1600:
                im = im.resize((1600, round(im.height * 1600 / im.width)), Image.LANCZOS)
            im.save(os.path.join(MEDIA, name), quality=85, optimize=True)
            print("wrote", name, os.path.getsize(os.path.join(MEDIA, name)) // 1024, "KB")


if __name__ == "__main__":
    lo, hi = wilson(24, 30)
    assert abs(lo - 0.627) < 0.002 and abs(hi - 0.905) < 0.002, (lo, hi)  # textbook value
    ap = argparse.ArgumentParser()
    ap.add_argument("--pdf", help="manuscript PDF, to re-crop the paper figures")
    args = ap.parse_args()
    os.makedirs(MEDIA, exist_ok=True)
    fig_model_sensitivity(); fig_failure_breakdown(); fig_distractor(); fig_calls_vs_success()
    fig_hw_legs(); fig_hw_route(); fig_hw_timeline()
    if args.pdf:
        paper_crops(args.pdf)
