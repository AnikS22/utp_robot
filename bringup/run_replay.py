#!/usr/bin/env python3
"""Replay a run as a top-down video: the map, the lidar, the plan Nav2 was following, the robot.

    python3 bringup/run_replay.py runs/<run> --map floor1
    python3 bringup/run_replay.py runs/<run> --map floor2 --fps 15

WHY, AND WHAT IT IS FOR. The first-person camera video (bringup/run_video.py) shows what the robot
SAW. It does not show what the robot BELIEVED, and on this project almost every failure lived in
the gap between the two: a pose that jumped 3.6 m while the wheels stood still, a plan routed
through a doorway the costmap had marked shut, an arm commanded at a target measured from an
origin the robot was not standing on. None of that is visible out of the front of the robot; all
of it is obvious from above.

So this draws, on one clock with the camera video and with events.jsonl:

    the frozen map          what the robot was localizing against, unchanged all session
    the lidar, in map frame every return placed by the pose the robot believed. When the belief is
                            right the scan lies on the map's walls; when it drifts, it visibly peels
                            off them, which is the single most legible failure signal this rig has
    the global plan         the route Nav2 intended at that instant, replanned as it went
    the footprint           the real 0.72 x 0.50 m rectangle, not a disc, because that is what the
                            planner was threading through 1.2 m lift doors
    the path so far         where it has actually been
    the event caption       the route's own account of what it was doing

Everything comes from what run_recorder.py already kept -- poses.jsonl, telemetry/global_plan.jsonl
and the bag's /scan_nav -- so any recorded run can be replayed after the fact.
"""
from __future__ import annotations
import argparse, bisect, json, math, sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO)); sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import cv2, numpy as np, yaml  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

FOOT = np.array([[0.36, 0.25], [0.36, -0.25], [-0.36, -0.25], [-0.36, 0.25]])


def jsonl(p: Path):
    if not p.exists():
        return []
    out = []
    for line in p.read_text(errors="replace").splitlines():
        if line.strip():
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out



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
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--scale", type=int, default=4, help="screen pixels per map cell")
    a = ap.parse_args()

    my = yaml.safe_load((REPO / "maps" / f"{a.map}.yaml").read_text())
    grid = cv2.imread(str(REPO / "maps" / my["image"] if Path(my["image"]).is_absolute()
                          else REPO / "maps" / Path(my["image"]).name), cv2.IMREAD_GRAYSCALE)
    if grid is None:
        print(f"could not read the map image for '{a.map}'", file=sys.stderr)
        return 2
    res, ox, oy = float(my["resolution"]), float(my["origin"][0]), float(my["origin"][1])
    H, W = grid.shape
    base = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
    base = cv2.resize(base, (W * a.scale, H * a.scale), interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        return (int((x - ox) / res * a.scale), int((H - 1 - (y - oy) / res) * a.scale))

    poses = jsonl(a.run / "poses.jsonl")
    if not poses:
        print("no poses.jsonl", file=sys.stderr)
        return 2
    plans = jsonl(a.run / "telemetry" / "global_plan.jsonl")
    plan_t = [p["stamp"] for p in plans]
    events = [(e["stamp"], e.get("kind", ""), e.get("detail", "")) for e in jsonl(a.run / "events.jsonl")]
    ev_t = [e[0] for e in events]

    scans = []
    bag = a.run / "rosbag"
    if bag.exists():
        rd = rosbag2_py.SequentialReader()
        rd.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
        rd.set_filter(rosbag2_py.StorageFilter(topics=["/scan_nav"]))
        while rd.has_next():
            _, data, t = rd.read_next()
            m = deserialize_message(data, LaserScan)
            r = np.asarray(m.ranges, dtype=float)
            ang = m.angle_min + np.arange(len(r)) * m.angle_increment
            ok = np.isfinite(r) & (r > m.range_min) & (r < 20.0)
            scans.append((t / 1e9, r[ok], ang[ok]))
        print(f"  {len(scans)} scans")
    scan_t = [s[0] for s in scans]

    # Drop everything before the lock -- see _first_lock.
    _lock = _first_lock([{"kind": k, "stamp": t} for t, k, _ in events])
    if _lock is not None:
        _before = len(poses)
        poses = [q for q in poses if q["stamp"] >= _lock] or poses
        if len(poses) < _before:
            print(f"  dropped {_before - len(poses)} poses from before the robot localized")

    t0, t1 = poses[0]["stamp"], poses[-1]["stamp"]
    out = a.out or (a.run / "artifacts" / "replay_topdown.mp4")
    out.parent.mkdir(parents=True, exist_ok=True)
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), a.fps,
                         (base.shape[1], base.shape[0]))
    if not vw.isOpened():
        print(f"could not open {out}", file=sys.stderr)
        return 1

    pose_t = [p["stamp"] for p in poses]
    trail = []
    n = 0
    t = t0
    while t <= t1:
        frame = base.copy()
        i = min(bisect.bisect_right(pose_t, t), len(poses) - 1)
        p = poses[i]["map"]
        trail.append(px(p["x"], p["y"]))

        # the lidar, placed by the pose the robot believed -- drift shows as the scan peeling off
        if scans:
            j = min(bisect.bisect_right(scan_t, t), len(scans) - 1)
            _, r, ang = scans[j]
            if r.size:
                c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
                xs = p["x"] + r * np.cos(ang) * c - r * np.sin(ang) * s
                ys = p["y"] + r * np.cos(ang) * s + r * np.sin(ang) * c
                for x, y in zip(xs, ys):
                    u, v = px(x, y)
                    if 0 <= u < frame.shape[1] and 0 <= v < frame.shape[0]:
                        cv2.circle(frame, (u, v), 1, (60, 170, 255), -1)

        if plan_t:
            k = bisect.bisect_right(plan_t, t) - 1
            if k >= 0 and t - plan_t[k] < 8.0:
                pts = [px(q["x"], q["y"]) for q in plans[k]["points"]]
                if len(pts) > 1:
                    cv2.polylines(frame, [np.array(pts, np.int32)], False, (255, 140, 0), 2)

        if len(trail) > 1:
            cv2.polylines(frame, [np.array(trail, np.int32)], False, (90, 200, 90), 2)

        c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
        quad = [px(p["x"] + fx * c - fy * s, p["y"] + fx * s + fy * c) for fx, fy in FOOT]
        cv2.polylines(frame, [np.array(quad, np.int32)], True, (40, 40, 230), 2)
        nose = px(p["x"] + 0.45 * c, p["y"] + 0.45 * s)
        cv2.line(frame, px(p["x"], p["y"]), nose, (40, 40, 230), 2)

        e = bisect.bisect_right(ev_t, t) - 1
        label = f"{events[e][1]} {events[e][2]}"[:78] if e >= 0 else ""
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(frame, f"t+{t - t0:6.1f}s   {label}", (8, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        vw.write(frame)
        n += 1
        t += 1.0 / a.fps
        if n % 300 == 0:
            print(f"  {n} frames ...", flush=True)

    vw.release()
    print(f"{out}  ({n} frames, {out.stat().st_size/1e6:.0f} MB, {n/a.fps:.0f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
