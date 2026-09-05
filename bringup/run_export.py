#!/usr/bin/env python3
"""Export one recorded run to analysis-ready CSV files and audit its completeness."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def rows(path: Path):
    if not path.exists(): return []
    out = []
    for line in path.read_text(errors="replace").splitlines():
        if line.strip():
            try: out.append(json.loads(line))
            except json.JSONDecodeError: pass
    return out


def flat(d, prefix=""):
    result = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict): result.update(flat(v, key))
        elif isinstance(v, (list, tuple)): result[key] = json.dumps(v, separators=(",", ":"))
        else: result[key] = v
    return result


def write_csv(path: Path, data):
    data = [flat(x) for x in data]
    if not data: return False
    fields = sorted({k for r in data for k in r})
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fields); w.writeheader(); w.writerows(data)
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__); ap.add_argument("run", type=Path)
    a = ap.parse_args(); run = a.run.resolve(); csvdir = run / "csv"
    outputs = []
    sources = [(run / "poses.jsonl", "trajectory.csv"), (run / "events.jsonl", "events.csv")]
    sources += [(p, p.stem + ".csv") for p in sorted((run / "telemetry").glob("*.jsonl"))] \
        if (run / "telemetry").exists() else []
    for src, name in sources:
        if write_csv(csvdir / name, rows(src)): outputs.append(str((csvdir / name).relative_to(run)))
    # Nav2 Path messages contain a list of points. Also emit one row per point so plotting a
    # planned-vs-actual path does not require parsing JSON inside a CSV cell.
    for stem in ("global_plan", "local_plan"):
        expanded = []
        for plan_i, msg in enumerate(rows(run / "telemetry" / f"{stem}.jsonl")):
            for point_i, point in enumerate(msg.get("points", [])):
                expanded.append({"stamp": msg.get("stamp"), "frame": msg.get("frame"),
                                 "plan_index": plan_i, "point_index": point_i, **point})
        name = f"{stem}_points.csv"
        if write_csv(csvdir / name, expanded): outputs.append(str((csvdir / name).relative_to(run)))
    # Trial/campaign records are already flat enough for tables; nested forensic fields remain JSON.
    for src in sorted((run / "records").glob("*.jsonl")) if (run / "records").exists() else []:
        name = src.stem + ".csv"
        if write_csv(csvdir / name, rows(src)): outputs.append(str((csvdir / name).relative_to(run)))

    poses = rows(run / "poses.jsonl")
    frames = list((run / "frames").glob("*.jpg")) if (run / "frames").exists() else []
    captures = list((run / "artifacts" / "captures").glob("*/rgb.png")) \
        if (run / "artifacts" / "captures").exists() else []
    traces = list((run / "artifacts").rglob("reasoning.jsonl")) \
        if (run / "artifacts").exists() else []
    bag_topics = {}
    bag_meta = run / "rosbag" / "metadata.yaml"
    if bag_meta.exists():
        try:
            import yaml
            info = (yaml.safe_load(bag_meta.read_text()) or {}).get("rosbag2_bagfile_information", {})
            for entry in info.get("topics_with_message_count", []):
                md = entry.get("topic_metadata", {})
                bag_topics[md.get("name", "")] = int(entry.get("message_count", 0))
        except Exception:
            pass
    expected_topics = set()
    for graph_file in (run / "ros_graph" / "start_topics.txt",
                       run / "ros_graph" / "end_topics.txt"):
        if graph_file.exists():
            for line in graph_file.read_text(errors="replace").splitlines():
                name = line.split()[0] if line.split() else ""
                if name.startswith("/") and not name.startswith("/rosbag2_recorder"):
                    expected_topics.add(name)
    missing_bag_topics = sorted(t for t in expected_topics if bag_topics.get(t, 0) <= 0)
    checks = {
        "meta": (run / "meta.json").is_file(), "trajectory": bool(poses),
        "events": bool(rows(run / "events.jsonl")), "camera_frames": bool(frames),
        "map_snapshot": bool(list((run / "provenance" / "maps").glob("*.yaml")))
            if (run / "provenance" / "maps").exists() else False,
        "odom": (run / "telemetry" / "odom.jsonl").is_file(),
        "nav_plan": (run / "telemetry" / "global_plan.jsonl").is_file(),
        "vlm_captures": bool(captures), "vlm_reasoning": bool(traces),
        "rosbag": bool(bag_topics), "live_slam_map": bag_topics.get("/map", 0) > 0,
        "tf_history": bag_topics.get("/tf", 0) > 0,
        "laser_history": bag_topics.get("/scan", 0) > 0,
        "pointcloud_history": bag_topics.get("/ouster/points", 0) > 0,
        "rgb_history": bag_topics.get("/mast_cam/color/image_raw", 0) > 0,
        "depth_history": any(bag_topics.get(t, 0) > 0 for t in (
            "/mast_cam/aligned_depth_to_color/image_raw", "/mast_cam/depth/image_rect_raw")),
        "global_costmap_history": bag_topics.get("/global_costmap/costmap", 0) > 0,
        "local_costmap_history": bag_topics.get("/local_costmap/costmap", 0) > 0,
        "heatmap_csv": (run / "heatmaps" / "dwell_cells.csv").is_file(),
        "heatmap_image": (run / "heatmaps" / "dwell_heatmap.png").is_file(),
        "all_observed_topics_bagged": bool(expected_topics) and not missing_bag_topics,
    }
    # A passive/open run legitimately has no VLM call. Keep detailed checks visible, while the
    # recorder's universal contract is provenance + trajectory + some visual evidence.
    required = ("meta", "trajectory", "map_snapshot", "rosbag", "tf_history")
    visual = checks["camera_frames"] or checks["vlm_captures"]
    complete = all(checks[k] for k in required) and visual
    black_box_required = ("rosbag", "live_slam_map", "tf_history", "laser_history",
                          "pointcloud_history", "rgb_history", "depth_history",
                          "heatmap_csv", "heatmap_image", "all_observed_topics_bagged")
    black_box_complete = complete and all(checks[k] for k in black_box_required)
    audit = {"complete": complete, "black_box_complete": black_box_complete,
             "black_box_required": list(black_box_required), "checks": checks, "counts": {
        "poses": len(poses), "events": len(rows(run / "events.jsonl")),
        "camera_frames": len(frames), "vlm_captures": len(captures),
        "reasoning_traces": len(traces)}, "csv_outputs": outputs,
        "rosbag_topics": bag_topics,
        "observed_topics": sorted(expected_topics), "missing_bag_topics": missing_bag_topics,
        "note": "nav_plan/VLM fields may be absent when those systems were not used; checks remain explicit"}
    (run / "audit.json").write_text(json.dumps(audit, indent=2) + "\n")
    print(json.dumps(audit, indent=2))
    return 0 if complete else 2


if __name__ == "__main__": raise SystemExit(main())
