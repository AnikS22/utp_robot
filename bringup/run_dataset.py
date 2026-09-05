#!/usr/bin/env python3
"""Run any robot command inside a self-contained, low-interference dataset recorder.

Example:
  python3 bringup/run_dataset.py --scene elevator --method ours -- \
      bash bringup/elevator_route.sh

The child receives UTP_RUN_DIR (route events) and UTP_OUTPUT_DIR (pipeline artifacts).
The recorder is a separate nice(10) process. On exit this script snapshots provenance,
collects only capture files written during this run, exports CSVs, and writes audit.json.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def snapshot_inputs(out: Path, map_name: str) -> list[str]:
    dst = out / "provenance"
    dst.mkdir(parents=True, exist_ok=True)
    wanted = [REPO / "config", REPO / "nav2_bringup" / "ranger_nav.launch.py",
              REPO / "maps" / "waypoints.yaml"]
    if map_name:
        wanted += [REPO / "maps" / f"{map_name}{ext}"
                   for ext in (".yaml", ".pgm", ".posegraph", ".data")]
    copied = []
    for src in wanted:
        if not src.exists():
            continue
        target = dst / src.relative_to(REPO)
        if src.is_dir():
            shutil.copytree(src, target, dirs_exist_ok=True,
                            ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
            copied.extend(str(p.relative_to(out)) for p in target.rglob("*") if p.is_file())
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, target)
            copied.append(str(target.relative_to(out)))
    manifest = []
    for rel in sorted(set(copied)):
        p = out / rel
        manifest.append({"path": rel, "bytes": p.stat().st_size, "sha256": sha256(p)})
    (dst / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return copied


def collect_captures(out: Path, started: float) -> list[str]:
    """Copy capture directories touched after start; never move or modify source evidence."""
    root = REPO / "captures"
    dest = out / "artifacts" / "captures"
    found = []
    if not root.exists():
        return found
    for src in root.iterdir():
        if not src.is_dir():
            continue
        files = [p for p in src.rglob("*") if p.is_file() and p.stat().st_mtime >= started - 2]
        if not files:
            continue
        target = dest / src.name
        target.mkdir(parents=True, exist_ok=True)
        for p in files:
            q = target / p.relative_to(src)
            q.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, q)
            found.append(str(q.relative_to(out)))
    return found


def jsonl_offsets() -> dict[Path, int]:
    root = REPO / "captures"
    return {p: p.stat().st_size for p in root.glob("*.jsonl")} if root.exists() else {}


def collect_jsonl_appends(out: Path, before: dict[Path, int]) -> list[str]:
    """Extract only rows appended during this command from the shared legacy log files."""
    root = REPO / "captures"
    collected = []
    for src in root.glob("*.jsonl") if root.exists() else []:
        offset = before.get(src, 0)
        if src.stat().st_size <= offset:
            continue
        with src.open("rb") as f:
            f.seek(offset); payload = f.read()
        dst = out / "records" / src.name
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(payload)
        collected.append(str(dst.relative_to(out)))
    return collected


def snapshot_ros_graph(out: Path, label: str, env: dict) -> None:
    graph = out / "ros_graph"; graph.mkdir(exist_ok=True)
    for kind, args in (("topics", ["topic", "list", "-t"]),
                       ("services", ["service", "list", "-t"]),
                       ("nodes", ["node", "list"])):
        try:
            r = subprocess.run(["ros2", *args], cwd=REPO, env=env, capture_output=True,
                               text=True, timeout=15)
            (graph / f"{label}_{kind}.txt").write_text(r.stdout + r.stderr)
        except Exception as e:
            (graph / f"{label}_{kind}.txt").write_text(f"ERROR {type(e).__name__}: {e}\n")


def tree_bytes(path: Path) -> int:
    return sum(p.stat().st_size for p in path.rglob("*") if p.is_file()) if path.exists() else 0


def free_bytes(path: Path) -> int:
    return shutil.disk_usage(path).free


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="unknown")
    ap.add_argument("--method", default="ours")
    ap.add_argument("--kind", choices=("troubleshooting", "full_trial"),
                    default="troubleshooting",
                    help="labels the dataset; use full_trial only when the operator declares it")
    ap.add_argument("--map", default=None)
    ap.add_argument("--dir", type=Path)
    ap.add_argument("--frames", choices=("on-event", "continuous"), default="on-event")
    ap.add_argument("--fps", type=float, default=2.0)
    ap.add_argument("--bag", choices=("full", "core", "off"), default="full",
                    help="full (default) records every ROS topic/service; core excludes raw point "
                         "cloud and continuous RGB-D; off disables rosbag")
    ap.add_argument("--reserve-gb", type=float, default=100.0,
                    help="stop cleanly before filesystem free space falls below this (default 100)")
    ap.add_argument("--max-gb", type=float, default=500.0,
                    help="stop cleanly when this run reaches this size (default 500; 0 disables)")
    ap.add_argument("--max-hours", type=float, default=8.0,
                    help="stop cleanly after this duration (default 8; 0 disables)")
    ap.add_argument("command", nargs=argparse.REMAINDER)
    a = ap.parse_args()
    command = a.command[1:] if a.command[:1] == ["--"] else a.command
    started = time.time()
    legacy_offsets = jsonl_offsets()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = (a.dir or REPO / "runs" / f"{stamp}_{a.method}_{a.scene}").resolve()
    out.mkdir(parents=True, exist_ok=False)
    reserve = int(a.reserve_gb * 1_000_000_000)
    initial_free = free_bytes(out)
    if initial_free <= reserve:
        print(f"[dataset] REFUSED: {initial_free/1e9:.1f} GB free, reserve is "
              f"{a.reserve_gb:.1f} GB", file=sys.stderr)
        out.rmdir()
        return 2
    map_name = a.map
    if map_name is None:
        try:
            map_name = (REPO / "maps" / ".loaded_map").read_text().split()[0]
        except Exception:
            map_name = ""
    snapshot_inputs(out, map_name)

    env = dict(os.environ)
    env.update(UTP_RUN_DIR=str(out), UTP_OUTPUT_DIR=str(out),
               PYTHONUNBUFFERED="1")
    snapshot_ros_graph(out, "start", env)
    rec_cmd = [sys.executable, str(REPO / "bringup" / "run_recorder.py"),
               "--dir", str(out), "--scene", a.scene, "--method", a.method,
               "--frames", a.frames, "--fps", str(a.fps)]
    if map_name:
        rec_cmd += ["--map", map_name]
    rec_log = (out / "recorder.log").open("w")
    bag_log = (out / "rosbag.log").open("w")
    run_log = (out / "run.log").open("w")
    recorder = subprocess.Popen(["nice", "-n", "10", *rec_cmd], cwd=REPO, env=env,
                                stdout=rec_log, stderr=subprocess.STDOUT,
                                start_new_session=True)
    bag = None
    if a.bag != "off":
        bag_cmd = ["ros2", "bag", "record", "-a", "--include-hidden-topics",
                   "--include-unpublished-topics", "--storage", "mcap",
                   "--storage-preset-profile", "zstd_fast", "--max-bag-size", "4294967296",
                   "--disable-keyboard-controls", "--output", str(out / "rosbag"),
                   "--custom-data", f"scene={a.scene}", f"method={a.method}"]
        if a.bag == "core":
            bag_cmd += ["--exclude-regex",
                        "^/(ouster/points|mast_cam/(color|depth|aligned_depth_to_color)/.*)$"]
        # idle-class disk I/O and low CPU priority: recording may lag, but it may never pre-empt
        # control or perception. audit.json exposes missing/zero-message topics afterward.
        launcher = ["nice", "-n", "15"]
        if shutil.which("ionice"):
            launcher += ["ionice", "-c", "3"]
        bag = subprocess.Popen([*launcher, *bag_cmd], cwd=REPO, env=env,
                               stdout=bag_log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    print(f"[dataset] {out}")
    print(f"[dataset] kind={a.kind}; free={initial_free/1e9:.1f} GB; "
          f"reserve={a.reserve_gb:.1f} GB; max_run={a.max_gb:.1f} GB")
    print(f"[dataset] command: {' '.join(command) if command else '(none; Ctrl-C to stop)'}")
    rc = 125
    stop_reason = ""
    try:
        time.sleep(1.0)
        if recorder.poll() is not None or (bag is not None and bag.poll() is not None):
            print(f"[dataset] recorder failed to start; see recorder.log and rosbag.log",
                  file=sys.stderr)
            rc = 125
        elif command:
            child = subprocess.Popen(command, cwd=REPO, env=env, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1)
            assert child.stdout is not None
            def relay():
                for line in child.stdout:
                    sys.stdout.write(line); sys.stdout.flush()
                    run_log.write(line); run_log.flush()
            relay_thread = threading.Thread(target=relay, daemon=True); relay_thread.start()
            while child.poll() is None:
                used = tree_bytes(out); free = free_bytes(out); elapsed = time.time() - started
                if free <= reserve:
                    stop_reason = f"free space reached reserve ({free/1e9:.1f} GB)"
                elif a.max_gb > 0 and used >= a.max_gb * 1_000_000_000:
                    stop_reason = f"run reached max size ({used/1e9:.1f} GB)"
                elif a.max_hours > 0 and elapsed >= a.max_hours * 3600:
                    stop_reason = f"run reached max duration ({elapsed/3600:.1f} h)"
                if stop_reason:
                    print(f"[dataset] STORAGE WATCHDOG: {stop_reason}; stopping cleanly",
                          file=sys.stderr)
                    child.send_signal(signal.SIGINT); break
                time.sleep(2)
            rc = child.wait(); relay_thread.join(timeout=5)
        else:
            rc = 0
            while True:
                used = tree_bytes(out); free = free_bytes(out); elapsed = time.time() - started
                if free <= reserve:
                    stop_reason = f"free space reached reserve ({free/1e9:.1f} GB)"; break
                if a.max_gb > 0 and used >= a.max_gb * 1_000_000_000:
                    stop_reason = f"run reached max size ({used/1e9:.1f} GB)"; break
                if a.max_hours > 0 and elapsed >= a.max_hours * 3600:
                    stop_reason = f"run reached max duration ({elapsed/3600:.1f} h)"; break
                time.sleep(2)
            print(f"[dataset] STORAGE WATCHDOG: {stop_reason}; stopping cleanly",
                  file=sys.stderr)
    except KeyboardInterrupt:
        rc = 130
        if "child" in locals() and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try: child.wait(timeout=10)
            except subprocess.TimeoutExpired: child.terminate()
    finally:
        if bag is not None and bag.poll() is None:
            bag.send_signal(signal.SIGINT)  # rosbag needs SIGINT to flush MCAP metadata/indexes
            try: bag.wait(timeout=30)
            except subprocess.TimeoutExpired: bag.terminate(); bag.wait(timeout=10)
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGTERM)
            try: recorder.wait(timeout=15)
            except subprocess.TimeoutExpired: recorder.kill(); recorder.wait()
        rec_log.close(); bag_log.close(); run_log.close()

    snapshot_ros_graph(out, "end", env)

    captures = collect_captures(out, started)
    records = collect_jsonl_appends(out, legacy_offsets)
    subprocess.run([sys.executable, str(REPO / "paper" / "make_heatmaps.py"), str(out)], cwd=REPO)
    export = subprocess.run([sys.executable, str(REPO / "bringup" / "run_export.py"), str(out)],
                            cwd=REPO)
    audit_path = out / "audit.json"
    audit = json.loads(audit_path.read_text()) if audit_path.exists() else {}
    mode_complete = (audit.get("black_box_complete") if a.bag == "full"
                     else audit.get("complete"))
    audit.update(command=command, command_exit_code=rc, captures_collected=captures,
                 legacy_records_collected=records,
                 rosbag_mode=a.bag,
                 run_kind=a.kind, stop_reason=stop_reason or "command_finished_or_operator_stop",
                 storage={"initial_free_bytes": initial_free,
                          "final_free_bytes": free_bytes(out), "run_bytes": tree_bytes(out),
                          "reserve_bytes": reserve,
                          "max_run_bytes": int(a.max_gb * 1_000_000_000)},
                 dataset_complete=bool(mode_complete) and rc == 0 and export.returncode == 0)
    audit_path.write_text(json.dumps(audit, indent=2) + "\n")
    print(f"[dataset] exit={rc}; complete={audit['dataset_complete']}; {out}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
