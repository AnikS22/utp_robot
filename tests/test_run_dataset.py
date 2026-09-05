from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def test_export_flattens_trajectory_and_audits_run(tmp_path):
    run = tmp_path / "run"
    (run / "frames").mkdir(parents=True)
    (run / "telemetry").mkdir()
    (run / "provenance" / "maps").mkdir(parents=True)
    (run / "rosbag").mkdir()
    (run / "meta.json").write_text('{"scene":"test"}\n')
    (run / "poses.jsonl").write_text('{"stamp":1,"map":{"x":2,"y":3,"yaw":0.5}}\n')
    (run / "events.jsonl").write_text('{"stamp":1,"kind":"start","detail":""}\n')
    (run / "telemetry" / "odom.jsonl").write_text('{"stamp":1,"x":2,"y":3}\n')
    (run / "telemetry" / "global_plan.jsonl").write_text(
        '{"stamp":1,"frame":"map","points":[{"x":2,"y":3,"yaw":0.1},'
        '{"x":4,"y":5,"yaw":0.2}]}\n')
    (run / "provenance" / "maps" / "test.yaml").write_text("resolution: 0.05\n")
    (run / "frames" / "1.000.jpg").write_bytes(b"jpeg-placeholder")
    (run / "rosbag" / "metadata.yaml").write_text("""rosbag2_bagfile_information:
  topics_with_message_count:
    - topic_metadata: {name: /tf, type: tf2_msgs/msg/TFMessage}
      message_count: 4
""")

    r = subprocess.run([sys.executable, str(REPO / "bringup" / "run_export.py"), str(run)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    audit = json.loads((run / "audit.json").read_text())
    assert audit["complete"] is True
    with (run / "csv" / "trajectory.csv").open() as f:
        row = next(csv.DictReader(f))
    assert row["map.x"] == "2" and row["map.y"] == "3"
    with (run / "csv" / "global_plan_points.csv").open() as f:
        plans = list(csv.DictReader(f))
    assert [(r["x"], r["y"]) for r in plans] == [("2", "3"), ("4", "5")]


def test_export_fails_closed_when_trajectory_is_missing(tmp_path):
    run = tmp_path / "run"; run.mkdir()
    (run / "meta.json").write_text("{}\n")
    r = subprocess.run([sys.executable, str(REPO / "bringup" / "run_export.py"), str(run)],
                       capture_output=True, text=True)
    assert r.returncode == 2
    assert json.loads((run / "audit.json").read_text())["checks"]["trajectory"] is False


def test_wrapper_advertises_shared_run_environment():
    src = (REPO / "bringup" / "run_dataset.py").read_text()
    assert "UTP_RUN_DIR" in src
    assert "UTP_OUTPUT_DIR" in src
    assert "nice\", \"-n\", \"10" in src
    assert "collect_captures" in src
    assert '"ros2", "bag", "record", "-a"' in src
    assert '"ionice", "-c", "3"' in src
    assert "--reserve-gb" in src and "--max-gb" in src and "--max-hours" in src
    assert 'choices=("troubleshooting", "full_trial")' in src


def test_heatmap_writes_point_grid_and_image(tmp_path):
    run = tmp_path / "run"; run.mkdir()
    (run / "poses.jsonl").write_text(
        '\n'.join('{"stamp":%s,"map":{"x":%s,"y":0,"yaw":0}}' % (i, i/10)
                  for i in range(5)) + '\n')
    r = subprocess.run([sys.executable, str(REPO / "paper" / "make_heatmaps.py"), str(run)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert (run / "heatmaps" / "dwell_cells.csv").is_file()
    assert (run / "heatmaps" / "dwell_seconds.npy").is_file()
    assert (run / "heatmaps" / "dwell_heatmap.png").is_file()
