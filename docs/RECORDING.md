# Self-contained run recording

Use one command for every measured hardware run:

```bash
python3 bringup/run_dataset.py --scene elevator --method ours -- \
  bash bringup/elevator_route.sh
```

For troubleshooting, start recording without wrapping a command and leave it running while using
other terminals. Stop it once with Ctrl-C; it will flush the bag and build the exports:

```bash
source bringup/env.sh
python3 bringup/run_dataset.py --kind troubleshooting --scene nav_debug \
  --bag full --frames continuous
```

When the operator explicitly declares a scored trial, label it separately:

```bash
python3 bringup/run_dataset.py --kind full_trial --scene elevator --method ours \
  --bag full --frames continuous -- bash bringup/elevator_route.sh
```

Storage safeguards default to a 100 GB free-space reserve, a 500 GB maximum per run, eight hours,
4 GiB MCAP splitting, and graceful shutdown so indexes are written. Change them explicitly with
`--reserve-gb`, `--max-gb`, and `--max-hours`; zero disables the corresponding size/time ceiling.
The wrapper never deletes an earlier run. Final byte counts and initial/final free space are stored
in `audit.json`.

The wrapper does not change the route or control path. It starts `run_recorder.py` as a separate
CPU-only process at niceness 10, never subscribes to the Ouster point cloud, depth image, costmaps,
or full-rate scan, and uses depth-one best-effort subscriptions. This keeps recording load away
from the detector and control loop. A second idle-I/O, niceness-15 rosbag process records the full
ROS graph directly to MCAP. Existing `captures/` evidence is copied, never moved.

Each invocation creates `runs/<UTC>_<method>_<scene>/` containing:

- `meta.json`, `run.log`, `recorder.log`, `audit.json`
- `poses.jsonl` and `csv/trajectory.csv`: actual map-frame path at 10 Hz
- `telemetry/`: odometry, commands, safety state, global and local Nav2 plans
- `rosbag/`: every discoverable ROS topic and service event, including TF, live maps, costmaps,
  scans, point clouds, RGB-D, arm/system state and diagnostics
- `ros_graph/`: start/end topic, service, and node inventories; the audit compares every observed
  topic with the MCAP message counts instead of merely trusting that rosbag started
- `csv/global_plan_points.csv`: one row per planned path point when Nav2 publishes `/plan`
- `events.jsonl`: route milestones written through `UTP_RUN_DIR`
- `frames/`: low-rate first-person event windows, or a continuous low-rate stream
- `artifacts/`: raw VLM reasoning plus RGB/depth/scan and annotated detections from this run
- `records/`: only TrialRecord/campaign JSONL rows appended during this command
- `provenance/`: exact config, waypoint, Nav2 launch, occupancy-grid and SLAM posegraph snapshots,
  with SHA-256 hashes
- `csv/`: analysis-ready exports for trajectories, events, telemetry, plans, and trial records
- `heatmaps/`: dwell-time grid as CSV/NumPy plus a rendered heatmap PNG

For continuous low-rate first-person coverage rather than event windows:

```bash
python3 bringup/run_dataset.py --scene elevator --method ours \
  --frames continuous --fps 2 -- bash bringup/elevator_route.sh
```

After an interrupted run, regenerate CSVs and the audit with:

```bash
python3 bringup/run_export.py runs/<run-directory>
```

`audit.json` never guesses. It explicitly reports missing trajectory, map, image, odometry, Nav2
plan, VLM capture, and VLM reasoning evidence. Runs without required map/trajectory/visual evidence
are marked incomplete. Nav2-plan and VLM checks may legitimately be false when those systems were
not used, but remain visible rather than being silently treated as recorded.

Third-person video still comes from the tripod phone; copy it into the run directory as
`thirdperson.mp4`. The robot has no independent third-person camera source.

Full MCAP recording is the default. It uses 4 GiB file splitting and fast chunk compression, but
raw Ouster and RGB-D streams can still consume substantial disk and DDS bandwidth. If measured
resource checks show that full recording perturbs a run, `--bag core` excludes only those raw
high-bandwidth streams while retaining maps, costmaps, TF, scans, navigation, arm and system state.
Such a run is deliberately not marked `black_box_complete`.
