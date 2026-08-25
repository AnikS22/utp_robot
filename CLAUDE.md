# utp_robot — read this first

You are working on the **physical robot** for "Unlocking the Path" (ICRA 2027): an AgileX Ranger
  Mini 3.0 + uFactory xArm6 + RealSense D435 + RPLIDAR A1M8. Deadline **2026-08-25**.

**Start with `docs/AGENT_BRIEF.md`.** Then, depending on the task:

| Task | Document |
|---|---|
| Setting the laptop up from scratch | `docs/LAPTOP_SETUP.md` |
| Any device ID, pinned version, or setting | `docs/HARDWARE_SPECS.md` |
| Measuring anything on the robot | `docs/CALIBRATION.md` |
| What a "trial" is, methods, missions, metrics | `docs/PIPELINE.md` |
| The reasoning VLM endpoint and its key | `docs/LLM_ENDPOINT.md` |
| What has actually been done and observed | `EXPERIMENT_LOG.md` |

## Non-negotiables

- **Software is the weakest safety layer.** Hardware E-stops are layer 0; the Ranger RC transmitter
  is layer 1 and revokes CAN authority below anything software can do. The person next to the robot
  holds the RC. Our twist mux is layer 2.
- **The base must not move unless the arm is stowed**, verified by measured joint angles, never by
  an FSM's belief about itself. All safety gates fail closed: never-seen and stale both mean
  "not permitted".
- **A gate is GREEN only when a human watched it pass.** Log observations, not expectations.
- **Never kill processes by a loose pattern** — scope by full command line AND by the executable
  living under this repo. A frame-name match once killed 22 of the sim campaign's TF publishers.
- **Do not edit the simulation repo.** Copy from it; never modify it in place.
- **Never commit the API key.** It lives in a gitignored `.env` in the pipeline repo. This repo is
  PUBLIC. See `docs/LLM_ENDPOINT.md`.
- **The reasoner must never emit pixel coordinates or boxes.** Separating semantics from geometry
  is the entire thesis; letting the VLM return a box deletes the experiment.
- **`use_sim_time:=false`** on all real hardware. The sim configs default it true and the failure
  is silent.

## Commands

```bash
source bringup/env.sh                 # ROS + workspace + ROS_DOMAIN_ID=9, conda scrubbed
bash bringup/setup_workspace.sh       # build drivers from pinned commits (idempotent, no sudo)
bash bringup/lidar.sh                 # /scan + base_link->lidar_link
python3 bringup/preflight.py -v       # collision + stale-port check
python3 bringup/probe_rplidar.py      # raw serial probe, no ROS
python3 bringup/check_scan_geometry.py --tf
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is required because ROS's `launch_testing` pytest plugin fails
to import (missing `lark`). Unrelated to our code.

## Repos

- This one: `https://github.com/AnikS22/utp_robot` (public)
- Pipeline: `https://github.com/AnikS22/unlocking-the-path` (**private** — run `gh auth login`
  before cloning, or you get a 404 that looks like a wrong URL rather than a permissions problem)

## Status

Working on physical hardware: D435 image/depth, RPLIDAR `/scan`, rear scan filtering, Ranger CAN and
50 Hz odometry, xArm Ethernet and local arm UI, SLAM Toolbox, RViz, and geotagged camera capture.
Repository unit suite: **104 passing** on 2026-08-24. The full automatic pipeline still has no
`world=real` backend, and no physical Nav2 goal or button press has passed yet.

## Mapping handoff — 2026-08-24

The first physical maps are **diagnostic only and must not be used for Nav2**. They contain doubled
rooms, bent/fragmented walls, and scan-matching jumps. The visual-only morphology output cannot
repair incorrect poses. Start a clean map next session.

The observed cause of the final mapping failure was Ranger **Spinning mode** (`/system_state`
reported `motion_mode: 2`). The Ranger Mini supports DualAckermann, parallel/crab, spinning, and
side-slip motion. During spin/crab motion, wheel odometry and lidar scan matching can disagree;
in a repetitive corridor, `map -> odom` was observed moving about 0.45 m in seven seconds while
the chassis twist was zero. Do not compensate by driving the corridor twice.

The mapping data path is now deliberately fail-closed:

```text
/scan -> filter_scan.py -> /scan_filtered -> mapping_scan_gate.py -> /scan_mapping -> SLAM
```

- `filter_scan.py` removes the chassis-occluded rear sector and retains -105° through +105°.
- `mapping_scan_gate.py` forwards scans only when a fresh (<=0.5 s) `/system_state` reports
  DualAckermann (`motion_mode: 0`). Spin, parallel/crab, side-slip, unknown, and stale state block
  SLAM without stopping raw sensor diagnostics.
- `config/slam.yaml` consumes `/scan_mapping`, with 0.15 s, 0.10 m, and 0.10 rad update thresholds
  and a 30-scan buffer. The previous 0.5 s/m/rad settings skipped too much overlap at driving speed.
- `bringup/mapping.sh` now starts the rear filter and Ackermann gate automatically.
- RViz displays `/scan_filtered`; seeing the robot on raw `/scan` is expected and harmless.

Disconnected ROS integration validation forwarded 6/6 Ackermann scans, 0 spin scans, and 0 scans
after chassis state became stale. Python compilation, shell syntax, diff checks, and all 104 unit
tests passed. This verifies the software gate; the next session still requires a human-observed
physical check before declaring mapping green.

For the next map: stow the arm, hold the E-stop/RC, select **DualAckermann before moving**, use broad
turns at no more than about 0.25 m/s, pause after turns, and close loops by returning via a different
route. If map updates stop, check RC motion mode and `/system_state`; never switch to spin/crab to
finish a turn. Start with:

```bash
bash bringup/mapping.sh
```

This starts read-only sensing/SLAM and works with the separately started Ranger driver under RC
control. Save both the occupancy map and pose graph after each good closed loop with
`bash bringup/save_map.sh`. Glass doors will not be represented reliably; mark them manually as
keepouts before Nav2. All ROS hardware processes were stopped before unplugging on 2026-08-24.
