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
bash bringup/lidar3d.sh               # OS0-128 -> /ouster/points
python3 bringup/preflight.py -v       # collision + stale-port check
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

The mapping data path (OS0-128, since 2026-08-30):

```text
/ouster/points -> pointcloud_to_laserscan -> /scan_filtered -> scan_relay.py -> /scan -> slam_toolbox
```

- `pointcloud_to_laserscan` projects with `target_frame:=base_link` and a `min_height`/`max_height`
  band, so the chassis is removed **geometrically**. `min_laser_range: 0.55` in
  `config/slam_os0.yaml` removes what is left. The A1M8-era rear-sector filter and Ackermann gate
  (`filter_scan.py` -> `mapping_scan_gate.py` -> `/scan_mapping`) are therefore retired; see
  `archive/README.md`.
- `scan_relay.py` is **not** optional: `pointcloud_to_laserscan` publishes BEST_EFFORT and
  slam_toolbox subscribes RELIABLE, and incompatible DDS QoS delivers zero messages with no error
  anywhere.
- `config/slam_os0.yaml` is the live SLAM config and `bringup/session.sh` launches with it. Never
  launch slam_toolbox with inline `-p` flags: that silently takes stock defaults for
  `do_loop_closing` (the map comes out bent) and `stack_size_to_use` (serializing a building-sized
  graph dies). Both verified to take effect 2026-09-01 by reading them back off the running node.
- **`min_laser_range` is inert on Jazzy** — slam_toolbox never declares it, measured. The chassis
  is kept out of the map by `pointcloud_to_laserscan`'s `range_min:=0.50` and its
  `min_height`/`max_height` band, not by SLAM.

DRIVING THE MAP: stow the arm, hold the RC, select **DualAckermann before moving**, broad turns at
no more than ~0.25 m/s, pause after turns, and **close the loop** by returning past your start via
a different route. The old gate that enforced Ackermann is gone, but the reason survives it: spin
mode caused metre-scale scan-matching jumps on 2026-08-24, and that was the *odometry*, not the
scan. Glass doors will not appear; mark them as keepouts by hand.

```bash
bash bringup/session.sh map              # every layer, in the order that works
python3 bringup/map_watch.py             # another terminal, while driving
bash bringup/map_persist.sh save <name>  # grid + pose graph + .loaded_map, all three or none
```

`map_persist.sh` is the ONE map script (`save` / `resume` / `list`). A `.pgm` without a
`.posegraph` cannot be relocalized into — and slam_toolbox does not error on one, it silently
starts a new graph at the robot's current pose. Full procedure: `docs/MAPPING.md`.

All ROS hardware processes were stopped before unplugging on 2026-08-24.
