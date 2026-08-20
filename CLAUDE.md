# utp_robot — read this first

You are working on the **physical robot** for "Unlocking the Path" (ICRA 2027): an AgileX Ranger
Mini 3.0 + uFactory xArm6 + RealSense D455 + RPLIDAR A1M8. Deadline **2026-08-25**.

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
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is required because ROS's `launch_testing` pytest plugin fails
to import (missing `lark`). Unrelated to our code.

## Repos

- This one: `https://github.com/AnikS22/utp_robot` (public)
- Pipeline: `https://github.com/AnikS22/unlocking-the-path` (**private** — run `gh auth login`
  before cloning, or you get a 404 that looks like a wrong URL rather than a permissions problem)

## Status

Working: lidar `/scan`, driver workspace, safety-stack logic (23 tests).
Not yet: CAN/base bring-up, arm (needs Ethernet link), RealSense, URDF, real `World` backend.
