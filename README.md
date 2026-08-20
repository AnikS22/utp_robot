# utp_robot — physical robot stack for *Unlocking the Path*

Everything needed to run the interactive-navigation benchmark on the physical
**AgileX Ranger Mini 3.0 + uFactory xArm6 + RPLIDAR A1M8**.

Self-contained and portable on purpose: clone onto the rover laptop, run one script, and you have
the same build as the workstation. Start at **[EXPERIMENT_LOG.md](EXPERIMENT_LOG.md)** — gate table
and dated findings.

## Quick start

```bash
git clone <this repo> ~/utp_robot && cd ~/utp_robot
bash bringup/setup_workspace.sh     # clones + patches + builds the drivers (no sudo, ~30 s)
bash bringup/lidar.sh               # publishes /scan
```

## Why this is a separate repo

The simulation stack (`Unlocking_the_path`) works and is not touched by any of this. Hardware code
lives here and diverges freely; anything needed from the sim side is copied, never edited in place.
The one seam back is `make_world()` in the sim repo's `utp/runner/batch.py`, where a real world
backend registers when it exists — `registry.py` already selects the real reasoner / grounder /
navigator for any non-mock backend.

## Layout

```
bringup/setup_workspace.sh   builds ros2_ws from pinned upstream commits + our patches
bringup/lidar.sh             brings up the RPLIDAR, publishes /scan
bringup/probe_rplidar.py     talks to the lidar over raw serial — no ROS, no driver, no sudo
patches/                     our diffs against upstream drivers, applied by setup_workspace.sh
safety/                      base-motion safety stack (arbiter + twist mux + arm monitor)
config/safety.yaml           mux wiring, interlock gates, speed and slew ceilings
tests/                       headless tests — no ROS, no Isaac
ros2_ws/                     GITIGNORED. Rebuilt by setup_workspace.sh.
```

`ros2_ws/` is deliberately not committed: it holds third-party checkouts with their own `.git`,
7 MB of vendored asio headers, and build artifacts baked with absolute paths that would be wrong on
the rover laptop. `setup_workspace.sh` is the reproducible artifact instead.

## Two upstream decisions that are easy to get wrong

**`ranger_ros2`: build the `humble` branch, on Jazzy.** The `jazzy` branch does **not** support the
Ranger Mini V3 — its `RangerSubType` enum stops at `kRangerMiniV2` and it ships no
`ranger_mini_v3.launch.py`. Only `humble` has `kRangerMiniV3`. Switching to the jazzy branch
silently loses V3 support.

**`rplidar_ros` needs our patch.** Our A1M8 runs firmware 1.29, which predates the scan-mode
negotiation protocol, so the driver's default express path fails
(`0x80008004` NOT_SUPPORT on mode enumeration, `0x80008000` INVALID_DATA on express scan). The patch
adds a `legacy_scan` parameter that issues the legacy SCAN command, which works. See
`patches/rplidar_ros-legacy-scan.patch`.

## Safety stack

Software is **layer 2** of three — do not mistake it for the whole story:

| Layer | Mechanism | Authority |
|-------|-----------|-----------|
| 0 | Chassis + arm-controller hardware E-stops | Cuts motor power. Nothing overrides it. |
| 1 | Ranger RC transmitter | Revokes CAN command authority at the driver, below any software. **The person next to the robot holds this.** |
| 2 | `safety/twist_mux_node.py` | Covers failures faster than a human reacts. |

- **`safety/arbiter.py`** — all decision logic, pure Python, no rclpy. Priority mux + interlocks +
  slew limiter. The part that can hurt someone is the part that is unit-tested.
- **`safety/twist_mux_node.py`** — the **only** publisher of `/cmd_vel`. Nav2, the pipeline's servo
  loops, and teleop each publish to their own topic and this node arbitrates. That single chokepoint
  is what makes every interlock possible.
- **`safety/arm_monitor_node.py`** — publishes `/safety/arm_stowed`. On hardware it reads **measured
  joint angles**, never an FSM's belief about itself: an FSM reporting `idle` after its owning
  process crashed is exactly what the interlock exists to catch.

**Every gate is fail-closed** — never-seen and stale both mean *not permitted*. **The mux publishes
every tick, zeros included**, because a mux that goes quiet when it blocks is indistinguishable
downstream from a mux that died.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q   # 23 tests
python3 safety/twist_mux_node.py
python3 safety/arm_monitor_node.py --backend scene_state   # sim
python3 safety/arm_monitor_node.py --backend xarm_sdk      # hardware
ros2 service call /safety/clear_estop std_srvs/srv/Trigger
ros2 topic echo /safety/status
```

Status: arbiter logic is tested; **the nodes have never run against a moving robot.** Gate H4.

## Documents

| | |
|---|---|
| `docs/AGENT_BRIEF.md` | start here — project, rules, order of work |
| `docs/LAPTOP_SETUP.md` | 10-stage provisioning runbook, each stage gated by a CHECK |
| `docs/HARDWARE_SPECS.md` | every device ID, baud, pinned commit, mount pose, known quirk |
| `docs/CALIBRATION.md` | nine calibrations in dependency order, with acceptance criteria |
| `docs/PIPELINE.md` | what a trial is: the loop, the method matrix, missions, metrics |
| `docs/LLM_ENDPOINT.md` | the FAU OwlChat reasoning endpoint — config, verification, failure modes |
| `EXPERIMENT_LOG.md` | what has actually been observed |

## Background (in the sim repo)

`docs/REAL_ROBOT_PLAN.md`, `docs/REAL_ROBOT_DESIGN_REASONING.md`, `docs/integration_contract.md`.
