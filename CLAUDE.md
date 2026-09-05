# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# utp_robot — read this first

You are working on the **physical robot** for "Unlocking the Path" (ICRA 2027): an AgileX Ranger
Mini 3.0 + uFactory xArm6 + RealSense D435 + Ouster OS0-128 (the RPLIDAR A1M8 it started on is
retired). Deadline **2026-08-25**.

**Start with `docs/AGENT_BRIEF.md`.** Then, depending on the task:

| Task | Document |
|---|---|
| Sitting down for a lab session (the runbook) | `docs/MORNING.md` |
| Setting the laptop up from scratch | `docs/LAPTOP_SETUP.md` |
| Any device ID, pinned version, or setting | `docs/HARDWARE_SPECS.md` |
| Measuring anything on the robot | `docs/CALIBRATION.md` |
| What a "trial" is, methods, missions, metrics | `docs/PIPELINE.md` |
| Mapping / Nav2 / recording a run | `docs/MAPPING.md`, `docs/NAV2.md`, `docs/RECORDING.md` |
| Riding the lift between floors | `docs/MULTIFLOOR.md` |
| Re-mapping floor 1 as one map (the current job) | `docs/FLOOR1_REMAP.md` |
| Running the unit suite, and triaging a red test | `docs/TESTING.md` |
| The reasoning VLM endpoint and its key | `docs/LLM_ENDPOINT.md` |
| What has actually been done and observed | `EXPERIMENT_LOG.md` |

## Non-negotiables

- **Software is the weakest safety layer.** Hardware E-stops are layer 0; the Ranger RC transmitter
  is layer 1 and revokes CAN authority below anything software can do. The person next to the robot
  holds the RC. Our twist mux is layer 2.
- **The base must not move unless the arm is stowed**, verified by measured joint angles, never by
  an FSM's belief about itself. All safety gates fail closed: never-seen and stale both mean
  "not permitted".
- **A gate is GREEN only when a human watched it pass.** Log observations, not expectations, in
  `EXPERIMENT_LOG.md`. Record negative results — the wrong theories are what stop the next person
  repeating them.
- **Never kill processes by a loose pattern** — scope by full command line AND by the executable
  living under this repo, or by the inherited `UTP_ROBOT_STACK` env var that `env.sh` exports. A
  frame-name match once killed 22 of the sim campaign's TF publishers.
- **Do not edit the simulation repo.** Copy from it; never modify it in place.
- **Never commit the API key.** It lives in a gitignored `.env` in the pipeline repo. This repo is
  PUBLIC. See `docs/LLM_ENDPOINT.md`.
- **The reasoner must never emit pixel coordinates or boxes.** Separating semantics from geometry
  is the entire thesis; letting the VLM return a box deletes the experiment. It selects from a
  fixed tool vocabulary; a separate open-vocabulary detector does all localization.
- **`use_sim_time:=false`** on all real hardware. The sim configs default it true and the failure
  is silent.

## Commands

```bash
source bringup/env.sh                 # ROS + workspace + ROS_DOMAIN_ID=9, conda scrubbed
bash bringup/setup_workspace.sh       # build drivers from pinned commits (idempotent, no sudo)
python3 bringup/preflight.py -v       # will we collide with another ROS graph / stale port?
python3 bringup/health.py             # is the robot ACTUALLY working right now? (--watch to loop)

bash bringup/stack.sh                 # BRING-UP GOES THROUGH THIS (--status checks only, --no-nav skips Nav2)
bash bringup/session.sh up|map|nav|campaign N|down    # layered front door; dies on the FIRST failed gate
bash bringup/lidar3d.sh               # OS0-128 -> /ouster/points -> /scan_filtered
bash bringup/camera.sh                # D435
bash bringup/safety.sh                # twist mux + arm gate
python3 bringup/map_watch.py          # another terminal, while driving a map
bash bringup/map_persist.sh save|resume|list <name>   # the ONE map script
bash bringup/map_insurance.sh start|stop|rebuild <name>   # record a mapping drive so a crash
                                      # costs seconds, not a walk. START IT BEFORE THE DRIVE.
python3 bringup/check_scan_geometry.py --tf
```

Tests (no ROS, no GPU, no hardware — the whole suite runs in ~25 s):

```bash
python3 -m pytest tests/ -q -p no:launch_testing
python3 -m pytest tests/test_press_veto.py -q -p no:launch_testing
python3 -m pytest tests/test_run_campaign.py::test_campaign_stops_on_collision -q -p no:launch_testing
```

`-p no:launch_testing` is required: ROS's `launch_testing` pytest plugin auto-registers on any
machine with a sourced ROS install, is incompatible with the pytest this suite is written against,
and without the flag **collection dies and zero tests run** — which is not obviously distinguishable
from a broken repo. Nothing under `tests/` uses it. See `docs/TESTING.md`.

## Bring-up goes through `bringup/stack.sh`

Not `session.sh`, which dies on the first failed gate and turns a four-fault morning into four
serial fix-and-re-run cycles. `stack.sh` starts everything it can, restarts what is wedged, and
prints one component table plus a `WHY` block naming the cause. Three principles are encoded in it,
and they are the lessons that have cost this project the most time.

**1. PROBE BY RATE, NOT BY EXISTENCE.** `ros2 node list` showing a node proves nothing. On
2026-09-04 the Ouster driver was running and publishing **0.00 Hz** — a stale process holding the
UDP socket across a robot power cycle — and, separately, the chassis driver was absent entirely, so
there was no `/odom`, so slam_toolbox could not publish `map->odom`, so localization presented as
"wrong in RViz": a symptom **three layers from its cause**. Existence checks saw neither. Every
probe in `stack.sh` measures a rate, and every guard is on the CAPABILITY, not the name — an
unconfigured `bt_navigator` sits happily in `ros2 node list` forever.

**And for lifecycle nodes the capability is not enough either — ask the lifecycle state.** On
2026-09-04 every Nav2 goal came back `rejected in 0.0s` while `ros2 action list` **did** show
`/navigate_to_pose`, because the action server is advertised *before* the node is activated. This
one is silent in three ways at once: node list shows a healthy Nav2, action list shows the action,
and RViz shows an empty world (inactive costmap nodes publish nothing) — which reads as an RViz
configuration problem and is not one. Only `ros2 lifecycle get /bt_navigator` → `inactive [2]` saw
it. `stack.sh` now requires `bt_navigator`, `planner_server` and `controller_server` to report
`active`. When you write that check yourself: **match the whole field, never a substring — the
string `inactive` contains `active`,** and a `grep -q active` reports a dead Nav2 as healthy.

**Read the goal status word.** `rejected` = the action server would not accept the goal at all
(usually lifecycle or config, not the world); `aborted` = it tried and failed; `blocked` in
`nav2_goto.py` = Nav2 `STATUS_ABORTED` specifically, the verdict that starts the press chain.

**2. `ros2 topic hz` IS NOT TRUSTWORTHY ON THIS STACK.** It has reported **1.7 Hz** and **10.0 Hz**
for the same topic minutes apart, against a repeatable 6.4 Hz. Measure with a counting subscriber;
`stack.sh`'s `rate()` is the reference implementation. Quote no rate in `EXPERIMENT_LOG.md` that
did not come from one.

**3. LAUNCHING A SECOND COPY OF SOMETHING IS A FAILURE MODE, not a harmless retry.** When a
component looks wrong, **check for duplicates before starting another one.** The same shape bit
this project three times on 2026-09-04 alone: two Nav2 stacks (two `lifecycle_manager` instances
contending for the same nodes, activation never completes, every goal rejected); two RealSense
driver instances racing for the USB device (the loser logs `No RealSense devices were found`, which
reads as a cable fault); and two `waypoint_markers` publishers on one topic. `stack.sh` counts
`bt_navigator`/`planner_server` processes and tears **all** of them down before launching one —
`count_matching` is the helper. Tear down the NODES, not just the `ros2 launch` wrapper: killing
the wrapper alone orphans the servers it started, and those orphans are exactly what the next
launch stacks on top of.

**Kill by verified full command line, never a loose pattern** (see Non-negotiables). `pkill -f`
matches the agent's own `bash -c` and has killed the calling shell twice. `stack.sh`'s
`kill_matching` reads `/proc/<pid>/cmdline`, refuses an empty pattern, and skips `$$`, `$BASHPID`,
the entire ancestor chain, and anything whose command line is byte-identical to its own. `$$` alone
is not enough: it does not change inside a subshell, it does not cover the shell that invoked you,
and `$(...)`/pipeline subshells are none of those three — but every bash subshell shares its
parent's command line, so the identical-cmdline test catches them at any depth.

## Small changes that became big failures (2026-09-04)

Each of these was a locally reasonable edit. Read them before making a one-line config change.

**The number that explains most of them: the `/scan` rate.** During the failures it was
**1.95 Hz**; during the good presses **8.0 Hz**, and localization fit went 56–64% → 76.1%.
slam_toolbox searches with `coarse_angle_resolution` 2.0°, so at `wz_max` 0.8 rad/s (46°/s) and
1.95 Hz consecutive scans are ~23° apart: the pose slides mid-turn and the controller drives
against a stale estimate. That is why Nav2 reported "arrived" at `lift_door` while the robot sat
**1.85 m** away, then drove the wrong way and hit a wall. The 3.1 MB point cloud
(512 × 128 × 48 B) is the bottleneck — ~73% lost in DDS — and it exists only to be flattened into
a 2D scan.

- **A. Lowered `inflation_radius` 0.30 → 0.20 to unblock a doorway. Caused a wall collision.** The
  diagnosis was right (the lift doorway is 1.08 m; at inflation 0.30 a corridor must exceed 1.16 m
  to contain any cell below cost 99, so that waypoint was mathematically unplannable). The fix was
  wrong: inflation is the margin that keeps planned paths off walls, and the operator had just
  asked for a *bigger* safety barrier. Reverted byte-exact. **A correct diagnosis does not license
  the first fix that unblocks it — check what the fix costs elsewhere.**
- **B. Pointed slam_toolbox at the driver's native `/ouster/scan` (9.9 Hz) instead of `/scan`
  (2–4 Hz) to fix drift. Put the robot in the wrong place.** `maps/elevator.*` was built from the
  **height-band** projection (min range over 0.20–1.20 m); the native scan is a **single ring** at
  one elevation. Matching a ring against a height-band map compares two different slices of the
  room. **The scan must be the same KIND the map was made from; rate is not the only property that
  matters.**
- **C. Edited `config/slam_os0.yaml` after launching slam_toolbox and assumed it applied.** It did
  not — the node keeps its launch-time params. The robot stayed mis-localized and it was reported
  as fixed. **After editing a params file, restart the node and verify with `ros2 param get`.**
- **D. Capped `wz_max` 0.8 → 0.20 to stop rotation outrunning the scan matcher.** Rejected by the
  operator, correctly: it slows every leg, including straight ones, to fix something that only
  happens while turning. Replaced with `bringup/settle.py`, which holds after each leg until the
  pose stops moving. **Prefer a fix scoped to the failing condition over a global slowdown.**
- **E. Ran a 5-leg route under a 2-minute foreground timeout**, killed a leg mid-drive, and
  reported it as a failure. **Long robot actions must be backgrounded.**
- **F. Reused one `UTP_RUN_DIR` across several attempts**, so the figure generator merged 11 events
  into one marker and matched a keyframe 972 s from its event. **One run directory per attempt.**

## Small changes that turn into big errors (2026-09-05)

The 2026-09-04 list above is about edits with a wrong value. This list is about something worse:
**things that report success while proving nothing.** A wrong value gets caught. A check that
cannot fail gets believed.

### A. Checks that pass while proving nothing

- **`grep -q active` reports a dead Nav2 as healthy, because the string `inactive` contains
  `active`.** Symptom: `stack.sh` printed a green Nav2 while every goal came back
  `rejected in 0.0s`. **Match the whole field**, never a substring — `[ "$state" = "active" ]`, or
  anchor the pattern.
- **An existence probe is not a rate probe.** `ros2 node list` showing `/ouster/os_driver` proves
  a process is alive, not that data is flowing. Symptom on 2026-09-04: the driver was up and
  publishing **0.00 Hz** (a stale process holding the UDP socket across a power cycle), and the
  fault surfaced three layers away as "localization is wrong in RViz". Every probe in `stack.sh`
  counts messages over a window. And `ros2 topic hz` is not a substitute — it has reported 1.7 Hz
  and 10.0 Hz for the same topic minutes apart against a repeatable 6.4 Hz.
- **A zero-timeout `can_transform` was measuring DDS discovery, not availability.** Symptom: the
  gate reported `base_link -> os_lidar` absent while `tf2_echo` resolved it instantly in the next
  terminal — so the transform was read as broken and time went into the TF tree, which was fine.
  A fresh node has not yet discovered the `/tf_static` publisher, and a latched static transform
  arrives only after discovery completes. **Fixed by giving it a 12 s timeout.** Any probe made by
  a just-started node needs one: with timeout 0 you are timing your own subscription setup.

The shape underneath all three: **ask the question that can come back "no".** A check that cannot
fail is not a check, and it is worse than no check because it ends the investigation.

### B. `config/floors.yaml` named a map and waypoints that did not exist

The floor-2 entry said map `elevator_f2` and waypoints `call_button_f2`, `car_panel_f2`. None of
those were ever on disk: the map is `maps/floor2.*` and the waypoints are **`f2_`-prefixed**, not
`_f2`-suffixed. The entry had been written from the naming convention in the file's own header
instead of from what was actually recorded. Symptom, had it not been caught:
`floor_swap.py --to 2 --go` failing to find its seed pose **with the robot already standing inside
the lift car**, doors about to close, on an unverified map.

`python3 bringup/floor_swap.py --check` caught it. It is pure offline Python — no ROS, no robot —
so there is no excuse for not running it after touching that file. **A config file is code that
nothing compiles; the only thing standing between it and the lift is a checker you actually run.**

### C. Seeding a floor swap at the wrong in-car pose

`floor_swap.py` seeded `map_start_pose` from `car_facing_out` unconditionally. On 2026-09-05 the
robot rode down parked at `car_panel` — **0.48 m and 116° away**. Seeding at `car_facing_out`
would have handed slam_toolbox a start pose the robot was not standing at.

Why that is not self-correcting: a closed lift car's scan is **four blank walls about a metre
away**. There is nothing in it to pull the estimate back. The matcher converges confidently onto
the seed you gave it, and the error only becomes visible once the doors open and the robot drives.
`--seed-role` now names where the robot physically is. **A seed is a claim about the world; in a
featureless room it is the only claim there is.**

### D. A `.pgm`/`.yaml` map with no `.posegraph` looks like a map

`maps/atrium.*` is a grid and nothing else. `slam_toolbox` `mode: localization` relocalizes by
deserializing the `.posegraph`; handed only a grid it does **not** error — it starts a new, empty
graph at the robot's feet, publishes `/map`, and reports `active`. Symptom: a healthy-looking
localization in which every stored waypoint is meaningless, because `map` is a fresh-SLAM frame
wearing the saved map's name. This is why floor 1 is being re-mapped; see `docs/FLOOR1_REMAP.md`.

Corollary that cost a second walk: **two such maps cannot be merged.** A pose graph is a chain of
scan matches from one continuous drive, and there was never a drive linking the lift lobby to the
ADA door. One continuous drive is the only fix.

`map_persist.sh save` writes all four files or none, and its overwrite guard now checks all four
extensions — guarding on `.pgm` alone let a name holding a good `.posegraph` be clobbered silently,
which is the half that cannot be regenerated.

### E. slam_toolbox keeps its LAUNCH-TIME parameters

Editing `config/slam_os0.yaml` after the node is launched changes nothing. Symptom on 2026-09-04:
the robot stayed mis-localized and it was reported as fixed. **Restart the node, then verify with
`ros2 param get`.** The same is true of every ROS node in this stack.

### F. A mapping drive lives only in RAM until it is serialized

`slam_toolbox` holds the pose graph in memory and serializes **only on request**. On 2026-09-05 a
floor-1 mapping drive died with its session: no partial map, nothing on disk, nothing to salvage,
walk repeated from scratch. Nothing was recording, so there was never a second chance to have.

`bringup/map_insurance.sh start <name>` records `/scan /scan_nav /tf /tf_static /odom` — what
slam_toolbox actually consumes, ~1–2 MB per minute — so a drive can be replayed and rebuilt
offline. Start it **before** the drive. (`ros2 bag record -a` is not an option: `/ouster/points` is
3.1 MB per cloud and the one such bag in `runs/` is 3.4 GB.)

### G. KNOWN TRAP — `session.sh`'s step-0 gate pings the xArm for EVERY session type

`bringup/session.sh:66-68` pings `192.168.1.119` (lidar), **`192.168.1.221` (xArm)** and
`192.168.1.1` (router), and `die`s on the first unreachable one. That gate runs before the command
word is looked at, so a **nav-only or mapping-only** run is blocked outright whenever the arm is
powered off — which is the normal state for a mapping drive, since the arm is stowed and unused.
It presents as `192.168.1.221 unreachable` and reads like a cable fault on the one cable that also
carries the lidar.

**Proposed, NOT applied:** make the arm's ping non-fatal unless the session actually needs the arm
— keep `.119` and `.1` fatal for every session, and demote `.221` to a warning for `map` and `nav`
while leaving it fatal for `campaign` and anything that presses. It is not applied here because
`session.sh` is live and the change belongs to whoever is next at the console, with the robot in
front of them.

## Conventions that will bite you

- **Nothing moves without `--go`.** Every motion script and node is dry-run by default and prints
  what it would do; `--go` (or `--dry-run` on the bash routes) is the switch. One documented
  exception: `press_run.sh`'s final STOW stage runs `stow_arm.py --go` unconditionally, so a
  `--dry-run` of a route that calls it *will* fold the arm.
- **Three Pythons, on purpose, and they cannot share a process.**
  - ROS's `python3` — `rclpy`, everything under `bringup/` that touches topics.
  - `.venv-arm/bin/python` — xArm SDK + `pyrealsense2` + `cv2`, no system site-packages and no
    `rclpy`. Anything that commands the arm (`stow_arm.py`, hand-eye, `approach_target.py`).
  - `~/unlocking-the-path/env/.venv/bin/python` — torch/GroundingDINO, the grounder and vetoes.
  The handoff between them is **a file**, `captures/<name>/detection.json`, which is also why the
  exact frame the detector saw is preserved and can go in the paper. When a bash route looks
  needlessly split into stages, this is why.
- **Waypoints are odom-frame unless recorded on a saved, NAMED map.** A chassis power-cycle
  silently invalidates odom-frame waypoints — a frame *identity* problem, not drift.
  `safety/map_frame.py` enforces the distinction; a 50-trial campaign must use `session.sh nav`.
- **`config/missions` is a gitignored symlink into the private pipeline repo.** A fresh clone has a
  dangling link, not a bug in the loader. See `config/pipeline/README.md`.
- **Angular commands must be held, not re-sent.** Re-issuing an angular twist at 20 Hz re-steers all
  four wheels every cycle and the body never commits to the turn (4WS mode-thrash).

## Architecture

The load-bearing split is **`safety/` (pure) vs `bringup/` (plumbing)**. Every rule that can stop
the robot is a plain-Python function with a test and no ROS import; the ROS nodes are thin shells
over it. That is the whole reason the suite runs headless on a laptop with nothing plugged in.
`arbiter.py` ↔ `twist_mux_node.py` is the canonical pair — copy that shape for anything new.

```
   mission (config/routes.yaml)
        │
        ▼  route_plan.py validates the WHOLE route against known waypoints before anything moves
   run_trial.py ──leg blocked?──▶ escalation.py ──▶ VLM reasoner (picks a TOOL, never a coordinate)
        │                                                    │
        │ action step                                        ▼
        │                                    GDINO grounder LOCALIZES  +  press_veto.py
        │                                    (never a fire alarm — see tests/test_press_veto.py)
        ▼                                                    │
   reach_envelope.py → base moves into reach → xArm6 presses ◀┘

   every twist, from every source, passes through:
   twist_mux_node.py — the ONLY publisher of /cmd_vel
   fail-closed gates: estop · arm_stowed · enable · override
```

- **The leg runs on the MAP via Nav2; the last metre runs on odom.** Deliberate: an AMCL correction
  mid-press would move the target under the arm. `nav2_goto.py` plans across the saved map;
  `approach_blockage`, the look ladder and the press chain stay in odom, and the visual servo
  closes the gap (measured to 3 mm across four runs).
- **The press chain is READY → LOOK → GROUND → REACH** (`press_run.sh`). READY comes before LOOK
  because with the arm stowed the folded arm occupies the lower-centre of the mast camera's frame —
  exactly where a plate 0.7 m ahead appears. `elevator_route.sh` is this same chain twice, with
  different waypoint names and query strings; there is deliberately no state machine.
- **A lift ride is a map handover, not a navigation problem** (`safety/floor_plan.py`,
  `bringup/floor_swap.py`, `multifloor_route.sh`). One saved map per floor; the swap RESTARTS
  slam_toolbox rather than deserializing in place, because a restart changes the DDS GID and so
  invalidates the old floor's waypoints through `map_frame.py` for free. The load-bearing fact: a
  lift car is geometrically identical on every floor, so a scan-match fit taken inside a closed car
  is high, confident, and says nothing about which floor you are on. The floor is only observable
  once the doors open, which is why the gate refuses a fit measured with them shut. Untested on
  hardware. `maps/floor2.*` exists and its five `f2_` waypoints have been driven; **floor 1 is
  mid-re-map** — `config/floors.yaml` already names `floor1` and five `f1_` waypoints that are not
  recorded yet, so `floor_swap.py --check` fails on purpose. See `docs/FLOOR1_REMAP.md`, then
  `docs/MULTIFLOOR.md`.
- **Recording a run**: `bringup/run_dataset.py` wraps any command in a low-interference recorder and
  writes `runs/<UTC>_<method>_<scene>/`. Routes mark moments with `event()` from `run_event.sh`,
  which is a silent no-op unless `UTP_RUN_DIR` is set — an instrumented run and a plain run must
  never be two different code paths.
- **`tests/test_stack_wiring.py` asserts that config and code agree** (stow pose in `safety.yaml`
  vs `STOW_DEG` in `stow_arm.py`, the scan chain wired the same way in every launcher, the SLAM
  range quoted in the config). Changing a constant in one place fails a test in a file you did not
  touch — that is the test doing its job, not a flaky test.

## Mapping data path (OS0-128, since 2026-08-30)

```text
/ouster/points -> pointcloud_to_laserscan -> /scan_filtered -> scan_relay.py -> /scan -> slam_toolbox
```

- The chassis is removed **geometrically** by `pointcloud_to_laserscan` (`target_frame:=base_link`,
  a `min_height`/`max_height` band, `range_min:=0.50`) — **not** by SLAM. `min_laser_range` is
  inert on Jazzy; slam_toolbox never declares it, measured. The A1M8-era rear-sector filter and
  Ackermann gate are retired; see `archive/README.md`.
- `scan_relay.py` is **not** optional: `pointcloud_to_laserscan` publishes BEST_EFFORT and
  slam_toolbox subscribes RELIABLE, and incompatible DDS QoS delivers zero messages with no error
  anywhere.
- **Never launch slam_toolbox with inline `-p` flags.** That silently takes stock defaults for
  `do_loop_closing` (the map comes out bent) and `stack_size_to_use` (serializing a building-sized
  graph dies). `config/slam_os0.yaml` is the live config; `bringup/session.sh` launches with it.
- A `.pgm` without a `.posegraph` cannot be relocalized into — and slam_toolbox does not error, it
  silently starts a new graph at the robot's current pose. `map_persist.sh save` writes grid +
  pose graph + `.loaded_map`, all three or none.
- **Driving the map**: stow the arm, hold the RC, select **DualAckermann before moving**, broad
  turns at ≤0.25 m/s, pause after turns, close the loop by returning past the start via a different
  route. Spin mode (`/system_state` `motion_mode: 2`) caused metre-scale scan-matching jumps on
  2026-08-24 — that was the *odometry*, not the scan. Glass doors will not appear; mark them as
  keepouts by hand.

## Status

Five of six stages are demonstrated on hardware (navigate a leg, detect a blockage and escalate,
ground the ADA plate, refuse to press a fire alarm, drive into arm reach). **The press landing on
the plate is NOT demonstrated** — the last attempt missed by ~10 cm, a perception-at-range problem,
not control. The fix (re-ground from the press pose with the arm raised) is implemented in
`reach_control.sh` / `press_run.sh` and has never run on hardware. Elevator interaction has never
run on any system. Do not read this repo as claiming a completed door-opening run.

**The unit suite is green, with two documented exceptions.** Measured 2026-09-05:
**478 passed, 29 skipped, 2 xfailed, 0 failed** (`-p no:launch_testing`; the old
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` invocation did not start the suite at all). The two
`xfail(strict)` cases are real bugs in directories the suite may not edit, each carrying its fix in
the marker: `bringup/ros_world.py`'s **default** nav path reports `blocked` on any Nav2 abort
without confirming it perceptually (the staged path already gates on it), and
`config/slam_os0.yaml` / `config/ouster.yaml` both cite `range_min:=0.30` when 0.45 is what ships.
Several skips are loud, by name: `captures/trial_ours_001/scan.json`, the glass-door near-miss four
tests are built on, was overwritten by a later run reusing the name and is not recoverable from
git. See `docs/TESTING.md` before treating a red or skipped test as either a regression or noise.
