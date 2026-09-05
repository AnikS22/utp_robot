# utp_robot — physical robot stack for *Unlocking the Path*

The hardware half of an interactive-navigation benchmark: a mobile manipulator that reasons about
a **closed door**, finds the **ADA push plate**, presses it, and drives through.

Platform: **AgileX Ranger Mini 3.0** (4-wheel independent steering) + **uFactory xArm6** +
**Intel RealSense D435** + **RPLIDAR A1M8**, on ROS 2 Jazzy / Ubuntu 24.04.

The simulation stack lives in a separate repository (`Unlocking_the_path`) and is never modified by
anything here — code is copied out of it, not edited in place. This repo is what happens when that
pipeline meets a real chassis.

---

## Status — read this first

The task decomposes into six stages. Five are demonstrated on the physical robot; the sixth is not.

| stage | status | evidence |
|---|---|---|
| Navigate a recorded route leg | **working** | repeated arrivals within 0.15 m; one 2-leg run at 14.3 cm final error |
| Detect that the way is blocked and escalate | **working** | corridor veto fires, route hands off to the pipeline instead of steering around |
| Ground the ADA plate in the image | **working** | score 0.526, 99×90 px, correctly preferred over a FIRE alarm 18 cm away |
| Refuse to press a fire alarm | **working** | veto refused a real alarm twice (4 of 4 forbidden queries on target) |
| Drive the base into arm reach of the plate | **working** | `positioned: 0.68 m from target, +0.0° off the press axis` |
| **The press landing on the plate** | **NOT DEMONSTRATED** | last attempt missed by ~10 cm; see below |

**The remaining failure is perception-at-range, not control.** The plate was grounded from 1.66 m,
where a 12 cm plate subtends roughly 50 px. A half-plate error at that distance is ~10 cm of lateral
error in the world, and the base then drove precisely onto the wrong point. The fix — re-ground from
the press pose, with the arm raised so it does not occlude the lower-centre of the frame, and correct
before committing — is implemented (`bringup/reach_control.sh`, `bringup/press_run.sh`) and has
**never been executed on hardware**. Do not read this repo as claiming a completed door-opening run.

Everything above is reproducible from `EXPERIMENT_LOG.md`, which is dated and includes the failures.

---

## The engineering result worth reporting

Six separate failures during bring-up shared one signature: **the system worked as designed and
reported the problem somewhere nobody was listening.**

| what actually happened | what it looked like |
|---|---|
| Safety mux discarding commands (gate not satisfied) | "the path planner is broken" |
| No `arm_stowed` publisher on hardware — interlock never satisfiable | same |
| Chassis latched in RC mode; CAN commands silently dropped | same |
| Waypoints recorded in an odom frame that died with a power-cycle | robot drives somewhere random |
| `/scan_filtered` absent, so the obstacle veto failed **open** | robot drives into things |
| `ROS_DOMAIN_ID` unset — talking to the wrong graph | topics exist but nothing responds |

None of these were logic errors. Each was a silent-degradation path. The response was to make every
one of them **refuse loudly**: `bringup/health.py` is a single preflight that fails on any of the six,
and the safety mux is fail-closed by construction. This is the most transferable finding here — the
hard part of moving a working simulated policy onto hardware was not the policy.

Also measured, and worth knowing before trusting any number from this platform:

* **Odometry is accurate — drift is not the problem.** Measured 2026-08-29: lateral drift on a
  straight run **−0.000 m**, position drift on a spin **0.001 m**, and lidar scan-match against odom
  on a 27° spin **1.02**. Three runs returning to one recorded waypoint landed within a **14 cm**
  spread. (An earlier revision of this README claimed ~1 m of drift per loop. That figure was wrong
  and no measurement supports it; it is corrected here because it was being used to justify adding
  fiducial localization the data does not call for.)
* **The angular control path is the problem, not localization.** Command scaling is linear 0.94 but
  angular **0.59–0.80 and inconsistent** — the chassis under-rotates by up to 41% of what the stack
  believes it asked for. Odometry reports that honestly, so closing the loop should absorb it, and
  it did not: `Limits` carried a stall floor for linear velocity (`v_min`) and **none for angular**,
  while the 4WS wheels must physically re-steer before the body turns. Every heading correction the
  controller emitted was below the rate the chassis will execute, so a turn converged until it
  stalled ~0.167 rad short of tolerance and then held there with `vx = 0` — the observed livelock.
  Fixed 2026-08-30 (`w_min`); **not yet confirmed on hardware.**
* **Waypoints are odom-frame, so a chassis power-cycle silently invalidates them.** This is a frame
  *identity* problem, not a drift problem, and it is the honest case for absolute localization
  (AprilTag or lidar anchoring): waypoints that survive between sessions, not waypoints that fight
  drift. `safety/scan_anchor.py` is written and unit-tested and has **not** been run on the robot.
* **4WS mode-thrash**: re-issuing an angular command at 20 Hz re-steers all four wheels every cycle,
  so the body never commits to the turn. Angular commands must be held, not re-sent.
* **The lidar sits 0.318 m forward of `base_link`** at z = 0.379 m — confirmed independently by the
  geometry of the robot's own self-hits in the scan.

---

## Architecture

```
              ┌─────────────────────────────────────────────┐
   mission ─▶ │ run_trial.py    one trial: legs + FSM        │
   (yaml)     └──────┬──────────────────────────────┬────────┘
                     │ leg blocked?                 │ action step
                     ▼                              ▼
          ┌──────────────────────┐      ┌────────────────────────────┐
          │ escalation.py        │      │ VLM reasoner  (picks TOOL) │
          │ geometry ran out —   │─────▶│   "look left" / "press"    │
          │ ask the pipeline     │      └─────────────┬──────────────┘
          └──────────────────────┘                    │ tool, never coordinates
                                                      ▼
                                        ┌────────────────────────────┐
                                        │ GDINO grounder (LOCALIZES) │
                                        │  + press_veto: not a fire  │
                                        │    alarm                   │
                                        └─────────────┬──────────────┘
                                                      ▼
                                        ┌────────────────────────────┐
                                        │ reach_envelope → base moves│
                                        │ into reach → xArm6 presses │
                                        └────────────────────────────┘

   every motion command, from every source, passes through:
          ┌──────────────────────────────────────────────────────────┐
          │ twist_mux_node.py — the ONLY publisher of /cmd_vel       │
          │ fail-closed gates: estop · arm_stowed · enable · override │
          └──────────────────────────────────────────────────────────┘
```

Two design commitments are load-bearing:

1. **The reasoner never emits coordinates.** It selects from a fixed tool vocabulary; a separate
   open-vocabulary detector does all localization. A VLM that hallucinates a pixel is a bug you cannot
   see; a VLM that picks the wrong tool is one you can.
2. **Motion is fail-closed.** `twist_mux_node` is the single `/cmd_vel` publisher and every gate must
   be *affirmatively* satisfied. A missing publisher stops the robot; it does not free it.

---

## Repository layout

```
safety/        pure decision logic — no ROS imports, so it is testable headlessly
               arbiter · twist_mux_node · route_plan · waypoint_drive · reach_envelope
               press_veto · escalation · look_policy · local_avoid · scan_anchor
               lidar_lift · scan_filter · mux_watch · waypoint_frame · teleop_guard
bringup/       the executable layer: ROS nodes, run scripts, one-shot diagnostics
               health.py       preflight — fails on any of the six silent-degradation paths
               run_trial.py    one trial · run_campaign.py  N trials, with drift budget
               session.sh      the front door: up · map · nav · campaign · down
               nav2_goto.py    drive one leg on the saved map · map_persist.sh  the map script
               waypoints.py    record / list / anchor odom-frame waypoints
               press_run.sh    READY → LOOK → GROUND → REACH
               reach_control.sh  ground, position, re-ground, press
config/        routes.yaml (missions) · safety.yaml (gates, speed and slew ceilings) · slam_os0.yaml
nav2_bringup/  Nav2 config, ported for real hardware (use_sim_time:=false, spin removed)
docs/          runbooks — start with MORNING.md, then MAPPING.md and NAV2.md
               FLOOR1_REMAP.md  the current job, in the order it must happen
               TESTING.md       what each test file guards, and how to triage a red one
tests/         all headless: no ROS, no GPU, no hardware. 478 passed / 29 skipped / 2 xfailed
               as of 2026-09-05; see docs/TESTING.md for what each file guards
patches/       our diffs against upstream drivers, applied by setup_workspace.sh
maps/          site maps (see maps/README.md) · archive/ retired scripts, see archive/README.md
ros2_ws/       GITIGNORED — rebuilt from pinned upstream commits by setup_workspace.sh
```

`safety/` holds no ROS imports on purpose. Every rule that can stop the robot is a pure function with
a test, and the ROS nodes are thin shells over it. That is why the suite runs in 22 seconds on a
laptop with nothing plugged in.

---

## Running it

```bash
git clone https://github.com/AnikS22/utp_robot ~/utp_robot && cd ~/utp_robot
bash bringup/setup_workspace.sh     # clones + patches + builds drivers (no sudo, ~30 s)
source bringup/env.sh
```

Tests need nothing but Python:

```bash
python3 -m pytest tests/ -q -p no:launch_testing     # 478 passed, 29 skipped, 2 xfailed
```

`-p no:launch_testing` disables a ROS-bundled pytest plugin that is incompatible with the pytest
version this suite is written against and otherwise stops pytest from starting at all. See
`docs/TESTING.md` for what each file guards and the triage rule for a failing test.

### Bring-up: one command

```bash
bash bringup/stack.sh            # brings everything up, restarts what is wedged
bash bringup/stack.sh --status   # check only, starts nothing
bash bringup/stack.sh --no-nav   # sensors + safety + slam only
```

`session.sh` brings the layers up in order and **stops at the first gate that fails**, so a morning
with four stale components is four serial fix-and-re-run cycles. `stack.sh` starts everything it can
in one pass, and probes each piece by **measured topic rate** with a counting subscriber rather than
by whether a node exists — which is the check that missed both of 2026-09-04's faults. Anything
publishing nothing is killed by verified PID and restarted.

It ends with a table of every component and, underneath it, a `WHY` block that names the actual
cause rather than the layer the symptom appeared in — "there is no `odom->base_link`, so slam cannot
publish `map->odom`", not "localization is wrong".

`MAP_NAME=elevator` (the default) selects the map to localize into. Floor 2 is `MAP_NAME=floor2`.
Floor 1 is **mid-re-map** into a single `floor1` map covering both the lift lobby and the ADA
door — `config/floors.yaml` already names it, so `floor_swap.py --check` fails on purpose until
the drive is done. Procedure and order: **[docs/FLOOR1_REMAP.md](docs/FLOOR1_REMAP.md)**.

### Nine things that stop a bring-up

Every one of these cost real time on 2026-09-04 because nothing checked for it. The right-hand
column is what `stack.sh` now does about it.

| | what goes wrong | how it presents | `stack.sh` |
|---|---|---|---|
| 1 | **Stale Ouster driver** holding the UDP socket across a robot power cycle | `ros2 node list` shows `/ouster/os_driver`; `/ouster/points` is 0.00 Hz | detects by rate, kills, restarts `lidar3d.sh` |
| 2 | **The scan chain is not restarted with the lidar** — `lidar3d.sh` starts only the driver + static TF | lidar healthy, `/scan_filtered` and `/scan` still dead | probes both separately, restarts `pointcloud_to_laserscan` and `scan_relay.py` |
| 3 | **Chassis driver absent**, so there is no `/odom` at all | "localization is wrong in RViz" — three layers from the cause: no `/odom` → no `odom->base_link` → slam_toolbox cannot publish `map->odom` | probes `/odom` + the TF, restarts, and the `WHY` block names the chassis |
| 4 | **`ranger_bringup` not found** when launched from a bare `source /opt/ros/jazzy/setup.bash` — the overlay lives in `ros2_ws/install` | a misleading "package not found", one wasted launch cycle | sources `bringup/env.sh` itself; **always** do the same |
| 5 | **slam_toolbox comes up UNCONFIGURED** — it is a lifecycle node | `/map` never publishes and everything downstream looks broken | runs `ros2 lifecycle set /slam_toolbox configure`, then `activate` |
| 6 | **Two Nav2 stacks, neither activated.** Repeated `ros2 launch` calls leave two `lifecycle_manager` instances contending for the same nodes, and the activation never completes | every goal comes back **"rejected in 0.0s"** and RViz shows an empty world. Silent three ways at once: `ros2 node list` shows a healthy-looking Nav2; `ros2 action list` **does** show `/navigate_to_pose`, because the action server is advertised *before* activation; and the empty RViz reads as an RViz config problem. `ros2 lifecycle get /bt_navigator` → `inactive [2]` is the only check that sees it | counts `bt_navigator`/`planner_server` processes and tears **all** of them down before starting one, then requires `bt_navigator`, `planner_server` and `controller_server` to report **active** — not merely present |
| 7 | **No initial pose.** `config/slam_os0.yaml` `map_start_pose` is an *atrium* coordinate | map loads, no `map->odom`, robot is nowhere | detects and says so — **you** set it: RViz 2D Pose Estimate (localization mode only) or `python3 bringup/relocalise.py` |
| 8 | **CAN authority.** `can0` can be UP while the chassis is still in RC mode, discarding every computer command *silently* | odom and the mux both look perfectly healthy; the robot does not move | detects via `chassis_mode.py` and says so — **you** flip SWB up and run `python3 bringup/claim_can.py` |
| 9 | **RViz shows nothing.** `os0_nav.rviz` displays only the Nav2 costmaps — no `/map`, no `/waypoint_markers` — so before Nav2 starts it renders an empty world | "the map did not load" | not touched. Use `nav2_bringup/elevator.rviz`, and set **Color Transformer: FlatColor** on the LaserScan — RViz defaults to colour-by-intensity and draws everything white |

**Read the goal status word — the three are not synonyms.** `rejected` means the action server would
not accept the goal at all, which is almost always lifecycle or config (row 6), not the world;
`aborted` means it tried and failed; `blocked` in `bringup/nav2_goto.py` means Nav2 `STATUS_ABORTED`
specifically, and is the verdict that starts reason → ground → press.

**When a component looks wrong, check for duplicates before starting another one.** Row 6 is one of
three instances of the same shape on 2026-09-04: two Nav2 stacks, two RealSense drivers racing for
the USB device (the loser logs "No RealSense devices were found"), and two `waypoint_markers`
publishers on one topic. A second `ros2 launch` is not a harmless retry.

Two more that read as errors and are not: `sudo ip link set can0 up` returning **"device busy"** means
can0 is *already* up; and touching the RC transmitter sticks reclaims RC authority at any moment,
including mid-run.

The same four layers by hand, if you want each driver's output in its own terminal:

```bash
ros2 launch ranger_bringup ranger_mini_v3.launch.py   # 1  chassis
bash bringup/lidar3d.sh                               # 2  OS0 -> /ouster/points -> /scan_filtered
bash bringup/camera.sh                                # 3  camera
bash bringup/safety.sh                                # 4  safety mux + arm gate
```

Then, **before anything moves**:

```bash
python3 bringup/health.py
```

It must report chassis mode `CAN` (not `RC`), `arm_stowed` at 100%, and exactly one publisher each on
`/odom`, `/scan`, `/cmd_vel`. It refuses rather than warns.

Record a route and run it (waypoints are odom-frame — record and run in one session):

```bash
python3 bringup/waypoints.py record start  --frame map
#  ... drive to the door ...
python3 bringup/waypoints.py record door   --frame map
#  ... drive to the press pose ...
python3 bringup/waypoints.py record button --frame map
python3 bringup/nav2_goto.py door --go     # one leg, on the saved map
bash bringup/session.sh campaign 50        # the full run
```

`--go` is required for any motion at all; without it every script prints what it would do. Full procedure,
including what to watch and when to stop it, is in **[docs/MORNING.md](docs/MORNING.md)**.

`ROS_DOMAIN_ID` is **9 for hardware, 42 for simulation**, set by `bringup/env.sh`. This separation is
not cosmetic — it is what keeps a simulated route from ever reaching a real chassis.

---

## Small changes that turn into big errors

The bring-up table above is faults that announce themselves. These are worse: **things that report
success while proving nothing.** A wrong value gets caught. A check that cannot fail gets believed.

### Checks that pass while proving nothing

| the check | why it proves nothing | what it produced |
|---|---|---|
| `grep -q active` on a lifecycle state | the string **`inactive` contains `active`** | a green Nav2 in the component table while every goal came back `rejected in 0.0s` |
| **existence** probe (`ros2 node list`, `ros2 topic list`) | a process being alive is not data flowing | the Ouster driver up and publishing **0.00 Hz** — a stale process holding the UDP socket across a power cycle. Surfaced three layers away as "localization is wrong in RViz" |
| `can_transform(..., timeout=0)` | a just-started node has not finished **DDS discovery**, and a latched `/tf_static` arrives only after it does — so timeout 0 times your own subscription setup, not the transform | `base_link -> os_lidar` reported absent while `tf2_echo` resolved it instantly in the next terminal. Hours into a TF tree that was fine. **Fixed by giving it 12 s** |

`ros2 topic hz` is also not a rate probe: it has reported 1.7 Hz and 10.0 Hz for the same topic
minutes apart against a repeatable 6.4 Hz. Measure with a counting subscriber — `stack.sh`'s
`rate()` is the reference. The shape underneath all of it: **ask the question that can come back
"no".** A check that cannot fail is worse than no check, because it ends the investigation.

### A config file is code that nothing compiles

`config/floors.yaml` named map `elevator_f2` and waypoints `call_button_f2` / `car_panel_f2`. None
of those existed — the map on disk is `maps/floor2.*` and the waypoints are **`f2_`-prefixed**, not
`_f2`-suffixed. The entry had been written from the naming convention in the file's own header
rather than from what was actually recorded. Uncaught, it would have surfaced as
`floor_swap.py --to 2 --go` failing to find its seed pose **with the robot already inside the lift
car**, doors about to close.

`python3 bringup/floor_swap.py --check` caught it, and is pure offline Python — no ROS, no robot.
Run it after every edit to that file.

### A seed pose is a claim about the world, and in a lift car it is the only one

The floor swap seeded `map_start_pose` from `car_facing_out` unconditionally. The robot actually
rides parked at `car_panel` — **0.48 m and 116° away**. That error is not self-correcting: a closed
car's scan is four blank walls about a metre away, so there is nothing in it to pull the estimate
back. The matcher converges confidently onto whatever seed it was given, and the mistake only
becomes visible once the doors open and the robot drives. `--seed-role` now names where the robot
physically is.

### A `.pgm`/`.yaml` map with no `.posegraph` looks exactly like a map

`slam_toolbox` `mode: localization` relocalizes by deserializing the `.posegraph`. Handed only a
grid it does **not** error — it starts a new, empty graph at the robot's feet, publishes `/map`,
and reports `active`. `maps/atrium.*` is such a map, and it is the one the ADA-button waypoints
were recorded in, so those waypoints name coordinates in a frame that no longer exists.

Two such maps also **cannot be merged**: a pose graph is a chain of scan matches from one
continuous drive, and there was never a drive linking the lift lobby to the ADA door. One
continuous drive is the only fix — that is `floor1`, and
**[docs/FLOOR1_REMAP.md](docs/FLOOR1_REMAP.md)** is the procedure.

### slam_toolbox keeps its LAUNCH-TIME parameters

Editing `config/slam_os0.yaml` after the node is running changes nothing. The robot stayed
mis-localized and it was reported as fixed. Restart the node, then verify with `ros2 param get`.

### A mapping drive lives only in RAM until it is serialized

`slam_toolbox` holds the pose graph in memory and serializes only on request. On 2026-09-05 a
floor-1 mapping drive died with its session: no partial map, nothing on disk, nothing to salvage,
the walk repeated from scratch. Nothing was recording, so there was never a second chance to have.

```bash
bash bringup/map_insurance.sh start floor1     # BEFORE the drive, in its own terminal
```

It records `/scan /scan_nav /tf /tf_static /odom` — what slam_toolbox actually consumes, ~1–2 MB
per minute — so the drive can be replayed and rebuilt offline. `ros2 bag record -a` is not an
option here: `/ouster/points` is 3.1 MB per cloud and the one such bag in `runs/` is 3.4 GB.

### Known trap: `session.sh` step 0 pings the xArm for every session type

`bringup/session.sh:66-68` pings `192.168.1.119` (lidar), **`192.168.1.221` (xArm)** and
`192.168.1.1` (router) and dies on the first unreachable one — before it looks at the command word.
So a **nav-only or mapping-only** run is blocked whenever the arm is powered off, which is the
normal state for a mapping drive. It presents as `192.168.1.221 unreachable`, which reads as a
fault on the one cable that also carries the lidar.

**Proposed, not applied:** keep `.119` and `.1` fatal for every session, demote `.221` to a warning
for `map` and `nav`, and leave it fatal for `campaign` and anything that presses. Not applied here
because `session.sh` is live; it belongs to whoever is next at the console.

---

## Known-open

1. **The press.** Untested since the READY → LOOK → GROUND → REACH reorder. This is the one thing
   between here and a complete run.
2. **Lidar re-anchoring** (`waypoints.py anchor` / `relocalize`). Written, unit-tested, never run on
   the robot. It is what removes the re-record-before-every-run tax — which is caused by the odom
   frame not surviving a power-cycle, not by drift within a run.
3. **Isaac depth is dead on this laptop** — the depth topic publishes 100% `inf` on every frame; the
   Isaac log names it (`Illegal cycle connection … WriterSyncGate … ignored`, an SDG graph problem).
   Fixing it would mean editing the simulation repo, which project policy forbids. Consequence: the
   sim can validate navigation and the state machine, but **cannot** validate grounding. This is why
   `safety/lidar_lift.py` exists — it lifts a 2D box to 3D from lidar when depth has nothing, which
   also covers glass doors, where depth genuinely fails on hardware too.
4. **Elevator interaction.** The lift has never been ridden. Floor 2's map (`maps/floor2.*`) and
   its five `f2_` waypoints exist and all four nav legs have been driven; the in-car press has
   been tuned by hand. The handover itself is untested.
5. **Floor 1 is mid-re-map.** It was two maps — `elevator` (lift lobby, relocalizable) and
   `atrium` (ADA door, `.pgm`/`.yaml` only, so not relocalizable and not mergeable). One
   continuous drive covering both is being recorded as `floor1`. `config/floors.yaml` already
   names it and its five `f1_` waypoints, so `floor_swap.py --check` fails until the drive is
   done — deliberately. `bringup/elevator_route.sh` still drives the old unsuffixed names and
   will need updating after. See [docs/FLOOR1_REMAP.md](docs/FLOOR1_REMAP.md).

---

## Reading order for a reviewer

1. `EXPERIMENT_LOG.md` — dated findings, including everything that failed.
2. `docs/MORNING.md` — current state and the exact runbook.
3. `safety/route_plan.py` and `safety/reach_envelope.py` — the mission and geometry rules, with the
   constants ported from the simulator and the reasoning for each.
4. `tests/test_press_veto.py` — the safety case that matters most: never press a fire alarm.
