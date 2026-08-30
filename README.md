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
   route  ──▶ │ route_run.py    waypoints + actions in order │
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
               route_run.py    the route executor
               waypoints.py    record / list / anchor odom-frame waypoints
               press_run.sh    READY → LOOK → GROUND → REACH
               reach_control.sh  ground, position, re-ground, press
config/        routes.yaml (missions) · safety.yaml (gates, speed and slew ceilings) · slam.yaml
nav2_bringup/  Nav2 config, ported for real hardware (use_sim_time:=false, spin removed)
docs/          runbooks — start with MORNING.md, then NAVTEST.md and ROUTES.md
tests/         243 tests, all headless: no ROS, no GPU, no hardware
patches/       our diffs against upstream drivers, applied by setup_workspace.sh
maps/          site maps from the SLAM attempt (see maps/README.md)
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
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/ -q     # 243 passed
```

With hardware, four terminals, left running:

```bash
ros2 launch ranger_bringup ranger_mini_v3.launch.py   # 1  chassis
bash bringup/lidar.sh                                 # 2  lidar + /scan_filtered
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
python3 bringup/waypoints.py record start
#  ... drive to the door ...
python3 bringup/waypoints.py record doors
#  ... drive to the press pose ...
python3 bringup/waypoints.py record button
python3 bringup/route_run.py press_and_out --go --confirm
```

`--confirm` pauses before every step; `--go` is required for any motion at all. Full procedure,
including what to watch and when to stop it, is in **[docs/MORNING.md](docs/MORNING.md)**.

`ROS_DOMAIN_ID` is **9 for hardware, 42 for simulation**, set by `bringup/env.sh`. This separation is
not cosmetic — it is what keeps a simulated route from ever reaching a real chassis.

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
4. **Elevator interaction.** Never run on any system.

---

## Reading order for a reviewer

1. `EXPERIMENT_LOG.md` — dated findings, including everything that failed.
2. `docs/MORNING.md` — current state and the exact runbook.
3. `safety/route_plan.py` and `safety/reach_envelope.py` — the mission and geometry rules, with the
   constants ported from the simulator and the reasoning for each.
4. `tests/test_press_veto.py` — the safety case that matters most: never press a fire alarm.
