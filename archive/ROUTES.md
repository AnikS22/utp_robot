# Running missions without a map

Waypoints on odometry, actions closed by vision. This is what the experiment runs on instead of
SLAM + Nav2, and it is a deliberate trade, not a workaround.

## Why

`slam_toolbox` could not hold a pose in this building (2026-08-25). Two measured reasons:

| | |
|---|---|
| scan smear | the A1M8 sweeps for **145 ms** and slam_toolbox treats that as instantaneous. `map->odom` correction went from **0.1 cm** per half-second below 0.10 rad/s to **13.7 cm** above 0.40 — 107x |
| corridor ambiguity | ~100 valid points per scan match nearly as well at many positions *along* a corridor, so the estimate flips between them |

**Nav2 was never the problem — it has never run on this robot.** It consumes a map and a pose; the
failure is upstream of it. Writing a different planner would not touch either cause.

## The principle

**Waypoints are approximate. Actions are visual.**

A leg only has to park the robot with the target **in the camera frame** — roughly ±0.3 m and
±15°. Everything that touches the world is closed by the grounder and the visual servo, which
repeated to **3 mm across four consecutive runs** on a real ADA plate. Odometry drift over a
15–20 m leg sits well inside that.

That moves the accuracy requirement to the one place accuracy has actually been measured.

## What it will not do

It will **not** plan around an obstacle. That needs a costmap, which needs the localisation we
just said we do not have. A blocked corridor **stops the route and says so**. Halting on an
unexpected obstruction is defensible; improvising a detour on a pose estimate we do not trust is
not.

## THE ONE THING THAT WILL BITE YOU

Waypoints are stored in the **`odom` frame**. `odom` is zeroed every time `ranger_base` starts.

**Restart the driver and every waypoint silently becomes wrong** — not missing, *wrong*, pointing
at wherever the robot happened to be. Record and drive inside one continuous driver session. If
the driver restarts, re-record.

## Procedure

### 1. Bring up sensing and the base (no SLAM needed)

```bash
source ~/utp_robot/bringup/env.sh
bash bringup/lidar.sh --no-tf &                 # /scan, for the corridor veto
bash bringup/camera.sh &                        # the actions need this
ros2 launch ranger_bringup ranger_mini_v3.launch.py use_sim_time:=false publish_odom_tf:=true
```

### 2. Record waypoints by driving to them

Drive on the RC. At each spot that matters:

```bash
python3 bringup/waypoints.py record door_approach
python3 bringup/waypoints.py list
python3 bringup/waypoints.py where              # sanity: distance and bearing to each
```

Record the pose you want the robot to *end up in*, including heading — the action runs from here,
so park it as you would want it parked.

### 3. Write the route

`config/routes.yaml`:

```yaml
routes:
  atrium_door:
    - goto: door_approach
    - action: press_button
      query: "the accessible door push button"
      standoff: 60
    - wait: 6
    - goto: through_door
```

### 4. Validate — this needs no robot

```bash
python3 bringup/route_run.py atrium_door        # dry run: validates, plans, moves nothing
```

Every unknown waypoint and action is reported **at once**, before anything moves. Fixing a route
one error per run means one drive per typo.

### 5. Drive it

```bash
bash bringup/teleop.sh &                        # the mux: route_run publishes THROUGH it
python3 bringup/route_run.py atrium_door --go
```

Preconditions, all of which fail closed:
- the **mux** is running — `route_run` publishes to `/cmd_vel_teleop`, never straight to `/cmd_vel`
- the **arm is stowed** — gated on measured joint angles
- **SWB on the RC is in command mode** — while the RC holds authority the chassis ignores CAN
  motion entirely and the robot will sit there doing nothing

## Files

| | |
|---|---|
| `safety/route_plan.py` | pure sequencing and validation, no ROS. 13 tests |
| `safety/waypoint_drive.py` | pure control: turn-then-drive, corridor veto. 18 tests |
| `bringup/waypoints.py` | record / list / where / goto |
| `bringup/route_run.py` | the executor |
| `config/routes.yaml` | the missions |
| `maps/waypoints.yaml` | recorded poses, odom frame |
