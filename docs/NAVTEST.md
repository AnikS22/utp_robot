# Navigation test — exact commands

Navigation only. No arm, no camera, no press.

## Every terminal, first line

```bash
cd ~/utp_robot
source bringup/env.sh
```

That is the whole setup. **There is no venv and no conda environment for this** — the navigation
stack runs on the system python at `/usr/bin/python3`, because `rclpy` is installed against it by
the ROS 2 apt packages and cannot be imported from a conda or venv interpreter. `env.sh` actively
STRIPS conda from PATH and unsets `PYTHONPATH`/`CONDA_PREFIX` for exactly this reason: colcon and
`ros2` run whatever `python3` comes first, and conda's has no rclpy bindings, which produces
errors that name neither conda nor python.

(The pipeline repo's venv at `~/unlocking-the-path/env/.venv` is a different thing — it holds
torch for the grounder, and only the press shells out to it. Navigation never touches it.)

`env.sh` also sets `ROS_DOMAIN_ID=9`. That matters more than it looks: unset means domain 0, an
empty graph, where every node starts happily and finds nothing. The scripts now refuse to run
without it rather than blaming the driver.

`cd ~/utp_robot` is only needed for the relative paths below; absolute paths work from anywhere
once `env.sh` is sourced.

## Once per session — bring the stack up

Four terminals. **Leave them running.** Restarting `ranger_base` re-zeroes odom and invalidates
every waypoint you recorded, which is the single most expensive mistake available here.

```bash
# 1  chassis
ros2 launch ranger_bringup ranger_mini_v3.launch.py

# 2  lidar (also starts the rear-sector filter -> /scan_filtered, which the corridor veto needs)
bash bringup/lidar.sh

# 3  safety mux + arm gate.  UTP_ARM_BACKEND=absent = "no arm is fitted", and the node
#    REFUSES to start if anything answers at the arm's IP, so the declaration cannot be false.
UTP_ARM_BACKEND=absent bash bringup/safety.sh

# 4  check before every run
python3 bringup/health.py --skip-arm
```

`health.py` must show:

| check | required |
|---|---|
| `chassis mode` | **CAN** — if it says RC, flip **SWB up**; in RC the chassis silently discards every command |
| `gate arm_stowed` | **100%** |
| `/odom` | ~50 Hz, **1 publisher** |
| `/scan` | ~6 Hz, **1 publisher** |
| `blocking:` | only `no_source` |

## A — straight line, no setup

Start point is wherever the robot is standing. There is nothing to set.

```bash
python3 bringup/twopoint.py                      # dry run, prints P1 and P2, no motion
python3 bringup/twopoint.py --go --pause         # stops at each end so you can measure
python3 bringup/twopoint.py --go --laps 10 --tol 0.05   # repeatability
```

Mark the floor first — **two marks per point**, not one. One gives position error only; two also
give heading error, and on a 4WS base heading is what accumulates. P2 is `--dist` metres (default
2.0) forward along the heading the robot has when you start.

## B — curvy path, any number of points

Drive by RC (**SWB down**), stopping at each pose you want, and record it. Recording works fine
under RC: odometry comes from the wheel encoders regardless of who is commanding.

```bash
python3 bringup/waypoints.py record p1     # at the start pose
#   ... drive to the next spot ...
python3 bringup/waypoints.py record p2
#   ... drive to the next spot ...
python3 bringup/waypoints.py record p3

python3 bringup/waypoints.py list          # check they are all there
```

Then **flip SWB up** (the chassis will not obey the computer otherwise) and drive it:

```bash
python3 bringup/route_run.py --goto p1,p2,p3            # dry run
python3 bringup/route_run.py --goto p1,p2,p3 --go --confirm
```

`--confirm` pauses before every leg: Enter runs it, `q` stops. Drop it once you trust the path.

An ad-hoc `--goto` route is parsed, validated and session-checked exactly like a route in
`config/routes.yaml`. Convenience is not a second, less-checked way to move the robot.

## You will not park it back on the start mark, and you do not have to

Hand-driving the robot back to "roughly where it started" is the normal case -- parking it on the
exact recorded pose by RC is not realistic. That is what the recorded `start` waypoint is for:
put it FIRST in the route and the robot squares itself onto the exact recorded start pose,
position and heading, before it sets off.

```bash
python3 bringup/route_run.py --goto start,finish --go --confirm
```

It corrects heading even when it is already inside the 15 cm arrival radius -- the controller
runs `final_heading` before `arrived`, so a robot parked on the right spot facing 20 deg off
still turns to match. Without that leading leg, every run starts from a different pose and the
finish scatter is measuring your RC parking, not the navigation.

For repeat runs, which is the same need:

```bash
python3 bringup/route_run.py --goto start,finish --loops 10 --go
```

That drives start -> finish -> start -> finish..., re-squaring on the recorded start every lap,
so run 1 and run 100 are actually comparable. Drop `--confirm` once you trust it, or keep it and
press Enter per leg.

## The path between waypoints is straight

Each leg is turn-to-bearing, then drive, then settle on the final heading. A "curvy" path is
therefore a polyline: more waypoints, closer together, is how you follow a curve. That is also
where the error goes — on a 4WS base odometry degrades in TURNS, not in distance, so ten short
legs drift more than one long one.

## If it will not move

Every one of these prints the reason and stops, rather than timing out:

| message | meaning |
|---|---|
| `chassis control_mode=RC` | SWB is down. The chassis is discarding commands; ROS cannot see this. |
| `STALE WAYPOINTS` | `ranger_base` restarted since you recorded. Re-record, or `waypoints.py rebase` if the robot has not physically moved. |
| `safety mux is blocking: arm_not_stowed` | the arm gate. Check `health.py`. |
| `no /safety/status` | the safety stack is not running (terminal 3). |
| `no /scan_filtered` | the lidar filter is not running; the corridor veto would be inactive, so it refuses. `--no-veto` overrides deliberately. |
| `corridor blocked` | something is inside the 0.90 x 0.80 m box ahead. |
