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

## Obstacles: steer, then ask

Three tiers, and each hands off to the next only when it genuinely cannot answer.

```bash
# 1. stop at obstacles (default) -- safe, and what the first successful run used
python3 bringup/route_run.py --goto start,finish --go

# 2. steer around them, using the live scan only. No map, no localisation.
python3 bringup/route_run.py --goto start,finish --avoid --go

# 3. and when there is no way around, ASK the pipeline what it is
python3 bringup/route_run.py --goto start,finish --avoid --on-blocked press_and_pass --go
```

**Why tier 3 is the interesting one.** A gap the robot fits through is arithmetic. A closed door
is not -- no amount of steering opens it, and a 2D lidar cannot tell a shut door from a wall
because geometrically they are the same thing. So local avoidance reporting "no way around" is
precisely the evidence that the problem is semantic, and that is the moment to stop reasoning
about shapes and ask what the obstruction MEANS.

What the answer may authorise is one pre-written, pre-validated route, after which the leg is
retried. The VLM chooses between reviewed plans -- act, or stop. It never composes motion.

It stops, rather than acting, whenever the answer is ambiguous:

| situation | what happens |
|---|---|
| VLM unreachable or unparseable | stop. Guessing in front of a glass door is the wrong way to be wrong. |
| lidar says blocked, VLM says clear | stop, and report the disagreement. Glass, an obstacle under the scan plane, or a mis-set lidar height all look like this. Picking whichever sensor suits us is how a robot ends up in a door. |
| `--escalations` budget spent (default 2) | stop. Repeating an action that did not work is not a plan. |

### Looking ahead: `--look-first`

MEASURED on this robot, 2026-08-29, pointed at closed glass double doors ~9 m away:

| sensor | says |
|---|---|
| camera | `{"kind": "door", "description": "closed glass double doors", "blocked": true}` |
| lidar | a return at 7.79 m; `corridor_blocked` **False**; `local_avoid` "clear toward the goal" |
| depth | doors at **8.96 m** (frame centre median) |

Read that carefully, because the obvious reading is wrong. This is **not** the lidar failing to
see glass. Both sensors saw the same thing at ~8-9 m. The veto box is 0.90 m and the avoidance
horizon 2.0 m, so neither geometry layer had any business reacting yet, and did not.

What `--look-first` actually buys is **knowing what is coming while there is still room to act on
it**. A door that needs a button pressed is a different PLAN, not a different steer, and that
decision is better made at 9 m than at 0.9 m. Use it on legs that end at a door.

```bash
python3 bringup/route_run.py --goto start,finish --avoid --look-first \
    --on-blocked press_and_pass --go
```

One VLM call per leg (~6 s). A clear look costs no escalation budget and just drives.

Glass being invisible to 2D lidar is a real risk (site risk S1) and this covers it -- but it has
NOT been demonstrated on this robot, and these doors carry tape precisely so that it is not the
failure mode here.

**What `--avoid` cannot do**, and no amount of tuning fixes: it has no memory and no map, so a
U-shape or a dead end traps it -- it steers into the pocket, finds no gap and stops. It cannot
see glass. And every detour adds turns, which is where 4WS odometry degrades, so a long way round
makes the goal coordinate itself less trustworthy. Avoidance is not free.

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
