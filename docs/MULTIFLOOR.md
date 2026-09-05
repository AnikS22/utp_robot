# Multi-floor navigation — riding the lift

**Status: nothing in here has run on the robot.** The single-floor elevator route
(`bringup/elevator_route.sh`) has been driven; the lift has never actually been ridden by anything
in this repo. This document describes a design with unit tests behind it, not an observation.
Update it with what happens when it is first run, and log the run in `EXPERIMENT_LOG.md`.

---

## The one fact everything here follows from

**A lift car is geometrically identical on every floor.**

With the doors shut the lidar sees four walls about a metre away (`docs/MORNING.md` measures the
car's side walls at 1.00–1.15 m) and nothing else. That scan matches the car in floor 1's map
exactly as well as it matches the car in floor 2's map.

So a scan-match fit taken inside a closed car is a **confident, well-formed answer to a question
nobody asked**. It is the `docs/AGENT_BRIEF.md` failure — "a device that answers is not a device
that works" — in its purest available form, and it is worse than no number at all, because a number
that high ends the investigation.

The floor becomes observable only when the doors open and the lobby comes into view. Every design
decision below is downstream of that sentence.

## Why a ride is not a navigation problem

Every other leg in this stack moves the robot continuously through one map, and both halves of the
localization stack assume it: slam_toolbox correlates each sweep against the last, and a Nav2
costmap is a picture of one floor. A ride breaks both at once — the robot is carried several metres
vertically into a room the loaded map does not contain, with no scans in between.

There is no trajectory and nothing to match against, so there is nothing for Nav2 to do. A ride is
a **handover between two maps**, and only two questions matter: which map is live, and has the
claim "we are on floor N" actually been checked.

## Why one map per floor, and why the SLAM node is restarted

Two floors cannot share a slam_toolbox pose graph — a graph is built from one continuous drive, and
there is no drive between floors. So each floor is its own saved map with its own origin, as
unrelated as `atrium` and `elevator` already are.

Given that, the handover could be done two ways, and the choice is not cosmetic:

| | effect on `safety/map_frame.py` |
|---|---|
| `/slam_toolbox/deserialize_map` on the running node | the node keeps its DDS GID, so `pose_source.slam_session_id` is unchanged — and **floor 1's waypoints go on validating after the robot reaches floor 2** |
| **restart the node on the new map** | new GID, new session id, so every floor-1 waypoint is refused the instant the swap happens, by machinery that already exists and is already tested |

`bringup/floor_swap.py` restarts. The correct behaviour falls out of the existing safety machinery
instead of being added on top of it.

It also deletes `maps/.loaded_map` **before** it stops anything, so a swap that dies half-way leaves
nothing certified rather than leaving floor 1's certification standing over a robot on floor 2.
Fail-closed means the failure state is the safe one, and here that state is "no map is loaded".

---

## Before any of this can run

`bringup/floor_swap.py --check` refuses today, and that is the intended state:

```
$ python3 bringup/floor_swap.py --check
config/floors.yaml is NOT drivable as it stands:
  - floor 2: map 'elevator_f2' is missing maps/elevator_f2.{pgm,yaml,posegraph,data}
  - floor 2: waypoint 'call_button_f2' (role call_button) is not recorded
  ...
```

**The destination floor's map does not exist and neither do its waypoints.** That is the whole of
the remaining work, and it is an afternoon with the robot, not a coding task.

### 1. Map the destination floor

Take the robot up in the lift by hand (RC transmitter, arm stowed) and map that floor exactly as
`docs/MAPPING.md` describes. Nothing about the procedure changes:

```bash
bash bringup/session.sh map                 # every layer, in the order that works
python3 bringup/map_watch.py                # another terminal, while driving
# ... drive it: DualAckermann selected BEFORE moving, broad turns at <= 0.25 m/s,
#     pause after turns, and CLOSE THE LOOP past your start by a different route ...
bash bringup/map_persist.sh save elevator_f2
```

Include the lift lobby **and the inside of the car** — with the doors held open, drive in, and let
the matcher see the car interior. The car has to be in the map or the seed pose has nothing to
land on.

### 2. Record that floor's five waypoints, in the same session

`map_persist.sh save` prints this reminder for a reason: a recording made after slam_toolbox
restarts is anchored to a different origin.

```bash
python3 bringup/waypoints.py record call_button_f2       --frame map
python3 bringup/waypoints.py record lift_door_reverse_f2 --frame map
python3 bringup/waypoints.py record car_facing_out_f2    --frame map
python3 bringup/waypoints.py record car_panel_f2         --frame map
python3 bringup/waypoints.py record lift_door_f2         --frame map
```

**The names must not collide with floor 1's.** `maps/waypoints.yaml` is one flat store shared by
every map, so recording a second `car_panel` overwrites the first and both floors then drive to
whichever survived. `check_building()` refuses a config that reuses a name; the `_f2` suffixes exist
for exactly this.

`car_facing_out_f2` is the important one — it is the **seed pose**, the thing the whole swap rests
on. Record it with the robot parked in the car the way it will be parked when it rides: in the
middle, square, facing the doors.

### 3. Check it offline, with the robot switched off

```bash
python3 bringup/floor_swap.py --check          # config, maps and waypoints agree
python3 bringup/floor_swap.py --plan 1 2       # the whole ride, printed
python3 bringup/check_waypoint.py call_button_f2 car_panel_f2 lift_door_f2   # footprints
```

`check_waypoint.py` reads each waypoint's own `map_name`, so it checks floor 2's poses against
floor 2's grid while you are standing anywhere.

---

## Running it

Bring the stack up on the **origin** floor's map, as usual:

```bash
bash bringup/session.sh up
MAP_NAME=elevator bash bringup/session.sh nav
python3 bringup/health.py                 # chassis CAN, arm_stowed, one publisher each
bash bringup/multifloor_route.sh --dry-run
bash bringup/multifloor_route.sh          # HAND ON THE RC
```

`--from` / `--to` pick the floors; the default is `1 -> 2`, and `--from 2 --to 1` comes back down.

### What the operator does, and when

The route stops and waits at four points. Three of them are doors.

| prompt | what to do |
|---|---|
| `DOORS -- hold them open` | hold the lift doors while the robot reverses in |
| `DOORS -- let them CLOSE` | let go. **The swap must not run with the doors open** — an open door lets the scan out into the origin floor's lobby, which is floor-1 geometry handed to a matcher holding a floor-2 seed |
| *(the swap runs here, during the ride)* | nothing — it takes tens of seconds and it is spending the ride, not the door hold |
| `DOORS -- the car has arrived and the doors are open` | hold them, and check the floor indicator **by eye** |

Then the gate runs. It is the only step in the whole route that checks which floor this is.

### If the gate refuses

```
GATE REFUSED. The base must not move.
```

Read the reason it prints. The three that matter:

* **"the doors were not observed open"** — the score is not about the floor. Wait for the doors and
  score again. Do not raise the threshold; the number is not too low, it is about the wrong thing.
* **"maps/.loaded_map says 'elevator' but this floor is 'elevator_f2'"** — the swap did not run or
  loaded the wrong map. Do not drive: those origins are unrelated and the exit waypoint names a
  different physical place.
* **"localization fit is NN%"** with the doors open — this *is* a real statement, and it says the
  robot is not where the seed put it. Check the floor indicator by eye first. If the floor is right,
  seed by hand (RViz **2D Pose Estimate**) or run `python3 bringup/relocalise.py`, then re-run
  `python3 bringup/floor_swap.py --verify 2 --doors-open`.

### Recovering a half-finished swap

`maps/.loaded_map` absent is the deliberate failure state: every map-frame waypoint is refused and
`nav2_goto.py` will not drive anywhere. Recover by hand, on whichever floor the robot is actually
standing on:

```bash
MAP_NAME=elevator_f2 bash bringup/session.sh nav
```

---

## Known-open

1. **The floor-button query cannot tell floor buttons apart.** `config/floors.yaml` uses
   `"the blue elevator button"`, which measured 0.443 against 0.267 for the tape-naming variant on
   2026-09-03 — but a panel has one button per floor and they are all the same blue. The grounder
   will return whichever it likes best. This is the fire-alarm failure shape (confident,
   well-formed, wrong object) with `press_veto.py` having nothing to say, because a wrong floor
   button is not a dangerous object. For a two-button lift the odds are what they are and
   `press_run.sh` shows the chosen box before the arm moves. **Do not read a successful demo as
   evidence that this query is discriminative.** The honest fix is spatial — panel buttons are
   vertically ordered and the order is known — which is a geometry question about the grounder's
   output, not a better sentence.
2. **Arrival is declared by the operator.** Nothing reads the floor indicator or counts door
   cycles. The operator's word starts the gate; the geometry check is what passes it, so a wrong
   declaration is caught rather than acted on — but an unattended run needs a real arrival signal.
   `bringup/doors_open.py` is wired in behind `floor_swap.py --verify --check-doors` and is a
   starting point, not a solution: it answers "are the doors open", not "which floor".
3. **The costmap swap is untested.** `floor_swap.py` clears both Nav2 costmaps after the handover
   and checks that the published `/map` matches the destination's saved grid in size and origin —
   which catches the documented silent failure where slam_toolbox comes up ACTIVE on a fresh empty
   graph. Whether Nav2's static layer picks up a whole new grid cleanly on a live system has not
   been observed.
4. **Two floors only, in a straight line.** `plan_ride` handles any itinerary (`--plan 1 2 1` is a
   round trip) but nothing chains rides that need an intermediate wait, and no floor beyond the two
   in `config/floors.yaml` exists.

## The files

| | |
|---|---|
| `safety/floor_plan.py` | pure logic: config validation, the ride plan, the handover gate. No ROS |
| `bringup/floor_swap.py` | the plumbing: `--check`, `--plan`, `--to`, `--verify` |
| `bringup/multifloor_route.sh` | the route. Linear, like `elevator_route.sh`, and for the same reason |
| `config/floors.yaml` | the building: one entry per floor |
| `tests/test_floor_plan.py` | 43 headless tests, including the script/plan agreement check |

`tests/test_floor_plan.py` asserts that `multifloor_route.sh` executes exactly the steps
`plan_ride()` plans, in that order. Reordering one without the other fails there rather than in a
lift.
