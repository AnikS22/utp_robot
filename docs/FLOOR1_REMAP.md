# Re-mapping floor 1 as `floor1`

**Status 2026-09-05: not done yet.** `config/floors.yaml` already names `floor1` and the five
`f1_` waypoints, so `python3 bringup/floor_swap.py --check` **fails**, listing exactly what is
missing. That is the intended fail-closed state, not a bug. Do not "fix" it by pointing floor 1
back at `elevator`.

## Why floor 1 is being re-mapped at all

Floor 1 was two maps, and neither could do the job:

| on disk | what it is | can localize into it? |
|---|---|---|
| `maps/elevator.*` | the lift lobby — `.pgm` `.yaml` `.posegraph` `.data` | **yes** |
| `maps/atrium.*` | the ADA door area, where the ADA-button task waypoints were recorded — `.pgm` and `.yaml` **only** | **no** |

A `.pgm`/`.yaml` pair is a **picture**. `slam_toolbox`'s `mode: localization` relocalizes by
deserializing the `.posegraph`; handed only a grid it does **not** error. It starts a brand-new,
empty pose graph whose origin is wherever the robot is standing, publishes `/map`, reports
`active`, and looks completely healthy. So the ADA map could never localize, and every waypoint
recorded in it names a coordinate in a frame that no longer exists — a fresh-SLAM frame wearing
the saved map's name.

**And the two could not be merged.** A pose graph comes from one continuous drive: it is a chain
of scan matches. There has never been a drive linking the lift lobby to the ADA door, so there is
no chain to merge along. Aligning the two grids by hand would produce exactly the thing described
above — a bigger picture, still no graph.

One continuous drive covering **both** areas is the only fix. That drive is `floor1`.

**The old maps are kept.** `maps/elevator.*` and `maps/atrium.*` stay on disk as backup.
`elevator` is still the only floor-1 map that has ever actually been driven, and it remains the
only one that works until `floor1` exists and passes `--check`.

## The order, and why it is this order

Each step depends on the one before it in a way that is invisible if you do them out of order.

### 1. Map both areas in ONE continuous drive

```bash
bash bringup/map_insurance.sh start floor1     # in its own terminal, FIRST
bash bringup/session.sh map
python3 bringup/map_watch.py                   # another terminal, watch it fill
```

Start `map_insurance.sh` **before** the drive. `slam_toolbox` holds the pose graph in RAM and
serializes only when asked, so until step 2 the entire drive exists nowhere else. A floor-1
mapping drive was lost exactly this way on 2026-09-05 — session died, nothing on disk, walk
repeated. The recorder costs 1–2 MB/min.

Driving discipline (`CLAUDE.md`, "Mapping data path"): stow the arm, hold the RC, select
**DualAckermann before moving**, broad turns at ≤0.25 m/s, pause after each turn.

**It must be one drive with the lift lobby and the ADA door in it, and it must close the loop** —
come back past the start by a different route. `do_loop_closing` only corrects drift when
slam_toolbox recognises a place it has already seen; an out-and-back gives a subtly bent map and
every waypoint inherits the bend. A drive that covers only one of the two areas reproduces the
exact problem this whole exercise exists to remove.

Glass doors will not appear in the scan. Mark them as keepouts by hand afterwards.

### 2. Save as `floor1` — WITHOUT stopping slam_toolbox

```bash
bash bringup/map_persist.sh save floor1
bash bringup/map_insurance.sh stop
ls -l maps/floor1.pgm maps/floor1.yaml maps/floor1.posegraph maps/floor1.data
```

All four files or none. If `.posegraph`/`.data` are absent you have made another `atrium` — a
picture — and steps 3–6 will all appear to work while the robot cannot localize.

`map_persist.sh` will refuse to overwrite an existing `floor1` under any of the four extensions
and will name which ones it found; that guard is deliberate, do not set `UTP_MAP_OVERWRITE` to
get past it unless you mean to discard the map that is there.

### 3. Delete the five old unsuffixed floor-1 waypoints

Edit `maps/waypoints.yaml` and remove, entirely:

```
call_button   car_facing_out   car_panel   lift_door   lift_door_reverse
```

**Delete them, do not rename them.** Their coordinates are in `elevator`'s frame. `floor1` is a
different map with an unrelated origin, so the same numbers name a different physical place. A
rename would produce five well-formed waypoints that pass every check and drive the robot into a
wall.

Do this **before** recording, not after: waypoint names are global across every map
(`maps/waypoints.yaml` is one flat store), and leaving the old names in place is how a later
floor silently overwrites an earlier one.

> `bringup/elevator_route.sh` drives the old names and will stop working the moment they are
> gone. That is expected and is not fixed here — see "What this breaks" below.

### 4. Record the five `f1_` waypoints, in ONE localization session on `floor1`

```bash
MAP_NAME=floor1 bash bringup/session.sh nav      # or: MAP_NAME=floor1 bash bringup/stack.sh
```

Confirm it is **localized into the saved map**, not running fresh: `maps/.loaded_map` must read
`floor1`, and the fit must be sane (`python3 bringup/relocalise.py` — above ~80% is localized,
50% is lost). A fresh-SLAM session will happily let you record five waypoints that carry no map
name and are portable nowhere.

Then drive to each spot and record, **without restarting slam_toolbox at any point**:

```bash
python3 bringup/waypoints.py record f1_call_button       --frame map
python3 bringup/waypoints.py record f1_lift_door_reverse --frame map
python3 bringup/waypoints.py record f1_car_facing_out    --frame map
python3 bringup/waypoints.py record f1_car_panel         --frame map
python3 bringup/waypoints.py record f1_lift_door         --frame map
```

Roles, from `safety/floor_plan.py` — these are the names `check_building()` requires, not a
preference:

| role | waypoint | where the robot stands |
|---|---|---|
| `call_button` | `f1_call_button` | outside the lift, facing the call plate |
| `door_reverse` | `f1_lift_door_reverse` | outside, **back** to the doors — floor 1 reverses in |
| `car_facing_out` | `f1_car_facing_out` | inside the car, facing the doors. **The seed pose** |
| `car_panel` | `f1_car_panel` | inside, square to the button panel |
| `exit` | `f1_lift_door` | back outside the doors |

`door_reverse` is required. `REQUIRED_ROLES` waives it only for a floor that enters nose-first and
therefore defines **both** of `FORWARD_ENTRY_ROLES` (`door_facing`, `car_facing_in`). That is
floor 2. Floor 1 reverses into the car, so all five names must exist.

`f1_car_facing_out` is the one that carries the most weight: it is the default `--seed-role`, the
pose a floor swap hands `slam_toolbox` as `map_start_pose`. If the robot will actually ride parked
at the panel, seed with `--seed-role car_panel` instead — see the trap in `CLAUDE.md`.

Then check the footprints before trusting any of them:

```bash
python3 bringup/check_waypoint.py f1_call_button f1_lift_door_reverse \
        f1_car_facing_out f1_car_panel f1_lift_door
```

### 5. `floor_swap.py --check`

```bash
python3 bringup/floor_swap.py --check
```

Pure offline Python — no ROS, no robot, safe to run at any time. It must print
`2 floors, all drivable as recorded` and exit 0. Anything else is a list of exactly what is still
missing; fix that list, do not work around it. This is the check that caught floor 2's config
naming `elevator_f2`/`call_button_f2` when the real names were `floor2` and `f2_*`.

Sanity-check the seed while you are here:

```bash
python3 bringup/floor_swap.py --seed 1 --seed-role car_facing_out
python3 bringup/floor_swap.py --plan 1 2
```

### 6. Only then, drive

Not before. Every step above is cheap; discovering at the lift that the map is a picture is not.

## What this breaks, and what to do about it

`bringup/elevator_route.sh` hardcodes the five old unsuffixed names (lines 14–19, 192, 216, 242,
252, 257) and asserts the `elevator` map (lines 12, 204). **It is the one route on this robot that
has actually been driven**, so it is deliberately left alone by this change. After step 3 it will
fail at its own `check_waypoint.py` preflight — loudly, before anything moves, which is the
correct failure. Updating it to the `f1_` names is a separate, deliberate edit to make once
`floor1` has been driven and trusted.

`tests/test_floor_plan.py::test_floor_one_of_the_shipped_config_matches_the_waypoints_on_disk`
**now fails**, and it is the only failure this change introduces. It asserts that the
lowest-numbered floor in `config/floors.yaml` — "the route that has actually been driven" — must
validate against `maps/` right now, which stops being true the moment floor 1 is pointed at a map
that has not been recorded yet. Its premise, not its logic, is what expired. It goes green again
at step 5. **Do not silence it by pointing floor 1 back at `elevator`** — that is the whole failure
this document exists to prevent. If floor 1 stays unrecorded for more than a session, the honest
edit is to make that test assert "every floor that HAS a map on disk validates", which keeps its
teeth for floor 2 today and floor 1 tomorrow. That edit is not made here (`tests/` was out of scope
for this change).

Other places still naming the old map or waypoints, none of which block this procedure:

| file:line | what |
|---|---|
| `bringup/session.sh:308` | `MAP_NAME=${MAP_NAME:-elevator}` — the bare default |
| `bringup/stack.sh:35`, `:7`, `:48` | `MAP_NAME="${MAP_NAME:-elevator}"` and its usage examples |
| `bringup/check_waypoint.py:182` | falls back to `map_name` `"elevator"` when a waypoint has none |
| `paper/make_figure.py:76`, `:5` | default map `"elevator"` for figures |
| `bringup/run_recorder.py:75` | `--scene` default `"elevator"` (a label, not a map) |
| `README.md:193`, `docs/MORNING.md:124`, `docs/MULTIFLOOR.md:132` | `MAP_NAME=elevator` in prose |
| `config/slam_os0.yaml:86` | `map_start_pose` is an **atrium** coordinate; never seed from it |

The `session.sh`/`stack.sh` defaults should move to `floor1` once floor 1 is driven on it, not
before: `tests/test_stack_wiring.py` asserts the bare default names a map some recorded waypoint
is actually in, and it would fail the moment the default pointed at a map with no waypoints yet.
