# Mapping the test site

Produces the `map` the mission runs localize against. Do this **after** the calibration items that
feed it — a map built on a wrong lidar pose or a mirrored scan looks perfectly plausible and
navigates catastrophically.

## Before you start

| Blocker | Why |
|---|---|
| `bringup/stale_cmd_test.py driver` **and** `firmware` both PASS | Mapping means driving. On 2026-08-20 the base ran away under teleop. Source says the *driver* cannot latch (`ranger_messenger.cpp:391` commands straight from the subscription callback; no repeat timer anywhere), but the *chassis firmware's* behaviour when commands stop arriving is unreadable from source and unmeasured. Two failure modes, two phases. |
| CALIBRATION ③ lidar mount pose | An unmeasured offset biases **every** obstacle by that offset. |
| CALIBRATION ④ scan direction / zero-angle | A mirrored scan builds a map that looks fine and is wrong everywhere. Physical check, cannot be skipped. |
| Gate **S1** — glass doors | 2D lidar sees *through* glass. It will not appear in the map, and the robot will drive at it. Safety, not data quality. |

Gate **S0** is not a mapping blocker but is the project's top risk: if the ADA doors are
motion-activated, `passive` succeeds and R1 measures nothing. Answer it on the same visit — one
walk-through settles it.

## Build the map

```bash
cd ~/utp_robot && bash bringup/session.sh map
```

That brings up every layer in the order that works — link, chassis, OS0, the 2D scan chain, the
safety mux — checks each one before the next, and only then starts slam_toolbox. Doing it by hand
means doing five things in the right order with no check between them; `session.sh` exists because
each of those steps has silently failed at least once.

Two parameters it passes that are not optional:

- `publish_odom_tf:=true` to the ranger launch. **Not** the default. Without it there is no `odom`
  frame at all, slam_toolbox has nothing to anchor to, and nothing says so.
- `slam_params_file:=config/slam_os0.yaml`. Stock slam_toolbox uses `base_frame: base_footprint`,
  which this stack does not have — with the stock value it publishes `/map` and looks healthy while
  **never emitting `map → odom`**. That file also sets `min_laser_range: 0.55` (the OS0 sees the
  chassis at ~0.2 m; at the default the robot is painted into the map at every pose it occupied),
  `do_loop_closing: true`, and a `stack_size_to_use` large enough to serialize a building.

### Driving technique, in rough order of how much it matters

- **Close the loop.** Return to somewhere you have already been, by a different route, and drive
  *past* your start. Loop closure is the only thing that removes accumulated drift; a map from a
  single out-and-back is a spiral, and every waypoint recorded on it inherits the bend.
- **Slowly**, and pause after each turn.
- **Broad turns.** Do not pivot or crab. Spin-mode odometry caused metre-scale scan-matching jumps
  on 2026-08-24. (The old `/scan_mapping` gate that enforced this belonged to the A1M8 chain and is
  in `archive/`; the OS0 chain removes the chassis geometrically instead, but the *driving* advice
  still holds because the jump was in the odometry, not the scan.)
- Cover every space a mission enters, **including the far side of each door**.

Watch it fill in another terminal:

```bash
python3 bringup/map_watch.py
```

It prints two independent answers to "is this being recorded" — wheel odometry, and occupied cells
in `/map`. Occupied cells are the signal; map *dimensions* jump on a single stray beam, so ignore
them. `wheels moving, MAP NOT GROWING` means stop.

## Save it

**While slam_toolbox is still running:**

```bash
bash bringup/map_persist.sh save atrium
```

One command for the whole thing, because doing it in four steps meant four places to get it wrong:

| it writes | why it is not optional |
|---|---|
| `maps/atrium.pgm` + `.yaml` | the grid Nav2's costmap plans on. A picture — nothing can relocalize into it |
| `maps/atrium.posegraph` + `.data` | slam_toolbox's own graph. **The only thing** `mode: localization` can deserialize |
| `maps/.loaded_map` | which named map is live, in which SLAM session |

Without the pose graph, `session.sh nav` cannot relocalize — and slam_toolbox does not error, it
starts a **new** graph at wherever the robot is standing, so you get a fresh map frame wearing the
saved map's name and every waypoint is off by the startup offset. Without `.loaded_map`, every
`waypoints.py record --frame map` stores as nameless and `nav2_goto.py` refuses to drive to it.

The services return success when nothing lands on disk, so `map_persist.sh` checks the **files**,
and reports the extent and occupied-cell count. Under 2000 occupied cells it warns: that is the
signature of a robot that barely moved — a valid map of one spot.

Then, **in the same session**, record the waypoints:

```bash
python3 bringup/waypoints.py record start  --frame map
python3 bringup/waypoints.py record door   --frame map
python3 bringup/waypoints.py record button --frame map
bash bringup/map_persist.sh list          # confirms which maps are campaign-usable
```

Saving does not stop mapping. Save after every closed loop, not once at the end.

## Use it

```bash
MAP_NAME=atrium bash bringup/session.sh nav
python3 bringup/nav2_goto.py door --go     # ONE leg, before committing 50 trials
```

`session.sh nav` starts slam_toolbox in **localization** mode against the saved graph and launches
Nav2 with `localization:=slam`, so neither `map_server` nor AMCL starts — exactly one source may
own `/map` and `map → odom`. It refuses outright if the pose graph is missing.

## Resume a partial map

```bash
bash bringup/map_persist.sh resume partial_01 [--at-pose X Y THETA]
```

Requires slam_toolbox running in mapping mode. The default assumes the robot is back where the
*original* session started; if it is not, pass `--at-pose`. Check RViz before driving on: the live
scan must lie **on** the walls already in the map. Beside them means it loaded at the wrong pose,
and the tear that follows looks exactly like ordinary drift. Save under a **new** name so a bad
resume cannot overwrite the good partial.

## Check the map before trusting it

```
CHECK:  drive to a known landmark; the pose in RViz matches where the robot physically is
        a measured corridor width in the map matches a tape measure within ~5 cm
        no doorway is walled shut by the inflation layer (inflation_radius 0.30)
        glass doors: confirm by hand whether they appear at all -- they usually do not
```

Record the corridor measurement and the loop-closure outcome in `EXPERIMENT_LOG.md`. A map with no
recorded check is a map nobody has any reason to trust.
