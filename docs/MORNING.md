# The door run — what to do when you sit down

Written overnight 2026-08-29/30. Straight status first, then the commands.

## Is it working?

**No — not end to end, and I am not going to tell you otherwise.** What changed overnight is
that every stage is now instrumented and the failure that was left at the end of the day is
fixed. The one thing still unproven on hardware is the press itself, because the last press
attempt missed by 10 cm and the fix for it has never been executed.

What IS proven on the real robot, from yesterday's runs:

| stage | evidence |
|---|---|
| navigation legs | `start`, `doors`, `button` each `arrived` at 0.15 m, repeatedly |
| a leg that ends at a wall | `arrived-short` at 0.92 m instead of failing STUCK (proven in sim) |
| grounding the ADA plate | 0.526, 99x90 px, beating the FIRE alarm 18 cm away |
| the fire-alarm veto | refused a real alarm pick twice (97%, 4 of 4 queries) |
| positioning into arm reach | `positioned: 0.68 m from the target, +0.0 deg off the press axis` |
| arm reach + retreat + stow | all five approach steps ran, clean retreat, stowed |
| the press landing on the plate | **NOT PROVEN — missed by 10 cm** |

## The fix that has never run

The miss was not aim, it was *what the arm was aimed at*. `reach_control` grounds the plate from
~1.7 m, the base then drives ~1 m to the press standoff, and the target was carried across that
drive by odometry — which is exactly the "base moved between observing and pressing" error the
sim documents. Ten centimetres, on a twelve-centimetre plate.

The frame taken with the arm extended showed the plate in plain view. It had been invisible only
because the *stowed* arm fills the lower-centre of the mast camera, which is where a plate 0.7 m
dead ahead appears. So `press_run.sh` is now **READY → LOOK → GROUND → REACH**: raise the arm
first, then photograph, then ground, then reach. The arm aims at a point measured from where the
arm actually is. The odometry-carried point is kept only as a cross-check and refuses the press
if the two disagree by more than 20 cm.

## Start here

```bash
cd ~/utp_robot && source bringup/env.sh
```

Four terminals, in order. **Leave them running.**

```bash
ros2 launch ranger_bringup ranger_mini_v3.launch.py     # 1  chassis
bash bringup/lidar.sh                                   # 2  lidar + /scan_filtered
bash bringup/camera.sh                                  # 3  camera
bash bringup/safety.sh                                  # 4  mux + arm gate  (arm must be powered)
```

Then, before anything moves:

```bash
python3 bringup/health.py
```

Required: `chassis mode CAN` (if it says RC, **flip SWB up**), `gate arm_stowed 100%`, `/odom`
and `/scan` and `/cmd_vel` each with **1 publisher**. If `can0` is missing:
`sudo systemctl restart utp-can-up@can0.service`.

## Record, then run

Waypoints are odom coordinates and odom drifts, so record all four back to back and run
immediately. Do not power-cycle the chassis or restart `ranger_base` in between.

```bash
python3 bringup/waypoints.py record start     # at the start pose
#   ... drive to ~1.5 m square to the closed doors ...
python3 bringup/waypoints.py record doors
#   ... drive to the press pose: square to the plate's wall, plate CENTRED in the camera ...
python3 bringup/waypoints.py record button
#   ... hold the doors open, drive through, outside ...
python3 bringup/waypoints.py record final
#   ... drive back to start, flip SWB up ...

python3 bringup/route_run.py recorded_press --go --confirm
```

`--confirm` pauses before every step. The route is
`start → doors → button → reach_control → press_button → doors → final`.

**Check the `button` pose before recording it** — it is the one that matters:

```bash
python3 bringup/grab_frame.py --name button_check
#   then look at captures/button_check/rgb.png: the plate should be centred and unobstructed
```

## What to watch, and when to stop it

* `reach_control` prints `positioned: X m from the target`. **X must be under 0.68 m.** If it
  is not, the arm will not be commanded — that is the envelope check doing its job.
* `SAFE: clear (N of 4 forbidden queries on the target)` — the fire-alarm veto passed. If it
  refuses, look at `captures/reach_*/detection.png` before overriding anything.
* **Stop it** if the arm reaches toward the red FIRE unit rather than the black plate beside it.

## Known-open, in the order I would fix them

1. **The press.** Untested since the READY→LOOK→GROUND→REACH reorder.
2. **Drift between recording and running.** `waypoints.py anchor <name>` / `relocalize <name>`
   are written and unit-tested (`safety/scan_anchor.py`) and have never run on the robot. They
   are what removes the re-record-every-time tax.
3. **Isaac depth is dead on this laptop.** 32FC1, 100% inf on every frame, three server variants
   tried. The Isaac log names it: the SDG pipeline drops the depth sync edge ("Illegal cycle
   connection … WriterSyncGate … ignored"). Fixing it means editing the sim repo, which
   CLAUDE.md forbids. The sim can still validate navigation and the FSM; it cannot validate
   grounding.
4. **Elevator.** Never run on any system.

## The chassis rules that cost the most time yesterday

* **SWB up** or the chassis silently discards every command while odom and the mux look healthy.
* **Do not disconnect the RC.** In RC mode a missing transmitter trips a lost-link failsafe and
  the chassis refuses mode changes entirely (`vehicle_state=EXCEPTION`).
* Touching the sticks reclaims RC at any moment, including mid-run.

---

# Map-frame waypoints (added 2026-08-30)

Waypoints can now be recorded in a **SLAM map frame** instead of the odom frame. Odom coordinates
die on every `ranger_base` restart and drift in between; map coordinates recorded against a
**saved, loaded map** survive between sessions. That is what removes the re-record-before-every-run
tax.

## The one distinction that matters

A MOLA started **fresh** also has a `map` frame -- with its origin wherever the robot happened to
boot. **The TF tree looks identical either way.** So:

| how MOLA was started | `map` frame origin | waypoints portable? |
|---|---|---|
| fresh, no map loaded | wherever the robot booted | **no** -- valid only while that MOLA runs |
| `map_load.sh <name>` | the saved map's own origin | **yes** -- any session localized in that map |

`safety/map_frame.py` enforces this. It refuses a waypoint recorded against map `atrium` when
MOLA is running fresh, and refuses one recorded in `atrium` when you are localized in `garage`.

## To get persistent waypoints

```bash
# 1. bring up the sensor and SLAM
bash bringup/lidar3d.sh                       # OS0 driver + mount TF
#    (MOLA launch -- see EXPERIMENT_LOG 2026-08-30b)

# 2. drive the route in RC mode (SWB down). MOLA needs only the lidar and its IMU:
#    not the chassis software, not the mux, not the waypoint controller.

# 3. save, and verify it reads back
bash bringup/map_save.sh atrium_os0

# 4. from now on, at the start of every session
bash bringup/map_load.sh atrium_os0           # or: ... atrium_os0 <x> <y> <yaw_deg>

# 5. record and run in the map frame
python3 bringup/waypoints.py record start --frame map
python3 bringup/route_run.py press_and_out --go --confirm
```

## Frame selection

`--frame auto` (the default) prefers the map when MOLA is publishing a usable one, falls back to
odom otherwise, and **always prints which it chose**. It never decides silently. `--frame map`
refuses rather than falling back; `--frame odom` is the old behaviour.

**The chassis must be running for the map frame to work at all.** MOLA publishes `map -> odom`
per REP-105, so without `odom -> base_link` from `ranger_base` there is no `map -> base_link` to
read. With the chassis off you will correctly get the odom fallback, or a refusal under
`--frame map`.

## What this does NOT fix

The angular stall floor (`w_min`, 2026-08-30). Perfect localization does not make the chassis
execute a 0.06 rad/s turn. Run the `characterise_twist.py --wz` sweep before trusting any run.
