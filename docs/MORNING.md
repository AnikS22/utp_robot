# The lab session — what to do when you sit down

> **NUMBERS CORRECTED 2026-09-02.** This file was written at 14:05 on 2026-09-01 and every sensor
> constant in it had since been superseded on the robot. It said `range_min: 0.70` (it is **0.45** --
> 0.70 hid a real door at 0.72 m, 0.30 exposed the packed arm at 0.31-0.36 m), `MASK_MAX_DEG 155`
> (it is **180**), and `MASK_MAX_M 1.00` (it is **0.90**). That last one matters most: the lift car's
> side walls sit at 1.00-1.15 m, so a 1.00 m mask was **deleting the walls the scan matcher needs**,
> which is why localization held in the atrium and broke on a turn by the lift. It also said
> `MAP_NAME=atrium`, which now refuses every elevator waypoint. Tune from `config/ouster.yaml` and
> `bringup/scan_relay.py`, never from a doc.



This is the runbook. It replaces the dated status notes that used to live here and in
`TOMORROW.md` (both now in `archive/`), because a status note goes stale in a day and a runbook
does not.

Read `## Honest state` before you start, so you know which of these steps have been proven on
hardware and which have only been proven offline.

---

## 0. The cable (30 seconds, do it first)

```bash
ip -brief link show | grep enx        # must NOT say NO-CARRIER
```

One USB-ethernet cable carries the **lidar (.119), the xArm (.221) and the router (.1)**. When it
dropped mid-session on 2026-08-30 it took all three down at once, and the adapter stays enumerated
when it happens — so `lsusb` looks perfectly healthy and `carrier` is the only check that matters.
**Strain-relieve it before any long drive.** `session.sh` checks this first and refuses to continue.

## 1. Bring-up

```bash
cd ~/utp_robot && bash bringup/session.sh up
```

Link → chassis → lidar + 2D scan chain → safety mux → health + gates 0-2, each verified before the
next. It will stop and tell you to start the deadman in another terminal:

```bash
python3 bringup/deadman.py            # open the URL and HOLD
```

**Nothing autonomous moves without it.** `config/safety.yaml` gates `nav` and `servo` on
`/safety/enable`, and a silent `/safety/enable` makes the robot look dead while every node behaves
correctly.

Once up, do not restart the ranger driver. It re-zeroes odom, and every odom-frame waypoint
silently becomes wrong.

**What the 2D scan chain does to the cloud, and why.** `pointcloud_to_laserscan` slices the OS0
cloud at `range_min: 0.45` (was 0.70, then 0.30), and `bringup/scan_relay.py` then applies a **self-occlusion
sector mask** — `MASK_MIN_DEG 74`, `MASK_MAX_DEG 180`, `MASK_MAX_M 0.90` — before republishing as
RELIABLE `/scan`. Both exist because the stowed arm and the mast sit inside the 0.20–1.20 m height
band the slice keeps, so the robot sees itself: measured 2026-09-01 over ten stationary scans on
open floor, minimum range **0.70–0.79 m in both rear quarters** — worst and most tightly pinned at
+105°..+150°, median 0.72 m — while the whole forward hemisphere (−90°..+60°) read **3.0–8.8 m**. Nav2's obstacle layer marked that **LETHAL around the footprint**, the
planner believed the robot was standing inside an obstacle, and it accepted goals and never moved
with 4.10 m clear ahead. The mask is asymmetric because the arm folds to one side; it is not a
tuning knob, it is a measurement of this robot in this stow pose. **If the arm is re-stowed or the
mast is moved, re-measure it** — and if `/scan` starts showing returns under 1.0 m astern again,
suspect the robot before you suspect the room.

## 2. The map

**`atrium` exists and it is relocalizable.** Saved 2026-09-01 with its pose graph:

    772 x 855 @ 0.05 m = 38.6 x 42.8 m, 10,452 occupied
    maps/atrium.posegraph (29 MB) + atrium.data (11 MB) + atrium.pgm + atrium.yaml

That is the difference that matters: every earlier map in `maps/` is a `.pgm`/`.yaml` pair — a
picture, with nothing slam_toolbox can resume a scan-matcher against. **You do not need to re-map.**
Go to §3.

### If you do need to re-map

Full detail in `MAPPING.md`. The short version:

```bash
bash bringup/session.sh map                  # then drive the loop — CLOSE it
python3 bringup/map_watch.py                 # in another terminal, while driving
bash bringup/map_persist.sh save atrium      # while slam_toolbox is STILL RUNNING
```

Then, **in the same session**, record the waypoints — a recording made after slam_toolbox restarts
is anchored to a different origin:

```bash
python3 bringup/waypoints.py record start  --frame map
python3 bringup/waypoints.py record door   --frame map
python3 bringup/waypoints.py record button --frame map
bash bringup/map_persist.sh list             # confirms the map is campaign-usable
```

### Park the robot where the map says, or tell it where you parked

`config/slam_os0.yaml` carries `map_start_pose: [0.2888, -0.4561, -0.5207]` — the pose the robot
was standing in when `atrium` was saved. **This is the seed, and it is not optional.**

* **With no `map_start_pose`, slam_toolbox silently discards the saved map.** It logs one line —
  `LocalizationSlamToolbox: Map starting pose not specified...` — and then comes up **ACTIVE
  anyway**, on a brand-new empty graph rooted at the robot's feet. Every check in `session.sh nav`
  passes. The only tell is the grid size: `atrium` is **772 x 855**; the empty graph published
  **486 x 585** with the robot at (0, 0). If the map on screen is smaller than the building, you
  are not localized in `atrium`, you are mapping again under its name.
* **A seed from the wrong parking spot is worse than none** — localization converges confidently
  to the wrong place. So if the robot is not parked where `map_start_pose` says, use RViz's
  **2D Pose Estimate** tool and click where it actually is:

```bash
ros2 run rviz2 rviz2 -d $PWD/nav2_bringup/slam_mapping.rviz
```

That config has both **2D Pose Estimate** (seed the localizer) and **2D Goal Pose** (drop a goal
and the robot drives there), plus `/map`, `/scan`, `/plan`, `/local_plan` and the global costmap.

## 3. Prove one leg before committing fifty

```bash
bash bringup/session.sh down                 # FIRST, if you have been mapping. See below.
MAP_NAME=elevator bash bringup/session.sh nav   # atrium can no longer be localized into:
                                               # its .posegraph/.data were destroyed and only
                                               # the grid came back from git
python3 bringup/nav2_goto.py door            # DRY RUN: prints the goal, moves nothing
python3 bringup/nav2_goto.py door --go       # THE ROBOT MOVES
```

**Never go `map` → `nav` without a `down` in between.** `session.sh nav` skips launching
localization if anything is already publishing `/map` (`if ! alive /map`), which is the right
behaviour when you are re-running `nav` and the wrong behaviour after a mapping drive: it attaches
Nav2 to the **still-MAPPING** slam_toolbox instead of a localization session on the saved map.
Everything comes up green — `/map` is there, `map->odom` resolves, `.loaded_map` gets written with
the map's name — and the robot is planning over a live, growing, origin-wherever-you-booted grid
that is not the map you saved. `down` first, every time.

**No leg has ever completed on hardware.** This is still the step most likely to need work, but
four of the reasons it could not work were removed on 2026-09-01, so read the log, not the picture:
**RViz keeps drawing a stale `Path` forever**, so a green line on screen is not evidence that
anything is being tracked. What to watch, and what to do:

| what you see | what it means | what to try |
|---|---|---|
| refuses before moving | the waypoint is odom-frame, or carries no map name | re-record it while localized in the named map |
| `no navigate_to_pose action server` | Nav2 came up unconfigured | `ros2 lifecycle get /bt_navigator` |
| `Nav2 REJECTED the goal` | goal is outside the map or in an inflated cell | check it in RViz against the costmap |
| `Lookup would require extrapolation into the future` → `Unable to transform goal pose` → `Goal failed`, in under a second | map→odom publish jitter exceeding `transform_tolerance` | it is 1.0 s in all four places in `nav2_params_os0_map.yaml` as of 2026-09-01; if it returns, raise it further, do not chase the planner |
| `Control loop missed its desired rate` | MPPI's cost critic with `consider_footprint: true` cannot do the work in the budget | the tuned pair is `controller_frequency 10.0` / `model_dt 0.1` / `time_steps 40` / `batch_size 1000`; keep `model_dt == 1/controller_frequency` and keep the horizon ≥ 4 s |
| accepts the goal, plans nothing usable, never moves, with clear floor ahead | the obstacle layer has marked the **robot's own arm/mast** lethal around the footprint | check `/scan` for returns under 1.0 m astern; §1's mask is the fix, re-measure it if the arm was re-stowed |
| plans, then wanders or saws | MPPI fighting the 4WS chassis' ~1.5 s re-steer lag | lengthen the MPPI horizon (it is 4.0 s now) before touching anything else; if it will not settle, `UTP_NAV_BACKEND=waypoints` falls back to odom legs |
| `blocked` at a door | **this is the correct outcome** | it is what starts the reason → ground → act loop |

Note the split that makes the pipeline's blockage check meaningful: `ros_world.navigate_to_goal()`
only asks the camera about a blockage within **`NEAR_GOAL_M = 2.5` m**. Beyond that the leg is
Nav2's problem. The rule is that the VLM is triggered when Nav2, on its path to the goal, discovers
it is blocked — not when a camera notices a door somewhere in frame.

## 4. The remaining hardware gates

`session.sh` runs gates 0-2 automatically. These need you:

```bash
bash bringup/lab_gates.sh 3          # chassis w_min — MANUAL, you watch and confirm
python3 bringup/characterise_twist.py --go --wz 0.30    # then 0.20, 0.12, 0.08
bash bringup/lab_gates.sh 5 6 7      # grounding + depth, arm, pipeline config + VLM
```

Also on this visit, and not software:

- **S0 — are the ADA doors motion-activated?** If they are, `passive` succeeds and the whole
  comparison measures nothing. One walk-through settles it. This is the project's top risk.
- **S1 — glass doors.** 2D lidar sees *through* glass; it will not appear in the map and the robot
  will drive at it. Mark them by hand.
- **S2 — riser height**, **H5 — hand-eye**.

## 5. The campaign

```bash
bash bringup/session.sh campaign 50
```

Per trial: run it, drive back to `start`, measure the residual against the **anchor** (not the
previous trial — 5 cm of creep per trial is invisible trial-to-trial and fatal by trial 20), append
one fsync'd record. It stops on drift past `--max-drift`, a collision, the deadman going silent, or
the endpoint dying — the things that make *later* trials unmeasurable. A failed trial is data and
does not stop it.

`--resume` tops up to `--trials` rather than restarting.

---

## Honest state

**Proven on hardware** (2026-08-25 → 08-30 runs): navigation legs arriving at 0.15 m repeatedly;
grounding the ADA plate at 0.526 confidence, beating the FIRE alarm 18 cm away; the fire-alarm veto
refusing a real alarm pick 4 of 4; positioning into arm reach at 0.68 m and +0.0°; arm reach,
retreat and stow.

**Proven on hardware 2026-09-01:** a map saved *with its pose graph* (`atrium`, 772 x 855) and
relocalized into; the OS0 self-occlusion measured and masked out of `/scan`; the VLM path end to
end on a real camera frame (1.645 s round trip, a correct abstain asking to look closer).

**Never proven on hardware:** the press landing on the plate (last attempt missed by 10 cm — the
target was carried across a ~1 m drive by odometry, which is exactly the "base moved between
observing and pressing" error), **Nav2 completing a leg** on a saved map, the chassis `w_min`
angular stall floor (§4, ten minutes, still not done), and the 50-trial loop. Glass doors remain
invisible to the lidar, so the costmap cannot mark them however well Nav2 is tuned.

**Proven offline only:** §3 above, and the campaign loop. 306 tests pass, and the map-persistence and
campaign paths are exercised behaviourally against fakes — but fake ROS proves the goal is correct
and the exit codes are honest. It says nothing about whether MPPI tracks a path on this chassis.
