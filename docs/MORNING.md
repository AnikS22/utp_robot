# The lab session — what to do when you sit down

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

## 2. The map

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

**None of the maps currently in `maps/` can be used.** They are grids with no pose graph, so
nothing can relocalize into them. You have to re-map.

## 3. Prove one leg before committing fifty

```bash
MAP_NAME=atrium bash bringup/session.sh nav
python3 bringup/nav2_goto.py door            # DRY RUN: prints the goal, moves nothing
python3 bringup/nav2_goto.py door --go       # THE ROBOT MOVES
```

This is the step that has never run on hardware and the one most likely to need work. What to
watch, and what to do:

| what you see | what it means | what to try |
|---|---|---|
| refuses before moving | the waypoint is odom-frame, or carries no map name | re-record it while localized in the named map |
| `no navigate_to_pose action server` | Nav2 came up unconfigured | `ros2 lifecycle get /bt_navigator` |
| `Nav2 REJECTED the goal` | goal is outside the map or in an inflated cell | check it in RViz against the costmap |
| plans, then wanders or saws | MPPI fighting the 4WS chassis' ~1.5 s re-steer lag | lengthen the MPPI horizon or lower `controller_frequency`; if it will not settle, `UTP_NAV_BACKEND=waypoints` falls back to odom legs |
| `blocked` at a door | **this is the correct outcome** | it is what starts the reason → ground → act loop |

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

**Never proven on hardware:** the press landing on the plate (last attempt missed by 10 cm — the
target was carried across a ~1 m drive by odometry, which is exactly the "base moved between
observing and pressing" error), Nav2 driving a leg on a saved map, and the 50-trial loop.

**Proven offline only:** everything in §2 and §3 above. 306 tests pass, and the map-persistence and
campaign paths are exercised behaviourally against fakes — but fake ROS proves the goal is correct
and the exit codes are honest. It says nothing about whether MPPI tracks a path on this chassis.
