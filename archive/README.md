# archive/

Nothing here is loaded, launched or imported by the live stack. It is kept because a measurement
or a hard-won lesson lives in the comments, not because it works — several of these scripts talk
to hardware that is no longer on the robot, or to a SLAM system this stack does not run.

**Do not run anything in here.** If you find yourself wanting to, move it back out and give it a
test first.

## Superseded by a consolidation

The map scripts overlapped, disagreed about which artefacts a "saved map" consists of, and split
the checks between them, so a save could report success with the campaign-critical half missing.
All four collapsed into `bringup/map_persist.sh` (`save` / `resume` / `list`).

| file | replaced by |
|---|---|
| `save_map.sh` | `map_persist.sh save` — plus the `.loaded_map` provenance it never wrote |
| `map_save.sh` | `map_persist.sh save` |
| `map_load.sh` | `session.sh nav` (loads via `map_file_name`), `map_persist.sh resume` |
| `resume_map.sh` | `map_persist.sh resume` |
| `mapping.sh` | `session.sh map` — which also brings up the layers mapping.sh assumed were there |
| `route_run.py` | `run_trial.py` (one trial) and `run_campaign.py` (N trials) |

## Belonged to the /scan_mapping chain

`config/slam.yaml` set `scan_topic: /scan_mapping`, fed by `filter_scan.py` → `mapping_scan_gate.py`.
That chain existed for the RPLIDAR A1M8, whose ~44-beam scans were wrecked by the chassis in the
rear sector and by crab/spin motion. The OS0-128 chain (`pointcloud_to_laserscan` with
`min_height`/`max_height`, then `min_laser_range: 0.55`) removes the chassis geometrically, so the
gate has nothing left to do. `config/slam_os0.yaml` is the live config.

`mapping_scan_gate.py`, `mapping_gate_policy.py`, `test_mapping_scan_gate.py`, `filter_scan.py`,
and with them the A1M8 driver scripts `lidar.sh` and `find_lidar.sh`. `bringup/lidar3d.sh` is the
live one. Note `safety/scan_filter.py` is NOT retired — that is the pure-logic corridor veto, a
different thing that happens to have a similar name.

## Hardware that is no longer on the robot

`probe_rplidar.py` — the A1M8 was replaced by the Ouster OS0-128 (44 valid returns vs 121,367).
`map_accum.py` — subscribed to MOLA's `/lidar_odometry/*`, which nothing publishes here. MOLA was
rejected: it produced 1.4 Hz against a 10 Hz input.

## One-off setup and diagnostics, already done or since covered

`provision.sh` (rover laptop, provisioned), `probe_xarm.py` and `check_yaw_scale.py` (now
`lab_gates.sh` gates 6 and 3), `record_map_images.py`, `map_markers.py`, `denoise_map.py`.

## UIs nothing points at

`drive_ui.py` (809-line browser map — `nav2_goto.py` and RViz cover it), `arm_ui.py`,
`ask_plan.py`.

## RViz configs superseded by `nav2_bringup/slam_mapping.rviz`

| file | why |
|---|---|
| `os0_mapping.rviz` | **Entirely MOLA-era.** Its displays were `/map_accum` and `/lidar_odometry/localmap_points` + `/lidar_odometry/pose` — topics nothing in this stack publishes, since MOLA was rejected on 2026-08-30c. Worse, it had **no `Map` display at all**, which is why `/map` could not be seen while a mapping drive was in progress: the operator was watching a config that could not show the thing being built. |
| `os0_2d.rviz` | A strict subset of `slam_mapping.rviz` — `Map /map`, `LaserScan /scan`, `TF`, and the SetGoal/SetInitialPose tools, all of which the new config has, alongside `/plan`, `/local_plan`, the global costmap, odometry and the raw cloud. Nothing referenced it but `archive/TOMORROW.md`, which is itself archived. |

`nav2_bringup/os0_nav.rviz` was checked at the same time and **kept**: it is the only config with a
`local_costmap/costmap` display and a `/scan_filtered` LaserScan, which are exactly what you need
to see the 2026-09-01 self-occlusion failure (the robot's own arm marked LETHAL around the
footprint). `slam_mapping.rviz` does not cover those two, so this is not dead yet.

## Retired documentation

`NAVTEST.md` and `ROUTES.md` describe the odom-frame route system that `route_run.py` executed.
`TOMORROW.md` was a dated status note. `docs/MORNING.md` is now the single lab runbook, and
`docs/MAPPING.md` + `docs/NAV2.md` cover the map and the planner. `docs/NAV2_OS0.md` was folded
into `docs/NAV2.md` rather than archived, since all of it was still true.

## Nav2 parameter files superseded by `nav2_bringup/nav2_params_os0_map.yaml` (2026-09-01)

There were **three** near-identical ~400-line Nav2 param files. There is now one.

The concrete cost, on 2026-09-01, in one day:

1. Both Nav2 costmaps were subscribed to `/scan_filtered`, which on the OS0 chain is the **raw**
   projection and contains the robot's own arm and mast (fixed 0.70–0.85 m returns across
   |bearing| 74–155°). Nav2 marked LETHAL cells around its own footprint, accepted goals, planned
   nothing, and never moved — with no error anywhere, because both ends are BEST_EFFORT so the
   wrong data arrived perfectly. It went unnoticed for hours. The one-line fix (`/scan_filtered`
   → `/scan`, in three places per file) then had to be typed **three times**, once per copy. The
   identical 22-line explanatory comment now appears verbatim in `git log` three times over.
2. Worse, the copies had already silently **diverged**. Commit `cd3dcc1`, the same day, fixed
   `transform_tolerance` 0.2 → **1.0 in four places** (the cause of the
   `Lookup would require extrapolation into the future` → `Goal failed` loop, `docs/MORNING.md`),
   `controller_frequency` 20.0 → **10.0** (measured: the 20 Hz loop actually ran at 7.2 Hz), and
   the MPPI horizon (`time_steps` 56→40, `model_dt` 0.05→0.1, `batch_size` 2000→1000) — in
   `nav2_params_os0_map.yaml` **only**. Neither copy received any of it. So the answer to "were
   all three updated?" is: the scan topic yes (by hand, three times), everything else no. A copy
   that is only sometimes updated is worse than no copy, because it still looks authoritative.

| file | why archived |
|---|---|
| `nav2_params.yaml` | The **sim mirror**, and the ancestor the other two were forked from. Nothing launched it: `bringup/session.sh` passes `params_file:=` explicitly. It was only reachable as the `params_file` *default* in `ranger_nav.launch.py` — and as a default it was actively dangerous, because it still carried `sensor_frame: lidar_link` (there is **no** `base_link → lidar_link` TF on this robot; `docs/NAV2.md` gotcha 4 calls this "the single most expensive bug this stack has had" — every scan dropped, costmaps empty, every path looks clear) and `motion_model: Omni` (the Ranger firmware drops `angular.z` whenever `linear.y` is non-zero). A bare `ros2 launch ranger_nav.launch.py` came up blind. That default now points at `nav2_params_os0_map.yaml`. |
| `nav2_params_os0.yaml` | The **rolling-window** variant, for driving before a map exists. Genuinely diagnostic-only, and referenced by **nothing executable** — one row of a comparison table in `docs/NAV2.md` was its entire reachability. Not turned into a small override file because there was no caller to point at one; a 400-line copy maintained for a smoke test nobody runs is exactly the duplication being removed. Its whole semantic delta from the live file is four keys: drop `static_layer` from `global_costmap.plugins`, add `rolling_window: true`, `width: 24`, `height: 24`, `resolution: 0.05`. Pass those as `--ros-args -p` overrides if you ever need it again — do not fork the file. |

Everything else in these two files was identical to the live config, verified by a **parsed-YAML**
key-by-key comparison (`yaml.safe_load`, flattened, 213–215 leaf keys each), not a text diff:
18 differing keys total, all listed above or in `docs/CONSOLIDATION.md`.

`nav2_bringup/nav2_params_os0_map.yaml` is now the only Nav2 config in the repo. `session.sh nav`
`sed`s a runtime copy of it into `/tmp/utp_nav2_params_runtime.yaml` (rewriting the two absolute
behaviour-tree paths) and launches that — a path untouched by this archiving.
