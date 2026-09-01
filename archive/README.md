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

## Retired documentation

`NAVTEST.md` and `ROUTES.md` describe the odom-frame route system that `route_run.py` executed.
`TOMORROW.md` was a dated status note. `docs/MORNING.md` is now the single lab runbook, and
`docs/MAPPING.md` + `docs/NAV2.md` cover the map and the planner. `docs/NAV2_OS0.md` was folded
into `docs/NAV2.md` rather than archived, since all of it was still true.
