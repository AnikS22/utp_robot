# Frozen-map localization and waypoint recording

This is the short field workflow for opening a saved map, keeping its grid unchanged, finding the
robot in that map, and recording map-frame waypoints. It separates the three jobs that are easy to
mix up: mapping, visualization, and localization.

## What each mode does

| Mode | Purpose | Does it change the map? |
|---|---|---|
| `slam_toolbox` mapping mode | Build a new pose graph and occupancy grid while driving. | Yes |
| `slam_toolbox` localization mode | Match live scans to an existing pose graph and publish `map -> odom`. | No |
| `nav2_map_server` | Serve a saved `.pgm`/`.yaml` grid for a visual reference. | No |
| `run_dataset.py` | Record ROS and camera evidence into a run directory. | No |

Only map while deliberately making a new map. Stop `run_dataset.py` with Ctrl-C when its recording
is complete; it closes the MCAP bag and exports the audit rather than losing the run.

## A saved map has four files

For a map named `20260190`, keep these together under `maps/`:

```
20260190.pgm        occupancy grid image
20260190.yaml       grid resolution and physical origin
20260190.posegraph  SLAM graph
20260190.data       SLAM scan data
```

The `.pgm` and `.yaml` are sufficient to display a map. The `.posegraph` and `.data` are required
for `slam_toolbox` localization. Do not overwrite an existing map when troubleshooting; save a new
name or make a full four-file copy first.

## Start frozen localization

Start one localization node with the saved pose graph. `map_update_interval` is set very high so
this session cannot rewrite the grid.

```bash
cd ~/utp_robot
source bringup/env.sh
ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
  --params-file "$PWD/config/slam_os0.yaml" \
  -p use_sim_time:=false \
  -p mode:=localization \
  -p map_update_interval:=1000000.0 \
  -p map_file_name:="$PWD/maps/20260190"

ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox
```

The final command must report `active`. Confirm the node loaded the intended map:

```bash
ros2 param get /slam_toolbox mode
ros2 param get /slam_toolbox map_file_name
```

Expected values are `localization` and the path ending in `maps/20260190`.

## Show the frozen grid in RViz

If a stationary localizer has not yet emitted its own `/map` message, serve the saved grid on a
separate topic. This does not touch the SLAM graph:

```bash
source bringup/env.sh
ros2 run nav2_map_server map_server --ros-args \
  -r __node:=utp_frozen_map_server \
  -r map:=/frozen_map \
  -p yaml_filename:="$PWD/maps/20260190.yaml"

ros2 lifecycle set /utp_frozen_map_server configure
ros2 lifecycle set /utp_frozen_map_server activate

/opt/ros/jazzy/bin/rviz2 -d "$PWD/maps/waypoint_recording.rviz"
```

`waypoint_recording.rviz` displays `/frozen_map`, while localization continues to own the `map`
frame and `map -> odom` transform. Do not run a second map server on `/map`; that produces two map
publishers and makes diagnosis ambiguous.

## Relocalize before recording

First score the current pose without changing it:

```bash
source bringup/env.sh
python3 bringup/relocalise.py --expected-map 20260190 --dry-run
```

The command searches the saved occupancy grid with the current laser scan. It refuses weak or
ambiguous matches. If it accepts a result, apply it and immediately check it:

```bash
python3 bringup/relocalise.py --expected-map 20260190
python3 bringup/relocalise.py --expected-map 20260190 --check
```

Only record waypoints after the post-check fit is credible and the robot marker visually agrees
with the robot's physical location. If the check is poor, use RViz **2D Pose Estimate** while
standing at a recognizable location, then check again. Do not record coordinates from a pose you
know is wrong.

## Record waypoints

At each physical location:

```bash
source bringup/env.sh
python3 bringup/waypoints.py record <name> --frame map
```

For example, record `start`, `elevator`, `call_button`, or `door`. Verify the saved-map binding:

```bash
python3 bringup/waypoints.py where --frame map
cat maps/.loaded_map
```

The output should identify map `20260190`. Keep the robot still while a point is recorded.

## Why this can be done quickly

The robot already runs a stable sensing chain: Ouster scans flow through `/scan`, wheel odometry
publishes `odom -> base_link`, and the saved map provides the graph and grid. The fast path is to
reuse those live inputs, start only the missing lifecycle nodes, verify their parameters and
transforms, and avoid restarting drivers or rebuilding maps. Each action above is reversible and
keeps mapping, recording, visualization, and localization as separate processes.
