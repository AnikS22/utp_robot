# Nav2 on the OS0, rolling window

Click-and-drag goals in RViz, with real path planning around obstacles. No saved map required.

## The two things that make it work, and would silently break it

**1. `/safety/enable` must be held.** `config/safety.yaml` marks the autonomous sources -- `nav`
(Nav2) and `servo` -- `requires_enable: true`. **Nothing in this repo had ever published that
topic**, so the mux was correctly discarding every autonomous command since the day it was
written. Nav2 would plan, run MPPI, publish to `/cmd_vel_nav`, and go nowhere, with odom flowing
and every node healthy. `bringup/deadman.py` is the publisher, and it is a real hold-to-enable,
not a Bool latched true.

**2. `sensor_frame` must be `base_link`, not `lidar_link`.** `/scan_filtered` is now the Ouster
cloud projected by `pointcloud_to_laserscan` with `target_frame:=base_link`. There is no
`base_link->lidar_link` TF on this robot at all. With the old value Nav2's ObservationBuffer
drops every scan and the obstacle layer never marks or clears -- costmaps stay empty and every
path looks clear. Fixed in `nav2_bringup/nav2_params_os0.yaml`.

Also changed there: `motion_model: Omni -> DiffDrive`, because MPPI-Omni emits strafe and yaw in
one twist and the Ranger firmware DROPS `angular.z` whenever `linear.y` is non-zero. The sim can
do Omni because Isaac drives the wheel joints directly; this chassis cannot.

## Run it

```bash
# already running from the main bringup: chassis, OS0, MOLA, pointcloud_to_laserscan, safety mux
ros2 launch nav2_bringup/ranger_nav.launch.py \
    params_file:=$PWD/nav2_bringup/nav2_params_os0.yaml \
    localization:=slam use_sim_time:=false

python3 bringup/deadman.py            # open http://127.0.0.1:8089 and HOLD
ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```

In RViz: set Fixed Frame to `map`, then use **2D Goal Pose** to click and drag a goal.
**The robot will not move unless the deadman is held.**

`localization:=slam` so neither `map_server` nor AMCL starts -- MOLA owns `map -> odom`, and
exactly one source may.

## What this mode cannot do

Plan around an obstacle it has never seen. The global costmap is a 24 m window that travels with
the robot, built from live scan. A goal behind an unobserved wall is planned straight through it
and replanned when the wall appears. Whole-floor planning needs a saved MOLA map projected to an
occupancy grid.

The OS0 sits at 1.146 m, so `/scan_filtered` starts at 0.50 m -- close-in ground obstacles are in
a blind cone that no software fixes.
