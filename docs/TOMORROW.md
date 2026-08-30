# Start here — 2026-08-31

Written at the end of 2026-08-30. State when we stopped: **a real 2D map exists and Nav2 plans on
it**, and the ethernet cable to the robot came unplugged.

## 0. The cable (30 seconds, do it first)

    ip -brief link show enx00e04c674c60      # must NOT say NO-CARRIER

That one USB-ethernet cable carries the **lidar (.119), the xArm (.221) and the router (.1)**. It
dropped mid-session at 17:39 and took all three down at once. The USB adapter stays enumerated when
this happens, so `lsusb` looks perfectly healthy — `carrier` is the check that matters.
**Strain-relieve it before any long drive.**

## 1. Bring-up, in order

```bash
cd ~/utp_robot && source bringup/env.sh          # ROS_DOMAIN_ID=9

sudo ip link set can0 up type can bitrate 500000 # needs your password
ros2 launch ranger_bringup ranger_mini_v3.launch.py publish_odom_tf:=true
```

`publish_odom_tf:=true` is **not** the launch default and everything downstream needs it: without
`odom -> base_link` there is no TF chain for SLAM or Nav2 to hang off.

```bash
bash bringup/lidar3d.sh                          # OS0 driver + base_link->os_sensor TF
```

Then the 2D scan chain — this is what SLAM actually eats:

```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/ouster/points -r scan:=/scan_filtered \
  -p target_frame:=base_link -p min_height:=0.20 -p max_height:=1.20 \
  -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.0061 \
  -p range_min:=0.50 -p range_max:=40.0 -p use_inf:=true

python3 bringup/scan_relay.py                    # /scan_filtered (BEST_EFFORT) -> /scan (RELIABLE)
```

**The relay is not optional.** pointcloud_to_laserscan publishes BEST_EFFORT and slam_toolbox
subscribes RELIABLE; those are incompatible in DDS, so without it slam_toolbox receives nothing,
reports nothing, and looks hung.

```bash
bash bringup/safety.sh                           # mux + arm gate (arm must be powered)
python3 bringup/health.py                        # must pass before anything moves
```

## 2. Mapping

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file $PWD/config/slam_os0.yaml -p use_sim_time:=false

ros2 lifecycle set /slam_toolbox configure       # REQUIRED
ros2 lifecycle set /slam_toolbox activate        # REQUIRED
```

**In Jazzy slam_toolbox is a lifecycle node and starts `unconfigured`.** Skip these two lines and
it loads no params, subscribes to nothing, and publishes no map — with no error.

```bash
ros2 run rviz2 rviz2 -d $PWD/nav2_bringup/os0_2d.rviz
```

Drive on the **RC (SWB down)**. SLAM needs only the lidar and odometry — not the mux, not the
controller, not the unverified stall floor. Slow, smooth, close loops, revisit from different
directions. Save when done:

```bash
ros2 run nav2_map_server map_saver_cli -f $PWD/maps/atrium2d
```

To re-record from blank: `deactivate` -> `cleanup` -> `configure` -> `activate` on the lifecycle.

## 3. Navigating

```bash
ros2 launch nav2_bringup/ranger_nav.launch.py \
  params_file:=$PWD/nav2_bringup/nav2_params_os0_map.yaml \
  localization:=slam use_sim_time:=false

python3 bringup/deadman.py                       # open :8089 and HOLD
ros2 run rviz2 rviz2 -d $PWD/nav2_bringup/os0_nav.rviz
```

RViz -> **2D Goal Pose** -> click and drag. **Nothing moves unless the deadman is held**: `nav` is
`requires_enable: true` in config/safety.yaml, and nothing published `/safety/enable` until
`deadman.py` was written yesterday.

## 4. DO THIS BEFORE TRUSTING ANY GOAL

```bash
python3 bringup/characterise_twist.py --go --wz 0.30   # should rotate
python3 bringup/characterise_twist.py --go --wz 0.20
python3 bringup/characterise_twist.py --go --wz 0.12   # expect little
python3 bringup/characterise_twist.py --go --wz 0.08   # expect nothing
```

The lowest `wz` that still turns the body **is** `w_min`; put it in `safety/waypoint_drive.py`.
Ten minutes. Until it is measured, a Nav2 goal that plans a perfect path and produces no motion is
ambiguous between "Nav2 is broken" and "the chassis will not execute that rate" — and the second is
the one we have evidence for.

## Two things the software cannot fix

**Glass.** The atrium doors are invisible to the lidar, absent from the map, and Nav2 will plan
straight through them. Keep the E-stop in hand near glass.

**The blind cone.** The OS0 sits at 1.146 m and `/scan_filtered` starts at 0.50 m, so close-in
ground obstacles are not seen at all.
