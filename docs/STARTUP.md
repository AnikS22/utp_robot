# Robot startup and shutdown

Use `bringup/bringup_all.sh` from `~/utp_robot`. This is the current startup entry point.
`stack.sh` and `session.sh` remain for older workflows; do not run multiple startup scripts in
sequence or use `sensing_chain.sh` to repair the current map's sensor chain.

## Power-on checklist and one-command input setup

1. Power on the **Ranger base itself**, arm controller and lidar; connect the USB-CAN adapter,
   robot Ethernet and RealSense USB 3 cable. Lidar/arm power alone does not power the chassis.
2. If `can0` is down, run this locally (it needs your sudo password):
   `sudo ip link set can0 up type can bitrate 500000`.
3. In an unmapped room, run `bash ~/utp_robot/bringup/inputs.sh up`.
   This starts hardware inputs, camera and measured arm/safety monitoring. It never starts
   SLAM, localization or Nav2 and never sends a motion goal or changes arm configuration.
4. To check an already running setup, use `bash ~/utp_robot/bringup/inputs.sh check`.
   This only subscribes and reads status; it does not restart components.

Each run prints its report folder under `captures/input_checks/`. `inputs.json` records message
rates for lightweight streams, sample freshness, publisher counts, image dimensions, valid aligned depth, scan beams, transforms,
CAN feedback, stow/estop state, zero outgoing velocity and absence of localization/navigation.
RGB, depth and point clouds are sampled individually to avoid making the audit itself a large
bandwidth load; this checks frame content, not their sustained throughput. The startup log is saved alongside it for `up`. A nonzero exit means a required check failed.
Scan rates below 6 Hz are reported separately as warnings; this input audit is not a driving test.

If the base is off, CAN can be `UP` but `ERROR-PASSIVE` with no received frames. The Ranger
ROS driver can still emit 50 Hz odometry in that state. Startup now fails the CAN check rather
than trusting the topic rate. In `inputs` mode the independent sensors still start so one
missing cable does not prevent checking the rest. Check power/cabling before restarting drivers.

On September 6 the full cold input startup took 108 seconds, including lidar/camera initialization.
After powering on the previously-off Ranger, the audit confirmed real chassis feedback, normal
state, 50.5 V battery, measured arm stowed, clear estop, fresh RGB and aligned depth, lidar/IMU,
and all required input transforms. The base remained in STANDBY; no authority was claimed.
A recorded input pass does not establish localization or validate waypoint positions. Re-running
startup on the healthy stack took about 9 seconds and retained the chassis process. The separate
content audit adds its own measurement and frame-sampling time.

## Start the required mode

```bash
cd ~/utp_robot
bash bringup/bringup_all.sh --mode inputs              # all inputs; no SLAM/localization/Nav2
bash bringup/bringup_all.sh --mode map                 # mapping, camera optional
bash bringup/bringup_all.sh --mode map --status        # inspect mapping; no component changes
```

For navigation, use the saved map and the robot's actual starting pose in that map:

```bash
SEED_POSE='X,Y,YAW' bash bringup/bringup_all.sh --mode nav --map floor1
```

Replace `X,Y,YAW` with measured map coordinates in metres and radians. Floor1 currently has no recorded waypoints; record them on the next visit. If parked at a recorded
waypoint, obtain its seed with `python3 bringup/floor_swap.py --seed 1 --seed-role car_panel`
(or the role where the robot actually stands). Do not use that role merely because it appears
in this example. The script rejects a cold localization start without a seed; the old
`config/slam_os0.yaml` seed belongs to a different map. Check the scan alignment in RViz before
driving. `--mode full` adds the camera and arm for the press workflow.

An unreachable arm is reported during mapping. The arm-stowed interlock still blocks software
motion; startup never declares an attached arm absent or clears a safety gate. Navigation and
full mode report a failed motion gate as a failure. Mapping readiness is not permission to drive.

## Map, save, then stop

Before driving a new map:

```bash
bash bringup/map_insurance.sh start floor1
python3 bringup/map_watch.py
```

Save while SLAM is running. Choose a new name for each checkpoint to retain earlier copies:

```bash
bash bringup/map_persist.sh save floor1_new_session
```

The save must verify four nonempty files under `maps/`: `.pgm`, `.yaml`, `.posegraph`, `.data`.
The image alone cannot resume SLAM localization. After a successful save:

```bash
python3 bringup/stop_stack.py
```

This stops processes carrying this checkout's ownership marker and hardware domain, including
SLAM, sensors, safety, RViz and recorders. It first sends SIGINT, then SIGTERM if needed so detached
recorders can close their bags. It reports survivors instead of claiming success. It does not
save the map itself, power off hardware, or shut down the laptop. Save first.

Do not use `session.sh down` for the consolidated stack: it tracks only some launchers and uses
an older cleanup path. A new session must localize again; old odometry cannot be reused.

## What changed to reduce startup time

- One initial subscriber measures topics together after discovery. Subsequent checks subscribe
  only to the component just started and its required transforms.
- The old unconditional 18-second chassis, 40-second lidar, 25-second camera and similar waits
  are replaced by bounded readiness checks. Healthy data ends the wait early.
- Missing TF edges share one deadline instead of each consuming a separate timeout.
- Process discovery reads `/proc` in one Python invocation instead of spawning shell utilities
  for every PID. Ownership and ROS domain checks still guard restarts.
- SLAM lifecycle activation checks the actual state and skips configure/activate when already
  active. Nav2 is polled for readiness rather than always sleeping 45 seconds.
- Bad arguments and missing map files fail early. A failed probe cannot trigger a stack restart.

The script prints elapsed seconds and logs launch output to `/tmp/utp_bringup.log`. Read the
component report and repair the first failed dependency. Avoid repeating health, gate, and full
startup checks after a successful run unless there is a new failure.

## Sensor chain for the saved floor1 map

`/ouster/points` → height-band projection → `/scan_filtered` → reliable `/scan` → SLAM.
The projection uses `base_link`, 0.20–1.20 m height, 0.45 m minimum range and 0.0061 radian bins.
The SLAM relay masks rear self-returns to 0.90 m. Nav2 uses `scan_temporal_filter.py` from `/scan`
to `/scan_nav`, including its existing rear mask. This matches the live process configuration
observed during the September 5 floor1 drive.

The earlier `bringup_all.sh` draft instead inserted a cloud artifact filter and used a second nav
relay. Those changes were removed from this startup path to preserve the completed mapping
setup. Do not change sensor geometry while diagnosing startup. Older maps may have been built
with other settings; verify their provenance before using them.

## Agent instructions

For startup, read this file and run the required mode once. Do not re-read experiment history,
calibration or pipeline documentation unless the reported failure calls for it. Do not start
individual duplicate nodes, reset a live map, restart healthy odometry, or switch mapping to
localization without saving and explicitly stopping the previous session.

Offline verification: `python3 -m pytest tests/test_startup_readiness.py -q -p no:launch_testing`.
Input startup and the read-only audit were exercised on connected hardware on September 6.
Localization, driving and button pressing were intentionally not exercised in the unseen room.

## Before the next multi-floor run

The saved `floor1` and `floor2` maps are present, but `floor_swap.py --check` currently reports
five missing floor1 waypoints: `f1_call_button`, `f1_lift_door_reverse`, `f1_car_facing_out`,
`f1_car_panel`, and `f1_lift_door`. Record those in the correct localized map on a later visit;
do not invent them while the robot is in an unseen room. Re-run that offline check afterwards.

Chassis STANDBY is an input/readback state, not permission to drive. When deliberately preparing
for motion in a known area, use the existing chassis authority and safety procedure. No setup
script should claim CAN authority or clear an interlock during an input-only check.

Arm readback on September 6 was TCP offset `[0,0,172,0,0,0]`, with no controller error/warning.
`arm_tool.py` expects that offset, while some historical hand-eye instructions say zero. Resolve
that calibration discrepancy before pressing; input startup reports it without writing settings.

The arm SDK initializes cached tool values to zero before its report thread receives data.
`arm_tool.py` now waits up to three seconds for a real report and fails if none arrives. This
prevents a startup race from being misdiagnosed as lost tool calibration. No tool setter is
called during input setup.
