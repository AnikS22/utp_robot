# 20260907T235743Z_ours_floor1_arrival

First clean floor-1 arrival after a real lift ride, 2026-09-07. Everything below shares one
clock, so a timestamp in the event list is a timestamp in the video.

## The video

`artifacts/mast_cam_color_image_raw.mp4` is the robot's own view, the full 200 s at 30 fps,
with the route's decisions burnt into the top-left of each frame.
`artifacts/aligned_depth_to_color_image_raw.mp4` is the same view as depth.

## What happens, and when

| video time | stage | detail |
|---|---|---|
| t+0s | loading the floor-1 map | floor1 |
| t+48s | map loaded | floor1 |
| t+49s | the way out of the lift reads clear | 3.0m/20deg |
| t+68s | knows where it is | floor1 2.3877,0.5592 yaw -77.60 fit 72.3% |
| t+77s | drive begins | f1_ada_button |
| t+110s | arrived | f1_ada_button arrived 33s |
| t+111s | press chain begins | the accessible door push button |
| t+124s | ARM REFUSED THE REACH -- base steps in and re-grounds | the accessible door push button shortfall 0.093 advance 0.193 |
| t+160s | gripper meets the plate | the accessible door push button |
| t+160s | drive begins | f1_outside |
| t+187s | arrived | f1_outside arrived 27s |
| t+188s | done | 2->1 |

## The moment worth cutting to

At t+124s the arm refuses: the ADA plate grounded 0.943 m away and the arm reaches 0.88 m.
It reports the 93 mm shortfall, the base drives in 193 mm, the detector re-grounds from the
new position at 0.816 m, and the press lands at t+160s. That whole loop is autonomous and
it is the first time it has run on hardware.

## Everything else in here

- `artifacts/trajectory.png` - the driven path on the floor-1 map, coloured by time, events marked
- `artifacts/captures/` - the two press attempts: RGB, depth, the detector's annotated pick, the 3D target
- `frames/` - 49 stills the recorder kept on each event
- `poses.jsonl`, `csv/trajectory.csv` - 1836 map-frame poses over 201 s
- `csv/` - odom, cmd_vel, the Nav2 global plans, safety status, events
- `rosbag/` - 21 GB mcap, every topic, if you want a different camera or the lidar
- `provenance/`, `audit.json` - git sha, ROS graph, topic rates at the time of the run
