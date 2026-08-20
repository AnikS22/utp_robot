# Host requirements

Tested on Ubuntu 24.04 + ROS 2 Jazzy.

- `ros-jazzy-desktop` (or at minimum `ros-jazzy-ros-base` + `ros-jazzy-nav2-*` for later stages)
- `colcon`, `rosdep`, `vcstool`, `git` — all present on the workstation already
- The user must be in the **`dialout`** group to open the lidar's serial port.
- CAN bring-up needs root (see EXPERIMENT_LOG.md). Nothing else here does.

Deliberately NOT required:
- `libasio-dev` — vendored as headers by `setup_workspace.sh`, so no `sudo apt` is needed.
- `pyserial` — `bringup/probe_rplidar.py` talks to the tty with stdlib `termios` only.

## conda

conda's `python3` must not be first on `PATH` when building: colcon runs ROS's
`package_xml_2_cmake.py` with whatever `python3` it finds, and conda's lacks `catkin_pkg`, which
makes every package fail at `ament_package()` with an opaque "returned error code 1". Both scripts
scrub conda from `PATH` themselves, so you do not need to deactivate it.
