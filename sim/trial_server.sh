#!/usr/bin/env bash
# Launch the sim repo's Isaac trial server ON THIS LAPTOP (pip venv, not the old workstation's
# ~/isaacsim kit install -- the sim repo's own launch scripts hardcode /home/minghanwei and are
# left untouched per CLAUDE.md).
#
#   bash sim/trial_server.sh                 # headless, domain 42, 2h
#
# Domain 42 is SIM ONLY. 9 is reserved for hardware; keeping them apart means a sim test can
# never publish a twist at the real chassis.
#
# System Jazzy is sourced FIRST (before `set -u`; ROS setup scripts read unset vars) so the
# Isaac ROS2 bridge binds the SAME rmw/libs the robot stack uses -- without it the bridge falls
# back to its internal copy and fails: "ROS2 Bridge startup failed" (measured 2026-08-27).
source /opt/ros/jazzy/setup.bash
set -euo pipefail
SIM_REPO="$HOME/unlocking-the-path"
exec env -u DISPLAY OMNI_KIT_ACCEPT_EULA=YES ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ISAAC_ASSETS_ROOT="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1" \
    "$HOME/isaacsim-venv/bin/python" "$HOME/utp_robot/sim/trial_server_patched.py" --seconds 7200 "$@"
