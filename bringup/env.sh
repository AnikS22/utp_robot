# Shared environment for every hardware script. Source, do not execute.
#
#     source bringup/env.sh
#
# ---------------------------------------------------------------------------------------------
# DOMAIN ISOLATION -- the whole point of this file
# ---------------------------------------------------------------------------------------------
# The workstation runs the SIMULATION campaign at the same time as hardware bring-up. Two ROS
# graphs on one host are isolated ONLY by ROS_DOMAIN_ID: different domain -> different DDS
# multicast port -> the two graphs cannot see each other at all.
#
# 9 is RESERVED for hardware. It was chosen because:
#   * the sim campaign allocates domains by walking UPWARD and never reusing (run_campaign_gpu.sh),
#     and is currently in the 137-225 band, well clear of 9;
#   * domains 42 and 43 are documented POISONED in run_campaign_gpu.sh (BUG #6) -- 43 was my first
#     pick and it was wrong;
#   * the DDS ceiling is 232, so staying low also stays far from that failure mode.
#
# This is a RESERVATION, not a guarantee. bringup/preflight.py enforces it by refusing to start if
# anything that is not ours is already talking on this domain -- a loud abort instead of a silent
# collision. On 2026-08-18 a careless cleanup of mine killed 22 of the sim campaign's TF publishers;
# every guard here exists because of that.
export ROS_DOMAIN_ID="${UTP_ROBOT_DOMAIN:-9}"

# ---------------------------------------------------------------------------------------------
# conda must not shadow ROS's python
# ---------------------------------------------------------------------------------------------
# colcon and ros2 run whatever python3 is first on PATH. conda's has no catkin_pkg/rclpy bindings,
# which produces errors that name neither conda nor python.
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)"
unset PYTHONPATH CONDA_PREFIX 2>/dev/null || true

# ROS setup files reference unset variables, so `set -u` must be off while sourcing them.
_utp_had_u=0; case "$-" in *u*) _utp_had_u=1; set +u;; esac
# shellcheck disable=SC1090,SC1091
source /opt/ros/jazzy/setup.bash
UTP_ROBOT_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$UTP_ROBOT_REPO/ros2_ws/install/setup.bash" ]; then
    source "$UTP_ROBOT_REPO/ros2_ws/install/setup.bash"
fi
[ "$_utp_had_u" = "1" ] && set -u
export UTP_ROBOT_REPO
