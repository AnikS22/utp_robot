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
# A SIM run must never resolve to the hardware domain. UTP_SIM=1 means the caller intends domain
# 42; if it still lands on 9 something upstream forgot UTP_ROBOT_DOMAIN, and the next thing this
# shell does may be to publish a twist at the real chassis. Refuse here, before any of that.
if [ "${UTP_SIM:-}" = "1" ] && [ "$ROS_DOMAIN_ID" = "9" ]; then
    echo "env.sh: UTP_SIM=1 but ROS_DOMAIN_ID resolved to 9 (hardware). Set UTP_ROBOT_DOMAIN=42." >&2
    return 1 2>/dev/null || exit 1
fi

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

# ---------------------------------------------------------------------------------------------
# OWNERSHIP MARKER -- how preflight tells our processes from someone else's
# ---------------------------------------------------------------------------------------------
# Every process launched from a shell that sourced this file inherits UTP_ROBOT_STACK in its
# environment. preflight.py reads /proc/<pid>/environ and treats a match as ours.
#
# This exists because the previous test -- "is the repo path in the command line?" -- was wrong in
# both directions. It missed `realsense2_camera_node` and `ros2 launch ranger_bringup` (their
# executables live in /opt/ros) and `bash bringup/teleop.sh` (relative path), so on 2026-08-21
# preflight refused to start the lidar because it had mistaken our OWN running stack for a foreign
# one. Widening it to a name or topic match is not the fix: matching `--child-frame-id lidar_link`
# is exactly what killed 22 of the sim campaign's TF publishers on 2026-08-18, because the sim uses
# the same frame names. An inherited env var is precise in both directions -- nothing we did not
# start can carry it, and everything we start does.
export UTP_ROBOT_STACK="$UTP_ROBOT_REPO"
if [ -f "$UTP_ROBOT_REPO/ros2_ws/install/setup.bash" ]; then
    source "$UTP_ROBOT_REPO/ros2_ws/install/setup.bash"
fi
[ "$_utp_had_u" = "1" ] && set -u
export UTP_ROBOT_REPO

# ---------------------------------------------------------------------------------------------
# Record where we are on the network, every time anything starts.
# ---------------------------------------------------------------------------------------------
# The laptop rides on the robot with the lid shut, so there is no screen to read an IP off. Campus
# DHCP re-leases and the robot roams between access points, so the address changes without warning.
# Stamping it here means the last known address is always on disk -- readable over whatever
# connection still works, or from the keyboard if none does. Costs a few bytes per invocation.
# Never allowed to fail: a read-only checkout must not stop the stack from starting.
{
    printf '# %s\nhost=%s\nuser=%s\n' "$(date -Is 2>/dev/null)" "$(hostname)" "${USER:-unknown}"
    for _a in $(hostname -I 2>/dev/null); do printf 'addr=%s\n' "$_a"; done
} > "$UTP_ROBOT_REPO/.last_address" 2>/dev/null || true
unset _a
