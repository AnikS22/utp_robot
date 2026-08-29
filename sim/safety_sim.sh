#!/usr/bin/env bash
# The REAL safety stack, pointed at the sim (domain 42): the same twist_mux_node that is the
# only /cmd_vel publisher on hardware, and the same arm_monitor -- scene_state backend, which
# reads the trial server's /scene/state and publishes /safety/arm_stowed. This is deliberately
# NOT bypassed in sim: the 2026-08-27 hardware freeze was almost certainly the arm_stowed gate
# failing closed, and a sim that skips the gate cannot catch that class of bug.
source /opt/ros/jazzy/setup.bash
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
export ROS_DOMAIN_ID=42
python3 "$REPO/safety/arm_monitor_node.py" --backend scene_state &
AM=$!
python3 "$REPO/safety/twist_mux_node.py" &
MUX=$!
# The rear-sector filter, so /scan_filtered exists on 42 -- approach_blockage, face_target and
# route_run all subscribe to it, and without it the corridor veto silently fails OPEN (the same
# gap lidar.sh closes on hardware). Guarded: two filters = two publishers.
if pgrep -f "filter_scan.py" >/dev/null 2>&1; then echo "[safety_sim] filter already running"; else
python3 "$REPO/bringup/filter_scan.py" &
FILT=$!
fi
trap 'kill $AM $MUX ${FILT:-} 2>/dev/null' EXIT
echo "[safety_sim] arm_monitor pid $AM, mux pid $MUX (domain 42). Ctrl-C stops both."
wait
