#!/usr/bin/env bash
# The safety stack on HARDWARE: the mux that owns /cmd_vel, and the arm monitor that gates it.
#
#     bash bringup/safety.sh
#
# WHY THIS FILE EXISTS. The sim had safety_sim.sh, which starts BOTH nodes. Hardware had no
# equivalent: teleop.sh starts the mux alone, and nothing anywhere started arm_monitor_node.py --
# it appears in README.md and LAPTOP_SETUP.md as a command to type, and in no launcher.
#
# So on hardware /safety/arm_stowed had NO PUBLISHER, the gate fail-closed forever, and every
# autonomous source was dead on arrival. Teleop still worked, because teleop is
# allows_arm_override and the UI has a checkbox that asserts /safety/override -- teleop.sh even
# tells you to tick it. That is exactly why this hid for so long: the half a human drives works,
# the half that drives itself does not, and the difference looks like a broken planner.
#
# route_run.py and twopoint.py publish to /cmd_vel_teleop, the SAME mux source as the UI, but
# nothing asserts override for them. Same topic, same priority, opposite outcome.
#
# THE ARM MUST BE POWERED AND REACHABLE. The xarm_sdk backend gates on MEASURED joint angles, so
# no session means no evidence means the gate stays shut -- deliberately. If the arm is off, the
# base does not drive, and that is the interlock working, not a fault to route around.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/bringup/env.sh"

BACKEND="${UTP_ARM_BACKEND:-xarm_sdk}"

CHILDREN=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${CHILDREN[@]:-}"; do
        [ -n "${pid:-}" ] || continue
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${CHILDREN[@]:-}"; do
        [ -n "${pid:-}" ] || continue
        kill -KILL -- "-$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    echo "[safety] children reaped"
}
trap cleanup EXIT INT TERM

# Exactly one mux. Two instances interleave commands on /cmd_vel -- the same duplicate-publisher
# class that cost 2026-08-26, and that health.py's check_duplicates now hunts.
if pgrep -f "twist_mux_node.py" >/dev/null 2>&1; then
    echo "[safety] a twist_mux_node is ALREADY running. Two muxes interleave commands on" >&2
    echo "         /cmd_vel. Stop the other one first (ros2 topic info /cmd_vel)." >&2
    exit 1
fi

echo "[safety] arm monitor (backend=$BACKEND) -> /safety/arm_stowed"
setsid python3 "$REPO/safety/arm_monitor_node.py" --backend "$BACKEND" &
CHILDREN+=($!)
sleep 2

echo "[safety] mux -> /cmd_vel   (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
setsid python3 "$REPO/safety/twist_mux_node.py" &
CHILDREN+=($!)

cat <<EOF

  Both up. Check the gate before driving:

      python3 bringup/health.py --skip-arm

  'gate arm_stowed' must read ~100%. If it reads 0%, the base will not move and the mux will
  say arm_not_stowed -- the arm is either not at the stow pose (bringup/stow_arm.py) or not
  reachable at all.

  Ctrl-C here stops both.
EOF
wait
