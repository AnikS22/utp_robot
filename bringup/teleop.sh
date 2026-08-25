#!/usr/bin/env bash
# Keyboard teleop for the Ranger Mini 3.0: safety mux + browser WASD UI.
#
#     bash bringup/teleop.sh          # then open http://127.0.0.1:8420
#
# Starts the TWO software pieces that belong together:
#   safety/twist_mux_node.py   the ONLY publisher of /cmd_vel
#   bringup/teleop_keyboard.py a teleop-priority source publishing /cmd_vel_teleop
#
# It deliberately does NOT start the Ranger driver. ranger_bringup calls EnableCommandedMode() on
# startup, which takes command authority away from the RC transmitter -- layer 1, the human
# takeover path. Handing that over is a decision a person makes while looking at the robot, not a
# side effect of running a teleop script. Start it yourself, in its own terminal:
#
#     ros2 launch ranger_bringup ranger_mini_v3.launch.py
#
# BEFORE THE WHEELS TURN, on the RC transmitter (RANGER MINI 3.0 User Manual):
#   * SWB to the TOP = command control mode. Until then the chassis is in CONTROL_MODE_RC (0x03)
#     and ignores CAN motion commands entirely. This is not a software fault and cannot be fixed
#     in software -- the RC sits below anything we can do.
#   * motion_mode kPark (3) means the chassis is parked/locked. Unpark it from the RC.
#
# And the measurement that outranks everything else the first time this drives (LAPTOP_SETUP.md
# stage 7): with the base moving, KILL THE PUBLISHER and watch the wheels. If the driver holds the
# last twist after its commander dies, that is a runaway, and the watchdog gets written before any
# further driving happens.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/bringup/env.sh"

PORT="${UTP_TELEOP_PORT:-8420}"

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
    echo "[teleop] children reaped"
}
trap cleanup EXIT INT TERM

echo "[teleop] safety mux -> /cmd_vel   (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
setsid python3 "$REPO/safety/twist_mux_node.py" &
CHILDREN+=($!)
sleep 2

echo "[teleop] UI -> http://127.0.0.1:$PORT"
setsid python3 "$REPO/bringup/teleop_keyboard.py" --port "$PORT" &
UI_PID=$!
CHILDREN+=("$UI_PID")

cat <<EOF

  open  http://127.0.0.1:$PORT

  hold W/A/S/D to move · release to stop · SPACE latches a stop
  tick "assert /safety/override" or the mux blocks teleop (arm gate is fail-closed)

  Ctrl-C here stops both.
EOF

wait "$UI_PID" || true
