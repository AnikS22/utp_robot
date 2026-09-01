#!/usr/bin/env bash
# Bring up the RPLIDAR A1M8 and publish /scan (+ the static base_link -> lidar_link transform).
#
#     bash bringup/lidar.sh            # publish /scan and the TF
#     bash bringup/lidar.sh --no-tf    # /scan only (something else owns base_link -> lidar_link)
#
# Settings, all learned the hard way (EXPERIMENT_LOG.md 2026-08-18):
#   * port DISCOVERED by bringup/find_lidar.sh, never hardcoded and never /dev/ttyUSBn. ttyUSBn
#     numbering DOES reorder -- observed live, the lidar moved from ttyUSB0 to ttyUSB1 after a
#     re-plug. by-id follows it, but a by-id path with the adapter's serial baked in still breaks
#     on a swap, so it is resolved fresh each start.
#   * 115200 baud -- correct for the A1M8. The wrong baud gives a SILENT no-data start.
#   * legacy_scan:=true -- our unit's firmware (1.29) predates scan-mode negotiation, so the
#     driver's default express path fails (0x80008000 / 0x80008004). Needs our patch.
#   * frame_id:=lidar_link -- Nav2's costmap sensor_frame. Publishing the frame is NOT enough:
#     something must publish base_link -> lidar_link too, or the costmap is blind. This script
#     does it. Pass --no-tf if a URDF/robot_state_publisher already owns that edge; two publishers
#     for one transform is its own bug.
#
# PROCESS HYGIENE: this script starts children and MUST reap them. An earlier version ended with
# `exec ros2 run ...`, which replaced the shell -- so the cleanup trap never fired and every run
# leaked an rplidar_node holding the serial port plus a TF publisher. The stale node made the next
# start fail with SL_RESULT_OPERATION_TIMEOUT (looks like a hardware fault; is not), and cleaning
# the leak up carelessly is what killed 22 of the sim campaign's TF publishers. No `exec` here.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/bringup/env.sh"

# Discovered, not hardcoded. The old default pinned the adapter's serial number (..._0001_...),
# which breaks on any swap and made "plug it in and go" impossible. find_lidar.sh honours
# $RPLIDAR_PORT if you set it, otherwise finds the single CP2102 and refuses to guess between two.
PORT="$(bash "$REPO/bringup/find_lidar.sh")" || exit 1
BAUD="${RPLIDAR_BAUD:-115200}"
FRAME="${RPLIDAR_FRAME:-lidar_link}"
PUBLISH_TF=1
for a in "$@"; do [ "$a" = "--no-tf" ] && PUBLISH_TF=0; done

RPLIDAR_PORT="$PORT" python3 "$REPO/bringup/preflight.py" --port "$PORT" || exit 1

# --- reap every child on exit, however we exit ------------------------------------------------
# Children are started with `setsid` so each becomes its own PROCESS GROUP LEADER, and cleanup
# kills the whole group with `kill -- -PGID`. This is required, not tidiness: `ros2 run` is a
# python wrapper that execs the real node binary as a CHILD, so killing the pid we hold leaves the
# actual node alive and holding the serial port. That is exactly how the last round of orphans got
# created, and a stale node makes the next start fail with SL_RESULT_OPERATION_TIMEOUT.
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
    echo "[lidar] children reaped"
}
trap cleanup EXIT INT TERM

if [ "$PUBLISH_TF" = "1" ]; then
    read -r MX MY MZ MR MP MYAW PARENT <<<"$(python3 - "$REPO/config/lidar.yaml" <<'EOF'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1])); m = c["mount"]
print(m["x_m"], m["y_m"], m["z_m"], m["roll_rad"], m["pitch_rad"], m["yaw_rad"], c["parent_frame"])
EOF
)"
    echo "[lidar] tf $PARENT -> $FRAME  xyz=($MX $MY $MZ) rpy=($MR $MP $MYAW)  [x,y from CAD; z +-0.07]"
    setsid ros2 run tf2_ros static_transform_publisher \
        --x "$MX" --y "$MY" --z "$MZ" --roll "$MR" --pitch "$MP" --yaw "$MYAW" \
        --frame-id "$PARENT" --child-frame-id "$FRAME" \
        --ros-args -r __node:=utp_robot_lidar_tf &
    CHILDREN+=($!)
fi

# The rear-sector filter belongs WITH the lidar, not with mapping.
#
# It used to be started only by mapping.sh. So outside a mapping session /scan_filtered did not
# exist -- and route_run.py and twopoint.py both subscribe to it for the corridor veto. Their
# guard reads `blocked = (self.scan is not None and corridor_blocked(...))`, so a MISSING scan
# means never blocked: the veto failed OPEN and the robot drove with no obstacle check at all,
# on every autonomous run ever made. CLAUDE.md documents the /scan -> /scan_filtered chain as
# though it always runs; it ran only while mapping.
#
# Guarded, because mapping.sh still starts one too and two publishers on one topic is the
# duplicate-publisher bug this repo has already paid for twice.
if pgrep -f "filter_scan.py" >/dev/null 2>&1; then
    echo "[lidar] rear-sector filter already running -- not starting a second"
else
    echo "[lidar] rear-sector filter -> /scan_filtered (keep +-${KEEP:-148} deg)"
    setsid python3 "$REPO/bringup/filter_scan.py" &
    CHILDREN+=($!)
fi

echo "[lidar] $PORT @ $BAUD, frame=$FRAME, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# The driver occasionally fails to start on a cold open (the CP2102 reopen path -- see
# EXPERIMENT_LOG.md; root cause not established, in-session it is rock solid). Retry rather than
# leave the operator guessing whether the hardware is broken.
for attempt in 1 2 3; do
    setsid ros2 run rplidar_ros rplidar_node --ros-args \
        -p serial_port:="$PORT" -p serial_baudrate:="$BAUD" -p frame_id:="$FRAME" \
        -p channel_type:=serial -p legacy_scan:=true -p angle_compensate:=true \
        -r __node:=utp_robot_rplidar &
    NODE_PID=$!
    CHILDREN+=("$NODE_PID")
    sleep 4
    if kill -0 "$NODE_PID" 2>/dev/null; then
        echo "[lidar] running (attempt $attempt). Ctrl-C to stop."
        wait "$NODE_PID" || true
        break
    fi
    echo "[lidar] driver died on attempt $attempt (cold-open flake); retrying" >&2
    sleep 2
done
