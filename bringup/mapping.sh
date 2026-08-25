#!/usr/bin/env bash
# Build the site map: lidar + odometry + slam_toolbox + a live view of the map being built.
#
#     bash bringup/mapping.sh                # sensing + SLAM + RViz. Nothing can move.
#     bash bringup/mapping.sh --with-base    # also start the Ranger driver and the teleop UI
#     bash bringup/mapping.sh --no-rviz      # headless (e.g. over ssh with no display)
#
# When the map looks right:  bash bringup/save_map.sh
#
# WHY THE BASE IS OPT-IN
# ranger_bringup calls EnableCommandedMode() at startup, which takes command authority away from
# the RC transmitter -- layer 1, the human takeover path. Handing that over is a decision a person
# makes while looking at the robot, not a side effect of running a mapping script. Everything this
# script starts by default is read-only: it cannot move the base even if every piece of it fails.
#
# BEFORE --with-base, ALL OF:
#   * bringup/stale_cmd_test.py driver AND firmware have both PASSED. Mapping means driving, and
#     on 2026-08-20 the base ran away under teleop. This is the gate; it is not optional.
#   * the arm is stowed (CLAUDE.md: the base must not move unless the arm is stowed)
#   * SWB on the RC transmitter is at the TOP = command control mode. Until then the chassis is in
#     CONTROL_MODE_RC (0x03) and ignores CAN motion entirely -- and because EnableCommandedMode()
#     is one-shot at driver startup, flipping SWB AFTER the driver starts does nothing. Flip first.
#   * motion_mode is not kPark (3). Unpark from the RC.
#   * the E-stop fob is in your hand.
#
# TWO SETTINGS THAT LOOK LIKE DEFAULTS AND ARE NOT. Both fail SILENTLY -- /map appears, RViz looks
# healthy, and no map->odom is ever published:
#   * publish_odom_tf:=true is NOT the ranger launch default. Without it there is no odom frame at
#     all and slam_toolbox has nothing to anchor to.
#   * config/slam.yaml exists only because stock slam_toolbox uses base_frame: base_footprint,
#     which this stack does not have. It needs base_link.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/bringup/env.sh"

WITH_BASE=0
RVIZ=1
for a in "$@"; do
    case "$a" in
        --with-base) WITH_BASE=1 ;;
        --no-rviz)   RVIZ=0 ;;
        -h|--help)   sed -n '2,8p' "$0"; exit 0 ;;
        *) echo "unknown option: $a" >&2; exit 2 ;;
    esac
done
[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] || RVIZ=0

# Children get setsid so each leads its own PROCESS GROUP and cleanup can kill the whole group.
# Required, not tidiness: `ros2 launch`/`ros2 run` are python wrappers that exec the real node as a
# CHILD, so killing the pid we hold leaves the node alive holding the serial port. That is how the
# last round of orphans happened, and a stale rplidar_node makes the next start fail with
# SL_RESULT_OPERATION_TIMEOUT -- which looks exactly like a hardware fault and is not.
CHILDREN=()
start() {  # start <label> <cmd...>
    local label="$1"; shift
    echo "[map] $label"
    setsid "$@" &
    CHILDREN+=($!)
}
cleanup() {
    trap - EXIT INT TERM
    echo
    for pid in "${CHILDREN[@]:-}"; do
        [ -n "${pid:-}" ] || continue
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${CHILDREN[@]:-}"; do
        [ -n "${pid:-}" ] || continue
        kill -KILL -- "-$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    echo "[map] children reaped. The map is NOT saved automatically -- if you did not run"
    echo "[map] bringup/save_map.sh, that drive is gone."
}
trap cleanup EXIT INT TERM

python3 "$REPO/bringup/preflight.py" || exit 1

# Under a watchdog: on 2026-08-21 the lidar's USB device re-enumerated three times and each
# time the driver stayed alive with /scan silent and nothing logged an error. Mid-drive that is
# worse than a crash -- SLAM keeps running and stacks every later scan at one pose, so you finish
# walking the building and only then discover the map stopped when a cable was nudged.
start "lidar -> /scan (watchdogged)" \
    python3 "$REPO/bringup/topic_watchdog.py" --topic /scan \
        --type sensor_msgs/msg/LaserScan --min-hz 3 -- bash "$REPO/bringup/lidar.sh"
sleep 6

# SLAM never consumes /scan directly. First remove the chassis-occluded rear sector, then block
# mapping scans unless fresh chassis state says DualAckermann. The 2026-08-24 run demonstrated
# metre-scale map jumps in spinning mode; allowing that data into the pose graph is unrecoverable.
start "rear-sector lidar filter -> /scan_filtered" \
    python3 "$REPO/bringup/filter_scan.py"
start "Ackermann-only mapping gate -> /scan_mapping" \
    python3 "$REPO/bringup/mapping_scan_gate.py"

if [ "$WITH_BASE" = "1" ]; then
    cat <<'WARN'

  ------------------------------------------------------------------------------
  The base is about to accept CAN commands. The RC transmitter loses authority.
  Arm stowed?  SWB up?  Unparked?  E-stop in your hand?  stale_cmd_test passed?
  ------------------------------------------------------------------------------
WARN
    read -r -p "  type yes to continue: " ok
    [ "$ok" = "yes" ] || { echo "  stopping."; exit 1; }
    start "ranger driver -> /odom + odom TF" \
        ros2 launch ranger_bringup ranger_mini_v3.launch.py \
            use_sim_time:=false publish_odom_tf:=true
    sleep 5
fi

start "slam_toolbox -> /map + map->odom" \
    ros2 launch slam_toolbox online_async_launch.py \
        use_sim_time:=false slam_params_file:="$REPO/config/slam.yaml"
sleep 5

[ "$RVIZ" = "1" ] && start "rviz2" rviz2 -d "$REPO/maps/mapping.rviz"

if [ "$WITH_BASE" = "1" ]; then
    start "teleop mux + UI" bash "$REPO/bringup/teleop.sh"
    sleep 3
fi

echo
echo "  ============================================================================"
echo "  MAPPING STACK UP."
if [ "$WITH_BASE" = "1" ]; then
    echo "  Drive from  http://127.0.0.1:${UTP_TELEOP_PORT:-8420}   (tick the override box)"
else
    echo "  Read-only. To drive, either re-run with --with-base, or in other terminals:"
    echo "      ros2 launch ranger_bringup ranger_mini_v3.launch.py \\"
    echo "           use_sim_time:=false publish_odom_tf:=true"
    echo "      bash bringup/teleop.sh"
fi
cat <<'HOWTO'

  HOW TO DRIVE, in order of how much it matters:
    1. CLOSE LOOPS. Come back to somewhere you have already been, by a DIFFERENT route.
       Loop closure is what removes accumulated drift. A single out-and-back is a spiral
       that never gets corrected -- and it looks fine until you try to navigate it.
    2. Keep the RC in DUAL ACKERMANN. /scan_mapping is automatically BLOCKED in spin,
       parallel/crab, side-slip, or when chassis state is stale.
    3. SLOWLY: no more than about 0.25 m/s. The A1M8 measures only ~7 scans/s.
       Use broad Ackermann turns, pause after each turn, then drive straight.
    4. Cover both sides of every door a mission goes through.

  WATCH RVIZ WHILE DRIVING. Stop and re-drive the loop, do NOT save, if you see:
    corridors that bend where they are straight ....... drift, needs a loop closed
    the same room appearing twice .................... loop closure failed
    walls thickening or ghosting ..................... driving too fast

  WHAT WILL NOT BE IN THE MAP: glass. 2D lidar sees straight through it, so glass doors
  and partitions are simply absent and the robot will happily plan through them. Gate S1.
  Note where the glass is BY HAND while you are there; no amount of driving finds it.

  When it looks right:   bash bringup/save_map.sh
  Ctrl-C here stops everything.
  ============================================================================
HOWTO

wait
