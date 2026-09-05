#!/usr/bin/env bash
# Record the MINIMUM needed to rebuild a mapping drive offline, so a crash costs seconds not a walk.
#
#   bash bringup/map_insurance.sh start floor1     # begin recording alongside a mapping drive
#   bash bringup/map_insurance.sh stop
#   bash bringup/map_insurance.sh rebuild floor1   # replay into slam_toolbox and re-serialize
#
# WHY THIS EXISTS. On 2026-09-05 a mapping drive was lost outright: slam_toolbox holds its pose
# graph in RAM and serializes ONLY when asked, so when the session died the whole drive went with
# it -- no partial map, nothing on disk, nothing to salvage. The map had to be redriven.
#
# WHY NOT JUST RECORD EVERYTHING. `ros2 bag record -a` on this robot captures /ouster/points at
# 3.1 MB per cloud; one such bag in runs/ is 3.4 GB and had to be gitignored. The four topics below
# are what slam_toolbox actually consumes -- a LaserScan is ~4 KB, so this runs about 1-2 MB per
# minute and can be left on for an entire drive without thinking about it.
#
# /tf_static matters as much as /tf: without base_link->os_lidar the replayed scans cannot be
# placed on the robot at all, and the rebuild fails in a way that looks like bad odometry.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
source bringup/env.sh 2>/dev/null || { echo "cannot source bringup/env.sh"; exit 1; }
BAGDIR="${UTP_BAGDIR:-$PWD/runs/mapbags}"
TOPICS=(/scan /scan_nav /tf /tf_static /odom)
NAME="${2:-map}"
case "${1:-}" in
  start)
    mkdir -p "$BAGDIR"
    OUT="$BAGDIR/${NAME}_$(date -u +%Y%m%dT%H%M%SZ)"
    setsid nohup ros2 bag record -o "$OUT" "${TOPICS[@]}" >/dev/null 2>&1 < /dev/null &
    echo "  recording ${TOPICS[*]}"
    echo "  -> $OUT"
    echo "  stop with: bash bringup/map_insurance.sh stop"
    ;;
  stop)
    # Scope the kill by the bag output path under THIS repo, never by a bare process name.
    n=0
    for pid in $(ps -eo pid,args --no-headers | grep 'ros2 bag record' | grep -v grep | awk '{print $1}'); do
      if tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -q "$BAGDIR"; then
        kill -INT "$pid" 2>/dev/null && n=$((n+1))   # INT so the mcap gets closed cleanly
      fi
    done
    sleep 3
    echo "  stopped $n recorder(s); bags in $BAGDIR"
    ;;
  rebuild)
    BAG="$(ls -dt "$BAGDIR"/${NAME}_* 2>/dev/null | head -1)"
    [ -n "$BAG" ] || { echo "  no bag named ${NAME}_* in $BAGDIR" >&2; exit 1; }
    echo "  rebuilding from $BAG"
    echo "  1. start mapping:  ros2 run slam_toolbox async_slam_toolbox_node --ros-args \\"
    echo "                       --params-file $PWD/config/slam_os0.yaml -p mode:=mapping"
    echo "  2. replay:         ros2 bag play '$BAG' --clock"
    echo "  3. save:           bash bringup/map_persist.sh save $NAME"
    echo "  Replay is faster than the drive but NOT instant -- slam_toolbox must process every scan."
    ;;
  *) sed -n '2,12p' "$0"; exit 2 ;;
esac
