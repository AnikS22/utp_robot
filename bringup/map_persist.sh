#!/usr/bin/env bash
# Save the live map, prove it reads back, load it again, and confirm it is active.
#
#     bash bringup/map_persist.sh atrium
#
# ONE command for the whole persistence loop, because doing it in four steps meant four places to
# get it wrong. It fails loudly at each stage rather than leaving you with a file you find out is
# useless later.
#
# WHAT "PERSISTENT" ACTUALLY REQUIRES, and the trap that makes this worth scripting: a MOLA
# started fresh ALSO has a `map` frame -- with its origin wherever the robot booted. The TF tree
# is identical either way, so a saved waypoint cannot tell the two apart by looking. Persistence
# only exists once a NAMED map is loaded and relocalized into, and that is what maps/.loaded_map
# records (name + the MOLA session it was loaded into, so the record cannot go stale silently).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"

NAME="${1:-}"
[ -n "$NAME" ] || { echo "usage: bash bringup/map_persist.sh <name>" >&2; exit 1; }
OUT="$REPO/maps/$NAME"
PLUGIN="$(ls /opt/ros/jazzy/lib/x86_64-linux-gnu/libmola_metric_maps.so* 2>/dev/null | head -1)"
[ -n "$PLUGIN" ] || { echo "map_persist: libmola_metric_maps.so not found (ros-jazzy-mola)" >&2; exit 1; }

echo "[1/4] saving ..."
ros2 service call /map_save mola_msgs/srv/MapSave "{map_path: '$OUT'}" | grep -q "success=True" \
    || { echo "  MOLA refused to save. Is it running?" >&2; exit 1; }
[ -f "$OUT.mm" ] || { echo "  no $OUT.mm written" >&2; exit 1; }
echo "      $(ls -lh "$OUT".* | awk '{printf "%s %s  ", $9, $5}')"

echo "[2/4] verifying it reads back ..."
INFO="$(mm-info -l "$PLUGIN" "$OUT.mm" 2>&1 | tail -2)"
echo "      $INFO"
KF="$(echo "$INFO" | grep -oE '[0-9]+ keyframes' | grep -oE '^[0-9]+' || echo 0)"
if [ "${KF:-0}" -lt 5 ]; then
    echo
    echo "  WARNING: only ${KF:-0} keyframes. MOLA lays keyframes down by DISTANCE TRAVELLED, so"
    echo "  this is the signature of a robot that barely moved. The file is valid and will load,"
    echo "  but it is a map of one spot, not of a room. A real drive gives dozens to hundreds."
    echo "  Check with: python3 bringup/map_watch.py   (run it WHILE driving)"
fi

echo "[3/4] loading it back ..."
ros2 service call /map_load mola_msgs/srv/MapLoad "{map_path: '$OUT'}" | grep -q "success=True" \
    || { echo "  MOLA refused to load its own map." >&2; exit 1; }

echo "[4/4] recording which map is active ..."
SESS="$(python3 - <<PY
import sys, time
sys.path.insert(0, "$REPO/bringup")
import rclpy
from rclpy.node import Node
from pose_source import mola_session_id
rclpy.init(); n = Node("utp_map_persist_probe")
try:
    end = time.monotonic() + 3.0; sid = None
    while time.monotonic() < end and sid is None:
        rclpy.spin_once(n, timeout_sec=0.1); sid = mola_session_id(n)
    print(sid or "")
finally:
    n.destroy_node(); rclpy.shutdown()
PY
)"
SESS="$(echo "$SESS" | tail -1)"
[ -n "$SESS" ] || { echo "  could not read the MOLA session id" >&2; exit 1; }
printf '%s %s\n' "$NAME" "$SESS" > "$REPO/maps/.loaded_map"

echo
echo "  MAP '$NAME' IS PERSISTENT AND ACTIVE  [mola session ${SESS:0:8}]"
echo "  Waypoints recorded now survive restarts:"
echo "      python3 bringup/waypoints.py record <name> --frame map"
echo "  Next session, before anything else:"
echo "      bash bringup/map_load.sh $NAME"
