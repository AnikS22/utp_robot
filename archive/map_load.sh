#!/usr/bin/env bash
# Load a saved MOLA map and relocalize into it, so map-frame waypoints mean what they meant
# when they were recorded.
#
#     bash bringup/map_load.sh atrium_os0              # relocalize at the map origin
#     bash bringup/map_load.sh atrium_os0 12.4 3.1 90  # ... near x y yaw_deg, if you know roughly
#
# WHY THIS MATTERS MORE THAN IT LOOKS. A MOLA started FRESH also has a `map` frame -- with its
# origin wherever the robot happened to boot. The TF tree is identical either way, so a stored
# coordinate cannot tell the two apart by looking. This script is what makes the difference
# recordable: it writes maps/.loaded_map with the map name AND the MOLA session it was loaded
# into, and safety/map_frame.py refuses named waypoints when that does not match.
#
# The session is recorded because the name alone goes stale: load 'atrium', restart MOLA fresh,
# and a name-only file would still claim 'atrium' while the frame origin had moved.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"

NAME="${1:-}"
[ -n "$NAME" ] || { echo "usage: bash bringup/map_load.sh <name> [x y yaw_deg]" >&2; exit 1; }
MAP="$REPO/maps/$NAME"
[ -f "$MAP.mm" ] || { echo "map_load: $MAP.mm does not exist. Saved maps:" >&2
                      ls "$REPO/maps"/*.mm 2>/dev/null >&2 || echo "  (none)" >&2; exit 1; }

echo "[map_load] loading $MAP.mm ..."
OUT="$(ros2 service call /map_load mola_msgs/srv/MapLoad "{map_path: '$MAP'}")"
echo "$OUT" | tail -3
echo "$OUT" | grep -q "success=True" || { echo "map_load: MOLA refused the map." >&2; exit 1; }

if [ "$#" -ge 4 ]; then
    X="$2"; Y="$3"; YAW_DEG="$4"
    echo "[map_load] relocalizing near ($X, $Y, ${YAW_DEG} deg) ..."
    read -r QZ QW <<<"$(python3 -c "
import math,sys
h=math.radians(float(sys.argv[1]))/2.0
print(math.sin(h), math.cos(h))" "$YAW_DEG")"
    ros2 service call /relocalize_near_pose mola_msgs/srv/RelocalizeNearPose \
        "{pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: $X, y: $Y, z: 0.0}, \
orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}}" | tail -3
else
    echo "[map_load] no pose given -- MOLA keeps its current estimate as the seed."
    echo "  If the robot is NOT near where it was when the map recording started, pass"
    echo "  an approximate x y yaw_deg, or relocalization may settle in the wrong place."
fi

# Record what is loaded, against the MOLA instance it was loaded into.
SESS="$(python3 - <<'PY'
import sys
sys.path.insert(0, "/home/weim/utp_robot/bringup")
import rclpy
from rclpy.node import Node
from pose_source import mola_session_id
rclpy.init()
n = Node("utp_map_load_probe")
try:
    import time
    end = time.monotonic() + 3.0
    sid = None
    while time.monotonic() < end and sid is None:
        rclpy.spin_once(n, timeout_sec=0.1)
        sid = mola_session_id(n)
    print(sid or "")
finally:
    n.destroy_node(); rclpy.shutdown()
PY
)"
SESS="$(echo "$SESS" | tail -1)"
[ -n "$SESS" ] || { echo "map_load: could not read the MOLA session id -- is MOLA running?" >&2; exit 1; }
printf '%s %s\n' "$NAME" "$SESS" > "$REPO/maps/.loaded_map"
echo "[map_load] map '$NAME' active in MOLA session ${SESS:0:8}"
echo "  Map-frame waypoints naming '$NAME' will now validate. Record with:"
echo "      python3 bringup/waypoints.py record <name> --frame map"
