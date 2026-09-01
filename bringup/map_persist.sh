#!/usr/bin/env bash
# Save the live map so that MAP-FRAME WAYPOINTS SURVIVE, and prove every part of that claim.
#
#     bash bringup/map_persist.sh atrium
#
# WHAT A "SAVED MAP" HAS TO BE, and why saving the .pgm is not enough.
#
#   maps/<name>.pgm + .yaml    the occupancy grid. What Nav2's costmap needs. NOT enough to
#                              relocalize into: it has no pose graph, so slam_toolbox cannot
#                              scan-match a new session onto the old coordinates.
#   maps/<name>.posegraph      slam_toolbox's serialized graph, plus <name>.data. THIS is what
#                              `mode: localization` + `map_file_name` deserializes, and therefore
#                              this is the only thing that makes trial 50's coordinates mean what
#                              trial 1's meant.
#   maps/.loaded_map           which named map is live, and in which SLAM session. A fresh SLAM
#                              also has a `map` frame -- with its origin wherever the robot
#                              booted -- and the TF tree looks identical. safety/map_frame.py
#                              refuses named waypoints when this does not match.
#
# ALL THREE OR NONE. Miss the posegraph and `session.sh nav` starts localization against nothing;
# miss .loaded_map and every `waypoints.py record --frame map` is stored as nameless, which
# nav2_goto.py then correctly refuses to drive to.
#
# MOLA is still handled (it saves a single .mm) but it is not the stack that runs the campaign:
# it produced 1.4 Hz against a 10 Hz input and was rejected. slam_toolbox is checked first.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"

NAME="${1:-}"
[ -n "$NAME" ] || { echo "usage: bash bringup/map_persist.sh <name>" >&2; exit 1; }
case "$NAME" in */*) echo "map name must not contain '/'" >&2; exit 1;; esac
OUT="$REPO/maps/$NAME"
mkdir -p "$REPO/maps"

has_service() { timeout 10 ros2 service list 2>/dev/null | grep -qx "$1"; }

session_id() {
    python3 - "$REPO" <<'PY'
import sys, time
sys.path.insert(0, sys.argv[1] + "/bringup")
import rclpy
from rclpy.node import Node
from pose_source import slam_session_id
rclpy.init(); n = Node("utp_map_persist_probe")
try:
    end = time.monotonic() + 3.0; sid = None
    while time.monotonic() < end and sid is None:
        rclpy.spin_once(n, timeout_sec=0.1); sid = slam_session_id(n)
    print(sid or "")
finally:
    n.destroy_node(); rclpy.shutdown()
PY
}

# --------------------------------------------------------------------------- slam_toolbox
if has_service /slam_toolbox/serialize_map; then
    echo "[1/4] serializing the pose graph -> $NAME.posegraph ..."
    timeout 120 ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph "{filename: '$OUT'}" | tail -2
    # The service returns result=0 on success, but the FILE is the thing that matters and the
    # service has been observed to answer before the write lands.
    for i in $(seq 1 20); do [ -f "$OUT.posegraph" ] && break; sleep 0.5; done
    [ -f "$OUT.posegraph" ] || { echo "  no $OUT.posegraph written -- serialize_map failed." >&2
                                 echo "  (is /slam_toolbox active? ros2 lifecycle get /slam_toolbox)" >&2
                                 exit 1; }
    [ -f "$OUT.data" ] || { echo "  $OUT.posegraph exists but $OUT.data does not; both are needed" >&2
                            exit 1; }

    echo "[2/4] saving the occupancy grid -> $NAME.pgm/.yaml ..."
    timeout 120 ros2 service call /slam_toolbox/save_map \
        slam_toolbox/srv/SaveMap "{name: {data: '$OUT'}}" | tail -2
    for i in $(seq 1 20); do [ -f "$OUT.yaml" ] && break; sleep 0.5; done
    [ -f "$OUT.yaml" ] && [ -f "$OUT.pgm" ] \
        || { echo "  no $OUT.pgm/.yaml -- Nav2 would have no costmap to plan on." >&2; exit 1; }

    echo "[3/4] checking the map is of a ROOM, not of one spot ..."
    # A robot that barely moved still yields a perfectly valid, perfectly useless map. Occupied
    # cells (value 0 in the pgm) are the cheap proxy: a parked scan gives a few hundred.
    OCC="$(python3 - "$OUT.pgm" <<'PY'
import sys
d = open(sys.argv[1], "rb").read()
# P5 header: magic, width, height, maxval -- then binary. Skip comment lines.
tok, i = [], 2
while len(tok) < 3:
    while i < len(d) and d[i:i+1].isspace(): i += 1
    if d[i:i+1] == b"#":
        while i < len(d) and d[i:i+1] != b"\n": i += 1
        continue
    j = i
    while j < len(d) and not d[j:j+1].isspace(): j += 1
    tok.append(d[i:j]); i = j
i += 1
print(sum(1 for b in d[i:] if b < 100))
PY
)"
    echo "      $OCC occupied cells at 0.05 m"
    if [ "${OCC:-0}" -lt 2000 ]; then
        echo
        echo "  WARNING: only ${OCC:-0} occupied cells. That is the signature of a robot that"
        echo "  barely moved -- a valid map of one spot, not of a corridor. Waypoints recorded"
        echo "  against it will relocalize badly. Drive the full loop and save again."
        echo "  Watch it fill while driving: python3 bringup/map_watch.py"
    fi
    SESS="$(session_id | tail -1)"
    [ -n "$SESS" ] || { echo "  could not read the SLAM session id (nothing publishing /map?)" >&2
                        exit 1; }
    printf '%s %s\n' "$NAME" "$SESS" > "$REPO/maps/.loaded_map"
    echo "[4/4] recorded maps/.loaded_map: $NAME  [slam ${SESS:0:8}]"
    echo
    echo "  MAP '$NAME' IS SAVED AND ACTIVE. Files:"
    ls -lh "$OUT".posegraph "$OUT".data "$OUT".pgm "$OUT".yaml | awk '{print "      " $9 "  " $5}'
    echo
    echo "  Record waypoints NOW, while this session is still up:"
    echo "      python3 bringup/waypoints.py record start --frame map"
    echo "  Next session, before anything else:"
    echo "      MAP_NAME=$NAME bash bringup/session.sh nav"
    exit 0
fi

# --------------------------------------------------------------------------- MOLA (legacy)
if has_service /map_save; then
    echo "[mola] /slam_toolbox not present; using MOLA's /map_save"
    PLUGIN="$(ls /opt/ros/jazzy/lib/x86_64-linux-gnu/libmola_metric_maps.so* 2>/dev/null | head -1)"
    ros2 service call /map_save mola_msgs/srv/MapSave "{map_path: '$OUT'}" | grep -q "success=True" \
        || { echo "  MOLA refused to save." >&2; exit 1; }
    [ -f "$OUT.mm" ] || { echo "  no $OUT.mm written" >&2; exit 1; }
    [ -n "$PLUGIN" ] && mm-info -l "$PLUGIN" "$OUT.mm" 2>&1 | tail -2
    ros2 service call /map_load mola_msgs/srv/MapLoad "{map_path: '$OUT'}" | grep -q "success=True" \
        || { echo "  MOLA refused to load its own map." >&2; exit 1; }
    SESS="$(session_id | tail -1)"
    [ -n "$SESS" ] || { echo "  could not read the SLAM session id" >&2; exit 1; }
    printf '%s %s\n' "$NAME" "$SESS" > "$REPO/maps/.loaded_map"
    echo "  MAP '$NAME' IS PERSISTENT AND ACTIVE  [mola ${SESS:0:8}]"
    exit 0
fi

echo "map_persist: no SLAM is running." >&2
echo "  Expected /slam_toolbox/serialize_map (slam_toolbox) or /map_save (MOLA)." >&2
echo "  Start mapping first:  bash bringup/session.sh map" >&2
exit 1
