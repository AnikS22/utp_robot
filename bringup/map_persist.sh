#!/usr/bin/env bash
# THE map script. Save a map so that MAP-FRAME WAYPOINTS SURVIVE, list what is on disk, or resume
# mapping into a partial one. This replaces save_map.sh, map_save.sh, map_load.sh and
# resume_map.sh, which overlapped, disagreed, and split the checks between them.
#
#     bash bringup/map_persist.sh save atrium     # while slam_toolbox is STILL RUNNING
#     bash bringup/map_persist.sh atrium          # same thing; `save` is the default verb
#     bash bringup/map_persist.sh list            # what is on disk, and what is usable
#     bash bringup/map_persist.sh resume partial_01 [--at-pose X Y THETA]
#
# WHAT A "SAVED MAP" HAS TO BE, and why the .pgm alone is not one:
#
#   maps/<name>.pgm + .yaml    the occupancy grid. What Nav2's costmap plans on. A PICTURE: it
#                              has no pose graph, so nothing can relocalize into it, ever.
#   maps/<name>.posegraph      slam_toolbox's own graph, plus <name>.data. THIS is what
#                              `mode: localization` deserializes, and therefore the only thing
#                              that makes trial 50's coordinates mean what trial 1's meant.
#   maps/.loaded_map           which named map is live, and in which SLAM session. A fresh SLAM
#                              also has a `map` frame -- origin wherever the robot booted -- and
#                              the TF tree looks identical. safety/map_frame.py refuses named
#                              waypoints when this does not match.
#
# ALL THREE OR NONE. No posegraph and `session.sh nav` cannot relocalize; no .loaded_map and every
# `waypoints.py record --frame map` stores as nameless, which nav2_goto.py then refuses to drive
# to. Saving does not stop mapping: save after every closed loop, not once at the end.
#
# MOLA is still handled where it is running (a single .mm), but it is not the campaign stack --
# it produced 1.4 Hz against a 10 Hz input and was rejected. slam_toolbox is checked first.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/bringup/env.sh"

usage() { sed -n '2,10p' "$0" | sed 's/^# \?//'; exit 2; }

VERB="${1:-}"
case "$VERB" in
    save|resume|list) shift ;;
    ""|-h|--help)     usage ;;
    *)                VERB=save ;;          # bare name: `map_persist.sh atrium`
esac

has_service() { timeout 10 ros2 service list 2>/dev/null | grep -qx "$1"; }

slam_session() {
    python3 - "$REPO" <<'PY' | tail -1
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

# ============================================================================== list
if [ "$VERB" = "list" ]; then
    echo "maps in $REPO/maps:"
    found=0
    for y in "$REPO"/maps/*.yaml; do
        [ -e "$y" ] || continue
        n="$(basename "$y" .yaml)"; s="$REPO/maps/$n"
        # maps/ also holds waypoints.yaml and site_markers.yaml. A map is a yaml WITH a grid.
        [ -s "$s.pgm" ] || continue
        found=1
        if [ -s "$s.posegraph" ] && [ -s "$s.data" ]; then
            printf '  %-44s USABLE for a campaign (grid + pose graph)\n' "$n"
        else
            printf '  %-44s grid only -- CANNOT be relocalized into; re-map and save\n' "$n"
        fi
    done
    [ "$found" = 1 ] || echo "  (none)"
    for m in "$REPO"/maps/*.mm; do
        [ -e "$m" ] && printf '  %-44s MOLA map (not the campaign stack)\n' "$(basename "$m" .mm)"
    done
    echo
    if [ -s "$REPO/maps/.loaded_map" ]; then
        read -r LN LS < "$REPO/maps/.loaded_map"
        LIVE="$(slam_session)"
        if [ -n "$LIVE" ] && [ "$LIVE" = "$LS" ]; then
            echo "  LIVE NOW: '$LN' [slam ${LS:0:8}] -- map-frame waypoints are valid"
        else
            echo "  .loaded_map names '$LN' but that SLAM session is gone. Map-frame waypoints"
            echo "  recorded against it are correctly refused until you localize into it again:"
            echo "      MAP_NAME=$LN bash bringup/session.sh nav"
        fi
    else
        echo "  no map is loaded (maps/.loaded_map absent)"
    fi
    exit 0
fi

NAME="${1:-}"
[ -n "$NAME" ] || usage
case "$NAME" in */*) echo "map name must be a bare filename, not a path: $NAME" >&2; exit 2 ;; esac
STEM="$REPO/maps/$NAME"
mkdir -p "$REPO/maps"

# ============================================================================== resume
if [ "$VERB" = "resume" ]; then
    [ -s "$STEM.posegraph" ] || { echo "no $STEM.posegraph. On disk:" >&2
        find "$REPO/maps" -maxdepth 1 -name '*.posegraph' -printf '  %f\n' 2>/dev/null \
            || echo "  (none)" >&2; exit 1; }
    has_service /slam_toolbox/deserialize_map \
        || { echo "slam_toolbox is not running on ROS_DOMAIN_ID=$ROS_DOMAIN_ID." >&2
             echo "  Start it first: bash bringup/session.sh map" >&2; exit 1; }
    shift
    MATCH=1; PX=0.0; PY=0.0; PT=0.0        # 1 = START_AT_FIRST_NODE
    if [ "${1:-}" = "--at-pose" ]; then
        MATCH=2; PX="${2:?--at-pose needs X Y THETA}"; PY="${3:?}"; PT="${4:?}"
    fi
    echo "[resume] loading $NAME.posegraph (match_type=$MATCH, pose $PX $PY $PT)"
    timeout 120 ros2 service call /slam_toolbox/deserialize_map \
        slam_toolbox/srv/DeserializePoseGraph \
        "{filename: '$STEM', match_type: $MATCH, initial_pose: {x: $PX, y: $PY, theta: $PT}}" \
        2>&1 | tail -3
    cat <<'NEXT'

  Loaded. BEFORE DRIVING: in RViz the live scan must lie ON the walls already in the map.
  Beside them means the graph loaded at the wrong pose -- reload with --at-pose rather than
  driving on, because every scan from here compounds the error and the tear looks like
  ordinary drift.

  Then drive, and save under a NEW name so a bad resume cannot overwrite the good partial:
      bash bringup/map_persist.sh save <new_name>
NEXT
    exit 0
fi

# ============================================================================== save
if has_service /slam_toolbox/serialize_map; then
    # A map costs a walk around a building. Do not let a bad one silently replace a good one.
    # CHECK ALL FOUR EXTENSIONS, not just .pgm. The four files come from two different steps --
    # serialize_map writes .posegraph/.data, map_saver writes .pgm/.yaml -- so an interrupted save
    # leaves some present and some absent. Guarding on .pgm alone means a name holding a good
    # .posegraph but no .pgm gets overwritten with no prompt at all, and the posegraph is the half
    # that cannot be regenerated from the other.
    _clobber=""
    for _e in pgm yaml posegraph data; do
        [ -f "$STEM.$_e" ] && _clobber="$_clobber .$_e"
    done
    if [ -n "$_clobber" ] && [ -z "${UTP_MAP_OVERWRITE:-}" ]; then
        _newest="$(ls -t "$STEM".pgm "$STEM".yaml "$STEM".posegraph "$STEM".data 2>/dev/null | head -1)"
        printf '  %s already exists (%s, modified %s). Overwrite? type yes: ' \
            "$NAME" "${_clobber# }" "$(date -r "$_newest" '+%Y-%m-%d %H:%M' 2>/dev/null)"
        read -r ok
        [ "$ok" = "yes" ] || { echo "  keeping the existing map."; exit 1; }
    fi

    echo "[1/4] pose graph     -> $NAME.posegraph / .data"
    timeout 180 ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph "{filename: '$STEM'}" >/dev/null 2>&1
    echo "[2/4] occupancy grid -> $NAME.pgm / .yaml"
    timeout 180 ros2 service call /slam_toolbox/save_map \
        slam_toolbox/srv/SaveMap "{name: {data: '$STEM'}}" >/dev/null 2>&1

    # THE SERVICES RETURN SUCCESS WHEN NOTHING LANDS ON DISK, and answer before the write
    # completes. Check the disk, with a grace period, and never the return code.
    for i in $(seq 1 20); do
        [ -s "$STEM.posegraph" ] && [ -s "$STEM.yaml" ] && break
        sleep 0.5
    done
    missing=0
    for ext in posegraph data pgm yaml; do
        if [ -s "$STEM.$ext" ]; then
            printf '      ok       %-24s %s\n' "$NAME.$ext" "$(du -h "$STEM.$ext" | cut -f1)"
        else
            printf '      MISSING  %s\n' "$NAME.$ext"; missing=1
        fi
    done
    if [ "$missing" != 0 ]; then
        echo >&2
        echo "  NOT SAVED. Do not stop mapping -- the graph is still in the live node." >&2
        echo "  Usually /slam_toolbox is not active: ros2 lifecycle get /slam_toolbox" >&2
        echo "  A serialize that dies on a large map is the default stack size; session.sh now" >&2
        echo "  launches with config/slam_os0.yaml, which sets stack_size_to_use." >&2
        exit 1
    fi

    echo "[3/4] extent + coverage"
    # "The map saved" and "the map covers the building" are different claims, and only the second
    # one matters. Occupied cells are the honest proxy: a robot that barely moved still yields a
    # perfectly valid, perfectly useless map, and dimensions grow on one stray beam.
    python3 - "$STEM" <<'PY'
import re, sys
stem = sys.argv[1]
try:
    res = float(re.search(r"^resolution:\s*([0-9.]+)", open(stem + ".yaml").read(), re.M).group(1))
    d = open(stem + ".pgm", "rb").read()
    tok, i = [], 2
    while len(tok) < 3:                       # P5 header: width height maxval, skipping comments
        while i < len(d) and d[i:i+1].isspace(): i += 1
        if d[i:i+1] == b"#":
            while i < len(d) and d[i:i+1] != b"\n": i += 1
            continue
        j = i
        while j < len(d) and not d[j:j+1].isspace(): j += 1
        tok.append(d[i:j]); i = j
    i += 1
    w, h = int(tok[0]), int(tok[1])
    occ = sum(1 for b in d[i:] if b < 100)
    print(f"      {w} x {h} cells at {res} m  =  {w*res:.1f} x {h*res:.1f} m, "
          f"{occ} occupied cells")
    if occ < 2000:
        print()
        print(f"      WARNING: only {occ} occupied cells. That is the signature of a robot that")
        print("      barely moved -- a valid map of one spot, not of a corridor. Waypoints")
        print("      recorded against it will relocalize badly. Drive the full loop, CLOSE it,")
        print("      and save again. Watch it fill: python3 bringup/map_watch.py")
except Exception as e:
    print(f"      (could not read extent: {e})")
PY

    SESS="$(slam_session)"
    [ -n "$SESS" ] || { echo "  could not read the SLAM session id -- is exactly one node" >&2
                        echo "  publishing /map? The map files ARE saved; re-run to record" >&2
                        echo "  provenance, or waypoints will store as nameless." >&2; exit 1; }
    printf '%s %s\n' "$NAME" "$SESS" > "$REPO/maps/.loaded_map"
    echo "[4/4] maps/.loaded_map -> $NAME [slam ${SESS:0:8}]"
    cat <<NEXT

  MAP '$NAME' IS SAVED AND ACTIVE. Record waypoints NOW, while this session is still up --
  a recording made after slam_toolbox restarts is anchored to a different origin:
      python3 bringup/waypoints.py record start  --frame map
      python3 bringup/waypoints.py record door   --frame map
  Next session, before anything else:
      MAP_NAME=$NAME bash bringup/session.sh nav
NEXT
    exit 0
fi

# ------------------------------------------------------------------------------ MOLA (legacy)
if has_service /map_save; then
    echo "[mola] /slam_toolbox absent; using MOLA's /map_save"
    ros2 service call /map_save mola_msgs/srv/MapSave "{map_path: '$STEM'}" \
        | grep -q "success=True" || { echo "  MOLA refused to save." >&2; exit 1; }
    [ -s "$STEM.mm" ] || { echo "  no $STEM.mm written" >&2; exit 1; }
    ros2 service call /map_load mola_msgs/srv/MapLoad "{map_path: '$STEM'}" \
        | grep -q "success=True" || { echo "  MOLA refused to load its own map." >&2; exit 1; }
    SESS="$(slam_session)"
    [ -n "$SESS" ] || { echo "  could not read the SLAM session id" >&2; exit 1; }
    printf '%s %s\n' "$NAME" "$SESS" > "$REPO/maps/.loaded_map"
    echo "  MAP '$NAME' IS PERSISTENT AND ACTIVE  [mola ${SESS:0:8}]"
    exit 0
fi

echo "map_persist: no SLAM is running on ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-?}." >&2
echo "  Expected /slam_toolbox/serialize_map (slam_toolbox) or /map_save (MOLA)." >&2
echo "  Start mapping first:  bash bringup/session.sh map" >&2
exit 1
