#!/usr/bin/env bash
# Continue mapping from a saved pose graph instead of starting the building over.
#
#     bash bringup/resume_map.sh partial_01        # load maps/partial_01.posegraph and keep going
#     bash bringup/resume_map.sh partial_01 --at-pose 3.2 -1.4 0.0
#
# Requires slam_toolbox to already be running in `mode: mapping` (bringup/mapping.sh does that).
# Loads the graph over the /slam_toolbox/deserialize_map service, so no config editing and no
# restart -- the alternative, setting map_file_name in slam.yaml, means every resume needs a
# different params file.
#
# WHERE THE ROBOT HAS TO BE STANDING
# START_AT_FIRST_NODE (the default here) tells slam_toolbox the robot is back at the pose it
# started the ORIGINAL session from. If it is somewhere else, the first scans get matched against
# the wrong part of the graph and the map tears -- and the tear looks like ordinary drift, so it
# is easy to miss until the map is finished and wrong.
#
# So either:
#   * drive back to roughly where the first session began, then resume with no options, or
#   * pass --at-pose X Y THETA (metres, radians, in the map frame) for where it actually stands.
#
# Either way, watch RViz for a few seconds after loading: the live scan should sit ON the walls
# already in the map. If it sits beside them, stop and reload with a corrected pose -- driving on
# from a bad match corrupts everything that follows.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/bringup/env.sh"

NAME="${1:-}"
[ -n "$NAME" ] || { sed -n '2,8p' "$0"; exit 2; }
shift
case "$NAME" in */*) echo "pass a bare name, not a path: $NAME" >&2; exit 2 ;; esac

STEM="$REPO/maps/$NAME"
[ -f "$STEM.posegraph" ] || {
    echo "no $STEM.posegraph" >&2
    echo "Available:" >&2
    find "$REPO/maps" -maxdepth 1 -name '*.posegraph' -printf '  %f\n' 2>/dev/null || echo "  (none)" >&2
    exit 1
}

MATCH=1            # START_AT_FIRST_NODE
PX=0.0; PY=0.0; PT=0.0
if [ "${1:-}" = "--at-pose" ]; then
    MATCH=2        # START_AT_GIVEN_POSE
    PX="${2:?--at-pose needs X Y THETA}"; PY="${3:?}"; PT="${4:?}"
fi

if ! ros2 service list 2>/dev/null | grep -q '/slam_toolbox/deserialize_map'; then
    echo "slam_toolbox is not running on ROS_DOMAIN_ID=$ROS_DOMAIN_ID." >&2
    echo "  Start the mapping stack first, then resume into it." >&2
    exit 1
fi

echo "[resume] loading $NAME.posegraph  (match_type=$MATCH, pose $PX $PY $PT)"
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
    "{filename: '$STEM', match_type: $MATCH, initial_pose: {x: $PX, y: $PY, theta: $PT}}" \
    2>&1 | tail -3

cat <<'NEXT'

  Loaded. Before driving anywhere:

    Look at RViz. The live red scan must lie ON the walls already drawn in the map.
    Sitting beside them means the graph loaded at the wrong pose -- reload with
    --at-pose rather than driving on, because every scan from here compounds the error.

  Then drive. Save with `bash bringup/save_map.sh <name>` -- use a NEW name, so a bad
  resume cannot overwrite the good partial you resumed from.
NEXT
