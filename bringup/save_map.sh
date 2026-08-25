#!/usr/bin/env bash
# Save the map slam_toolbox is currently holding. Run it while mapping.sh is still up.
#
#     bash bringup/save_map.sh            # -> maps/site.{pgm,yaml,posegraph,data}
#     bash bringup/save_map.sh lobby_v2   # -> maps/lobby_v2.*
#
# Saves BOTH artefacts, because they are not interchangeable and losing either costs a re-drive:
#   .pgm + .yaml   the occupancy grid. This is what Nav2's map_server loads. It is a picture --
#                  you cannot continue mapping from it, ever.
#   .posegraph     slam_toolbox's own state. The ONLY thing that lets you resume mapping later
#                  and add a wing you missed, instead of driving the whole building again.
#
# Saving does not stop mapping. You can save, keep driving, and save again over the top -- and
# that is the right habit: save after every closed loop, not once at the end.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/bringup/env.sh"

NAME="${1:-site}"
case "$NAME" in
    */*|"") echo "name must be a bare filename, not a path: $NAME" >&2; exit 2 ;;
esac
mkdir -p "$REPO/maps"
STEM="$REPO/maps/$NAME"

if ! ros2 service list 2>/dev/null | grep -q '/slam_toolbox/save_map'; then
    echo "slam_toolbox is not running on ROS_DOMAIN_ID=$ROS_DOMAIN_ID." >&2
    echo "  There is nothing to save. Start bringup/mapping.sh, drive, THEN save." >&2
    exit 1
fi

# Refuse to clobber a good map with a bad one by accident. A map costs a walk around a building.
if [ -f "$STEM.pgm" ]; then
    printf '  %s.pgm already exists (%s). Overwrite? type yes: ' "$NAME" \
        "$(date -r "$STEM.pgm" '+%Y-%m-%d %H:%M')"
    read -r ok
    [ "$ok" = "yes" ] || { echo "  keeping the existing map."; exit 1; }
fi

echo "[save] occupancy grid -> $STEM.pgm / .yaml"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '$STEM'}}" >/dev/null

echo "[save] pose graph     -> $STEM.posegraph / .data"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$STEM'}" >/dev/null

# The services return success even when nothing lands on disk, so check the disk, not the return.
missing=0
for ext in pgm yaml posegraph data; do
    if [ -s "$STEM.$ext" ]; then
        printf '  ok   %-28s %s\n' "$NAME.$ext" "$(du -h "$STEM.$ext" | cut -f1)"
    else
        printf '  MISSING  %s\n' "$NAME.$ext"
        missing=1
    fi
done
[ "$missing" = "0" ] || { echo; echo "Not saved. Do not stop mapping." >&2; exit 1; }

echo
# Report the extent, because "the map saved" and "the map covers the building" are different
# claims and only the second one matters. A map far smaller than the floor you walked means
# slam_toolbox lost the odom anchor partway and quietly kept going.
python3 - "$STEM" <<'PY'
import re, sys
stem = sys.argv[1]
try:
    res = float(re.search(r"^resolution:\s*([0-9.]+)", open(stem + ".yaml").read(),
                          re.M).group(1))
    with open(stem + ".pgm", "rb") as f:
        head = f.read(128).split()
    w, h = int(head[1]), int(head[2])
    print(f"  {w} x {h} cells at {res} m/cell  =  {w * res:.1f} x {h * res:.1f} m")
except Exception as e:
    print(f"  (could not read extent: {e})")
PY
cat <<'NEXT'

  Open maps/NAME.pgm in any image viewer before trusting it. You are looking for straight
  corridors, walls one cell thick, and no room drawn twice. Those faults are invisible from
  inside RViz mid-drive and obvious in the finished picture.

  Then hand it to Nav2:
      ros2 launch nav2_bringup ranger_nav.launch.py \
           localization:=amcl map:=maps/NAME.yaml use_sim_time:=false
  and give it an initial pose in RViz (2D Pose Estimate) -- AMCL is launched with
  set_initial_pose: false, so until you do, map->odom does not exist and nothing plans.
NEXT
