#!/usr/bin/env bash
# Save the live MOLA map and immediately verify it can be read back.
#
#     bash bringup/map_save.sh atrium_os0
#
# WHY THE VERIFY STEP. `mm-info <map>.mm` fails on a MOLA-saved map with
#
#     Stored object has class 'mola::KeyframePointCloudMap' which is not registered!
#
# and a wall of MRPT backtrace, which reads exactly like a corrupt file. The map is fine: the
# standalone mm-* tools do not link MOLA's map classes, so they must be handed the plugin with
# -l. Every mm-* tool takes it (mm-info, mm-filter, mm-viewer). Getting this wrong after a long
# mapping drive would look like losing the drive.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"

NAME="${1:-}"
[ -n "$NAME" ] || { echo "usage: bash bringup/map_save.sh <name>" >&2; exit 1; }
OUT="$REPO/maps/$NAME"
mkdir -p "$REPO/maps"

PLUGIN="$(ls /opt/ros/jazzy/lib/x86_64-linux-gnu/libmola_metric_maps.so* 2>/dev/null | head -1)"
[ -n "$PLUGIN" ] || { echo "map_save: libmola_metric_maps.so not found -- is ros-jazzy-mola installed?" >&2; exit 1; }

echo "[map_save] saving to $OUT ..."
ros2 service call /map_save mola_msgs/srv/MapSave "{map_path: '$OUT'}" | tail -3

[ -f "$OUT.mm" ] || { echo "map_save: no $OUT.mm was written" >&2; exit 1; }
echo "[map_save] wrote: $(ls -lh "$OUT".* | awk '{print $9, $5}' | tr '\n' ' ')"

echo "[map_save] verifying with the MOLA map plugin loaded ..."
mm-info -l "$PLUGIN" "$OUT.mm" | tail -3
