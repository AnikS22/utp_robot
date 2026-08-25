#!/usr/bin/env bash
# Print the RPLIDAR's serial device path, or fail with a reason. Source or call.
#
#     PORT="$(bash bringup/find_lidar.sh)" || exit 1
#
# Exists because the port was hardcoded to a by-id path containing the adapter's serial number
# (..._0001-if00-port0). That breaks the moment the adapter is swapped, and on 2026-08-21 an
# accidental unplug re-enumerated the device -- the path survived, but nothing here should depend
# on that. Plugging in should just work.
#
# Resolution order:
#   1. $RPLIDAR_PORT if set and present  -- explicit override always wins
#   2. exactly one CP2102 under /dev/serial/by-id  -- the normal case
#   3. more than one  -- refuse and list them, because guessing which is the lidar risks opening
#      something else's device. Set RPLIDAR_PORT to choose.
#
# by-id, never /dev/ttyUSBn: the numbering DOES reorder. Observed live -- the lidar moved from
# ttyUSB0 to ttyUSB1 after a re-plug. A hardcoded ttyUSB0 would have pointed at nothing.
set -euo pipefail

if [ -n "${RPLIDAR_PORT:-}" ]; then
    if [ -e "$RPLIDAR_PORT" ]; then
        echo "$RPLIDAR_PORT"; exit 0
    fi
    echo "RPLIDAR_PORT is set to '$RPLIDAR_PORT' but that path does not exist." >&2
    exit 1
fi

BYID=/dev/serial/by-id
[ -d "$BYID" ] || { echo "no $BYID -- no USB serial device is plugged in at all." >&2; exit 1; }

mapfile -t FOUND < <(find "$BYID" -maxdepth 1 -name '*CP2102*' 2>/dev/null | sort)

case "${#FOUND[@]}" in
    1) echo "${FOUND[0]}" ;;
    0)
        echo "No CP2102 adapter under $BYID. The RPLIDAR A1M8 presents as a Silicon Labs" >&2
        echo "CP2102 UART bridge (USB 10c4:ea60). Present devices:" >&2
        find "$BYID" -maxdepth 1 -mindepth 1 -printf '  %f\n' >&2 2>/dev/null || echo "  (none)" >&2
        exit 1 ;;
    *)
        echo "${#FOUND[@]} CP2102 adapters present -- refusing to guess which is the lidar," >&2
        echo "because opening the wrong one talks to someone else's hardware. Choose:" >&2
        for f in "${FOUND[@]}"; do echo "  RPLIDAR_PORT=$f" >&2; done
        exit 1 ;;
esac
