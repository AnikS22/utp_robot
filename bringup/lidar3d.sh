#!/usr/bin/env bash
# Bring up the Ouster OS0-128 and publish /ouster/points (+ base_link -> os_sensor).
#
#     bash bringup/lidar3d.sh            # driver + mount TF
#     bash bringup/lidar3d.sh --no-tf    # driver only (a URDF owns the TF)
#
# WHY THIS SENSOR. slam_toolbox could not hold a pose in this building on the A1M8, which
# delivers 44 valid points per scan against the several hundred its correlative matcher needs.
# Measured here 2026-08-30: this sensor returns 121,367 of 131,072 beams (92.6%) with full 360
# degree coverage and no sparse sectors. The mapping problem was the sensor, not the algorithm.
#
# THE udp_dest TRAP. The sensor was found configured to stream to 192.168.1.106, which is not
# this machine, while reporting "status": "RUNNING" over its HTTP API. ouster_ros rewrites
# udp_dest on connect so the driver is immune -- but anything that does NOT reconfigure (a raw
# packet capture, a viewer pointed at the sensor) sees zero packets from a healthy sensor.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"

PUBLISH_TF=1
for a in "$@"; do [ "$a" = "--no-tf" ] && PUBLISH_TF=0; done

HOST="$(python3 -c "import yaml;print(yaml.safe_load(open('$REPO/config/ouster.yaml'))['host'])")"
if ! timeout 5 curl -sf "http://$HOST/api/v1/sensor/metadata/sensor_info" >/dev/null 2>&1; then
    echo "lidar3d: sensor $HOST is not answering its HTTP API." >&2
    echo "  is it powered, and is the interface on its subnet up? (ip -brief addr)" >&2
    exit 1
fi
echo "[lidar3d] sensor $HOST: $(timeout 5 curl -s "http://$HOST/api/v1/sensor/metadata/sensor_info" \
    | python3 -c 'import json,sys;d=json.load(sys.stdin);print(d["prod_line"], d["prod_sn"], d["status"])')"

CHILDREN=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${CHILDREN[@]:-}"; do
        [ -n "${pid:-}" ] || continue
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${CHILDREN[@]:-}"; do
        [ -n "${pid:-}" ] || continue
        kill -KILL -- "-$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    echo "[lidar3d] children reaped"
}
trap cleanup EXIT INT TERM

if [ "$PUBLISH_TF" = "1" ]; then
    read -r MX MY MZ MR MP MYAW PARENT SENSOR <<<"$(python3 - "$REPO/config/ouster.yaml" <<'PY'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1])); m = c["mount"]
print(m["x_m"], m["y_m"], m["z_m"], m["roll_rad"], m["pitch_rad"], m["yaw_rad"],
      c["parent_frame"], c["sensor_frame"])
PY
)"
    echo "[lidar3d] $PARENT -> $SENSOR at ($MX, $MY, $MZ) m"
    setsid ros2 run tf2_ros static_transform_publisher \
        --x "$MX" --y "$MY" --z "$MZ" --roll "$MR" --pitch "$MP" --yaw "$MYAW" \
        --frame-id "$PARENT" --child-frame-id "$SENSOR" &
    CHILDREN+=($!)
fi

setsid ros2 launch ouster_ros driver.launch.py \
    params_file:="$REPO/config/ouster_driver.yaml" viz:=false &
CHILDREN+=($!)
wait
