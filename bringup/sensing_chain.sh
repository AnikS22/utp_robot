#!/usr/bin/env bash
# THE sensing chain. One definition, used for mapping AND localization, so a map is always built
# from the same geometry it is later matched against.
#
#     bash bringup/sensing_chain.sh          # start/restart the whole chain
#     bash bringup/sensing_chain.sh --check  # report what is running, start nothing
#
# WHY THIS FILE EXISTS. On 2026-09-05 the floor-2 map was built while the chain was being edited
# underneath it: the map came from /scan_nav (a 1.30 m rear mask), then range_min moved 0.45 -> 0.60,
# then a cloud artifact filter was inserted that removes ~1150 points per cloud. Every one of those
# changes was individually justified. Together they meant the saved map contained walls the live
# sensor no longer produces, so localization sat at 63% and waypoints would not hold.
#
# Floor 1 worked for exactly one reason: its map and its live scan came from the same chain. That
# is the whole property this file protects. IF YOU CHANGE ANYTHING BELOW, THE MAPS BUILT BEFORE THE
# CHANGE ARE NO LONGER VALID -- rebuild them, or expect the fit to fall.
#
#   /ouster/points                     OS0 driver, 512x10
#        |  cloud_artifact_filter      drops range < 1.4 m AND reflectivity <= 1
#        |                             (near-field crosstalk: reflectivity pinned at the floor
#        |                              value of 1 on 15 of 128 downward rings. Not the mast --
#        |                              a real surface returns with variation.)
#        v
#   /ouster/points_clean
#        |  pointcloud_to_laserscan    height band 0.20-1.20 m, range_min 0.45
#        |                             (0.70 hid a real door at 0.72 m; 0.30 exposed the packed
#        |                              arm at 0.31-0.36 m; 0.45 clears both)
#        v
#   /scan_filtered                     BEST_EFFORT
#        |  scan_relay  x2             BEST_EFFORT -> RELIABLE, plus a rear self-return mask
#        |
#        +--> /scan       mask 0.90 m  ->  slam_toolbox   (needs far returns: a 2 m lift car puts
#        |                                                 its side walls ~1.0-1.15 m astern)
#        +--> /scan_nav   mask 1.30 m  ->  Nav2 costmaps   (must not see the robot astern or it can
#                                                           never reverse into anything)
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh" >/dev/null 2>&1 || { echo "env.sh failed"; exit 1; }
CHECK=0; [ "${1:-}" = "--check" ] && CHECK=1

kill_by_cmd() {  # verified full command line only -- never a loose pattern
  local pat="$1" pid a me; me="$(tr '\0' ' ' < /proc/$$/cmdline 2>/dev/null)"
  for pid in $(ps -eo pid --no-headers); do
    a="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)" || continue
    [ "$pid" = "$$" ] || [ "$pid" = "$BASHPID" ] && continue
    [ "$a" = "$me" ] && continue
    case "$a" in *"$pat"*) kill -INT "$pid" 2>/dev/null ;; esac
  done
}
rate() {
  python3 - "$1" "$2" "$3" <<'PY' 2>/dev/null || echo 0.00
import importlib, sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
topic, kind, rel = sys.argv[1], sys.argv[2], sys.argv[3]
mod, cls = ("sensor_msgs.msg", kind)
M = getattr(importlib.import_module(mod), cls)
q = (QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
     if rel == "reliable" else qos_profile_sensor_data)
rclpy.init(); n = Node("chain_probe"); c=[0]
n.create_subscription(M, topic, lambda m: c.__setitem__(0,c[0]+1), q)
t0=time.time()
while time.time()-t0 < 4: rclpy.spin_once(n, timeout_sec=0.05)
print(f"{c[0]/(time.time()-t0):.2f}")
PY
}

if [ "$CHECK" = 0 ]; then
  kill_by_cmd cloud_artifact_filter; kill_by_cmd pointcloud_to_laserscan; kill_by_cmd scan_relay.py
  sleep 4
  nohup setsid python3 "$REPO/safety/cloud_artifact_filter.py" >/tmp/utp_chain_filter.log 2>&1 </dev/null &
  sleep 6
  nohup setsid ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r cloud_in:=/ouster/points_clean -r scan:=/scan_filtered -p target_frame:=base_link \
    -p min_height:=0.20 -p max_height:=1.20 -p angle_min:=-3.14159 -p angle_max:=3.14159 \
    -p angle_increment:=0.0061 -p range_min:=0.45 -p range_max:=40.0 -p use_inf:=true \
    >/tmp/utp_chain_p2l.log 2>&1 </dev/null &
  sleep 8
  nohup setsid python3 "$REPO/bringup/scan_relay.py" >/tmp/utp_chain_relay_slam.log 2>&1 </dev/null &
  sleep 3
  UTP_SCAN_IN=/scan_filtered UTP_SCAN_OUT=/scan_nav UTP_MASK_MAX_M=1.30 \
    nohup setsid python3 "$REPO/bringup/scan_relay.py" >/tmp/utp_chain_relay_nav.log 2>&1 </dev/null &
  sleep 10
fi

echo
printf '  %-24s %8s\n' TOPIC Hz
for spec in "/ouster/points PointCloud2 sensor" "/ouster/points_clean PointCloud2 sensor" \
            "/scan_filtered LaserScan sensor" "/scan LaserScan reliable" "/scan_nav LaserScan reliable"; do
  set -- $spec
  printf '  %-24s %8s\n' "$1" "$(rate "$1" "$2" "$3")"
done
echo
echo "  slam_toolbox must read /scan;  Nav2 costmaps must read /scan_nav."
echo "  A map is only valid for the chain that built it. Change this file -> rebuild the maps."
