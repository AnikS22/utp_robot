#!/usr/bin/env bash
# ONE SESSION, START TO FINISH. Brings the stack up in the order that works, verifies each layer
# before the next, and hands off to the campaign.
#
#   bash bringup/session.sh up              # bring everything up, run the gates, then STOP
#   bash bringup/session.sh map             # ... plus a mapping drive (you pilot; Ctrl-C to save)
#   bash bringup/session.sh campaign 50     # ... then 50 trials of `ours`
#   bash bringup/session.sh down            # kill everything this script started
#
# WHAT NEEDS WHAT — read this once, it saves an afternoon.
#
#   THE LEG runs on the MAP via Nav2. The LAST METRE runs on odom. That split is deliberate and
#   is docs/NAV2.md's: "an AMCL correction mid-press would move the target under the arm." So
#   Nav2 plans across the saved map to the door; approach_blockage, the look ladder and the press
#   chain stay in odom where motion is smooth and continuous, and the visual servo closes the gap
#   (measured to 3 mm across four runs).
#
#   You do not choose between odometry and SLAM -- ROS composes them. map->odom (slam_toolbox)
#   over odom->base_link (wheel odometry) IS the fusion: odometry supplies smooth high-rate
#   motion, SLAM supplies the drift correction. Both edges already exist here.
#
#   FOR A 50-TRIAL SESSION, USE A SAVED MAP IN LOCALIZATION MODE (`session.sh nav`). Odom-frame
#   waypoints drift continuously AND die outright when ranger_base restarts; a fresh-SLAM map has
#   its origin wherever the robot booted, so its coordinates are not portable between sessions.
#   Only a saved, NAMED map makes a waypoint mean the same thing at trial 50 as at trial 1 --
#   safety/map_frame.py enforces exactly that distinction.
#
# EVERY STEP BELOW EXISTS BECAUSE SOMETHING SILENTLY FAILED. See EXPERIMENT_LOG.md.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)
source bringup/env.sh 2>/dev/null || { echo "cannot source bringup/env.sh"; exit 1; }

PIDS_FILE=/tmp/utp_session_pids
say()  { echo; echo "### $*"; }
die()  { echo "STOP: $*" >&2; exit 1; }
bg()   { setsid "$@" >/dev/null 2>&1 & echo $! >> "$PIDS_FILE"; }   # setsid: kill the GROUP later
alive(){ timeout 8 ros2 topic echo "$1" --once >/dev/null 2>&1; }
waitfor() { local t=$1 topic=$2; local i=0
            while [ $i -lt "$t" ]; do alive "$topic" && return 0; sleep 1; i=$((i+1)); done; return 1; }

cmd=${1:-up}; arg=${2:-}

# ------------------------------------------------------------------------------ down
if [ "$cmd" = "down" ]; then
  say "stopping everything this script started"
  if [ -f "$PIDS_FILE" ]; then
    while read -r p; do [ -n "$p" ] && kill -- -"$p" 2>/dev/null; done < "$PIDS_FILE"
    rm -f "$PIDS_FILE"
  fi
  # Scoped by full command line AND to this repo — never by topic or frame name. The 2026-08-18
  # incident killed 22 of the sim campaign's TF publishers because they shared a child frame.
  pkill -f "$ROOT/bringup" 2>/dev/null
  echo 'done. run: ros2 node list   — no utp_robot nodes should remain'
  exit 0
fi

: > "$PIDS_FILE"

# ------------------------------------------------------------------------------ 0. link
say "0/6  physical link"
IFACE=$(ip -brief link show | awk '/^enx/{print $1; exit}')
[ -n "$IFACE" ] || die "no enx* USB-ethernet interface"
ip -brief link show "$IFACE" | grep -q NO-CARRIER && \
  die "$IFACE NO-CARRIER. That one cable carries the lidar (.119), the xArm (.221) AND the router. Reseat, then strain-relieve it."
for a in 192.168.1.119 192.168.1.221 192.168.1.1; do
  ping -c1 -W1 "$a" >/dev/null 2>&1 || die "$a unreachable"
done
echo "  link ok ($IFACE)"

# ------------------------------------------------------------------------------ 1. chassis
say "1/6  chassis"
if ! ip link show can0 2>/dev/null | grep -q "state UP"; then
  echo "  bringing can0 up (needs your password)"
  sudo ip link set can0 up type can bitrate 500000 || die "can0 would not come up"
fi
if ! alive /odom; then
  # publish_odom_tf:=true is NOT the launch default and everything downstream needs odom->base_link.
  bg ros2 launch ranger_bringup ranger_mini_v3.launch.py publish_odom_tf:=true
  waitfor 25 /odom || die "no /odom after 25 s"
fi
echo "  /odom ok"
timeout 6 ros2 run tf2_ros tf2_echo odom base_link >/dev/null 2>&1 \
  || die "TF odom->base_link missing — was the launch given publish_odom_tf:=true ?"
echo "  TF odom->base_link ok"
echo "  NOTE: from here on, do NOT restart the ranger driver. It re-zeroes odom and every"
echo "        recorded waypoint silently becomes wrong."

# ------------------------------------------------------------------------------ 2. lidar
say "2/6  lidar + 2D scan chain"
alive /ouster/points || { bg bash bringup/lidar3d.sh; waitfor 40 /ouster/points \
  || die "no /ouster/points — check udp_dest: a sensor streaming to another host still reports RUNNING"; }
echo "  /ouster/points ok"
if ! alive /scan_filtered; then
  bg ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
     -r cloud_in:=/ouster/points -r scan:=/scan_filtered \
     -p target_frame:=base_link -p min_height:=0.20 -p max_height:=1.20 \
     -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.0061 \
     -p range_min:=0.50 -p range_max:=40.0 -p use_inf:=true
  waitfor 20 /scan_filtered || die "no /scan_filtered"
fi
echo "  /scan_filtered ok"
# The relay is NOT optional: p2l publishes BEST_EFFORT, slam_toolbox subscribes RELIABLE, and
# incompatible DDS QoS delivers zero messages with no error anywhere.
alive /scan || { bg python3 bringup/scan_relay.py; waitfor 20 /scan || die "no /scan — relay failed"; }
echo "  /scan ok (relay running)"

# ------------------------------------------------------------------------------ 3. safety
say "3/6  safety"
bg bash bringup/safety.sh; sleep 3
echo "  mux + arm gate started"
echo
echo "  >>> START THE DEADMAN IN ANOTHER TERMINAL AND HOLD IT:"
echo "  >>>     cd $ROOT && python3 bringup/deadman.py"
echo "  >>> Nothing autonomous moves without it. safety.yaml gates nav+servo on /safety/enable,"
echo "  >>> and a silent /safety/enable makes the robot look dead while behaving correctly."
echo
printf "  waiting for /safety/enable ... "
waitfor 180 /safety/enable && echo "ok" || die "/safety/enable never appeared"

# ------------------------------------------------------------------------------ 4. health
say "4/6  health + gates"
python3 bringup/health.py || die "health.py reported a critical failure"
bash bringup/lab_gates.sh 0 2 || die "gates 0-2 failed — fix before anything moves"

# ------------------------------------------------------------------------------ 5. map (optional)
if [ "$cmd" = "map" ]; then
  say "5/6  mapping drive"
  # slam_toolbox in Jazzy is a LIFECYCLE node: it starts `unconfigured` and looks exactly like a
  # hung node until it is configured AND activated.
  bg ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
     -p use_sim_time:=false -p base_frame:=base_link -p odom_frame:=odom -p map_frame:=map \
     -p scan_topic:=/scan -p resolution:=0.05
  sleep 5
  ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1
  ros2 lifecycle set /slam_toolbox activate  >/dev/null 2>&1
  STATE=$(ros2 lifecycle get /slam_toolbox 2>/dev/null | head -1)
  echo "  slam_toolbox: ${STATE:-unknown}  (must be active)"
  waitfor 30 /map || die "no /map — slam_toolbox not activated, or /scan not reaching it"
  echo "  /map ok."
  echo
  echo "  DRIVE THE FULL LOOP SLOWLY with the teleop -- and CLOSE IT, returning past where you"
  echo "  started. slam_toolbox only corrects accumulated drift when it recognises a place it has"
  echo "  already seen; an out-and-back gives you a map that is subtly bent, and every waypoint"
  echo "  inherits the bend. Watch it fill:  python3 bringup/map_watch.py"
  echo
  echo "  Then, WITHOUT stopping slam_toolbox:"
  echo "      bash bringup/map_persist.sh <name>      # grid + pose graph + .loaded_map"
  echo "      python3 bringup/waypoints.py record start  --frame map"
  echo "      python3 bringup/waypoints.py record door   --frame map"
  echo "      python3 bringup/waypoints.py record button --frame map"
  echo "  Waypoints MUST be recorded while a named map is loaded, or nav2_goto refuses them."
  exit 0
fi

# ------------------------------------------------------------------------------ 5b. nav (map)
MAP_NAME=${MAP_NAME:-atrium2d}
start_nav() {
  say "5/6  localization + Nav2 on the SAVED map '$MAP_NAME'"
  [ -f "maps/$MAP_NAME.yaml" ] || die "maps/$MAP_NAME.yaml not found. Make one: bash bringup/session.sh map"
  # A .pgm/.yaml pair is NOT a map you can relocalize into. slam_toolbox's `mode: localization`
  # deserializes map_file_name as <name>.posegraph + <name>.data; given only the grid it comes up
  # ACTIVE, publishes a /map, and silently starts a brand-new graph whose origin is wherever the
  # robot is standing -- i.e. exactly the fresh-SLAM frame safety/map_frame.py exists to refuse,
  # except now wearing the saved map's name. Every waypoint would be off by the startup offset.
  for ext in posegraph data; do
    [ -f "maps/$MAP_NAME.$ext" ] || die "maps/$MAP_NAME.$ext is missing, so '$MAP_NAME' cannot be
        relocalized into -- only drawn. Re-map and save with bringup/map_persist.sh, which writes
        the pose graph as well as the grid:
            bash bringup/session.sh map
            bash bringup/map_persist.sh $MAP_NAME"
  done
  if ! alive /map; then
    # LOCALIZATION mode, not mapping: 50 passes through the same corridor must not keep rewriting
    # the map underneath the waypoints. slam_toolbox owns BOTH /map and map->odom here, which is
    # why Nav2 is launched with localization:=slam and starts neither map_server nor AMCL --
    # exactly one source may own each.
    bg ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
       -p use_sim_time:=false -p base_frame:=base_link -p odom_frame:=odom -p map_frame:=map \
       -p scan_topic:=/scan -p resolution:=0.05 \
       -p map_file_name:="$ROOT/maps/$MAP_NAME" -p mode:=localization
    sleep 5
    ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1
    ros2 lifecycle set /slam_toolbox activate  >/dev/null 2>&1
    waitfor 40 /map || die "no /map — slam_toolbox localization did not activate"
  fi
  echo "  /map ok"
  timeout 10 ros2 run tf2_ros tf2_echo map odom >/dev/null 2>&1 \
    || die "TF map->odom missing — slam_toolbox has not localized into the map yet. Drive a few
        metres so it can match, then re-run."
  echo "  TF map->odom ok (localized)"
  # Record WHICH named map is live and in WHICH slam session. Without this every
  # `waypoints.py record --frame map` is stored nameless, and nav2_goto.py then refuses to drive
  # to it -- correctly, because a nameless recording is not portable. The session id is what stops
  # the file going stale after a slam_toolbox restart.
  SESS="$(python3 - "$ROOT" <<'PYEOF' | tail -1
import sys, time
sys.path.insert(0, sys.argv[1] + "/bringup")
import rclpy
from rclpy.node import Node
from pose_source import slam_session_id
rclpy.init(); n = Node("utp_session_slam_probe")
try:
    end = time.monotonic() + 3.0; sid = None
    while time.monotonic() < end and sid is None:
        rclpy.spin_once(n, timeout_sec=0.1); sid = slam_session_id(n)
    print(sid or "")
finally:
    n.destroy_node(); rclpy.shutdown()
PYEOF
)"
  [ -n "$SESS" ] || die "cannot identify the slam session (is exactly one node publishing /map?)"
  printf '%s %s\n' "$MAP_NAME" "$SESS" > "$ROOT/maps/.loaded_map"
  echo "  maps/.loaded_map -> $MAP_NAME [slam ${SESS:0:8}]"
  # THE BEHAVIOUR-TREE PATHS IN THE PARAMS ARE ABSOLUTE AND POINT AT THE SIM CHECKOUT.
  # nav2_params_os0_map.yaml carries
  #   default_nav_to_pose_bt_xml: "/home/<someone>/Desktop/Unlocking_the_path/nav2_bringup/..."
  # which is a path on the WORKSTATION. On the rover laptop it does not exist, bt_navigator fails
  # to load its tree, and Nav2 comes up looking healthy while navigate_to_pose never works --
  # exactly the silent half-failure docs/NAV2.md warns about. This repo ships its own copies of
  # both trees, so rewrite the two lines to point at them, wherever this repo happens to live.
  RUNTIME_PARAMS=/tmp/utp_nav2_params_runtime.yaml
  sed -E "s#(default_nav_to_pose_bt_xml:).*#\1 \"$ROOT/nav2_bringup/behavior_trees/navigate_to_pose_no_spin.xml\"#; \
          s#(default_nav_through_poses_bt_xml:).*#\1 \"$ROOT/nav2_bringup/behavior_trees/navigate_through_poses_no_spin.xml\"#" \
      "$ROOT/nav2_bringup/nav2_params_os0_map.yaml" > "$RUNTIME_PARAMS"
  for f in navigate_to_pose_no_spin navigate_through_poses_no_spin; do
    [ -f "$ROOT/nav2_bringup/behavior_trees/$f.xml" ] || die "missing behaviour tree $f.xml"
  done
  grep -q "$ROOT/nav2_bringup/behavior_trees" "$RUNTIME_PARAMS" \
    || die "behaviour-tree path rewrite failed — bt_navigator would load nothing"
  echo "  behaviour trees -> $ROOT/nav2_bringup/behavior_trees (rewritten from the sim path)"
  if ! ros2 node list 2>/dev/null | grep -q bt_navigator; then
    bg ros2 launch "$ROOT/nav2_bringup/ranger_nav.launch.py" \
       params_file:="$RUNTIME_PARAMS" localization:=slam
    sleep 8
  fi
  for i in $(seq 1 30); do
    ros2 action list 2>/dev/null | grep -q navigate_to_pose && break; sleep 1
  done
  ros2 action list 2>/dev/null | grep -q navigate_to_pose \
    || die "no navigate_to_pose action — Nav2 came up unconfigured (ros2 lifecycle get /bt_navigator)"
  echo "  Nav2 ok (navigate_to_pose available)"
}

if [ "$cmd" = "nav" ]; then start_nav; say "nav up. next: session.sh campaign 50"; exit 0; fi

# ------------------------------------------------------------------------------ 6. campaign
say "5/6  waypoints"
python3 bringup/waypoints.py list || true
echo
echo "  Record them NOW if they are not listed, in THIS driver session:"
echo "      python3 bringup/waypoints.py record start     # records in the MAP frame once localized"
echo "      python3 bringup/waypoints.py record door"
echo "  Recorded while localized in a NAMED map, these survive a ranger_base restart and 50 trials."
echo "  A pre-recorded BUTTON waypoint is a human pointing at the control. Use it to debug, but"
echo "  a scored trial that relies on it is not measuring grounding — see approach_blockage()."

if [ "$cmd" = "campaign" ]; then
  N=${arg:-50}
  start_nav                      # the legs plan over the saved map; the press chain stays on odom
  say "6/6  campaign: $N trials"
  echo "  dry run first (nothing moves) ..."
  python3 bringup/run_campaign.py --trials 2 --method ours --start start --dry-run \
    || die "dry run failed — do not start a live campaign"
  echo
  echo "  live campaign starting. HAND ON THE RC. Ctrl-C stops cleanly after the current trial."
  sleep 3
  python3 bringup/run_campaign.py --trials "$N" --method ours --start start
  exit $?
fi

say "up. next:"
echo "  bash bringup/lab_gates.sh 3      # chassis characterisation — MANUAL, the robot moves"
echo "  bash bringup/session.sh map      # mapping drive — do this ONCE, then save"
echo "  bash bringup/session.sh nav      # localization + Nav2 on the saved map"
echo "  bash bringup/session.sh campaign 50"
echo "  bash bringup/session.sh down     # stop everything this script started"
