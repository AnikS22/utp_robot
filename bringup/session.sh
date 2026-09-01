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
#   The PIPELINE does NOT use SLAM or Nav2. RosWorld.navigate_to_goal drives with
#   `waypoints.py goto --go`, i.e. ODOMETRY WAYPOINTS, and the visual servo closes the last metre
#   (measured to 3 mm across four runs). This is deliberate: route_run.py records that
#   slam_toolbox could not hold a pose in this building, so accuracy was moved to the one place it
#   had been measured. A map is therefore OPTIONAL for running trials.
#
#   Keep the map for what it is genuinely good for: seeing where the robot is while you watch,
#   and the OS0 finally makes it viable (977 valid beams vs the A1M8's 44). But do NOT block a
#   trial session on getting SLAM perfect. If you want the pipeline to plan with Nav2 instead of
#   waypoints, that is a change to navigate_to_goal and a revalidation — not a bring-up flag, and
#   not something to attempt the morning of.
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
  echo "  /map ok. Drive the loop slowly with the teleop, then:"
  echo "      bash bringup/map_persist.sh <name>"
  echo "  Watch it fill: python3 bringup/map_watch.py"
  exit 0
fi

# ------------------------------------------------------------------------------ 6. campaign
say "5/6  waypoints"
python3 bringup/waypoints.py list || true
echo
echo "  Record them NOW if they are not listed, in THIS driver session:"
echo "      python3 bringup/waypoints.py record start"
echo "      python3 bringup/waypoints.py record door"
echo "  A pre-recorded BUTTON waypoint is a human pointing at the control. Use it to debug, but"
echo "  a scored trial that relies on it is not measuring grounding — see approach_blockage()."

if [ "$cmd" = "campaign" ]; then
  N=${arg:-50}
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
echo "  bash bringup/session.sh map      # mapping drive (optional; the pipeline does not need it)"
echo "  bash bringup/session.sh campaign 50"
echo "  bash bringup/session.sh down     # stop everything this script started"
