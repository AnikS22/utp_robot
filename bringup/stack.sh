#!/usr/bin/env bash
# Bring the whole stack up, check every piece by RATE, restart what is wedged, print a table.
#
#     bash bringup/stack.sh              # bring up everything, nav included
#     bash bringup/stack.sh --no-nav     # sensors + safety + slam only
#     bash bringup/stack.sh --status     # check only, start nothing
#     MAP_NAME=elevator bash bringup/stack.sh
#
# WHY THIS EXISTS, given session.sh already claims to do it.
#
# 1. session.sh DIES ON THE FIRST FAILED GATE. One stale component and you get a single error
#    line, fix it, re-run, and hit the next one. On 2026-09-04 that turned bring-up into a
#    twenty-minute serial hunt. This starts everything it can, then reports the whole picture at
#    once, so one pass tells you all of what is wrong.
#
# 2. session.sh checks whether a NODE EXISTS. That is the wrong question, and it hid both of
#    today's faults:
#      * the Ouster driver was running and publishing NOTHING -- a stale process holding the UDP
#        socket after a power cycle. `ros2 node list` showed it; /ouster/points was 0.00 Hz.
#      * the chassis driver was absent entirely, so there was no /odom, so slam_toolbox could not
#        publish map->odom, so localization was "wrong" in RViz. The visible symptom was three
#        layers away from the cause.
#    Every probe here measures a RATE with a counting subscriber. `ros2 topic hz` is not used: it
#    is unreliable on this stack and has reported 1.7 and 10.0 Hz for the same topic minutes apart.
#
# 3. A wedged component is RESTARTED, not just reported. If a probe fails and the process exists,
#    it is killed by verified PID (never a loose pattern -- that has killed this shell twice) and
#    started again.
#
# It is idempotent: healthy components are left alone, so re-running costs a few seconds of probes.
set -uo pipefail            # NOT -e: this script's whole job is to continue past a failure

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh" >/dev/null 2>&1 || { echo "env.sh failed"; exit 1; }
MAP_NAME="${MAP_NAME:-elevator}"
WANT_NAV=1; STATUS_ONLY=0
for a in "$@"; do
  case "$a" in
    --no-nav) WANT_NAV=0 ;;
    --status) STATUS_ONLY=1 ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
  esac
done

RESULT=()          # "name|state|detail"
WHY=()             # "name|why it failed and what to do" -- only populated on FAIL/WARN
# Every stored field is folded to ONE LINE. The report reads rows back with
# `IFS='|' read -r n d`, and `read` stops at the FIRST NEWLINE, so a multi-line reason came out
# truncated to its first line -- worse than no reason at all, because it reads as a complete
# sentence that is missing its point.
#
# `sed 's/[[:space:]]\+/ /g'` did NOT fold it: sed works one line at a time and the newline is
# never in its pattern space, so that collapsed the indentation inside each continuation line and
# left every newline exactly where it was. `tr` is not line-oriented, so it does. The reason is
# re-wrapped for the terminal at print time with `fold -s -w 88`.
fold1() { printf '%s' "$1" | tr -s '[:space:]' ' ' | sed 's/^ *//; s/ *$//'; }
why() { WHY+=("$1|$(fold1 "$2")"); }
note() { printf '  %s\n' "$*"; }
record() { RESULT+=("$1|$2|$(fold1 "$3")"); }

# ---------------------------------------------------------------- probes
# Measure a topic's rate with a real subscriber. Returns the rate on stdout, "0.00" if silent.
rate() {  # rate <topic> <msgtype-module> <msgtype-class> <seconds> <reliable|sensor>
  local out lim
  # Hard ceiling: rclpy.init() against a sick DDS can block forever, and a bring-up script that
  # hangs on a probe is indistinguishable from a bring-up script that is working.
  lim=$(LC_ALL=C awk -v s="${4:-3}" 'BEGIN{printf "%d", s+25}')
  out="$(timeout "$lim" python3 - "$1" "$2" "$3" "$4" "$5" <<'PY' 2>/dev/null
import importlib, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
topic, mod, cls, secs, rel = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), sys.argv[5]
M = getattr(importlib.import_module(mod), cls)
qos = (QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
       if rel == "reliable" else qos_profile_sensor_data)
rclpy.init(); n = Node("utp_rate_probe"); c = [0]
n.create_subscription(M, topic, lambda m: c.__setitem__(0, c[0] + 1), qos)
t0 = time.time()
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.05)
el = time.time() - t0
print(f"{c[0]/el:.2f}")
n.destroy_node()
try: rclpy.shutdown()
except Exception: pass
PY
)"
  # Whatever comes back, hand the caller a bare number. Two ways this used to leak:
  #   * the probe prints its rate and THEN exits non-zero (a throw out of destroy_node), so the
  #     old `|| echo "0.00"` appended a SECOND line -- and a newline inside $r lands in a RESULT
  #     row, where `read -r n s d` truncates the table. Same defect as the WHY block above.
  #   * timeout kills it mid-write and $r is empty, which `ge` reads as 0 but which prints as
  #     "/ouster/points  Hz" -- a blank where the number should be.
  # First line only; digits and dots only; never empty.
  out="${out%%$'\n'*}"
  case "$out" in
    ''|*[!0-9.]*) printf '0.00\n' ;;
    *)            printf '%s\n' "$out" ;;
  esac
}

tf_ok() {  # tf_ok <parent> <child>   -- grep for a Translation line; tf2_echo never exits
  local out
  out="$(timeout 8 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 | head -20)"
  printf '%s\n' "$out" | grep -q 'Translation:'
}

# Numeric compare that must not fail on an empty or malformed rate: "" and "0.00" both read as 0
# and the comparison simply returns false. LC_ALL=C because the default awk here is mawk, whose
# string->number conversion goes through strtod and IS locale-sensitive: under a comma-decimal
# locale "1.5" would convert to 1 and every threshold in this script would silently be wrong.
# ros2 lifecycle get prints "active [3]" / "inactive [2]" / nothing if the node is not there.
# MATCH THE WHOLE FIELD, NEVER A SUBSTRING: "inactive" CONTAINS "active", so a `grep -q active`
# here would report a dead Nav2 as healthy -- this repo's signature bug, in one word.
nav_state() {  # nav_state <node> -> active | inactive | unconfigured | absent
  local s
  s="$(timeout 8 ros2 lifecycle get "$1" 2>/dev/null | head -1 | awk '{print $1}')"
  printf '%s\n' "${s:-absent}"
}

ge() { LC_ALL=C awk -v a="${1:-0}" -v b="${2:-0}" 'BEGIN{exit !(a+0 >= b+0)}'; }

# Kill by VERIFIED full command line, never a loose pattern. A `pkill -f` here matches this
# script's own bash -c and has killed the calling shell twice.
#
# The `$$` guard alone was not enough, for three reasons:
#   * `$$` is the ORIGINAL shell's pid and does not change inside a subshell, while a forked
#     subshell shares its parent's /proc/<pid>/cmdline -- so it protected this script but not a
#     subshell of it. `$BASHPID` is the one that moves.
#   * it protected only this process, not the shell that INVOKED it. A `bash -c` wrapper whose
#     command line happens to contain the pattern was still fair game -- which is exactly the
#     "killed the calling shell" failure. Skip the whole ancestor chain.
#   * an empty pattern made `case "$a" in *""*)` match EVERY process on the machine. Refuse it.
#   * and the pid list is walked inside `$(...)` and pipelines, which fork subshells that are
#     NEITHER $$ NOR $BASHPID NOR an ancestor -- so pid-identity alone can never be complete.
#     Every bash subshell shares its parent's /proc/<pid>/cmdline, so skipping anything whose
#     command line is byte-identical to our own covers all of them at any nesting depth.
# SIGINT is also not by itself a restart: a wedged Ouster driver that ignores INT keeps the UDP
# socket, and the fresh driver then comes up publishing 0.00 Hz -- the exact fault this script
# exists to clear. Wait for the victims to actually go, then SIGKILL what is left.
pids_matching() {
  local pat="${1:-}" pid a p self skip=" $$ $BASHPID "
  [ -n "$pat" ] || { note "pids_matching: empty pattern refused (it would match every process)"; return 1; }
  self="$( { tr '\0' ' ' < "/proc/$$/cmdline"; } 2>/dev/null )"
  p="$PPID"
  while [ -n "${p:-}" ] && [ "$p" -gt 1 ] 2>/dev/null; do
    skip="$skip$p "
    p="$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')"
  done
  for pid in $(ps -eo pid --no-headers); do
    case "$skip" in *" $pid "*) continue ;; esac
    # The 2>/dev/null must wrap the REDIRECTION, not just tr: a pid that exits between `ps`
    # and this read makes the shell itself print "No such file or directory", and that happens on
    # essentially every run.
    a="$( { tr '\0' ' ' < "/proc/$pid/cmdline"; } 2>/dev/null )" || continue
    [ -n "$a" ] || continue                    # kernel threads and zombies have no command line
    [ "$a" = "$self" ] && continue             # a subshell of this script, at any nesting depth
    case "$a" in *"$pat"*) printf '%s\n' "$pid" ;; esac
  done
}

# HOW MANY copies are running. Ask this BEFORE starting anything: launching a second copy of a
# component is a failure mode, not a harmless retry, and it has bitten this project three times in
# one day -- two Nav2 stacks (neither activated, every goal "rejected"), two RealSense drivers
# racing for the USB device (the loser logs "No RealSense devices were found"), and two
# waypoint_markers publishers on one topic.
count_matching() { pids_matching "$1" | wc -l; }

kill_matching() {
  local pid a victims=() alive i
  mapfile -t victims < <(pids_matching "${1:-}")
  [ "${#victims[@]}" -gt 0 ] || return 0
  kill -INT "${victims[@]}" 2>/dev/null
  for i in 1 2 3 4 5; do
    sleep 1
    alive=0
    for pid in "${victims[@]}"; do
      a="$( { tr -d '\0' < "/proc/$pid/cmdline"; } 2>/dev/null )" || continue
      [ -n "$a" ] && alive=1                   # empty cmdline == gone or reaped
    done
    [ "$alive" = 0 ] && return 0
  done
  for pid in "${victims[@]}"; do kill -KILL "$pid" 2>/dev/null; done
  return 0
}

start_bg() { nohup setsid "$@" >>"/tmp/utp_stack.log" 2>&1 < /dev/null & sleep 1; }

echo
echo "=== utp stack   map=$MAP_NAME   $( [ "$WANT_NAV" = 1 ] && echo 'with nav' || echo 'no nav' )"
# Only truncate the log when this run might actually write to it. In --status the log is the
# record of whatever brought the stack up, and emptying it destroys the evidence a check-only run
# exists to collect.
[ "$STATUS_ONLY" = 1 ] || : > /tmp/utp_stack.log

# ---------------------------------------------------------------- 0. links that need a human
if /sbin/ip link show can0 >/dev/null 2>&1; then
  record can0 ok "$(/sbin/ip -details link show can0 | grep -oE 'state [A-Z-]+' | head -1)"
else
  record can0 FAIL "missing -- sudo ip link set can0 up type can bitrate 500000"
fi
IFACE=$(/sbin/ip -brief link show 2>/dev/null | awk '/^enx/{print $1; exit}')
if [ -n "$IFACE" ]; then record ethernet ok "$IFACE"; else record ethernet FAIL "no enx* interface"; fi

# ---------------------------------------------------------------- 1. chassis  -> /odom, odom TF
r=$(rate /odom nav_msgs.msg Odometry 3 sensor)
if ! ge "$r" 5; then
  [ "$STATUS_ONLY" = 1 ] || { note "chassis silent ($r Hz) -- restarting"
    kill_matching ranger_mini_v3.launch; kill_matching ranger_base_node; sleep 3
    start_bg ros2 launch ranger_bringup ranger_mini_v3.launch.py publish_odom_tf:=true
    sleep 18; r=$(rate /odom nav_msgs.msg Odometry 3 sensor); }
fi
if ge "$r" 5 && tf_ok odom base_link; then record chassis ok "/odom ${r} Hz, odom->base_link"
else
  record chassis FAIL "/odom ${r} Hz (want >=5) or odom->base_link missing"
  if ! /sbin/ip link show can0 >/dev/null 2>&1; then
    why chassis "can0 does not exist, so the driver has nothing to talk to. The USB-CAN adapter is
               not enumerated. Plug it in, then: sudo ip link set can0 up type can bitrate 500000"
  else
    _cm="$(timeout 25 python3 "$REPO/bringup/chassis_mode.py" 2>&1 | grep -oE 'control_mode=[A-Z]+' | head -1)"
    case "$_cm" in
      *RC*) why chassis "the chassis answers but control_mode=RC: the TRANSMITTER holds authority and
               every computer command is discarded silently. Flip SWB UP, then python3 bringup/claim_can.py" ;;
      *)    why chassis "can0 exists and the chassis is ${_cm:-not answering}. If it is not answering the
               driver process is dead or wedged -- see /tmp/utp_stack.log. /odom at full rate with
               ALL-ZERO velocity is a different fault: that means the CAN link is down, not the driver." ;;
    esac
  fi
fi

# ---------------------------------------------------------------- 2. lidar -> cloud -> scan
r=$(rate /ouster/points sensor_msgs.msg PointCloud2 3 sensor)
if ! ge "$r" 1.5; then
  [ "$STATUS_ONLY" = 1 ] || { note "lidar silent ($r Hz) -- restarting (a stale driver holds the socket and publishes nothing)"
    kill_matching os_driver; kill_matching lidar3d.sh; sleep 4
    start_bg bash "$REPO/bringup/lidar3d.sh"; sleep 40
    r=$(rate /ouster/points sensor_msgs.msg PointCloud2 3 sensor); }
fi
if ge "$r" 1.5; then record lidar ok "/ouster/points ${r} Hz"
else
  record lidar FAIL "/ouster/points ${r} Hz"
  if ping -c1 -W2 192.168.1.119 >/dev/null 2>&1; then
    why lidar "the sensor ANSWERS on 192.168.1.119 but no cloud arrives. That is a stale/wedged
               driver holding the UDP socket -- it survives a robot power cycle and 'ros2 node list'
               still shows it. This script already tried a restart; if it persists check
               /tmp/utp_stack.log for 'poll_client timed out' or 'Couldn't get active config'."
  else
    why lidar "192.168.1.119 does not answer. That is the NETWORK, not the driver: one USB-ethernet
               cable carries the lidar, the xArm and the router. Check it is seated and that
               'ip -brief link show' does not say NO-CARRIER."
  fi
fi

r=$(rate /scan_filtered sensor_msgs.msg LaserScan 3 sensor)
if ! ge "$r" 1.5; then
  [ "$STATUS_ONLY" = 1 ] || { note "projection silent -- restarting pointcloud_to_laserscan"
    kill_matching pointcloud_to_laserscan; sleep 2
    start_bg ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
      -r cloud_in:=/ouster/points -r scan:=/scan_filtered -p target_frame:=base_link \
      -p min_height:=0.20 -p max_height:=1.20 -p angle_min:=-3.14159 -p angle_max:=3.14159 \
      -p angle_increment:=0.0061 -p range_min:=0.45 -p range_max:=40.0 -p use_inf:=true
    sleep 8; r=$(rate /scan_filtered sensor_msgs.msg LaserScan 3 sensor); }
fi
ge "$r" 1.5 && record projection ok "/scan_filtered ${r} Hz" || record projection FAIL "/scan_filtered ${r} Hz"

# The relay is NOT optional: p2l publishes BEST_EFFORT, slam_toolbox subscribes RELIABLE, and
# incompatible QoS delivers zero messages with no error anywhere.
r=$(rate /scan sensor_msgs.msg LaserScan 3 reliable)
if ! ge "$r" 1.5; then
  [ "$STATUS_ONLY" = 1 ] || { note "/scan silent -- restarting scan_relay"
    kill_matching scan_relay.py; sleep 2
    start_bg python3 "$REPO/bringup/scan_relay.py"; sleep 8
    r=$(rate /scan sensor_msgs.msg LaserScan 3 reliable); }
fi
if ge "$r" 1.5; then
  ge "$r" 6 && record scan ok "/scan ${r} Hz" \
             || { record scan WARN "/scan ${r} Hz -- below 6"
                  why scan "slam_toolbox searches with coarse_angle_resolution 2.0 deg. At ${r} Hz and
                            wz_max 0.8 rad/s (46 deg/s) consecutive scans are far more than 2 deg
                            apart, so the pose SLIDES during a turn and the controller drives against
                            a stale estimate. On 2026-09-03 that put the robot 1.85 m from where Nav2
                            said it had arrived, and into a wall. The 3.1 MB point cloud is the
                            bottleneck (~73% lost in DDS); the driver's native /ouster/scan runs at
                            9.9 Hz but sees a single ring, which does not match a height-band map."; }
else
  record scan FAIL "/scan ${r} Hz"
  why scan "the relay is not delivering. It converts BEST_EFFORT -> RELIABLE, and without it
            slam_toolbox (which subscribes RELIABLE) receives ZERO messages with no error anywhere.
            Check /scan_filtered above: if that is healthy the relay is the problem, if it is also
            down the fault is upstream in the lidar or the projection."
fi

# ---------------------------------------------------------------- 3. camera (optional)
if [ "${UTP_NO_CAMERA:-0}" = "1" ]; then
  record camera skip "UTP_NO_CAMERA=1 -- grounding and pressing will NOT work"
else
  r=$(rate /mast_cam/color/camera_info sensor_msgs.msg CameraInfo 3 sensor)
  if ! ge "$r" 10; then
    [ "$STATUS_ONLY" = 1 ] || { note "camera silent ($r Hz) -- restarting"
      kill_matching realsense2_camera_node; kill_matching camera.sh; sleep 3
      start_bg bash "$REPO/bringup/camera.sh"; sleep 25
      r=$(rate /mast_cam/color/camera_info sensor_msgs.msg CameraInfo 3 sensor); }
  fi
  ge "$r" 10 && record camera ok "camera_info ${r} Hz" \
              || { record camera FAIL "camera_info ${r} Hz"
                   _sp=""; for _d in /sys/bus/usb/devices/*/idVendor; do
                     [ "$(cat "$_d" 2>/dev/null)" = "8086" ] || continue
                     _pp=$(dirname "$_d")
                     case "$(cat "$_pp/product" 2>/dev/null)" in *RealSense*) _sp="$(cat "$_pp/speed" 2>/dev/null)";; esac
                   done
                   if [ -z "$_sp" ]; then
                     why camera "the D435 is NOT ENUMERATED on USB at all. Reseat it."
                   elif [ "$_sp" = "480" ]; then
                     why camera "the D435 negotiated 480 Mbps -- USB 2. config/camera.yaml asks for
                                 1280x720x30 colour + 848x480x30 depth, which USB 2 physically cannot
                                 carry, so librealsense opens NOTHING and loops on
                                 xioctl(VIDIOC_S_FMT) errno=5. Restarting the driver will not help.
                                 Re-plug until it comes up at 5000, or lower the profile."
                   else
                     why camera "link is ${_sp} Mbps so bandwidth is fine -- the driver is wedged, or a
                                 second instance is fighting for the device. Check for more than one
                                 mast_cam node."
                   fi; }
fi

# ---------------------------------------------------------------- 4. safety mux
r=$(rate /safety/status std_msgs.msg String 3 sensor)
if ! ge "$r" 5; then
  [ "$STATUS_ONLY" = 1 ] || { note "safety mux silent -- restarting"
    kill_matching twist_mux_node.py; kill_matching arm_monitor_node.py; sleep 3
    start_bg bash "$REPO/bringup/safety.sh"; sleep 10
    r=$(rate /safety/status std_msgs.msg String 3 sensor); }
fi
ge "$r" 5 && record safety ok "/safety/status ${r} Hz" || record safety FAIL "/safety/status ${r} Hz"

# ---------------------------------------------------------------- 5. slam localization
if [ "$WANT_NAV" = 1 ]; then
  # One row per COMPONENT, not one per missing file: three missing extensions used to emit three
  # rows all named "slam" and count three times towards `bad`, so the summary line lied about how
  # many things were down.
  _miss=""
  for f in yaml posegraph data; do
    [ -f "$REPO/maps/$MAP_NAME.$f" ] || _miss="$_miss .$f"
  done
  [ -z "$_miss" ] || { record slam FAIL "maps/$MAP_NAME missing$_miss -- cannot localize"; MAPBAD=1; }
  if [ -z "${MAPBAD:-}" ]; then
    if ! timeout 8 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once >/dev/null 2>&1; then
      [ "$STATUS_ONLY" = 1 ] || { note "no /map -- starting slam_toolbox localization on '$MAP_NAME'"
        kill_matching slam_toolbox; sleep 4
        start_bg ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
          --params-file "$REPO/config/slam_os0.yaml" -p use_sim_time:=false -p mode:=localization \
          -p map_file_name:="$REPO/maps/$MAP_NAME"
        sleep 20
        # It is a LIFECYCLE node: it comes up unconfigured and publishes nothing until told.
        timeout 20 ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1
        sleep 3
        timeout 90 ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1
        sleep 8; }
    fi
    if timeout 8 ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once >/dev/null 2>&1; then
      if tf_ok map odom; then record slam ok "map '$MAP_NAME' loaded, map->odom present"
      else
        record slam WARN "map loaded but NO map->odom"
        if tf_ok odom base_link; then
          why slam "the map is loaded and odom->base_link is fine, so slam_toolbox simply has no pose
                    yet: config/slam_os0.yaml map_start_pose is an ATRIUM coordinate and seeding from
                    it would converge into the wrong corridor. Set it by hand -- RViz 2D Pose Estimate
                    (works only in LOCALIZATION mode, silently ignored while mapping), or run
                    python3 bringup/relocalise.py for a global search."
        else
          why slam "there is no odom->base_link, so slam CANNOT publish map->odom no matter how good
                    the scan is. Fix the chassis first -- this failure is three layers from its cause
                    and presents as 'localization is wrong' in RViz."
        fi
      fi
    else record slam FAIL "no /map after starting localization"; fi
  fi

  # ------------------------------------------------------------- 6. nav2
  # Guard on the CAPABILITY, not the node name: an unconfigured bt_navigator still appears in
  # `ros2 node list` while navigate_to_pose never arrives.
  #
  # AND THE ACTION IS NOT ENOUGH EITHER -- measured 2026-09-04, and this is the most deceptive
  # failure in the stack, because it is silent in three different ways at once. Every goal came
  # back "rejected in 0.0s" while:
  #   * `ros2 node list` showed bt_navigator, planner_server, controller_server, behavior_server
  #     -- a perfectly healthy-looking Nav2;
  #   * `ros2 action list` DID show /navigate_to_pose, because the action server is ADVERTISED
  #     BEFORE the node is activated. Guarding on the action is better than guarding on the node
  #     name and is still not sufficient;
  #   * RViz showed an empty world, because inactive costmap nodes publish nothing -- which reads
  #     as an RViz configuration problem and is not one.
  # `ros2 lifecycle get /bt_navigator` -> "inactive [2]" was the only check that saw it. The cause
  # was TWO Nav2 stacks, from repeated `ros2 launch` calls during debugging, NEITHER activated:
  # two lifecycle_manager instances contend for the same nodes and the transition never completes.
  # So this section refuses to stack a second copy, and requires ACTIVE, not merely present.
  nav_probe() {   # sets _act _bt _pl _ct _dups
    _act=$(timeout 10 ros2 action list 2>/dev/null | grep -c navigate_to_pose)
    _bt=$(nav_state /bt_navigator); _pl=$(nav_state /planner_server); _ct=$(nav_state /controller_server)
    _dups=$(count_matching bt_navigator)
    _pdups=$(count_matching planner_server)
    [ "$_pdups" -gt "$_dups" ] && _dups="$_pdups"
    return 0
  }
  nav_healthy() {
    [ "${_act:-0}" -ge 1 ] && [ "${_dups:-0}" -le 1 ] \
      && [ "$_bt" = active ] && [ "$_pl" = active ] && [ "$_ct" = active ]
  }

  nav_probe
  if ! nav_healthy; then
    [ "$STATUS_ONLY" = 1 ] || {
      [ "$_dups" -le 1 ] || note "$_dups Nav2 stacks are running and none is usable -- tearing down ALL of them"
      note "Nav2 not active (action=$_act bt=$_bt planner=$_pl controller=$_ct procs=$_dups) -- launching one"
      # Tear down the NODES, not just the launch wrapper. Killing `ros2 launch` alone can orphan
      # the servers it started, and the orphans are exactly what the next launch stacks on top of.
      for _p in ranger_nav.launch bt_navigator planner_server controller_server behavior_server \
                smoother_server velocity_smoother waypoint_follower lifecycle_manager; do
        kill_matching "$_p"
      done
      sleep 4
      RUNTIME=/tmp/utp_nav2_params_runtime.yaml
      sed -E "s#(default_nav_to_pose_bt_xml:).*#\1 \"$REPO/nav2_bringup/behavior_trees/navigate_to_pose_no_spin.xml\"#; \
              s#(default_nav_through_poses_bt_xml:).*#\1 \"$REPO/nav2_bringup/behavior_trees/navigate_through_poses_no_spin.xml\"#" \
          "$REPO/nav2_bringup/nav2_params_os0_map.yaml" > "$RUNTIME"
      start_bg ros2 launch "$REPO/nav2_bringup/ranger_nav.launch.py" params_file:="$RUNTIME" localization:=slam
      sleep 45
      nav_probe; }
  fi
  if nav_healthy; then
    record nav2 ok "navigate_to_pose + bt_navigator/planner/controller ACTIVE"
  else
    record nav2 FAIL "action=$_act bt_navigator=$_bt planner=$_pl controller=$_ct procs=$_dups"
    if [ "$_dups" -gt 1 ]; then
      why nav2 "there are $_dups bt_navigator/planner_server processes: TWO Nav2 STACKS are running,
                from repeated 'ros2 launch' calls. Two lifecycle_manager instances contend for the
                same nodes and the activation NEVER COMPLETES, so every goal comes back 'rejected
                in 0.0s'. Kill both stacks and start exactly one. Launching a second copy of a
                component is a failure mode, not a harmless retry -- the same shape gives two
                RealSense drivers racing for the USB device (the loser logs 'No RealSense devices
                were found') and two publishers on one marker topic."
    elif [ "$_act" -ge 1 ]; then
      why nav2 "Nav2's nodes are present and /navigate_to_pose IS advertised, but the nodes are
                INACTIVE (bt_navigator=$_bt planner_server=$_pl controller_server=$_ct; check
                behavior_server too). The action server is advertised BEFORE activation, so the
                action existing proves nothing. 'ros2 lifecycle get /bt_navigator' is the only
                check that sees this: node list shows a healthy-looking Nav2, action list shows the
                action, and RViz shows an empty world because inactive costmap nodes publish
                nothing -- which reads as an RViz problem and is not one. The goal status is the
                other tell: REJECTED means the server would not accept it at all (usually lifecycle
                or config), ABORTED means it tried and failed, and 'blocked' in nav2_goto.py means
                Nav2 STATUS_ABORTED specifically. Check /tmp/utp_stack.log for a transition failure,
                and confirm map->odom exists: without it the costmaps never come up and
                bt_navigator stays down."
    else
      why nav2 "Nav2 launched but /navigate_to_pose never appeared. Its nodes are LIFECYCLE nodes
                and can sit unconfigured while still showing in 'ros2 node list' -- which is why
                this script guards on the ACTION and the lifecycle STATE, not the node name. Check
                /tmp/utp_stack.log for a transition failure, and confirm map->odom exists: without
                it the costmaps never come up and bt_navigator stays down."
    fi
  fi
fi

# ---------------------------------------------------------------- report
echo
printf '  %-12s %-6s %s\n' COMPONENT STATE DETAIL
printf '  %-12s %-6s %s\n' "------------" "-----" "--------------------------------------------"
bad=0; warn=0
for row in "${RESULT[@]}"; do
  IFS='|' read -r n s d <<<"$row"
  printf '  %-12s %-6s %s\n' "$n" "$s" "$d"
  [ "$s" = "FAIL" ] && bad=$((bad+1))
  [ "$s" = "WARN" ] && warn=$((warn+1))
done
echo
if [ ${#WHY[@]} -gt 0 ]; then
  echo "  WHY:"
  for row in "${WHY[@]}"; do
    IFS='|' read -r n d <<<"$row"
    printf '    %s:\n' "$n"
    printf '%s\n' "$d" | fold -s -w 88 | sed 's/^/      /'
    echo
  done
fi
if [ "$bad" -gt 0 ]; then
  echo "  $bad component(s) DOWN. Fix those before driving."
  exit 1
fi
[ "$warn" -gt 0 ] && echo "  $warn warning(s) -- usable, but read them."
echo "  stack up. Next: check the pose (bringup/relocalise.py --check, want >=80%)."
exit 0
