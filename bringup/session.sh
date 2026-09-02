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

# THE USB PHYSICAL LAYER. Added 2026-09-01 after an evening lost to it.
#
# Both of these dropped off the bus within one hour and EVERY downstream symptom looked like a
# software fault: Nav2 "aborted" a goal it had planned perfectly 25 times over ten minutes, the
# camera published 0.0 Hz while its node sat there looking healthy, and the gate ladder failed on
# checks that were fine. ranger_base_node logged 21,766 consecutive "Failed to send CAN frame" and
# then died with "Resource deadlock avoided", while /odom kept streaming at 45.8 Hz with every
# velocity sample identically zero. Hours went into the planner. The planner was never wrong.
#
# These two checks cost 20 ms and are the difference between "reseat the adapter" and a night.
if ! ip link show can0 >/dev/null 2>&1; then
  die "can0 DOES NOT EXIST -- the USB-CAN adapter is not enumerated.
        The chassis cannot receive a single command. Nav2 will plan perfectly and the robot will
        not move, /odom will stream at full rate with all-zero velocity, and every failure will
        present as a navigation or costmap problem. It is none of those. Plug the adapter in:
          lsusb
          sudo ip link set can0 up type can bitrate 500000
          python3 bringup/claim_can.py"
fi

# The D435 must be on USB 3. config/camera.yaml asks for 1280x720x30 colour + 848x480x30 depth;
# a USB 2 link (480 Mbps) cannot carry that, so librealsense opens NOTHING, camera_info reads
# 0.0 Hz, and the driver loops forever on xioctl(VIDIOC_S_FMT) errno=5. Restarting camera.sh does
# not help, and health.py used to advise exactly that.
_cam_speed=""
for _d in /sys/bus/usb/devices/*/idVendor; do
  [ "$(cat "$_d" 2>/dev/null)" = "8086" ] || continue
  _p="$(dirname "$_d")"
  case "$(cat "$_p/product" 2>/dev/null)" in *RealSense*) _cam_speed="$(cat "$_p/speed" 2>/dev/null)";; esac
done
if [ -z "$_cam_speed" ]; then
  die "the RealSense D435 is NOT ENUMERATED on USB. Reseat it in a blue USB 3 port."
elif [ "$_cam_speed" != "5000" ] && [ "$_cam_speed" != "10000" ]; then
  die "the RealSense D435 negotiated ${_cam_speed} Mbps -- that is USB 2, not USB 3.
        It cannot open the profiles in config/camera.yaml and will publish nothing at all while
        looking perfectly alive in ros2 node list. Move it to a blue USB 3 port."
fi
echo "  usb ok (can0 present, D435 at ${_cam_speed} Mbps)"

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
# Grep for the transform; do NOT test tf2_echo's exit code. tf2_echo never returns on its own, so
# `timeout 6 ... || die` (what used to be here) always saw 124 and always died -- the check could
# only ever fail. It went unnoticed because nav2 was usually already running and the relaunch below
# is guarded, so this line was rarely reached. A Translation line is positive proof.
_tf="$(timeout 8 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -20)"
printf '%s\n' "$_tf" | grep -q 'Translation:' \
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
     -p range_min:=0.45 -p range_max:=40.0 -p use_inf:=true
  waitfor 20 /scan_filtered || die "no /scan_filtered"
fi
echo "  /scan_filtered ok"
# The relay is NOT optional: p2l publishes BEST_EFFORT, slam_toolbox subscribes RELIABLE, and
# incompatible DDS QoS delivers zero messages with no error anywhere.
alive /scan || { bg python3 bringup/scan_relay.py; waitfor 20 /scan || die "no /scan — relay failed"; }

# 2b. THE CAMERA. session.sh has always GATED on the camera without ever STARTING it: health.py
# fails CRITICAL on camera_info < 20 Hz and lab_gates gate 2 requires base_link -> mast_cam_link,
# but camera.sh appeared nowhere in this script -- only in README.md as a manual step. So a clean
# boot could not pass its own bring-up, and the advice printed on failure ("restart camera.sh")
# was the step the script should have taken itself.
# Guarded on camera_info, not on the node: a leaked realsense node holds the USB device while
# publishing nothing, which is the state that produced 0.0 Hz with a healthy-looking node list.
if ! alive /mast_cam/color/camera_info; then
  # camera.sh DOES NOT RETURN. It ends in `wait "$NODE_PID"` so it can own the driver's lifetime and
  # clean up its process group on Ctrl-C. Calling it synchronously here hung bring-up forever at
  # this line -- no camera, no safety stage, no gates, no error. It has to be backgrounded like
  # every other long-lived process this script starts.
  #
  # And check for a WEDGED driver before starting another. camera.sh has no already-running guard,
  # so a second invocation adds a second static TF publisher for base_link->mast_cam_link and a
  # second realsense node fighting for the same USB device -- which is exactly what happened at
  # 19:21 yesterday, when two nodes raced and the loser reported "No RealSense devices were found".
  # A node that exists while camera_info is silent is the wedged state, and stacking another on top
  # of it makes the situation worse, so say so instead.
  if ros2 node list 2>/dev/null | grep -q mast_cam; then
    die "a /mast_cam node is running but /mast_cam/color/camera_info is silent. That is a WEDGED
        driver, not a missing one, and starting a second would leave two nodes fighting for the USB
        device. Kill the existing one first, then check the link speed: on a USB 2 port the D435
        cannot open the profiles in config/camera.yaml and loops on xioctl(VIDIOC_S_FMT) errno=5."
  fi
  bg bash "$ROOT/bringup/camera.sh"
  waitfor 40 /mast_cam/color/camera_info || die "no camera_info 40 s after starting camera.sh.
        Check the USB link speed FIRST: on a USB 2 port the D435 cannot open the configured
        profiles at all and retries forever. Restarting will not help."
fi
echo "  camera ok"
echo "  /scan ok (relay running)"

# ------------------------------------------------------------------------------ 3. safety
say "3/6  safety"
bg bash bringup/safety.sh; sleep 3
echo "  mux + arm gate started"

# THE ARM'S TOOL GEOMETRY -- REPORT IT, DO NOT SET IT.
#
# An earlier version of this block ran `arm_tool.py --set`, writing tcp_offset [0,0,172,0,0,0].
# That was WRONG and would have broken the press chain that works. The hand-eye calibration this
# stack aims with was captured on 2026-08-21 with the arm at tcp_offset [0,0,0,0,0,0]
# (EXPERIMENT_LOG.md:875), and calib/pairs/*.json store arm_xyz_m straight from get_position(), so
# calib/handeye.json's marker_on_flange_mm is FLANGE-relative. approach_target.py:254-291 reads
# get_position() and commands set_position() on that basis. Install a 172 mm tool offset and both
# calls start referring to the TOOL TIP instead: the commanded flange retreats by the tool length
# and the marker lands 172 mm SHORT -- with no contact sensor to notice (get_ft_sensor_data answers
# zeros, collision_sensitivity is 0), so approach_target returns 0, press_run prints done, and the
# route reports ROUTE COMPLETE over a press that touched nothing.
#
# So the correct state for the CURRENT calibration is tcp_offset ZERO, and this reports rather than
# writes. Note arm_tool.py's readback cannot settle this either way: EXPERIMENT_LOG.md:2149 --
# "there is NO live getter for the TCP in SDK 1.18.4, the tcp_offset property is a local cache".
# Resolving it properly needs a physical measurement (touch one fixed point from two arm
# configurations), which is docs/CALIBRATION.md item 2 and is still open.
if [ -x "$ROOT/.venv-arm/bin/python" ]; then
  "$ROOT/.venv-arm/bin/python" "$ROOT/bringup/arm_tool.py" 2>&1 | sed 's/^/  /' || true
  echo "  (reported, not set -- the hand-eye calibration assumes tcp_offset ZERO)"
fi
# ASK FOR THE DEADMAN ONLY IF SOMETHING IS ACTUALLY GATED ON IT.
# This block used to demand /safety/enable unconditionally. That is wrong once the operator has set
# requires_enable: false on nav and servo -- the mux then never consults /safety/enable, so waiting
# for it blocks bringup on a signal that changes nothing. It is also the WRONG SAFETY TRADE on this
# robot: the deadman is a browser button, and holding it costs the operator the hand that would
# otherwise be on the physical e-stop. The operator has stated this repeatedly and it is their call.
# The e-stop and arm_stowed gates are untouched and still hard-block every source.
# Read the config rather than assuming, so this tracks whatever safety.yaml actually says.
if grep -qE '^[[:space:]]*requires_enable:[[:space:]]*true' "$ROOT/config/safety.yaml"; then
  echo
  echo "  >>> START THE DEADMAN IN ANOTHER TERMINAL AND HOLD IT:"
  echo "  >>>     cd $ROOT && python3 bringup/deadman.py"
  echo "  >>> safety.yaml has a source with requires_enable: true, and a silent /safety/enable"
  echo "  >>> makes the robot look dead while behaving correctly."
  echo
  printf "  waiting for /safety/enable ... "
  waitfor 180 /safety/enable && echo "ok" || die "/safety/enable never appeared"
else
  echo "  deadman not required — no source in safety.yaml sets requires_enable: true"
  echo "  (e-stop and arm_stowed gates remain active; operator is on the physical e-stop)"
fi

# ------------------------------------------------------------------------------ 4. health
say "4/6  health + gates"
python3 bringup/health.py || die "health.py reported a critical failure"
bash bringup/lab_gates.sh 0 2 || die "gates 0-2 failed — fix before anything moves"

# ------------------------------------------------------------------------------ 5. map (optional)
if [ "$cmd" = "map" ]; then
  say "5/6  mapping drive"
  # slam_toolbox in Jazzy is a LIFECYCLE node: it starts `unconfigured` and looks exactly like a
  # hung node until it is configured AND activated.
  # PARAMS FILE, NOT INLINE FLAGS. The inline version that used to be here set six parameters and
  # took slam_toolbox's DEFAULTS for everything else in config/slam_os0.yaml. Two of those decide
  # whether the map is usable at all, and both were VERIFIED to take effect on 2026-09-01 by
  # launching the node against this file and reading the parameters back:
  #   do_loop_closing true  without it a closed loop does not close; the map comes out bent and
  #                         every waypoint inherits the bend.
  #   stack_size_to_use     serializing a building-sized graph overflows the default stack, so
  #                         map_persist.sh's save would die on exactly the map worth keeping.
  # Also verified taking effect: scan_topic /scan, base_frame base_link, resolution, mode,
  # max_laser_range. NOT min_laser_range -- see config/slam_os0.yaml, it is inert on Jazzy.
  bg ros2 launch slam_toolbox online_async_launch.py \
     use_sim_time:=false slam_params_file:="$ROOT/config/slam_os0.yaml"
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
# DEFAULT MUST BE A MAP THAT EXISTS AND IS RELOCALIZABLE. 'atrium2d' was the default while every
# waypoint on disk carried map_name 'atrium', and atrium2d is grid-only (no .posegraph/.data), so
# the default could not be relocalized into AND disagreed with every waypoint. Nothing compared
# them until nav2_goto gained its map-match check on 2026-09-01.
# DEFAULT TO THE MAP THE WAYPOINTS WERE ACTUALLY RECORDED IN. This said `atrium` while all five
# elevator waypoints carry map_name: elevator and maps/.loaded_map reads elevator. A bare
# `session.sh nav` therefore loaded atrium, rewrote .loaded_map, and nav2_goto.py then refused
# every leg -- "recorded in map 'elevator' but the map currently loaded is 'atrium'". Loud, but it
# costs a morning. atrium cannot even be localized into any more: its .posegraph and .data were
# destroyed by a test fixture and only the grid came back from git.
# tests/test_stack_wiring.py asserts this equals the waypoints' map_name; it was failing on the
# shipped value, which is exactly what it exists to catch.
MAP_NAME=${MAP_NAME:-elevator}
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
  # AN EXISTING /map IS NOT EVIDENCE THAT THE RIGHT MAP IS LOADED, and trusting it was a real
  # bug. `alive /map` is true for a still-running MAPPING session, and for a localization session
  # holding a DIFFERENT map. In either case this used to skip the launch below and then write
  # maps/.loaded_map certifying that $MAP_NAME is live -- manufacturing the exact provenance that
  # waypoints.py and nav2_goto.py were built to trust. The documented flow map -> nav with no
  # `down` in between lands you there, silently, in the wrong coordinate frame.
  #
  # So interrogate the node instead of the topic. A publisher on /map answers "something is
  # running"; the parameters answer "what, and on which map".
  if alive /map; then
    LIVE_MODE=$(timeout 8 ros2 param get /slam_toolbox mode 2>/dev/null | tail -1)
    LIVE_MAP=$(timeout 8 ros2 param get /slam_toolbox map_file_name 2>/dev/null | tail -1)
    case "$LIVE_MODE" in
      *localization*) ;;
      *) die "something is already publishing /map and it is NOT in localization mode
        (mode reads: ${LIVE_MODE:-unreadable}). A mapping session keeps rewriting the map under
        your waypoints, and this script would have certified '$MAP_NAME' as loaded anyway.
        Stop it first:  bash bringup/session.sh down" ;;
    esac
    # ros2 prints `String value is: /path/to/map`. Compare the final path component exactly:
    # substring matching makes requested map `atrium` incorrectly accept live map `atrium2d`.
    LIVE_MAP_VALUE=${LIVE_MAP##*: }
    LIVE_MAP_VALUE=${LIVE_MAP_VALUE#\"}; LIVE_MAP_VALUE=${LIVE_MAP_VALUE%\"}
    LIVE_MAP_VALUE=${LIVE_MAP_VALUE#\'}; LIVE_MAP_VALUE=${LIVE_MAP_VALUE%\'}
    LIVE_MAP_NAME=${LIVE_MAP_VALUE##*/}
    if [ "$LIVE_MAP_NAME" = "$MAP_NAME" ]; then
      echo "  /map already served by localization on '$MAP_NAME'"
    else
      die "slam_toolbox is localizing in a DIFFERENT map (map_file_name reads:
        ${LIVE_MAP:-unreadable}), not '$MAP_NAME'. Their origins are unrelated, so every waypoint
        would resolve to the wrong physical place. Stop it first:
            bash bringup/session.sh down"
    fi
  fi
  if ! alive /map; then
    # LOCALIZATION mode, not mapping: 50 passes through the same corridor must not keep rewriting
    # the map underneath the waypoints. slam_toolbox owns BOTH /map and map->odom here, which is
    # why Nav2 is launched with localization:=slam and starts neither map_server nor AMCL --
    # exactly one source may own each.
    # Same params file as mapping -- a map built with one set of scan-matcher settings and
    # localized with another matches worse for no reason -- overriding only the two that must
    # differ. --ros-args after --params-file wins, so the override is the last word.
    bg ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
       --params-file "$ROOT/config/slam_os0.yaml" \
       -p use_sim_time:=false -p mode:=localization \
       -p map_file_name:="$ROOT/maps/$MAP_NAME"
    sleep 5
    ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1
    ros2 lifecycle set /slam_toolbox activate  >/dev/null 2>&1
    waitfor 40 /map || die "no /map — slam_toolbox localization did not activate"
  fi
  echo "  /map ok"
  # Same grep-not-exit-code rule as the odom->base_link check above. This copy was missed when
  # that one was fixed, 170 lines earlier in the same file, and it sits in start_nav() -- the
  # path taken by `session.sh nav` and `session.sh campaign`. So every Nav2 (re)start died here
  # with "slam_toolbox has not localized", whether or not map->odom was healthy.
  _tf="$(timeout 10 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -20)"
  printf '%s\n' "$_tf" | grep -q 'Translation:' \
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
  # THE BEHAVIOUR-TREE PATHS IN THE PARAMS ARE ABSOLUTE AND POINT AT THE SIM CHECKOUT
  # (default_nav_{to_pose,through_poses}_bt_xml -> /home/<someone>/Desktop/...). Unresolved,
  # bt_navigator loads no tree, the lifecycle manager aborts, and Nav2 comes up looking healthy
  # while navigate_to_pose never works -- the silent half-failure docs/NAV2.md warns about.
  #
  # REDUNDANCY CLAIM, SETTLED 2026-09-01: ranger_nav.launch.py DOES already resolve both keys from
  # its own __file__ and passes them to bt_navigator as the LAST entry of `parameters=`, which wins
  # over params_file -- so the launch file, not this sed, is what decides at runtime. The rewrite
  # is kept anyway: it keeps the params file we hand Nav2 agreeing with the node that will run
  # (a params file that lies about the running config is its own trap), and it is asserted by
  # tests/test_session_e2e.py and tests/test_stack_wiring.py. Drop it only together with those.
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
  # GUARD ON THE CAPABILITY, NOT THE NAME. This used to test `ros2 node list | grep bt_navigator`.
  # A bt_navigator that came up unconfigured -- the exact failure warned about below and in
  # nav2_goto.py -- still appears in `ros2 node list`, so the relaunch was skipped, the wait for
  # navigate_to_pose then timed out, and the script died without ever retrying. Orphaned nodes
  # from a killed `ros2 launch` hit this the same way. The action either exists or it does not.
  if ! ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
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
