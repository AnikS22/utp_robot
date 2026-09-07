#!/usr/bin/env bash
# Canonical robot startup. Read docs/STARTUP.md for modes, saving and shutdown.
#   bash bringup/bringup_all.sh --mode map
#   SEED_POSE=x,y,yaw bash bringup/bringup_all.sh --mode nav --map floor1
#   bash bringup/bringup_all.sh --mode map --status
#   bash bringup/bringup_all.sh --mode inputs   # sensors, camera, arm readback; no SLAM/Nav2
# Healthy components are retained; motion gates remain enforced.
set -uo pipefail          # NOT -e: the whole job of this script is to survive a failure and report

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh" >/dev/null 2>&1 || { echo "cannot source bringup/env.sh"; exit 2; }

LOG=/tmp/utp_bringup.log
MODE=nav
# The default must be the map the WAYPOINTS were actually recorded in. `bringup/session.sh` and
# `bringup/stack.sh` both default to floor1 and tests/test_stack_wiring.py pins them to agree with
# maps/waypoints.yaml; a bring-up that quietly loads a different map hands every waypoint a
# different physical meaning, and safety/map_frame.py then refuses all of them.
MAP_NAME="${MAP_NAME:-floor1}"
STATUS_ONLY=0
NEED_ARM="${UTP_NEED_ARM:-0}"

while [ $# -gt 0 ]; do
  case "$1" in
    --mode)   [ "$#" -ge 2 ] || { echo "--mode requires a value" >&2; exit 2; }; MODE="$2"; shift 2 ;;
    --mode=*) MODE="${1#*=}"; shift ;;
    --map)    [ "$#" -ge 2 ] || { echo "--map requires a value" >&2; exit 2; }; MAP_NAME="$2"; shift 2 ;;
    --map=*)  MAP_NAME="${1#*=}"; shift ;;
    --status) STATUS_ONLY=1; shift ;;
    -h|--help) sed -n '2,6p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1  (see --help)" >&2; exit 2 ;;
  esac
done
case "$MODE" in
  inputs|map|nav|full) ;;
  *) echo "--mode must be inputs, map, nav or full (got '${MODE}')" >&2; exit 2 ;;
esac
[ "$STATUS_ONLY" = 1 ] && PROBE_ERROR_LOG=/dev/null
case "$MAP_NAME" in
  ''|*[!a-zA-Z0-9_.-]*|.|..) echo "--map must be a bare map name" >&2; exit 2 ;;
esac
if [ -n "${SEED_POSE:-}" ]; then
  # VALIDATE **AND NORMALISE TO FLOATS**. Not cosmetic: ROS types a parameter from the literal it
  # is given, so map_start_pose:=[0,0,0] arrives as an integer_array while slam_toolbox declares
  # the parameter double_array, and configure dies with
  #   "parameter {map_start_pose} is of type {double_array}, setting it to {integer_array}
  #    is not allowed"
  # Four failed configure transitions and then a segfault, reported by this script as the useless
  # "lifecycle=absent, /map seen=0". Measured 2026-09-06 with SEED_POSE=0,0,0 -- the most natural
  # thing anyone would type. The same seed written 0.0,0.0,0.0 works, which is exactly the kind of
  # difference no operator should have to know about.
  SEED_POSE="$(python3 - "$SEED_POSE" <<'PYSEED'
import math, sys
try:
    values = [float(v) for v in sys.argv[1].split(',')]
    assert len(values) == 3 and all(math.isfinite(v) for v in values)
except (ValueError, AssertionError):
    print('SEED_POSE must contain three finite numbers: x,y,yaw', file=sys.stderr)
    raise SystemExit(2)
print(','.join(repr(float(v)) for v in values))
PYSEED
)" || exit 2
fi
{ [ "$MODE" = full ] || [ "$MODE" = inputs ]; } && NEED_ARM=1
WANT_NAV=0; [ "$MODE" = nav ] || [ "$MODE" = full ] && WANT_NAV=1
# nav IS IN THIS LIST, and leaving it out cost a run on 2026-09-07. mission.sh brings the stack
# up in `nav` mode, so the camera was never started and never checked -- and every earlier run
# worked only because a camera left alive by some previous session happened to still be
# publishing. The first cold start after a recharge exposed it: the call press died at its
# very first step with "rgb=0 depth=0 info=no", the arm never moved, and nothing in the
# preflight had said a word, because the preflight checked odometry, lidar, scan, safety,
# slam and Nav2 -- everything except the sensor two of the three mission steps aim with.
WANT_CAMERA=0; { [ "$MODE" = full ] || [ "$MODE" = inputs ] || [ "$MODE" = nav ]; } \
    && [ "${UTP_NO_CAMERA:-0}" != "1" ] && WANT_CAMERA=1

if [ "$WANT_NAV" = 1 ] && [ "$STATUS_ONLY" = 0 ]; then
  for ext in pgm yaml posegraph data; do
    [ -s "$REPO/maps/$MAP_NAME.$ext" ] || {
      echo "Missing or empty maps/$MAP_NAME.$ext; no components started." >&2
      exit 2
    }
  done
fi

# ---------------------------------------------------------------- report plumbing
START_SECONDS=$SECONDS
RESULT=()   # "name|state|detail"
WHY=()      # "name|cause, fix, and what the symptom looks like downstream"
declare -A S    # stage -> state, so a later stage can ask whether its input actually came up

# Every stored field is folded to ONE LINE: the report reads rows back with `IFS='|' read`, and
# read stops at the first newline -- a multi-line reason came out truncated to its first line,
# which is worse than no reason, because it reads as a complete sentence missing its point.
fold1() { printf '%s' "$1" | tr -s '[:space:]' ' ' | sed 's/^ *//; s/ *$//'; }
why()    { WHY+=("$1|$(fold1 "$2")"); }
record() { RESULT+=("$1|$2|$(fold1 "$3")"); S["$1"]="$2"; }
note()   { printf '  %s\n' "$*"; }
up()     { [ "${S[$1]:-}" = "ok" ] || [ "${S[$1]:-}" = "WARN" ]; }

# Numeric compare that must not fail on an empty or malformed rate: "" and "0.00" both read as 0.
# LC_ALL=C because the default awk here is mawk, whose string->number conversion goes through
# strtod and IS locale-sensitive: under a comma-decimal locale "1.5" converts to 1 and every
# threshold in this script would silently be wrong.
ge() { LC_ALL=C awk -v a="${1:-0}" -v b="${2:-0}" 'BEGIN{exit !(a+0 >= b+0)}'; }

# ---------------------------------------------------------------- process identity
# Ours = the full command line matches AND the process carries this repo's ownership marker and
# this domain. env.sh exports both into everything it starts, so nothing we did not start can
# carry them and everything we did start does. Matching on a name, a frame or a topic is what
# killed 22 sim TF publishers on 2026-08-18.
_is_ours() {
  local pid="$1" e
  e="$( { tr '\0' '\n' < "/proc/$pid/environ"; } 2>/dev/null )" || return 1
  printf '%s\n' "$e" | grep -qxF "UTP_ROBOT_STACK=$REPO" || return 1
  printf '%s\n' "$e" | grep -qxF "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" || return 1
  return 0
}

# Walk every pid, compare the WHOLE command line, and skip:
#   * this pid and $BASHPID -- $$ does not change inside a subshell, $BASHPID does;
#   * the entire ancestor chain -- a `bash -c` wrapper whose command line happens to contain the
#     pattern is exactly the "killed the calling shell" failure, twice now;
#   * anything whose command line is byte-identical to ours -- $(...) and pipeline subshells are
#     none of the above, and every bash subshell shares its parent's cmdline, so this catches
#     them at any nesting depth.
#   * an EMPTY pattern, which would make `case "$a" in *""*)` match every process on the machine.
pids_matching() {
  python3 "$REPO/bringup/startup_processes.py" "${1:-}"
}

pids_ours()     { local p; while read -r p; do [ -n "$p" ] && _is_ours "$p" && printf '%s\n' "$p"; done < <(pids_matching "$1"); }
pids_foreign()  { local p; while read -r p; do [ -n "$p" ] && ! _is_ours "$p" && printf '%s\n' "$p"; done < <(pids_matching "$1"); }
count_ours()    { pids_ours "$1" | wc -l; }
count_foreign() { pids_foreign "$1" | wc -l; }

# SIGINT alone is not a restart: a wedged Ouster driver that ignores INT keeps the UDP socket and
# the fresh driver then comes up publishing 0.00 Hz -- the exact fault this clears. Wait for the
# victims to actually go, then SIGKILL what is left.
kill_ours() {
  local pid a victims=() alive i
  mapfile -t victims < <(pids_ours "${1:-}")
  [ "${#victims[@]}" -gt 0 ] || return 0
  note "stopping ${#victims[@]} existing '$1' process(es): ${victims[*]}"
  kill -INT "${victims[@]}" 2>/dev/null
  for i in 1 2 3 4 5; do
    sleep 1; alive=0
    for pid in "${victims[@]}"; do
      a="$( { tr -d '\0' < "/proc/$pid/cmdline"; } 2>/dev/null )" || continue
      [ -n "$a" ] && alive=1               # an empty cmdline means gone or reaped
    done
    [ "$alive" = 0 ] && return 0
  done
  for pid in "${victims[@]}"; do kill -KILL "$pid" 2>/dev/null; done
  return 0
}

# TRAP 8. `nohup ... &` alone dies with the shell that started it -- a mapping node launched that
# way was gone minutes later. setsid detaches the process group, </dev/null stops it stealing the
# terminal, and disown removes it from this shell's job table.
start_bg() {
  setsid nohup "$@" >>"$LOG" 2>&1 < /dev/null &
  disown 2>/dev/null || true
  sleep 1
}

# A foreign copy is never killed and never stacked on: report it with the pids and let the human
# decide. Returns 1 when starting would create a duplicate.
foreign_blocked() {   # foreign_blocked <component> <pattern>
  local n pids
  n=$(count_foreign "$2")
  [ "$n" -gt 0 ] || return 0
  pids="$(pids_foreign "$2" | tr '\n' ' ')"
  record "$1" FAIL "$n process(es) matching '$2' are NOT ours -- not started, not killed"
  why "$1" "there are $n process(es) whose command line matches '$2' but which do NOT carry this
       repo's UTP_ROBOT_STACK marker or this ROS_DOMAIN_ID ($ROS_DOMAIN_ID), so they were started
       outside bringup/env.sh -- by hand, by another checkout, or by a session in another domain.
       This script refuses to kill them (a loose pattern kill has taken out the calling shell
       twice and 22 of the sim campaign's TF publishers once) and refuses to start a second copy
       (two publishers on one topic interleave; two Nav2 stacks never activate). Look at them and
       kill them BY PID yourself: ps -o pid,cmd -p ${pids% }; then kill -INT ${pids% }"
  return 1
}

# ---------------------------------------------------------------- the probe
# ONE node, EVERY topic, discovery first, counters reset, then count. See trap 5 in the header.
SETTLE=3.0; WINDOW=4.0; TF_BUDGET=3
declare -A P
probe() {
  local out lim
  lim="${PROBE_LIMIT:-30}"
  # Preserve unrelated observations; invalidate every requested field before probing.
  unset 'P[err]'
  local spec kind x y rest
  for spec in "$@"; do
    IFS=: read -r kind x y rest <<< "$spec"
    case "$kind" in
      topic) P["hz:$x"]=0; P["seen:$x"]=0 ;;
      tf) P["tf:$x>$y"]=MISSING ;;
      gates) P[gaten]=0; unset 'P[gate:arm_stowed]' 'P[gate:estop_latched]' ;;
    esac
  done
  out="$(timeout "$lim" python3 "$REPO/bringup/startup_probe.py" \
    "$SETTLE" "$WINDOW" "$TF_BUDGET" "$@" 2>>"${PROBE_ERROR_LOG:-$LOG}")" || P[err]="probe failed or timed out"
  while IFS='|' read -r k v; do
    [ -n "${k:-}" ] || continue
    P["$k"]="$v"
  done <<<"$out"
}
hz()    { printf '%s' "${P[hz:$1]:-0.00}"; }
seen()  { printf '%s' "${P[seen:$1]:-0}"; }
tfst()  { printf '%s' "${P[tf:$1>$2]:-MISSING}"; }
tfok()  { [ "$(tfst "$1" "$2")" = "ok" ]; }

# ros2 lifecycle get prints "active [3]" / "inactive [2]", or nothing when the node is absent.
# MATCH THE WHOLE FIELD, NEVER A SUBSTRING: "inactive" CONTAINS "active", and a `grep -q active`
# here reports a dead Nav2 as healthy. That is this repo's signature bug in one word.
lc_state() {
  local s
  s="$(timeout 10 ros2 lifecycle get "$1" 2>/dev/null | head -1 | awk '{print $1}')"
  printf '%s\n' "${s:-absent}"
}
lc_active() { [ "$(lc_state "$1")" = "active" ]; }

source "$REPO/bringup/startup_helpers.sh"

# ---------------------------------------------------------------- the topics this mode cares about
SPEC=(
  "topic:/odom:nav_msgs.msg:Odometry:sensor"
  "topic:/ouster/points:sensor_msgs.msg:PointCloud2:sensor"
  "topic:/scan_filtered:sensor_msgs.msg:LaserScan:sensor"
  # /scan and /scan_nav are probed RELIABLE ON PURPOSE, matching their consumers. A BEST_EFFORT
  # subscriber would read them green even if the relay were publishing BEST_EFFORT -- and then
  # slam_toolbox, which subscribes RELIABLE, would receive ZERO messages with no error anywhere.
  # The probe must fail in the same way the consumer fails.
  "topic:/scan:sensor_msgs.msg:LaserScan:reliable"
  "topic:/scan_nav:sensor_msgs.msg:LaserScan:reliable"
  "topic:/safety/status:std_msgs.msg:String:sensor"
  "gates:/safety/status"
  "tf:odom:base_link"
  "tf:base_link:os_sensor"
  "tf:base_link:os_lidar"
)
[ "$MODE" = inputs ] || SPEC+=("topic:/map:nav_msgs.msg:OccupancyGrid:latched" "tf:map:odom")
[ "$WANT_CAMERA" = 1 ] && SPEC+=("topic:/mast_cam/color/camera_info:sensor_msgs.msg:CameraInfo:sensor")

echo
echo "=== utp bring-up   mode=$MODE   map=$MAP_NAME   domain=$ROS_DOMAIN_ID   $( [ "$STATUS_ONLY" = 1 ] && echo '(STATUS ONLY -- nothing will be started)' )"
# Only truncate the log when this run might write to it: in --status the log is the record of
# whatever brought the stack up, and emptying it destroys the evidence a check-only run collects.
[ "$STATUS_ONLY" = 1 ] || : > "$LOG"

# ============================================================================================
# STAGE 0 -- the things no software can fix. No ROS, no sudo, no waiting.
# ============================================================================================
HUMAN_NEEDED=0

IFACE=$(/sbin/ip -brief link show 2>/dev/null | awk '/^enx/{print $1; exit}')
if [ -z "$IFACE" ]; then
  record ethernet FAIL "no enx* USB-ethernet interface"
  why ethernet "the USB-ethernet adapter is not enumerated. That ONE cable carries the lidar
       (192.168.1.119), the xArm (192.168.1.221) and the router (192.168.1.1), so all three go
       down together and it reads like three separate faults. Reseat the adapter, then:
       ip -brief link show"
  HUMAN_NEEDED=1
elif /sbin/ip -brief link show "$IFACE" 2>/dev/null | grep -q NO-CARRIER; then
  record ethernet FAIL "$IFACE NO-CARRIER"
  why ethernet "$IFACE is enumerated but has NO CARRIER, so lsusb looks perfectly healthy while
       nothing on 192.168.1.x answers. Reseat the cable at BOTH ends and strain-relieve it; it
       has come adrift mid-session before and took the lidar, the arm and the router with it."
  HUMAN_NEEDED=1
else
  record ethernet ok "$IFACE carrier up"
fi

# TRAP 4. The lidar and the router are fatal for every mode. The ARM is fatal only when the mode
# actually uses it -- session.sh dies on 192.168.1.221 for every session type, which blocks a
# mapping or nav-only run on a device that is deliberately powered off.
for pair in 192.168.1.119:lidar 192.168.1.1:router; do
  a=${pair%%:*}; n=${pair##*:}
  if ping -c1 -W2 "$a" >/dev/null 2>&1; then record "net_$n" ok "$a reachable"
  else
    record "net_$n" FAIL "$a unreachable"
    why "net_$n" "the $n at $a does not answer. Check $IFACE first -- one cable carries the lidar,
         the arm and the router, so a single unseated plug presents as three device faults.
         Without the lidar there is no cloud, no /scan, and slam_toolbox cannot match anything."
    HUMAN_NEEDED=1
  fi
done
if ping -c1 -W2 192.168.1.221 >/dev/null 2>&1; then
  record net_arm ok "192.168.1.221 reachable"
elif [ "$NEED_ARM" = 1 ]; then
  record net_arm FAIL "192.168.1.221 unreachable and --mode $MODE needs the arm"
  why net_arm "the xArm at 192.168.1.221 does not answer, and this mode drives it. Power the arm
       on and wait for its controller to boot (~30 s), or run --mode nav / --mode map, where the
       arm is not touched and this check is only a note."
  HUMAN_NEEDED=1
else
  record net_arm WARN "192.168.1.221 unreachable -- not needed for --mode $MODE"
  why net_arm "the xArm is off or unplugged. That is NORMAL for mapping and nav-only work and is
       NOT fatal here (session.sh's step-0 gate dies on it for every session type, which is the
       trap this replaces). Two consequences to know about: nothing can press anything, and the
       arm_stowed safety gate has no MEASURED evidence, so it fails closed and the base will not
       accept autonomous twists. Restore the arm connection and verify its stow pose before
       requesting software motion."
fi

# TRAP 3. can0 needs a password. Detect, print the command, STOP. Never run sudo from here: the
# prompt is invisible behind a backgrounded bring-up and reads exactly like a hang.
if [ ! -e /sys/class/net/can0 ]; then
  record can0 FAIL "interface absent -- the USB-CAN adapter is not enumerated"
  why can0 "can0 DOES NOT EXIST, so the chassis driver has nothing to talk to: no /odom, no
       odom->base_link, and therefore slam_toolbox can never publish map->odom. The symptom
       surfaces THREE LAYERS AWAY as 'localization is wrong in RViz'. Plug the adapter in and
       bring the link up (this needs your password, so this script will not do it):
         lsusb
         sudo ip link set can0 up type can bitrate 500000
         python3 bringup/claim_can.py"
  HUMAN_NEEDED=1
elif [ "$(cat /sys/class/net/can0/operstate 2>/dev/null)" != "up" ]; then
  record can0 FAIL "present but $(cat /sys/class/net/can0/operstate 2>/dev/null) -- needs sudo"
  why can0 "can0 is enumerated but DOWN. Bringing it up prompts for a password, which cannot be
       answered by a bring-up script -- so this stops here instead of hanging on a prompt you
       cannot see. Run this one line, then re-run this script (it is idempotent):
         sudo ip link set can0 up type can bitrate 500000
       Without it Nav2 will plan perfectly and the robot will not move, and /odom will stream at
       full rate with every velocity sample identically zero."
  HUMAN_NEEDED=1
else
  # can0 can sit 'up' with a dead chassis on the far end: healthy interface, zero rx, climbing tx
  # errors because nothing is left to ACK. Frames ARRIVING is the real test.
  _r1=$(cat /sys/class/net/can0/statistics/rx_packets 2>/dev/null || echo 0)
  sleep 1
  _r2=$(cat /sys/class/net/can0/statistics/rx_packets 2>/dev/null || echo 0)
  _rx=$(( _r2 - _r1 ))
  _can_state=$(ip -j -details link show can0 | python3 -c 'import json,sys; print(json.load(sys.stdin)[0].get("linkinfo",{}).get("info_data",{}).get("state","UNKNOWN"))')
  if [ "$_rx" -gt 50 ] && [ "$_can_state" = ERROR-ACTIVE ]; then
    record can0 ok "$_can_state, ~${_rx} frames/s"
  else
    HUMAN_NEEDED=1
    record can0 FAIL "$_can_state, only ~${_rx} received frames/s; odometry is not verified"
    why can0 "can0 is configured but the bus is not delivering healthy chassis feedback. Check
         Ranger power and the CAN cable at both ends; ERROR-PASSIVE indicates bus errors. /odom may still publish at
         full rate carrying nothing but zeros, which is indistinguishable from a stationary robot
         until you command a motion that never happens."
  fi
fi

if [ "$HUMAN_NEEDED" = 1 ] && [ "$STATUS_ONLY" = 0 ] && [ "$MODE" != inputs ]; then
  echo
  printf '  %-14s %-7s %s\n' COMPONENT STATE DETAIL
  printf '  %-14s %-7s %s\n' "--------------" "-------" "------------------------------------------"
  for row in "${RESULT[@]}"; do IFS='|' read -r n s d <<<"$row"; printf '  %-14s %-7s %s\n' "$n" "$s" "$d"; done
  echo
  echo "  WHY:"
  for row in "${WHY[@]}"; do
    IFS='|' read -r n d <<<"$row"
    printf '    %s:\n' "$n"
    printf '%s\n' "$d" | fold -s -w 88 | sed 's/^/      /'
    echo
  done
  echo "  NOTHING WAS STARTED. These faults are physical: no amount of relaunching fixes them,"
  echo "  and starting the stack on top of them produces nodes that are alive and silent forever."
  echo "  Fix the above, then re-run:  bash bringup/bringup_all.sh --mode $MODE --map $MAP_NAME"
  exit 2
fi

# ============================================================================================
# First full probe. Everything below decides from THIS, and re-probes after it starts anything.
# ============================================================================================
note "probing (one node, every topic, ~$(LC_ALL=C awk -v s="$SETTLE" -v w="$WINDOW" 'BEGIN{printf "%.0f", s+w}') s plus TF) ..."
probe "${SPEC[@]}"
if [ -n "${P[err]:-}" ]; then
  echo "Probe failed: ${P[err]}. No components restarted; see $LOG." >&2
  exit 1
fi
if [ "$WANT_NAV" = 1 ] && [ "$STATUS_ONLY" = 0 ] && [ "$(seen /map)" -eq 0 ] && [ -z "${SEED_POSE:-}" ]; then
  echo "Cold localization needs SEED_POSE=x,y,yaw in '$MAP_NAME'. No components started." >&2
  exit 2
fi

# ============================================================================================
# STAGE 1 -- chassis: /odom and odom->base_link
# ============================================================================================
if [ "$MODE" = inputs ] && ! up can0; then
  record chassis BLOCKED "can0 is down; other inputs can still start"
else
r=$(hz /odom)
if ! ge "$r" 5 && [ "$STATUS_ONLY" = 0 ]; then
  if foreign_blocked chassis ranger_base_node; then
    note "chassis silent ($r Hz) -- restarting the ranger driver"
    kill_ours ranger_mini_v3.launch; kill_ours ranger_base_node; sleep 3
    # publish_odom_tf:=true is NOT the launch default, and everything downstream needs
    # odom->base_link. Without it slam_toolbox and Nav2 both come up and neither works.
    start_bg ros2 launch ranger_bringup ranger_mini_v3.launch.py publish_odom_tf:=true
    wait_ready 30 /odom 5 "topic:/odom:nav_msgs.msg:Odometry:sensor" "tf:odom:base_link"
    r=$(hz /odom)
  fi
fi
if [ "${S[chassis]:-}" != "FAIL" ]; then
  if ge "$r" 5 && tfok odom base_link; then
    record chassis ok "/odom ${r} Hz, odom->base_link present"
  elif ge "$r" 5; then
    record chassis FAIL "/odom ${r} Hz but odom->base_link MISSING"
    why chassis "the driver publishes /odom but not the TRANSFORM. ranger_mini_v3.launch.py
         defaults publish_odom_tf to false, and this stack must launch it with
         publish_odom_tf:=true. Without odom->base_link slam_toolbox cannot publish map->odom no
         matter how good the scan is, Nav2's costmaps never come up, and the whole thing presents
         as 'localization is wrong' -- three layers from the cause."
  else
    record chassis FAIL "/odom ${r} Hz (want >=5)"
    _cm="$(timeout 25 python3 "$REPO/bringup/chassis_mode.py" 2>&1 | grep -oE 'control_mode=[A-Z]+' | head -1)"
    case "$_cm" in
      *RC*) why chassis "the chassis answers on CAN but control_mode=RC: the TRANSMITTER holds
                 authority and every computer command is discarded IN FIRMWARE, below anything
                 ROS can observe. /odom keeps flowing and the mux keeps reporting 'permitted'.
                 Flip SWB UP on the transmitter, then: python3 bringup/claim_can.py" ;;
      *)    why chassis "can0 is up and the chassis is ${_cm:-not answering}. If it is not
                 answering, the driver is dead or wedged -- read $LOG. /odom at full rate with
                 ALL-ZERO velocity is a DIFFERENT fault: that is a dead CAN link, not a dead
                 driver, and it looks identical to a robot standing still." ;;
    esac
  fi
fi

fi

# ============================================================================================
# STAGE 2 -- lidar: the mount TF *and* the driver, from bringup/lidar3d.sh, in that order
# ============================================================================================
# TRAP 2, THE SILENT KILLER. base_link->os_sensor comes from lidar3d.sh reading config/ouster.yaml's
# `mount` block, NOT from the driver. This never launches ouster_ros/driver.launch.py directly.
r=$(hz /ouster/points)
if { ! ge "$r" 1.5 || ! tfok base_link os_lidar; } && [ "$STATUS_ONLY" = 0 ]; then
  if foreign_blocked lidar os_driver; then
    note "lidar not healthy (/ouster/points $r Hz, base_link->os_lidar $(tfst base_link os_lidar)) -- restarting via bringup/lidar3d.sh"
    kill_ours "$REPO/bringup/lidar3d.sh"; kill_ours os_driver
    kill_ours "--child-frame-id os_sensor"        # our own mount publisher only; see _is_ours
    sleep 4
    start_bg bash "$REPO/bringup/lidar3d.sh"
    wait_ready 55 /ouster/points 1.5 "topic:/ouster/points:sensor_msgs.msg:PointCloud2:sensor" "tf:base_link:os_sensor" "tf:base_link:os_lidar"
    r=$(hz /ouster/points)
  fi
fi

MOUNT="$(python3 - "$REPO/config/ouster.yaml" <<'PY' 2>/dev/null
import sys, yaml
m = yaml.safe_load(open(sys.argv[1]))["mount"]
print(f"({m['x_m']}, {m['y_m']}, {m['z_m']}) m")
PY
)"
if [ "${S[lidar]:-}" != "FAIL" ]; then
  if ge "$r" 1.5; then record lidar ok "/ouster/points ${r} Hz"
  else
    record lidar FAIL "/ouster/points ${r} Hz"
    if ping -c1 -W2 192.168.1.119 >/dev/null 2>&1; then
      why lidar "the sensor ANSWERS on 192.168.1.119 but no cloud arrives. That is a stale or
           wedged driver holding the UDP socket -- it survives a robot power cycle and still
           shows in 'ros2 node list'. A restart was already attempted; if it persists, read $LOG
           for 'poll_client timed out' or \"Couldn't get active config\". The other cause is the
           udp_dest trap: the sensor was once found streaming to 192.168.1.106 while reporting
           status RUNNING over HTTP. ouster_ros rewrites udp_dest on connect, so anything that
           does NOT reconfigure sees zero packets from a perfectly healthy sensor."
    else
      why lidar "192.168.1.119 does not answer, so this is the NETWORK and not the driver. One
           USB-ethernet cable carries the lidar, the xArm and the router. Check that
           'ip -brief link show' does not say NO-CARRIER before touching any software."
    fi
  fi
fi

# The mount transform, checked as its own component because it fails ON ITS OWN, silently, and
# everything downstream of it dies quietly.
if tfok base_link os_lidar; then
  record mount_tf ok "base_link->os_sensor->os_lidar resolves, mount $MOUNT"
elif tfok base_link os_sensor; then
  record mount_tf FAIL "base_link->os_sensor ok but base_link->os_lidar MISSING"
  why mount_tf "the mount transform is published but os_sensor->os_lidar is not, which is the
       DRIVER's half of the chain (config/ouster_driver.yaml: sensor_frame/lidar_frame,
       pub_static_tf: true). If /ouster/points is also silent, fix the driver first."
else
  record mount_tf FAIL "base_link->os_sensor MISSING -- p2l will drop every cloud in silence"
  why mount_tf "THIS IS THE SILENT KILLER. base_link->os_sensor is published by
       bringup/lidar3d.sh from the \`mount\` block in config/ouster.yaml -- currently $MOUNT --
       and by NOTHING ELSE. If you (or a script) ran 'ros2 launch ouster_ros driver.launch.py'
       directly, the cloud flows and that transform never appears. pointcloud_to_laserscan then
       cannot transform into target_frame base_link, so it DROPS EVERY CLOUD without a word:
       /scan sits at exactly 0.00 Hz, every node reports healthy, and no error is printed
       anywhere. Start the lidar the only supported way:  bash bringup/lidar3d.sh
       (--no-tf exists for the case where a URDF owns the transform; nothing here publishes one)."
fi

# ============================================================================================
# STAGE 3 -- match the cloud chain to the MAP being loaded
# ============================================================================================
# A MAP IS ONLY VALID FOR THE CHAIN THAT BUILT IT, and the two maps in this building were built
# with DIFFERENT chains:
#
#   floor1   raw /ouster/points. Driven 2026-09-05 without the artifact filter.
#   floor2   /ouster/points_clean. config/floors.yaml calls it "the CLEAN re-map, made after
#            safety/cloud_artifact_filter.py stopped the OS0 near-field crosstalk from stamping
#            phantom walls", and keeps the contaminated run beside it: 3467 occupied cells against
#            2884, the 583-cell difference being the artifact.
#
# This stage was hardcoded to skip the filter and to SAY "matches the saved floor1 mapping chain"
# whatever --map asked for. Localizing floor2 that way feeds the matcher a ring of phantom returns
# 0.85-1.20 m behind the robot that its map does not contain. Measured 2026-09-06: fit 27.8% on
# floor2 -- lost by this stack's own scale -- and the robot then grounded a decoy at the call plate
# because it was not where it believed it was. The operator saw it first, as "the dots behind the
# lidar that shouldn't exist came back".
case "${UTP_CLOUD_CHAIN:-auto}" in
  raw)      CLOUD_TOPIC=/ouster/points ;;
  filtered) CLOUD_TOPIC=/ouster/points_clean ;;
  *) case "$MAP_NAME" in
       floor2|floor2.*) CLOUD_TOPIC=/ouster/points_clean ;;
       *)               CLOUD_TOPIC=/ouster/points ;;
     esac ;;
esac
if [ "$CLOUD_TOPIC" = /ouster/points ]; then
  record filter skip "raw /ouster/points: matches the '$MAP_NAME' mapping chain"
elif [ "$STATUS_ONLY" = 1 ]; then
  r=$(hz /ouster/points_clean)
  ge "$r" 1.5 && record filter ok "/ouster/points_clean ${r} Hz" \
               || record filter FAIL "/ouster/points_clean ${r} Hz"
elif foreign_blocked filter cloud_artifact_filter.py; then
  r=$(hz /ouster/points_clean)
  if ! ge "$r" 1.5; then
    note "'$MAP_NAME' was mapped with the artifact filter -- starting it"
    kill_ours cloud_artifact_filter.py; sleep 2
    start_bg python3 "$REPO/safety/cloud_artifact_filter.py"
    wait_ready 25 /ouster/points_clean 1.5 \
      "topic:/ouster/points_clean:sensor_msgs.msg:PointCloud2:sensor"
    r=$(hz /ouster/points_clean)
  fi
  ge "$r" 1.5 && record filter ok "/ouster/points_clean ${r} Hz (matches the '$MAP_NAME' chain)" \
    || { record filter FAIL "/ouster/points_clean ${r} Hz"
         why filter "'$MAP_NAME' was built from filtered clouds. Localizing it against raw
              /ouster/points hands the matcher phantom near-field returns its map does not
              contain -- measured 27.8% fit on floor2, which is lost."; }
fi

# ============================================================================================
# STAGE 4 -- projection: /ouster/points -> /scan_filtered   (needs the mount TF)
# ============================================================================================
r=$(hz /scan_filtered)
if ! ge "$r" 1.5 && [ "$STATUS_ONLY" = 0 ]; then
  if ! tfok base_link os_lidar; then
    record projection BLOCKED "not started: base_link->os_lidar does not resolve"
    why projection "pointcloud_to_laserscan transforms every cloud into target_frame base_link.
         Without base_link->os_lidar it drops all of them and publishes NOTHING, with no error,
         while looking perfectly healthy in 'ros2 node list'. Starting it now would manufacture
         exactly that state, so it was not started. See the mount_tf row above."
  elif ! up lidar; then
    record projection BLOCKED "not started: /ouster/points is down"
    why projection "p2l subscribes to /ouster/points. Started against a silent input it is
         alive and silent forever. Fix the lidar row above first."
  elif foreign_blocked projection pointcloud_to_laserscan; then
    note "/scan_filtered silent ($r Hz) -- starting pointcloud_to_laserscan on $CLOUD_TOPIC"
    kill_ours pointcloud_to_laserscan; sleep 2
    # These numbers ARE the chain: the height band and range_min decide what the map contains,
    # and a map is only valid for the chain that built it. They match the floor1 mapping session.
    # range_min 0.45: 0.70 hid a real door at 0.72 m; 0.30 exposed the packed arm at 0.31-0.36 m.
    start_bg ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
      -r cloud_in:="$CLOUD_TOPIC" -r scan:=/scan_filtered -p target_frame:=base_link \
      -p min_height:=0.20 -p max_height:=1.20 -p angle_min:=-3.14159 -p angle_max:=3.14159 \
      -p angle_increment:=0.0061 -p range_min:=0.45 -p range_max:=40.0 -p use_inf:=true
    wait_ready 25 /scan_filtered 1.5 "topic:/scan_filtered:sensor_msgs.msg:LaserScan:sensor"
    r=$(hz /scan_filtered)
  fi
fi
# A HEALTHY /scan_filtered IS NOT EVIDENCE IT IS THE RIGHT ONE. If p2l is already up against the
# other cloud topic, every rate above is fine and the scans are simply wrong for this map. Read the
# running command line rather than trusting the rate.
if [ "$STATUS_ONLY" = 0 ] && ge "$r" 1.5; then
  _p2l_wrong=0
  for _p in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
    _c=$(tr '\0' ' ' < "/proc/$_p/cmdline" 2>/dev/null) || continue
    case "$_c" in
      *pointcloud_to_laserscan_node*)
        case "$_c" in *"cloud_in:=$CLOUD_TOPIC"*) ;; *) _p2l_wrong=1; kill -INT "$_p" 2>/dev/null ;; esac ;;
    esac
  done
  if [ "$_p2l_wrong" = 1 ]; then
    note "pointcloud_to_laserscan was consuming the WRONG cloud for '$MAP_NAME' -- restarting on $CLOUD_TOPIC"
    sleep 3
    start_bg ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
      -r cloud_in:="$CLOUD_TOPIC" -r scan:=/scan_filtered -p target_frame:=base_link \
      -p min_height:=0.20 -p max_height:=1.20 -p angle_min:=-3.14159 -p angle_max:=3.14159 \
      -p angle_increment:=0.0061 -p range_min:=0.45 -p range_max:=40.0 -p use_inf:=true
    wait_ready 25 /scan_filtered 1.5 "topic:/scan_filtered:sensor_msgs.msg:LaserScan:sensor"
    r=$(hz /scan_filtered)
  fi
fi
if [ -z "${S[projection]:-}" ]; then
  ge "$r" 1.5 && record projection ok "/scan_filtered ${r} Hz from $CLOUD_TOPIC" \
               || { record projection FAIL "/scan_filtered ${r} Hz"
                    why projection "pointcloud_to_laserscan is not publishing. The two ways this
                         happens are (a) its input is silent -- check /ouster/points above,
                         and (b) it cannot transform into base_link, which drops every cloud with
                         no error at all. Check the mount_tf row: that failure produces exactly
                         0.00 Hz here with every node reporting healthy."; }
fi

# ============================================================================================
# STAGE 5 -- the two relays. NOT optional: p2l publishes BEST_EFFORT, slam_toolbox subscribes
# RELIABLE, and incompatible DDS QoS delivers zero messages with no error anywhere.
# ============================================================================================
# UTP_RELAY_ROLE is not read by scan_relay.py. It is passed through `env` so that the two
# otherwise byte-identical relays have DISTINGUISHABLE command lines, and one can be restarted
# without touching the other. Killing "scan_relay.py" would take out both.
start_relay() {   # start_relay <role> <out topic> <mask m>
  start_bg env "UTP_RELAY_ROLE=$1" UTP_SCAN_IN=/scan_filtered "UTP_SCAN_OUT=$2" \
    "UTP_MASK_MAX_M=$3" python3 "$REPO/bringup/scan_relay.py"
}

r=$(hz /scan)
if ! ge "$r" 1.5 && [ "$STATUS_ONLY" = 0 ]; then
  if ! up projection; then
    record scan BLOCKED "not started: /scan_filtered is down"
    why scan "the relay subscribes to /scan_filtered. Started against a silent input it is alive
         and silent forever, and slam_toolbox then sits there matching nothing while every node
         in the graph reports healthy. Fix the projection row first."
  elif [ "$(count_ours 'UTP_RELAY_ROLE=nav')" -gt 0 ] || [ "$(count_foreign 'UTP_RELAY_ROLE=nav')" -gt 0 ]; then
    record scan FAIL "alternate nav relay is running; inspect the sensor profile before repair"
  elif foreign_blocked scan scan_relay.py; then
    note "/scan silent ($r Hz) -- starting the slam relay (mask 0.90 m)"
    kill_ours scan_relay.py; sleep 2
    start_relay slam /scan 0.90
    wait_ready 25 /scan 1.5 "topic:/scan:sensor_msgs.msg:LaserScan:reliable"
    r=$(hz /scan)
  fi
fi
if [ -z "${S[scan]:-}" ]; then
  if ge "$r" 6; then record scan ok "/scan ${r} Hz (rear mask 0.90 m)"
  elif ge "$r" 1.5; then
    record scan WARN "/scan ${r} Hz -- below 6"
    why scan "slam_toolbox searches with coarse_angle_resolution 2.0 deg. At ${r} Hz and wz_max
         0.8 rad/s (46 deg/s) consecutive scans are far more than 2 deg apart, so the pose SLIDES
         through every turn and the controller drives against a stale estimate. That is what put
         the robot 1.85 m from where Nav2 said it had arrived, and into a wall. The 3.1 MB point
         cloud is the bottleneck (~73% of messages lost in DDS). Do NOT 'fix' it by pointing slam
         at the driver's native /ouster/scan: that is a single ring at one elevation and the maps
         were built from the height-band projection -- matching a ring against a height-band map
         puts the robot in the wrong place."
  else
    record scan FAIL "/scan ${r} Hz"
    why scan "the slam relay is not delivering. It is NOT optional: pointcloud_to_laserscan
         publishes BEST_EFFORT, slam_toolbox subscribes RELIABLE, and incompatible DDS QoS
         delivers ZERO messages with no error anywhere. This probe subscribes RELIABLE on
         purpose, exactly as slam_toolbox does, so it fails the same way slam_toolbox fails.
         If /scan_filtered above is healthy, the relay is the problem; if it is not, look
         upstream."
  fi
fi

r=$(hz /scan_nav)
if ! ge "$r" 1.5 && [ "$STATUS_ONLY" = 0 ]; then
  if ! up scan; then
    record scan_nav BLOCKED "not started: /scan is down"
  elif foreign_blocked scan_nav scan_temporal_filter.py; then
    # A second relay from older startup attempts must not share /scan_nav.
    if [ "$(count_ours 'UTP_RELAY_ROLE=nav')" -gt 0 ] || [ "$(count_foreign 'UTP_RELAY_ROLE=nav')" -gt 0 ]; then
      record scan_nav FAIL "another nav relay owns /scan_nav; inspect it before restarting"
    else
      kill_ours scan_temporal_filter.py
      start_bg python3 "$REPO/safety/scan_temporal_filter.py"
      wait_ready 25 /scan_nav 1.5 "topic:/scan_nav:sensor_msgs.msg:LaserScan:reliable"
      r=$(hz /scan_nav)
    fi
  fi
fi
if [ -z "${S[scan_nav]:-}" ]; then
  ge "$r" 1.5 && record scan_nav ok "/scan_nav ${r} Hz (temporal filter from /scan)" \
    || record scan_nav FAIL "/scan_nav ${r} Hz; check scan_temporal_filter.py"
fi

# ============================================================================================
# STAGE 6 -- the safety mux. It is the ONLY publisher of /cmd_vel; without it nothing drives.
# ============================================================================================
r=$(hz /safety/status)
if ! ge "$r" 5 && [ "$STATUS_ONLY" = 0 ]; then
  if foreign_blocked safety twist_mux_node.py; then
    note "/safety/status silent ($r Hz) -- starting the mux and the arm monitor"
    kill_ours twist_mux_node.py; kill_ours arm_monitor_node.py; sleep 3
    start_bg bash "$REPO/bringup/safety.sh"
    wait_ready 25 /safety/status 5 "topic:/safety/status:std_msgs.msg:String:sensor" "gates:/safety/status"
    r=$(hz /safety/status)
  fi
fi
if [ -z "${S[safety]:-}" ]; then
  if ge "$r" 5; then
    _stow="${P[gate:arm_stowed]:-}"; _estop="${P[gate:estop_latched]:-}"
    record safety ok "/safety/status ${r} Hz, arm_stowed ${_stow:-?}%, estop_latched ${_estop:-?}%"
    if [ -n "$_stow" ] && [ "$_stow" -lt 99 ] 2>/dev/null; then
      _gate_level=FAIL; [ "$MODE" = map ] && _gate_level=WARN
      record safety_gate "$_gate_level" "arm_stowed permits only ${_stow}% of ticks; motion blocked"
      why safety_gate "the arm_stowed gate is fail-closed and it is BLOCKING. Every autonomous
           twist is being discarded by the mux, so Nav2 will plan a perfect path, publish it, and
           the robot will not move -- for the full leg timeout, and then report 'leg timed out':
           a navigation symptom for an interlock cause. Days have gone into the planner for this.
           Restore the arm connection and verify its measured stow pose before software motion.
           Starting a mapping recorder does not open this gate.
           A gate that FLAPS is the expensive case: sampled once it looks fine and still blocks
           most ticks, which is why this is a duty cycle over ${P[gaten]:-0} messages."
    elif [ -n "$_estop" ] && [ "$_estop" -ge 1 ] 2>/dev/null; then
      record safety_gate FAIL "estop_latched on ${_estop}% of ticks"
      why safety_gate "the e-stop is LATCHED. Releasing the physical button is not enough -- the
           latch is cleared with the /safety/clear_estop service. Until then nothing drives, and
           it is correct that nothing drives."
    fi
  else
    record safety FAIL "/safety/status ${r} Hz"
    why safety "the twist mux is not running, and it is the ONLY publisher of /cmd_vel. Nothing
         forwards a command to the chassis, so every source -- teleop, Nav2, the servo -- is dead
         on arrival while every node looks healthy. Start it: bash bringup/safety.sh
         (it starts the arm monitor too; without that /safety/arm_stowed has NO publisher and the
         gate fail-closes forever, which is how the autonomous half of this robot stayed dead
         while teleop kept working)."
  fi
fi

# ============================================================================================
# STAGE 7 -- camera (only --mode full; the press chain is the only thing that needs it)
# ============================================================================================
if [ "$WANT_CAMERA" = 1 ]; then
  r=$(hz /mast_cam/color/camera_info)
  if ! ge "$r" 10 && [ "$STATUS_ONLY" = 0 ]; then
    if foreign_blocked camera realsense2_camera_node; then
      note "camera silent ($r Hz) -- restarting camera.sh"
      kill_ours realsense2_camera_node; kill_ours "$REPO/bringup/camera.sh"; sleep 3
      start_bg bash "$REPO/bringup/camera.sh"
      wait_ready 40 /mast_cam/color/camera_info 10 "topic:/mast_cam/color/camera_info:sensor_msgs.msg:CameraInfo:sensor"
      r=$(hz /mast_cam/color/camera_info)
    fi
  fi
  if [ -z "${S[camera]:-}" ]; then
    if ge "$r" 10; then record camera ok "camera_info ${r} Hz"
    else
      record camera FAIL "camera_info ${r} Hz"
      _sp=""
      for _d in /sys/bus/usb/devices/*/idVendor; do
        [ "$(cat "$_d" 2>/dev/null)" = "8086" ] || continue
        _pp=$(dirname "$_d")
        case "$(cat "$_pp/product" 2>/dev/null)" in *RealSense*) _sp="$(cat "$_pp/speed" 2>/dev/null)";; esac
      done
      if [ -z "$_sp" ]; then
        why camera "the D435 is NOT ENUMERATED on USB at all. Reseat it in a blue USB 3 port."
      elif [ "$_sp" = "480" ]; then
        why camera "the D435 negotiated 480 Mbps -- that is USB 2. config/camera.yaml asks for
             1280x720x30 colour plus 848x480x30 depth, which USB 2 physically cannot carry, so
             librealsense opens NOTHING and loops on xioctl(VIDIOC_S_FMT) errno=5 while the node
             sits there looking alive. RESTARTING WILL NOT HELP. Move it to a blue USB 3 port, or
             run --mode nav, or set UTP_NO_CAMERA=1 and accept that grounding and pressing do not
             work in this session."
      else
        why camera "the link is ${_sp} Mbps so bandwidth is fine: the driver is wedged, or a
             second instance is racing for the device (the loser logs 'No RealSense devices were
             found', which reads as a cable fault and is not one). Check for more than one
             mast_cam node before starting another."
      fi
    fi
  fi
fi

# ============================================================================================
# STAGE 7b -- do the waypoints on disk actually live in the map we are about to load?
# ============================================================================================
# Offline, no ROS, costs nothing, and catches the failure BEFORE the whole stack is up: a
# waypoint recorded in one map means a different physical place in another, because two maps'
# origins are unrelated. nav2_goto.py refuses those legs (correctly) and safety/map_frame.py
# enforces the distinction -- but only once you are standing there with the robot.
if [ "$WANT_NAV" = 1 ]; then
  _wp="$(python3 - "$REPO/maps/waypoints.yaml" "$MAP_NAME" <<'PY' 2>/dev/null
import sys, yaml
try:
    d = yaml.safe_load(open(sys.argv[1])) or {}
except Exception:
    print("UNREADABLE ")
    raise SystemExit(0)
names = sorted({v.get("map_name") for v in d.values()
                if isinstance(v, dict) and v.get("frame") == "map" and v.get("map_name")})
print(("OK " if sys.argv[2] in names else "MISS ") + ",".join(names))
PY
)"
  case "$_wp" in
    OK*)   record waypoints ok "maps/waypoints.yaml has map-frame waypoints in '$MAP_NAME'" ;;
    MISS*) record waypoints WARN "no waypoint is recorded in '$MAP_NAME' (recorded in: ${_wp#MISS })"
           why waypoints "every map-frame waypoint on disk was recorded in a DIFFERENT map
                (${_wp#MISS }), and two maps' origins are unrelated -- the same numbers name a
                different physical place in each. nav2_goto.py will refuse every leg with
                'recorded in map X but the map currently loaded is $MAP_NAME', which is loud and
                correct but only reaches you once the robot is standing there. Either load the
                map they belong to (--map <name>), or record them again while localized in
                '$MAP_NAME': python3 bringup/waypoints.py record <name> --frame map" ;;
    *)     record waypoints WARN "maps/waypoints.yaml could not be read" ;;
  esac
fi

# ============================================================================================
# STAGE 8 -- slam_toolbox. LIFECYCLE: present in `ros2 node list` while completely inactive.
# ============================================================================================
slam_live_mode() {   # what is the running slam_toolbox actually doing?  mapping|localization|""
  timeout 8 ros2 param get /slam_toolbox mode 2>/dev/null | tail -1 | sed 's/.*: *//' | tr -d "\"'"
}
slam_live_map() {
  local v; v="$(timeout 8 ros2 param get /slam_toolbox map_file_name 2>/dev/null | tail -1)"
  v="${v##*: }"; v="${v%\"}"; v="${v#\"}"; v="${v%\'}"; v="${v#\'}"; printf '%s\n' "${v##*/}"
}

if [ "$MODE" != inputs ]; then
_map_seen=$(seen /map)
_slam_state="$(lc_state /slam_toolbox)"

if [ "$MODE" = map ]; then
  # ---- MAPPING ---------------------------------------------------------------------------
  _live="$(slam_live_mode)"
  if [ "$_map_seen" -ge 1 ] && [ "$_slam_state" = active ] && [ "$_live" = mapping ]; then
    record slam ok "MAPPING, lifecycle active, /map published"
  elif [ "$_live" = localization ]; then
    record slam FAIL "a slam_toolbox is running in LOCALIZATION mode, not mapping"
    why slam "something is already publishing /map in localization mode on '$(slam_live_map)'.
         Starting a mapping session on top of it gives two publishers of /map and of map->odom,
         and EXACTLY ONE source may own each. Stop it first: bash bringup/session.sh down"
  elif [ "$_slam_state" = active ] || [ "$_live" = mapping ]; then
    # NEVER restart an active slam_toolbox in mapping mode on a partial reading. The pose graph
    # lives in RAM and is serialized only on request, so an unnecessary restart costs the walk.
    record slam WARN "ACTIVE but not fully confirmed (mode='${_live:-unreadable}', /map seen=$_map_seen) -- NOT restarted"
    why slam "slam_toolbox reports lifecycle 'active' but this script could not confirm it is
         mapping (reading its mode parameter timed out, or /map has not been published yet on a
         session that has only just started). It was deliberately NOT restarted: the pose graph
         exists only in RAM until something serializes it, so a needless restart throws the drive
         away and you walk it again. Confirm by hand, and if it is a drive worth keeping, save it
         before touching anything:
           ros2 param get /slam_toolbox mode
           bash bringup/map_persist.sh save <name>"
  elif [ "$STATUS_ONLY" = 1 ]; then
    record slam FAIL "no mapping session (/map seen=$_map_seen, lifecycle $_slam_state)"
  elif ! up scan; then
    record slam BLOCKED "not started: /scan is down"
    why slam "slam_toolbox subscribes to /scan (RELIABLE). Started against a silent scan it
         builds a map from every scan stacked at the origin -- which is a map-shaped file that
         means nothing -- or simply sits there active and empty. Fix the scan chain first."
  elif ! up chassis; then
    record slam BLOCKED "not started: there is no odom->base_link"
    why slam "without odom->base_link slam_toolbox cannot publish map->odom no matter how good
         the scan is. It would come up active and produce nothing usable. Fix the chassis first."
  elif foreign_blocked slam slam_toolbox; then
    note "starting slam_toolbox in MAPPING mode (config/slam_os0.yaml)"
    kill_ours slam_toolbox; sleep 4
    # PARAMS FILE, NEVER INLINE -p FLAGS. Inline flags silently take stock defaults for
    # do_loop_closing (the map comes out bent and every waypoint inherits the bend) and
    # stack_size_to_use (serializing a building-sized graph dies -- on exactly the map worth
    # keeping).
    start_bg ros2 launch slam_toolbox online_async_launch.py \
      use_sim_time:=false slam_params_file:="$REPO/config/slam_os0.yaml"
    ensure_active /slam_toolbox 90 || note "SLAM activation failed; see $LOG"
      wait_ready 30 /map latched "topic:/map:nav_msgs.msg:OccupancyGrid:latched" "tf:map:odom"
      _map_seen=$(seen /map); _slam_state="$(lc_state /slam_toolbox)"
    if [ "$_map_seen" -ge 1 ] && [ "$_slam_state" = active ]; then
      record slam ok "MAPPING, lifecycle active, /map published"
    else
      record slam FAIL "lifecycle=$_slam_state, /map seen=$_map_seen"
      why slam "slam_toolbox is a LIFECYCLE node: it comes up 'unconfigured', appears in
           'ros2 node list', declares no subscriptions, publishes no /map and no map->odom, and
           is indistinguishable from a hung node. It needs configure THEN activate, which this
           script issued. Read $LOG for the transition failure. Note that its state is compared
           EXACTLY here -- 'inactive' contains the substring 'active', and a grep for it reports
           a dead node as healthy."
    fi
  fi
else
  # ---- LOCALIZATION ----------------------------------------------------------------------
  _miss=""
  for f in pgm yaml posegraph data; do
    [ -s "$REPO/maps/$MAP_NAME.$f" ] || _miss="$_miss .$f"
  done
  if [ -n "$_miss" ]; then
    record slam FAIL "maps/$MAP_NAME missing$_miss -- cannot relocalize into it"
    why slam "a .pgm/.yaml pair is NOT a map you can localize into. slam_toolbox's
         mode: localization relocalizes by DESERIALIZING <name>.posegraph + <name>.data; handed
         only a grid it does NOT error -- it starts a brand-new empty graph at the robot's feet,
         publishes /map, and reports active. The result is a healthy-looking localization in
         which every stored waypoint is meaningless, because 'map' is a fresh-SLAM frame wearing
         the saved map's name. Missing here:$_miss. Either name a map that has all four files
         (ls maps/*.posegraph), or drive one:
           bash bringup/bringup_all.sh --mode map
           bash bringup/map_persist.sh save $MAP_NAME"
  else
    _live="$(slam_live_mode)"; _livemap="$(slam_live_map)"
    if [ "$_map_seen" -ge 1 ] && [ "$_slam_state" = active ] && [ "$_live" = localization ] && [ "$_livemap" = "$MAP_NAME" ]; then
      if tfok map odom; then record slam ok "localizing in '$MAP_NAME', lifecycle active, map->odom present"
      else
        record slam WARN "'$MAP_NAME' loaded but NO map->odom yet"
        why slam "the map is loaded and the node is active, so slam_toolbox simply has no pose
             yet. Give it one: RViz '2D Pose Estimate' (which works ONLY in localization mode and
             is silently ignored while mapping), or python3 bringup/relocalise.py for a global
             search. Do NOT seed from config/slam_os0.yaml's map_start_pose -- that is an ATRIUM
             coordinate and seeding from it converges into the wrong corridor. A global search
             also needs a scan with information in it: inside a lift car with the doors shut the
             scan is four blank walls and the search will still return a confident answer."
      fi
    elif [ "$_map_seen" -ge 1 ] && [ "$_live" = mapping ]; then
      record slam FAIL "a MAPPING session is publishing /map, not localization on '$MAP_NAME'"
      why slam "a mapping session keeps rewriting the map underneath your waypoints, and
           certifying '$MAP_NAME' as loaded while it runs manufactures exactly the provenance
           that waypoints.py and nav2_goto.py exist to trust. Stop it (save it first if it is
           worth keeping: bash bringup/map_persist.sh save <name>), then re-run."
    elif [ "$_map_seen" -ge 1 ] && [ -n "$_livemap" ] && [ "$_livemap" != "$MAP_NAME" ]; then
      record slam FAIL "slam_toolbox is localizing in '$_livemap', not '$MAP_NAME'"
      why slam "two maps' origins are unrelated, so every waypoint would resolve to the wrong
           physical place. Stop the running session first: bash bringup/session.sh down"
    elif [ "$_slam_state" = active ]; then
      # An ACTIVE node whose parameters cannot be read is not evidence that it is wrong. Say so
      # rather than restarting it: /map already being published is not proof the RIGHT map is
      # loaded, and neither is a timed-out `ros2 param get`.
      record slam WARN "ACTIVE but unconfirmed (mode='${_live:-unreadable}', map='${_livemap:-unreadable}') -- NOT restarted"
      why slam "slam_toolbox is active, but this script could not read back WHICH map it holds or
           in which mode. An existing /map is not evidence that the right map is loaded -- it is
           equally true of a still-running mapping session and of a localization session holding
           a DIFFERENT map, and certifying '$MAP_NAME' as loaded in either case manufactures the
           exact provenance waypoints.py and nav2_goto.py were built to trust. Interrogate the
           node, not the topic:
             ros2 param get /slam_toolbox mode
             ros2 param get /slam_toolbox map_file_name
           Then either accept it, or: bash bringup/session.sh down  and re-run this script."
    elif [ "$STATUS_ONLY" = 1 ]; then
      record slam FAIL "no localization session (/map seen=$_map_seen, lifecycle $_slam_state)"
    elif ! up scan; then
      record slam BLOCKED "not started: /scan is down"
      why slam "slam_toolbox subscribes RELIABLE to /scan. With no scan it activates, publishes a
           map and never matches anything, so the pose is wherever the seed said it was -- which
           is the worst of all outcomes because it looks like a working localization."
    elif ! up chassis; then
      record slam BLOCKED "not started: there is no odom->base_link"
      why slam "without odom->base_link slam_toolbox cannot publish map->odom, and the failure
           surfaces three layers away as 'localization is wrong in RViz'."
    elif foreign_blocked slam slam_toolbox; then
      if [ -z "${SEED_POSE:-}" ]; then
        echo "Set SEED_POSE=x,y,yaw to the robot's known pose in '$MAP_NAME'; refusing the stale config seed." >&2
        exit 2
      fi
      note "starting slam_toolbox in LOCALIZATION mode on '$MAP_NAME'"
      kill_ours slam_toolbox; sleep 4
      # Same params file as mapping -- a map built with one set of scan-matcher settings and
      # localized with another matches worse for no reason -- overriding only what must differ.
      # --ros-args after --params-file wins, so the override is the last word.
      # FREEZE THE PUBLISHED MAP FOR THE WHOLE LOCALIZATION SESSION.
      #
      # chmod a-w on maps/*.pgm protects the FILE. It does not protect the map Nav2 actually uses,
      # which is the /map TOPIC, and slam_toolbox regenerates that every map_update_interval
      # (5.0 s in config/slam_os0.yaml) from its pose graph plus the scans it has taken since --
      # in localization mode as well as mapping. Measured 2026-09-07 on floor2: the live grid had
      # 2972 occupied cells against the saved file's 2884. Eighty-eight walls that exist only at
      # runtime, and the operator watched one of them appear where the robot was standing, after
      # which Nav2's static layer treated it as an obstacle and planned around a wall that is not
      # in the building.
      #
      # A huge interval means the map is published once on activation and never regenerated, so
      # what Nav2 sees for the rest of the session is exactly what was loaded from disk. The 5.0 s
      # default stays correct for mapping, which is why this is overridden HERE and not in the
      # shared params file. UTP_MAP_UPDATE_INTERVAL exists for anyone who deliberately wants the
      # live behaviour back.
      start_bg ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
        --params-file "$REPO/config/slam_os0.yaml" -p use_sim_time:=false -p mode:=localization \
        -p map_update_interval:="${UTP_MAP_UPDATE_INTERVAL:-1000000.0}" \
        -p map_file_name:="$REPO/maps/$MAP_NAME" -p map_start_pose:="[$SEED_POSE]"
      ensure_active /slam_toolbox 90 || note "SLAM activation failed; see $LOG"
      wait_ready 30 /map latched "topic:/map:nav_msgs.msg:OccupancyGrid:latched" "tf:map:odom"
      _map_seen=$(seen /map); _slam_state="$(lc_state /slam_toolbox)"
      if [ "$_map_seen" -ge 1 ] && [ "$_slam_state" = active ]; then
        if tfok map odom; then record slam ok "localizing in '$MAP_NAME', lifecycle active, map->odom present"
        else
          record slam WARN "'$MAP_NAME' loaded but NO map->odom yet"
          why slam "slam_toolbox is active on the saved map but has not matched into it. Set the
               pose by hand (RViz 2D Pose Estimate) or run python3 bringup/relocalise.py --check
               and expect >=80% before driving. Nav2 below will not come up without map->odom:
               its costmaps never activate, so bt_navigator stays down and every goal is
               rejected."
        fi
      else
        record slam FAIL "lifecycle=$_slam_state, /map seen=$_map_seen"
        why slam "slam_toolbox is a LIFECYCLE node and needs configure THEN activate; both were
             issued. Read $LOG. Compare the state EXACTLY -- 'inactive' contains 'active'."
      fi
    fi
  fi
fi

else
  record slam skip "inputs-only: no mapping or localization"
fi

# Named waypoint commands require provenance from this actual localization session.
if [ "$WANT_NAV" = 1 ] && [ "$STATUS_ONLY" = 0 ] && up slam && tfok map odom; then
  if timeout 12 python3 "$REPO/bringup/startup_mark_map.py" "$MAP_NAME"; then
    record map_session ok "saved map matched to the live SLAM session"
  else
    record slam FAIL "could not verify live map provenance; Nav2 startup blocked"
  fi
fi

# ============================================================================================
# STAGE 9 -- Nav2. Every server is a lifecycle node, and the action is advertised BEFORE
# activation, so neither the node list nor the action list can see this failure.
# ============================================================================================
if [ "$WANT_NAV" = 1 ]; then
  # KEEPOUT MASK -- opt-in by FILE EXISTENCE, never by a flag. maps/<map>_keepout.yaml is the
  # operator's hand-painted "the planner may not route through here"; it is the half of the
  # 2026-09-06 runaway that allow_unknown: false cannot cover, because the areas that must be
  # forbidden are MAPPED FREE (docs/KEEPOUT.md). Two extra lifecycle nodes serve it.
  #
  # It has to be off when the file is absent, and off in a way that cannot half-work: the
  # lifecycle manager waits on a bond from every name it is given, so a mask server that is named
  # but not launched does not degrade navigation, it HANGS the whole activation and leaves the
  # robot with no planner at all. ranger_nav.launch.py derives both the node list and the launch
  # conditions from one os.path.isfile on this same path, so the two cannot disagree.
  KEEPOUT="$REPO/maps/${MAP_NAME}_keepout.yaml"
  [ -f "$KEEPOUT" ] || KEEPOUT=""
  nav_probe() {
    _act=$(timeout 12 ros2 action list 2>/dev/null | grep -c navigate_to_pose)
    _bt=$(lc_state /bt_navigator); _pl=$(lc_state /planner_server)
    _ct=$(lc_state /controller_server); _bh=$(lc_state /behavior_server)
    _km=$(lc_state /filter_mask_server); _ki=$(lc_state /costmap_filter_info_server)
    _dups=$(count_ours bt_navigator); _pdups=$(count_ours planner_server)
    [ "$_pdups" -gt "$_dups" ] && _dups="$_pdups"
    return 0
  }
  nav_healthy() {
    # The keepout servers are checked ONLY when a mask exists, and they are checked on lifecycle
    # STATE like everything else here: a map_server that loaded no image still appears in
    # `ros2 node list` and still advertises, so the node name proves nothing. A mask that is
    # present on disk but not ACTIVE is the dangerous case -- the operator painted a forbidden
    # area, the run looks healthy, and the planner routes straight through it.
    if [ -n "$KEEPOUT" ] && { [ "$_km" != active ] || [ "$_ki" != active ]; }; then
      return 1
    fi
    [ "${_act:-0}" -ge 1 ] && [ "${_dups:-0}" -le 1 ] \
      && [ "$_bt" = active ] && [ "$_pl" = active ] && [ "$_ct" = active ] && [ "$_bh" = active ]
  }
  nav_probe
  if ! nav_healthy && [ "$STATUS_ONLY" = 0 ]; then
    if [ "${S[slam]:-}" != "ok" ] && [ "${S[slam]:-}" != "WARN" ]; then
      record nav2 BLOCKED "not started: slam is not up"
      why nav2 "Nav2's costmaps need map->odom. Launched without it they never activate, the
           lifecycle manager's transition never completes, bt_navigator stays down, and every
           goal comes back 'rejected in 0.0s' -- which reads as a Nav2 bug and is a SLAM
           problem. Fix the slam row first."
    elif ! tfok map odom; then
      record nav2 BLOCKED "not started: map->odom does not exist yet"
      why nav2 "slam_toolbox is up but has not localized into '$MAP_NAME', so there is no
           map->odom. Nav2 launched now would come up, fail to activate its costmaps, and hand
           back 'rejected in 0.0s' for every goal -- a Nav2-shaped symptom with a localization
           cause. Give slam a pose first: RViz '2D Pose Estimate' (localization mode only), or
             python3 bringup/relocalise.py --check     # want >=80%
           then re-run this script."
    elif ! up scan_nav; then
      record nav2 BLOCKED "not started: /scan_nav is down"
      why nav2 "the global and local costmaps both read /scan_nav. Without it Nav2 activates,
           plans across a static map only, and cannot see a single obstacle it did not already
           know about."
    elif foreign_blocked nav2 bt_navigator; then
      [ "${_dups:-0}" -le 1 ] || note "$_dups Nav2 stacks are running and none is usable -- tearing down ALL of them"
      note "Nav2 not active (action=$_act bt=$_bt planner=$_pl controller=$_ct behavior=$_bh procs=$_dups) -- launching one"
      # Tear down the NODES, not just the launch wrapper: killing `ros2 launch` alone orphans the
      # servers it started, and those orphans are exactly what the next launch stacks on top of.
      for _p in ranger_nav.launch bt_navigator planner_server controller_server behavior_server \
                smoother_server velocity_smoother waypoint_follower lifecycle_manager \
                filter_mask_server costmap_filter_info_server; do
        kill_ours "$_p"
      done
      sleep 4
      RUNTIME=/tmp/utp_nav2_params_runtime.yaml
      sed -E "s#(default_nav_to_pose_bt_xml:).*#\1 \"$REPO/nav2_bringup/behavior_trees/navigate_to_pose_no_spin.xml\"#; \
              s#(default_nav_through_poses_bt_xml:).*#\1 \"$REPO/nav2_bringup/behavior_trees/navigate_through_poses_no_spin.xml\"#" \
          "$REPO/nav2_bringup/nav2_params_os0_map.yaml" > "$RUNTIME"
      if ! grep -q "$REPO/nav2_bringup/behavior_trees" "$RUNTIME"; then
        record nav2 FAIL "behaviour-tree path rewrite failed"
        why nav2 "the absolute default_nav_to_pose_bt_xml path in nav2_params_os0_map.yaml points
             at a sim checkout. Unresolved, bt_navigator loads NO tree, the lifecycle manager
             aborts, and Nav2 comes up looking healthy while navigate_to_pose never works."
      else
        if [ -n "$KEEPOUT" ]; then
          note "keepout mask maps/${MAP_NAME}_keepout.yaml -- filter_mask_server + costmap_filter_info_server join the Nav2 lifecycle group"
        else
          note "no maps/${MAP_NAME}_keepout.yaml -- no keepout filter; nothing on this floor is forbidden"
        fi
        _keepout_args=()
        [ -n "$KEEPOUT" ] && _keepout_args+=("keepout:=$KEEPOUT")
        start_bg ros2 launch "$REPO/nav2_bringup/ranger_nav.launch.py" \
          params_file:="$RUNTIME" localization:=slam "${_keepout_args[@]}"
        _nav_deadline=$((SECONDS + 60))
        while :; do
          nav_probe
          nav_healthy && break
          [ "$SECONDS" -ge "$_nav_deadline" ] && break
          note "waiting for Nav2 lifecycle activation"
          sleep 1
        done
      fi
    fi
  fi
  if [ -z "${S[nav2]:-}" ]; then
    if nav_healthy; then
      if [ -n "$KEEPOUT" ]; then
        record nav2 ok "navigate_to_pose + bt_navigator/planner/controller/behavior ACTIVE, keepout mask '${MAP_NAME}_keepout' ACTIVE"
      else
        record nav2 ok "navigate_to_pose + bt_navigator/planner/controller/behavior ACTIVE, no keepout mask"
      fi
    else
      record nav2 FAIL "action=$_act bt=$_bt planner=$_pl controller=$_ct behavior=$_bh keepout=$_km/$_ki procs=$_dups"
      if [ -n "$KEEPOUT" ] && { [ "$_km" != active ] || [ "$_ki" != active ]; }; then
        why nav2 "maps/${MAP_NAME}_keepout.yaml EXISTS, so the operator has painted areas the
             planner must not enter, but filter_mask_server=$_km and
             costmap_filter_info_server=$_ki -- they are not both active, so the KeepoutFilter in
             both costmaps is receiving nothing and every painted area is being planned through as
             ordinary free floor. This is a fail-OPEN direction: navigation will look completely
             healthy. Read $LOG for the map_server load error (a mask whose .pgm is missing or
             whose header GIMP rewrote unreadably fails HERE, in configure), then
               python3 bringup/keepout_mask.py --check
             which compares the mask on disk to the live /map."
      elif [ "${_dups:-0}" -gt 1 ]; then
        why nav2 "there are $_dups bt_navigator/planner_server processes: TWO NAV2 STACKS are
             running, from repeated 'ros2 launch' calls. Two lifecycle_manager instances contend
             for the same nodes, the activation NEVER COMPLETES, and every goal comes back
             'rejected in 0.0s'. Kill both and start exactly one -- and tear down the NODES, not
             just the launch wrapper, because killing the wrapper orphans the servers and the
             next launch stacks on top of them."
      elif [ "${_act:-0}" -ge 1 ]; then
        why nav2 "Nav2's nodes are present and /navigate_to_pose IS advertised, but the servers
             are INACTIVE (bt=$_bt planner=$_pl controller=$_ct behavior=$_bh). This failure is
             silent in three ways at once: 'ros2 node list' shows a healthy Nav2; 'ros2 action
             list' shows the action, because the server is advertised BEFORE activation; and
             RViz shows an empty world, because inactive costmap nodes publish nothing -- which
             reads as an RViz configuration problem and is not one. Only 'ros2 lifecycle get
             /bt_navigator' sees it. Read the goal status word too: REJECTED means the server
             would not accept the goal at all (usually lifecycle or config), ABORTED means it
             tried and failed. Check $LOG for a transition failure, and confirm map->odom exists
             -- without it the costmaps never come up and bt_navigator stays down."
      else
        why nav2 "Nav2 launched but /navigate_to_pose never appeared. Its servers are LIFECYCLE
             nodes and can sit unconfigured while still showing in 'ros2 node list', which is why
             this guards on the ACTION and the lifecycle STATE and never on the node name. Read
             $LOG for a transition failure, and confirm map->odom exists."
      fi
    fi
  fi
fi

# ============================================================================================
# STAGE 10 -- the arm (--mode full only). Reported, never configured: the hand-eye calibration
# assumes tcp_offset ZERO and writing one makes every press land 172 mm short, silently.
# ============================================================================================
if [ "$NEED_ARM" = 1 ]; then
  if [ "${S[net_arm]:-}" != "ok" ]; then
    record arm FAIL "192.168.1.221 unreachable"
  elif [ -x "$REPO/.venv-arm/bin/python" ]; then
    if _arm="$(timeout 15 "$REPO/.venv-arm/bin/python" "$REPO/bringup/arm_tool.py" 2>&1 | tr '\n' ' ')"; then
      record arm ok "SDK tool readback succeeded; details in log"
    else
      record arm FAIL "SDK tool readback failed or tool differs from arm_tool.py expectations"
    fi
    [ "$STATUS_ONLY" = 1 ] || printf '%s\n' "$_arm" >> "$LOG"
    note "arm tool reported only; no offset or load changed"
  else
    record arm WARN "reachable but .venv-arm/bin/python is missing, cannot read its state"
    why arm "the xArm SDK lives in its own venv (no rclpy, no system site-packages) and it is not
         installed here. Anything that commands the arm runs under .venv-arm/bin/python; see
         docs/LAPTOP_SETUP.md."
  fi
fi

# ============================================================================================
# REPORT
# ============================================================================================
echo
printf '  %-14s %-7s %s\n' COMPONENT STATE DETAIL
printf '  %-14s %-7s %s\n' "--------------" "-------" "------------------------------------------"
bad=0; warn=0; blocked=0
for row in "${RESULT[@]}"; do
  IFS='|' read -r n s d <<<"$row"
  printf '  %-14s %-7s %s\n' "$n" "$s" "$d"
  case "$s" in
    FAIL)    bad=$((bad+1)) ;;
    BLOCKED) blocked=$((blocked+1)) ;;
    WARN)    warn=$((warn+1)) ;;
  esac
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

echo "  elapsed: $((SECONDS - START_SECONDS)) seconds"
if [ $((bad + blocked)) -gt 0 ]; then
  echo "  $bad component(s) DOWN, $blocked not started because their input was down."
  echo "  A stage is never launched into a silent input: a node started before its input exists"
  echo "  is ALIVE AND SILENT FOREVER and reports nothing. Fix the top failure, then re-run --"
  echo "  this script is idempotent and will not restart anything that is already healthy:"
  echo "      bash bringup/bringup_all.sh --mode $MODE --map $MAP_NAME"
  exit 1
fi
[ "$warn" -gt 0 ] && echo "  $warn warning(s) -- usable, but read them."

echo "  everything --mode $MODE needs is up. Next:"
case "$MODE" in
  inputs) echo "  Inputs checked. No SLAM, localization or Nav2 started." ;;
  map)
    echo "      bash bringup/map_insurance.sh start <name>   # START THIS BEFORE THE DRIVE."
    echo "        slam_toolbox holds the pose graph in RAM and serializes only on request; a"
    echo "        session that dies mid-drive leaves nothing on disk and you walk it again."
    echo "      python3 bringup/map_watch.py                 # another terminal, while driving"
    echo "      # stow the arm, hold the RC, select DualAckermann BEFORE moving, <=0.25 m/s,"
    echo "      # broad turns, pause after each, and close the loop by returning past the start"
    echo "      # via a different route. Glass doors will not appear -- mark them by hand."
    echo "      bash bringup/map_persist.sh save $MAP_NAME   # grid + pose graph + .data + .loaded_map"
    ;;
  nav|full)
    echo "      python3 bringup/relocalise.py --check        # want >=80% before driving"
    echo "      python3 bringup/waypoints.py list            # must be recorded in map '$MAP_NAME'"
    ;;
esac
exit 0
