#!/usr/bin/env bash
# LAB GATE LADDER — run these in order. Do not skip a gate because you are in a hurry;
# every one of them exists because something silently failed in a way that looked healthy.
#
#   bash bringup/lab_gates.sh            # run every gate, stop at the first failure
#   bash bringup/lab_gates.sh 3          # run gate 3 only
#   bash bringup/lab_gates.sh 3 7        # run gates 3..7
#
# Gates 0-2 move NOTHING. Gate 3 is the first that drives the base — stand clear, hand on the
# RC transmitter. Gates 6+ move the arm.
#
# Written 2026-08-31 from EXPERIMENT_LOG.md. Each gate names the failure it is guarding against.
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
source bringup/env.sh 2>/dev/null || { echo "cannot source bringup/env.sh"; exit 1; }

PASS=0; FAIL=0
ok()   { echo "   PASS  $*"; PASS=$((PASS+1)); }
bad()  { echo "   FAIL  $*"; FAIL=$((FAIL+1)); }
note() { echo "         $*"; }
gate() { echo; echo "=== GATE $1 — $2"; }

FROM=${1:-0}; TO=${2:-99}
want() { [ "$1" -ge "$FROM" ] && [ "$1" -le "$TO" ]; }

# ---------------------------------------------------------------- 0. the cable
if want 0; then
gate 0 "physical link (30 s, nothing moves)"
# GUARDS: the USB-ethernet cable carries the lidar (.119), the xArm (.221) AND the router (.1).
# It dropped mid-session on 2026-08-30 and took all three down at once. The adapter stays
# enumerated when this happens, so `lsusb` looks perfectly healthy — carrier is the check.
IFACE=$(ip -brief link show | awk '/^enx/{print $1; exit}')
if [ -z "$IFACE" ]; then bad "no enx* USB-ethernet interface found"; else
  if ip -brief link show "$IFACE" | grep -q "NO-CARRIER"; then
    bad "$IFACE has NO-CARRIER — reseat the cable, then strain-relieve it"
  else ok "$IFACE carrier up"; fi
fi
for ip_addr in 192.168.1.119:lidar 192.168.1.1:router; do
  a=${ip_addr%%:*}; n=${ip_addr##*:}
  if ping -c1 -W1 "$a" >/dev/null 2>&1; then ok "$n reachable ($a)"; else bad "$n UNREACHABLE ($a)"; fi
done
# THE ARM IS NOT PART OF EVERY SESSION, so it must not fail the gate that every session runs.
# This used to sit in the loop above alongside the lidar and the router, so a mapping or nav-only
# run -- where the arm is deliberately stowed and POWERED OFF -- failed gate 0 on a device it
# never touches, and session.sh (which runs `lab_gates.sh 0 2`) died there. It also reads like a
# cable fault on the one cable that carries the lidar too, which sends you looking in the wrong
# place. Fatal only when the ladder actually reaches the arm: gates 6+ move it, and UTP_NEED_ARM=1
# forces it for anything that presses.
ARM_NEEDED="${UTP_NEED_ARM:-0}"
[ "$TO" -ge 6 ] && ARM_NEEDED=1
if ping -c1 -W1 192.168.1.221 >/dev/null 2>&1; then
  ok "xArm reachable (192.168.1.221)"
elif [ "$ARM_NEEDED" = "1" ]; then
  bad "xArm UNREACHABLE (192.168.1.221) — gates 6+ move the arm and cannot run without it"
else
  note "xArm unreachable (192.168.1.221) — NOT a failure for gates $FROM..$TO; it is normal for"
  note "mapping and nav-only work. Two consequences: nothing can press, and the arm_stowed gate"
  note "has no MEASURED evidence so it fail-closes and the base will refuse autonomous twists."
  note "If no arm is fitted or powered: UTP_ARM_BACKEND=absent bash bringup/safety.sh — and record"
  note "that against any trial, because a gate satisfied by declaration is not one by measurement."
fi
fi

# ---------------------------------------------------------------- 1. sensing
if want 1; then
gate 1 "sensing alive and correctly shaped (nothing moves)"
# GUARDS: the OS0 udp_dest trap — a sensor configured to stream to another host answers HTTP
# "RUNNING" and sends its cloud to nobody.
# MEASURE THE RATE WITH ONE COUNTING SUBSCRIBER, NOT `ros2 topic hz`, AND NOT ONE NODE PER TOPIC.
#
#   * `ros2 topic hz` is not trustworthy on this stack: it has reported 1.7 Hz and 10.0 Hz for the
#     same topic minutes apart against a repeatable 6.4 Hz. Worse for a gate, it reports SOMETHING
#     for a topic that is barely alive, and the old test here was `[ -n "$R" ]` — a check that
#     passes on any output at all, including a rate far below what the scan matcher needs.
#   * A FRESH NODE PER TOPIC MEASURES DDS DISCOVERY, NOT RATE. A just-created node has not yet
#     discovered the publisher, so the first seconds of every window are empty; that mistake
#     produced 0.00 Hz readings for healthy topics twice on 2026-09-05. One node subscribes to
#     everything, spins ~3 s so discovery completes, RESETS the counters, and only then counts.
#   * /scan and /scan_nav are subscribed RELIABLE, exactly as slam_toolbox and Nav2's costmaps do.
#     A BEST_EFFORT probe would read them green while the RELIABLE consumers received nothing —
#     which is the QoS mismatch this gate exists to catch, hidden by the gate itself.
SENSE="$(timeout 45 python3 - <<'PY' 2>/dev/null
import math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2

REL = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
rclpy.init()
n = Node("utp_gate1_probe")
count, last = {}, {}
def sub(topic, typ, qos):
    count[topic] = 0
    def cb(m):
        count[topic] += 1
        last[topic] = m
    n.create_subscription(typ, topic, cb, qos)
sub("/ouster/points", PointCloud2, qos_profile_sensor_data)
sub("/ouster/points_clean", PointCloud2, qos_profile_sensor_data)
sub("/scan_filtered", LaserScan, qos_profile_sensor_data)
sub("/scan", LaserScan, REL)
sub("/scan_nav", LaserScan, REL)
t0 = time.time()
while time.time() - t0 < 3.0:            # discovery; nothing counted here is believed
    rclpy.spin_once(n, timeout_sec=0.02)
for k in count:
    count[k] = 0
t1 = time.time()
while time.time() - t1 < 4.0:
    rclpy.spin_once(n, timeout_sec=0.02)
el = max(time.time() - t1, 1e-6)
for k in sorted(count):
    print(f"{k} {count[k] / el:.2f}")
m = last.get("/scan")
if m is None:
    print("beams 0 0")
else:
    print("beams %d %d" % (sum(1 for r in m.ranges
                               if math.isfinite(r) and m.range_min < r < m.range_max),
                           len(m.ranges)))
n.destroy_node()
try:
    rclpy.shutdown()
except Exception:
    pass
PY
)"
val()  { printf '%s' "$SENSE" | awk -v t="$1" '$1==t{print $2; exit}'; }
gehz() { LC_ALL=C awk -v a="${1:-0}" -v b="${2:-0}" 'BEGIN{exit !(a+0 >= b+0)}'; }

R=$(val /ouster/points)
gehz "$R" 1.5 && ok "/ouster/points @ ${R:-0.00} Hz" \
  || bad "/ouster/points @ ${R:-0.00} Hz — driver down, or the udp_dest trap (a sensor streaming to another host still answers HTTP \"RUNNING\")"
R=$(val /ouster/points_clean)
gehz "$R" 1.5 && ok "/ouster/points_clean @ ${R:-0.00} Hz" \
  || bad "/ouster/points_clean @ ${R:-0.00} Hz — safety/cloud_artifact_filter.py is not running or its input is silent"
R=$(val /scan_filtered)
gehz "$R" 1.5 && ok "/scan_filtered @ ${R:-0.00} Hz" \
  || bad "/scan_filtered @ ${R:-0.00} Hz — pointcloud_to_laserscan down, OR it cannot transform into base_link (see gate 2: without base_link->os_lidar it silently drops EVERY cloud and sits at exactly 0.00 Hz)"
# GUARDS: the QoS mismatch. p2l publishes BEST_EFFORT, slam_toolbox subscribes RELIABLE —
# incompatible in DDS, zero messages delivered, NO ERROR ANYWHERE.
R=$(val /scan)
if gehz "$R" 6; then ok "/scan @ ${R} Hz (relay running, RELIABLE as slam_toolbox reads it)"
elif gehz "$R" 1.5; then
  bad "/scan @ ${R} Hz — below 6. slam_toolbox searches at 2.0 deg; at wz_max 0.8 rad/s consecutive scans are >20 deg apart and the pose slides through every turn"
else
  bad "/scan @ ${R:-0.00} Hz — start bringup/scan_relay.py (NOT optional: p2l is BEST_EFFORT, slam_toolbox is RELIABLE, and the mismatch delivers zero messages silently)"
fi
R=$(val /scan_nav)
gehz "$R" 1.5 && ok "/scan_nav @ ${R} Hz (RELIABLE, as Nav2's costmaps read it)" \
  || bad "/scan_nav @ ${R:-0.00} Hz — Nav2's costmaps read /scan_nav, not /scan; without it Nav2 plans over the static map and cannot see a single new obstacle"
VALID=$(printf '%s' "$SENSE" | awk '$1=="beams"{print $2; exit}')
if [ -n "$VALID" ] && [ "$VALID" -gt 400 ] 2>/dev/null; then
  ok "/scan has $VALID valid beams (OS0 projection healthy; the A1M8 gave 44)"
else
  bad "/scan has only ${VALID:-0} valid beams — 2D SLAM will fail as it did on 2026-08-25"
fi
fi

# ---------------------------------------------------------------- 2. TF + safety
if want 2; then
gate 2 "TF chain and the deadman (nothing moves)"
# GREP FOR THE TRANSFORM, DO NOT TEST tf2_echo'S EXIT CODE. tf2_echo never returns on its own, so
# `timeout 6 ... && ok || bad` reported MISSING for every transform on the robot, including ones
# session.sh had just confirmed present two lines earlier. Measured: exit code 124 with a healthy
# odom->base_link. A "Translation:" line in the output is positive proof the lookup succeeded; the
# first line is a "frame does not exist" notice printed while the listener buffer fills, so read
# several lines before deciding.
tf() {
  local out
  out="$(timeout 8 ros2 run tf2_ros tf2_echo "$1" "$2" 2>&1 | head -20)"
  printf '%s\n' "$out" | grep -q 'Translation:' && ok "TF $1 -> $2" || bad "TF $1 -> $2 MISSING"
}
tf odom base_link      # needs ranger launched with publish_odom_tf:=true (NOT the default)
tf base_link os_sensor
# THE ONE THAT KILLS QUIETLY. pointcloud_to_laserscan runs with target_frame:=base_link, so it
# must resolve base_link -> os_lidar (the cloud's own frame) for EVERY cloud. base_link->os_sensor
# is published by bringup/lidar3d.sh from config/ouster.yaml's `mount` block and by nothing else;
# os_sensor->os_lidar comes from the driver. Launch `ros2 launch ouster_ros driver.launch.py`
# directly and the mount half never appears — then p2l DROPS EVERY CLOUD with no error, /scan sits
# at exactly 0.00 Hz, and every node in the graph reports healthy. Start the lidar the supported
# way: bash bringup/lidar3d.sh
tf base_link os_lidar
if [ "${UTP_NO_CAMERA:-0}" = "1" ]; then
  note "TF base_link -> mast_cam_link SKIPPED (UTP_NO_CAMERA=1) -- pressing will not work"
else
  tf base_link mast_cam_link
fi
# GUARDS: two publishers of odom->base_link is the 2026-08-21b two-publishers-on-/map bug again.
N=$(timeout 8 ros2 topic info /tf 2>/dev/null | awk '/Publisher count/{print $3}')
note "/tf publisher count: ${N:-?} (each extra one is a candidate duplicate — check if TF flickers)"
# GUARDS: the deadman that never existed. safety.yaml gates nav+servo on /safety/enable and
# NOTHING in the repo published it, so every autonomous command was correctly discarded and the
# system looked dead while behaving exactly as designed.
# Counting subscriber, not `ros2 topic hz`: same reason as gate 1, and here it matters twice
# over, because a deadman that is publishing INTERMITTENTLY is the dangerous reading and a
# tool that has reported 1.7 and 10.0 Hz for one topic cannot tell you that.
R=$(timeout 30 python3 - <<'PY' 2>/dev/null
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
rclpy.init()
n = Node("utp_gate2_enable_probe")
c = [0]
n.create_subscription(Bool, "/safety/enable", lambda m: c.__setitem__(0, c[0] + 1), 10)
t0 = time.time()
while time.time() - t0 < 3.0:            # discovery, not measurement
    rclpy.spin_once(n, timeout_sec=0.05)
c[0] = 0
t1 = time.time()
while time.time() - t1 < 3.0:
    rclpy.spin_once(n, timeout_sec=0.05)
r = c[0] / max(time.time() - t1, 1e-6)
print(f"{r:.1f}" if r > 0.5 else "")
n.destroy_node()
try:
    rclpy.shutdown()
except Exception:
    pass
PY
)
# Only a FAILURE if safety.yaml actually gates something on it. The operator runs supervised with a
# hand on the physical e-stop and requires_enable: false on nav and servo; there a silent
# /safety/enable changes nothing and must not block the ladder. estop and arm_stowed are untouched.
if [ -n "$R" ]; then
  ok "/safety/enable publishing @ ${R} Hz"
elif grep -qE '^[[:space:]]*requires_enable:[[:space:]]*true' "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/config/safety.yaml" 2>/dev/null; then
  bad "/safety/enable SILENT — a source in safety.yaml requires it; every autonomous command is dropped"
else
  ok "/safety/enable silent, and no source requires it (supervised, operator on the e-stop)"
fi
fi

# ---------------------------------------------------------------- 3. chassis (MOVES)
if want 3; then
gate 3 "chassis characterisation — THE ROBOT WILL MOVE. Hand on the RC."
# GUARDS: `Limits` had v_min=0.06 but NO w_min. Proportional rotation drove w below the ~0.20 rad/s
# the 4WS chassis needs, so turns livelocked below the exit tolerance and vx never started. This is
# "wheels rotating, robot isn't moving", the 90 s door livelock, and "it didn't move, it stared".
# The fix (w_min=0.20) is VERIFIED IN SIM ONLY. The sim uses a hard cutoff; the real chassis has a
# re-steer COST, not a cliff. THE LOWEST wz THAT STILL ROTATES THE BODY *IS* w_min.
note "run these one at a time, watch the body, and record which actually rotate:"
note "  python3 bringup/characterise_twist.py --go --wz 0.30   # expect rotation"
note "  python3 bringup/characterise_twist.py --go --wz 0.20    # expect rotation (scale 0.59-0.80 before)"
note "  python3 bringup/characterise_twist.py --go --wz 0.12    # expect little or nothing"
note "  python3 bringup/characterise_twist.py --go --wz 0.08    # expect nothing"
note "then set Limits.w_min to the LOWEST value that still turned the body, and rerun gate 3."
note "GATE 3 IS MANUAL — it cannot be automated because the pass criterion is your eyes on the robot."
fi

# ---------------------------------------------------------------- 4. localisation (MOVES)
if want 4; then
gate 4 "map + localisation — the robot will drive"
if timeout 6 ros2 topic echo /map --once >/dev/null 2>&1; then ok "/map is being published"; else bad "/map absent — slam_toolbox not configured+activated?"; fi
# GUARDS: slam_toolbox in Jazzy is a LIFECYCLE node and starts `unconfigured`. It has no
# subscribers beyond /parameter_events, scan_topic does not read back, no /map and no map->odom
# appear — indistinguishable from a hung node.
S=$(timeout 6 ros2 lifecycle get /slam_toolbox 2>/dev/null | head -1)
[ -n "$S" ] && note "slam_toolbox lifecycle state: $S  (must be 'active')"
tf map odom
note "drive a short loop and confirm the map does not smear and the pose does not jump."
fi

# ---------------------------------------------------------------- 5. perception -> 3D
if want 5; then
gate 5 "grounding and the depth lift (nothing moves)"
# GUARDS: depth-to-colour misalignment presents as "grounding is right but the 3D point is wrong",
# which is very easily misdiagnosed as hand-eye error.
python3 bringup/check_depth_alignment.py 2>/dev/null && ok "depth/colour alignment check passed" \
  || bad "depth alignment check failed or unavailable — see docs/CALIBRATION.md"
note "stand the robot in front of a real door control, then:"
note "  python3 bringup/find_control.sh          # detector must return a box ON the control"
note "  python3 bringup/check_calib.py           # 3D point vs a tape-measured truth"
note "ACCEPT ONLY IF the lifted 3D point is within ~2 cm of where you measure the control to be."
fi

# ---------------------------------------------------------------- 6. arm (MOVES THE ARM)
if want 6; then
gate 6 "arm — THE ARM WILL MOVE. Clear the workspace."
# GUARDS: H3 is RED in the gate table — the xArm SDK Cartesian API is MILLIMETRES and our stack is
# METRES. A 1000x error is not subtle, but it is silent until something moves.
note "  python3 bringup/arm_workspace.py         # reach envelope, no surprises"
note "  python3 bringup/handeye_verify.py        # H5 — UNCALIBRATED as of the last log entry"
note "  python3 bringup/check_press_safe.py      # press pose without contact"
note "MEASURE THE RISER (S2) and put it in base_link -> link_base. Unmeasured means every press"
note "misses vertically by exactly the riser height, and it will look like a calibration bug."
fi

# ---------------------------------------------------------------- 7. the loop
if want 7; then
gate 7 "the pipeline itself"
# GUARDS: Config.load() with no argument silently loads the SIM repo's config, so this laptop's
# single-GPU detector overrides were dead. run_trial.py now defaults to config/pipeline/.
python3 - <<'PY' && ok "pipeline config resolves to the rover-laptop copy" || bad "pipeline config check failed"
import os, sys
from pathlib import Path
repo = Path(__file__).resolve().parents[0] if False else Path.cwd()
cfgdir = Path(os.environ.get("UTP_CONFIG_DIR", repo / "config" / "pipeline"))
assert (cfgdir / "methods.yaml").is_file(), f"no methods.yaml in {cfgdir}"
assert (cfgdir / "detectors.yaml").is_file(), f"no detectors.yaml in {cfgdir}"
import re
# strip trailing comments first — the override lines legitimately EXPLAIN cuda:1 in prose
lines = [l.split("#", 1)[0] for l in (cfgdir / "detectors.yaml").read_text().splitlines()]
bad_dev = [l.strip() for l in lines if re.search(r"cuda:[1-9]", l)]
assert not bad_dev, f"single-GPU laptop but config names another device: {bad_dev}"
print(f"   config dir: {cfgdir}")
PY
# GUARDS: the VLM endpoint is a university HPC service and the test site's connectivity is unknown.
bash bringup/check_llm.sh >/dev/null 2>&1 && ok "VLM endpoint reachable from here" \
  || bad "VLM endpoint UNREACHABLE — campus network / VPN? ours+direct_vlm cannot run"
note "then, in order:"
note "  python3 bringup/run_trial.py --method ours --dry-run    # every stage runs, nothing moves"
note "  python3 bringup/run_trial.py --method passive           # the control: must NOT open a door"
note "  python3 bringup/run_trial.py --method ours              # the real thing"
note "A trial that does not write a TrialRecord is not a data point."
fi

echo
echo "================ $PASS passed, $FAIL failed ================"
[ "$FAIL" -eq 0 ] || echo "Fix the first failure before moving up the ladder — later gates assume earlier ones hold."
exit $(( FAIL > 0 ))
