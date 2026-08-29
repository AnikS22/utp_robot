#!/usr/bin/env python3
"""One place that answers "is the robot actually working right now?".

    python3 bringup/health.py              # check everything, exit 0 if all critical checks pass
    python3 bringup/health.py --watch      # re-check every 5 s until interrupted

Run it first thing after plugging in, and any time something behaves oddly.

WHY THIS EXISTS
On 2026-08-21 six separate failures all presented the same way: a component stayed alive,
produced plausible output, and was wrong. Nothing errored.

    lidar re-enumerated  -> node alive, /scan silent
    camera re-enumerated -> node alive, no topics
    CAN link dropped     -> driver alive, /odom publishing ZEROS at 50 Hz
    SLAM with no /scan    -> a map built from every scan stacked at the origin
    detector, no target  -> a confident box on a fire alarm
    wrong hand-eye sign  -> a clean 3 mm residual, 0.8 m out

Each cost between ten minutes and an hour, and each was found by eventually thinking to look
rather than by being told. The common cause is that "the process is running" was being used as a
proxy for "the thing is working". This checks the second one.

The specific trap it exists to catch: a driver holding a stale file descriptor after its USB
device re-enumerated. The process is up, the topic is advertised, and no data flows. `pgrep` says
everything is fine. Only the data rate reveals it.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

CRITICAL = "critical"
INFO = "info"
# Non-fatal but not merely informational. It was USED by check_duplicates and never
# defined, so the "0 publishers" branch -- the one a half-started bring-up takes --
# raised NameError instead of reporting the missing publisher.
WARN = "warn"

USB_IDS = [
    ("8086:0b07", "RealSense D435", CRITICAL),
    ("10c4:ea60", "RPLIDAR (CP2102)", CRITICAL),
    ("1d50:606f", "USB-CAN adapter", CRITICAL),
]


class Report:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail, level=CRITICAL):
        self.rows.append((name, ok, detail, level))

    def render(self):
        width = max(len(r[0]) for r in self.rows)
        bad = 0
        for name, ok, detail, level in self.rows:
            mark = "ok  " if ok else ("FAIL" if level == CRITICAL else "warn")
            if not ok and level == CRITICAL:
                bad += 1
            print(f"  [{mark}] {name:<{width}}  {detail}")
        return bad


def check_usb(rep):
    try:
        out = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=10).stdout
    except Exception as e:
        rep.add("usb", False, f"lsusb failed: {e}")
        return
    for vid, label, level in USB_IDS:
        rep.add(label, vid in out, "present" if vid in out else "NOT ON USB -- replug", level)


def check_can(rep):
    path = "/sys/class/net/can0"
    if not os.path.exists(path):
        rep.add("can0", False, "interface absent (adapter unplugged, or udev rule not installed)")
        return
    state = open(f"{path}/operstate").read().strip()
    if state != "up":
        rep.add("can0", False, f"{state} -- sudo systemctl restart can0.service")
        return
    # Frames ARRIVING is the real test. can0 can sit 'up' with a dead chassis on the far end,
    # which is what an unplugged CAN cable looks like: healthy interface, zero rx, climbing tx
    # errors because nothing is left to ACK.
    r1 = int(open(f"{path}/statistics/rx_packets").read())
    time.sleep(1.5)
    r2 = int(open(f"{path}/statistics/rx_packets").read())
    rate = (r2 - r1) / 1.5
    rep.add("can0", rate > 100, f"up, {rate:.0f} frames/s"
            + ("" if rate > 100 else "  <- chassis silent: powered off, or CAN cable adrift"))


def check_arm(rep, ip="192.168.1.221"):
    ok = subprocess.run(["ping", "-c1", "-W1", ip],
                        capture_output=True).returncode == 0
    if not ok:
        rep.add("xArm6", False, f"{ip} unreachable")
        return
    try:
        sys.path.insert(0, os.path.expanduser(
            "~/utp_robot/.venv-arm/lib/python3.12/site-packages"))
        from xarm.wrapper import XArmAPI
        a = XArmAPI(ip, is_radian=False, do_not_open=False)
        err, state = a.error_code, a.state
        tcp = a.tcp_offset
        load = a.tcp_load
        a.disconnect()
        rep.add("xArm6", err == 0, f"{ip} state={state} err={err}"
                + ("" if err == 0 else "  <- clean_error() before moving"))
        configured = any(abs(v) > 1e-6 for v in (tcp[:3] if tcp else [0, 0, 0]))
        rep.add("arm tool", configured,
                f"tcp_offset={tcp} load={load}"
                + ("" if configured else "  <- NOT SET: arm thinks its tool is a bare flange, so "
                                         "collision thresholds are wrong (CALIBRATION item 2)"),
                INFO)
    except Exception as e:
        rep.add("xArm6", False, f"{ip} reachable but SDK failed: {e}")


def check_chassis_mode(rep):
    """Will the chassis obey the computer, or is the RC holding authority?

    Invisible from ROS: in CONTROL_MODE_RC the chassis DISCARDS every CAN motion command while
    /odom keeps flowing at 50 Hz and the mux keeps reporting "permitted". The discard happens in
    firmware, below anything ROS can observe, so every other check here goes green.
    """
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from chassis_mode import ADVICE, GOOD, chassis_mode
        st = chassis_mode()
    except Exception as e:
        rep.add("chassis mode", False, f"could not read 0x211: {e}", WARN)
        return
    if st is None:
        rep.add("chassis mode", False, "no 0x211 -- bus silent, or rover unpowered", CRITICAL)
        return
    vehicle, mode, batt, err = st
    rep.add("chassis mode", mode == GOOD,
            f"{mode}" + ("" if mode == GOOD else f"  <- {ADVICE.get(mode, '')}"), CRITICAL)
    rep.add("chassis state", vehicle == "NORMAL",
            f"{vehicle}" + ("" if vehicle == "NORMAL" else "  <- nothing will move"),
            CRITICAL if vehicle == "ESTOP" else INFO)
    # 48 V nominal pack; below ~46 V the chassis browns out under load before it warns.
    rep.add("battery", batt > 46.0, f"{batt:.1f} V" + ("" if batt > 46.0 else "  <- LOW"),
            INFO if batt > 46.0 else CRITICAL)
    if err:
        rep.add("chassis error", False, f"0x{err:04X}", CRITICAL)


def check_topics(rep):
    """Rates, not existence. An advertised topic with no publisher looks identical to a live one."""
    script = r'''
import sys, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, CameraInfo
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
rclpy.init(); n = Node("utp_health")
c = {"scan":0,"cam":0,"odom":0}; edges=set(); moving=0
def od(m):
    global moving
    c["odom"] += 1
    v = m.twist.twist
    if abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z) > 1e-6: moving += 1
n.create_subscription(LaserScan,"/scan",lambda m:c.__setitem__("scan",c["scan"]+1),qos_profile_sensor_data)
n.create_subscription(CameraInfo,"/mast_cam/color/camera_info",lambda m:c.__setitem__("cam",c["cam"]+1),qos_profile_sensor_data)
n.create_subscription(Odometry,"/odom",od,10)
n.create_subscription(TFMessage,"/tf",lambda m:[edges.add((t.header.frame_id,t.child_frame_id)) for t in m.transforms],10)
t0=n.get_clock().now().nanoseconds
while n.get_clock().now().nanoseconds-t0 < 5e9: rclpy.spin_once(n,timeout_sec=0.2)
d=(n.get_clock().now().nanoseconds-t0)/1e9
print(f"{c['scan']/d:.1f} {c['cam']/d:.1f} {c['odom']/d:.1f} {moving} " +
      ",".join(f"{p}>{ch}" for p,ch in sorted(edges)))
n.destroy_node(); rclpy.shutdown()
'''
    try:
        r = subprocess.run([sys.executable, "-c", script], capture_output=True,
                           text=True, timeout=60)
        parts = r.stdout.strip().split(None, 4)
        scan, cam, odom, moving = float(parts[0]), float(parts[1]), float(parts[2]), int(parts[3])
        edges = parts[4] if len(parts) > 4 else ""
    except Exception as e:
        rep.add("ros topics", False, f"could not sample: {e}")
        return

    rep.add("/scan", scan > 3.0, f"{scan:.1f} Hz (expect ~6.5)"
            + ("" if scan > 3.0 else "  <- lidar node alive but silent? restart bringup/lidar.sh"))
    rep.add("camera", cam > 20.0, f"{cam:.1f} Hz on camera_info (expect ~30)"
            + ("" if cam > 20.0 else "  <- restart bringup/camera.sh"))
    rep.add("/odom", odom > 20.0, f"{odom:.1f} Hz"
            + ("" if odom > 20.0 else "  <- ranger driver not publishing"))
    # /odom at full rate carrying nothing but zeros is what a dead CAN link looks like from ROS.
    # It is indistinguishable from a stationary robot, which is why it needs saying out loud.
    if odom > 20.0:
        rep.add("/odom content", True,
                f"{moving} of {int(odom*5)} samples non-zero"
                + ("" if moving else "  <- all zeros. Correct if the robot is stationary; if it "
                                     "is MOVING, the CAN link is dead"), INFO)
    for want in ("odom>base_link", "map>odom", "base_link>lidar_link"):
        rep.add(f"tf {want}", want in edges,
                "present" if want in edges else "MISSING",
                CRITICAL if want == "odom>base_link" else INFO)



def _arm_declared_absent() -> bool:
    """Is the arm_stowed gate being satisfied by a declaration rather than a measurement?

    A gate reading 100% looks identical whether it was measured or asserted. It must not.
    """
    try:
        r = subprocess.run(["pgrep", "-af", "arm_monitor_node.py"],
                           capture_output=True, text=True, timeout=5)
        return "absent" in r.stdout
    except Exception:
        return False


def check_gates(rep):
    """Would the base move if something asked it to?

    THE FAILURE THIS CATCHES. Every safety gate is fail-closed, so the DEFAULT state of this robot
    is "will not move". Each gate going False is correct behaviour, reported correctly on
    /safety/status -- and until 2026-08-29 the only subscriber was the human teleop page. An
    autonomous run therefore published twists into a mux that discarded all of them, for the full
    180 s leg timeout, and then reported "leg timed out": a navigation symptom for an interlock
    cause. Days went into the planner. The planner was never running.

    Duty cycle, not a snapshot. A gate that FLAPS is the expensive case: sampled once it looks
    fine, and it still blocks the majority of ticks. arm_stowed measured 30 True / 91 False in
    sim, and every one of those False ticks was a discarded command."""
    script = r'''
import json, rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init(); n = Node("utp_health_gates")
seen = {"n":0}; gates = {}; blocks = {}
def on(m):
    try: st = json.loads(m.data)
    except Exception: return
    seen["n"] += 1
    for k, v in (st.get("gates") or {}).items():
        gates[k] = gates.get(k, 0) + (1 if v else 0)
    b = st.get("blocked_by")
    if b: blocks[b] = blocks.get(b, 0) + 1
n.create_subscription(String, "/safety/status", on, 10)
t0 = n.get_clock().now().nanoseconds
while n.get_clock().now().nanoseconds - t0 < 5e9: rclpy.spin_once(n, timeout_sec=0.2)
print(json.dumps({"n": seen["n"], "gates": gates, "blocks": blocks}))
n.destroy_node(); rclpy.shutdown()
'''
    try:
        r = subprocess.run([sys.executable, "-c", script], capture_output=True,
                           text=True, timeout=60)
        d = json.loads(r.stdout.strip().splitlines()[-1])
    except Exception as e:
        rep.add("safety mux", False, f"could not sample /safety/status: {e}", CRITICAL)
        return

    n = int(d.get("n", 0))
    if n == 0:
        rep.add("safety mux", False,
                "NO /safety/status - the mux is not running, so nothing forwards commands to "
                "/cmd_vel. Nothing will drive.", CRITICAL)
        return
    rep.add("safety mux", True, f"{n/5.0:.1f} Hz on /safety/status")

    # estop_latched is inverted: True there means blocked, unlike every other gate.
    for name, count in sorted((d.get("gates") or {}).items()):
        pct = 100.0 * count / n
        permitting = (pct < 1.0) if name == "estop_latched" else (pct > 99.0)
        detail = f"{pct:.0f}% of {n} ticks"
        if name == "estop_latched" and pct >= 1.0:
            detail += "  <- LATCHED. Clear it with the /safety/clear_estop service; releasing " \
                      "the physical button is not enough."
        elif name == "arm_stowed" and permitting and _arm_declared_absent():
            detail += "  <- by DECLARATION (arm_monitor backend=absent), NOT measured. Valid " \
                      "only while no arm is fitted; record this against any trial."
        elif name == "arm_stowed" and not permitting:
            detail += "  <- base blocked. Either stow the arm, or the monitor's evidence is " \
                      "going stale between messages (see arm_monitor.<backend>.stale_after_s)."
        elif name in ("enable",) and not permitting:
            detail += "  <- deadman not held; nav and servo sources are dead (teleop is not)"
        rep.add(f"gate {name}", permitting, detail,
                CRITICAL if name in ("arm_stowed", "estop_latched") else INFO)

    for reason, count in sorted((d.get("blocks") or {}).items(), key=lambda kv: -kv[1]):
        # no_source with nothing driving is the correct, expected report, not a fault.
        rep.add(f"blocking: {reason}", reason == "no_source",
                f"{100.0*count/n:.0f}% of ticks"
                + ("  (expected while nothing is commanding)" if reason == "no_source" else ""),
                INFO if reason == "no_source" else CRITICAL)


# Topics that must have EXACTLY ONE publisher, and what a second one does.
SINGLE_PUBLISHER = {
    "/odom":     "two ranger_base drivers -> readers get a BLEND of two odom frames",
    "/scan":     "two rplidar drivers -> they split the serial stream between them",
    "/cmd_vel":  "something is bypassing the safety mux",
}


def check_duplicates(rep):
    """Exactly one publisher per critical topic.

    THE FAILURE THIS CATCHES, which cost most of 2026-08-26. Two ranger_base_node processes were
    running -- orphans left by restarts whose `ros2 launch` parent was killed but whose node
    survived. Both published /odom. Subscribers got an interleaved blend of two odometry frames,
    so the robot's pose jumped between them every few messages, a heading controller chased the
    jumps, and the wheels thrashed without the base moving.

    Nothing errored. Both drivers were healthy. `ros2 topic hz /odom` looked perfect -- BETTER
    than perfect, because it was counting two publishers. The only visible signature is the
    publisher COUNT, which nothing was checking. Two symptom-level bugs were found and fixed
    before anyone looked at it.
    """
    for topic, why in SINGLE_PUBLISHER.items():
        try:
            r = subprocess.run(["ros2", "topic", "info", topic],
                               capture_output=True, text=True, timeout=12)
            line = next((l for l in r.stdout.splitlines() if "Publisher count" in l), "")
            n = int(line.split(":")[1]) if line else -1
        except Exception:
            rep.add(f"publishers {topic}", False, "could not query", WARN)
            continue
        if n == 1:
            rep.add(f"publishers {topic}", True, "1")
        elif n == 0:
            rep.add(f"publishers {topic}", False, "0 - nothing publishing", WARN)
        else:
            rep.add(f"publishers {topic}", False, f"{n} PUBLISHERS - {why}", CRITICAL)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--skip-arm", action="store_true")
    a = ap.parse_args()

    while True:
        rep = Report()
        print(f"\n=== utp_robot health  {time.strftime('%H:%M:%S')} "
              f"(ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'UNSET')}) ===")
        check_usb(rep)
        check_can(rep)
        if not a.skip_arm:
            check_arm(rep)
        check_chassis_mode(rep)
        check_topics(rep)
        check_gates(rep)
        check_duplicates(rep)
        bad = rep.render()
        print(f"\n  {'ALL CRITICAL CHECKS PASS' if bad == 0 else f'{bad} CRITICAL FAILURE(S)'}")
        if not a.watch:
            return 0 if bad == 0 else 1
        time.sleep(5)


if __name__ == "__main__":
    sys.exit(main())
