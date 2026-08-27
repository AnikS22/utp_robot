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
import os
import subprocess
import sys
import time

CRITICAL = "critical"
INFO = "info"

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
        check_topics(rep)
        check_duplicates(rep)
        bad = rep.render()
        print(f"\n  {'ALL CRITICAL CHECKS PASS' if bad == 0 else f'{bad} CRITICAL FAILURE(S)'}")
        if not a.watch:
            return 0 if bad == 0 else 1
        time.sleep(5)


if __name__ == "__main__":
    sys.exit(main())
