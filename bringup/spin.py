#!/usr/bin/env python3
"""Turn in place slowly enough that the scan matcher can follow, then re-localize.

    python3 bringup/spin.py 180              # turn 180 deg, then relocalize
    python3 bringup/spin.py -90 --no-reloc   # turn 90 deg clockwise, skip the relocalize
    python3 bringup/spin.py 180 --rate 0.10  # even slower

WHY THIS EXISTS INSTEAD OF A NAV2 GOAL. Asking Nav2 to reach a pose that differs only in heading
works, but it took 67 s on 2026-09-05 and left the pose estimate worse than it found it. Two
reasons, both structural:

  * MPPI is free to choose its own angular velocity up to wz_max (0.8 rad/s = 46 deg/s), and it
    uses it. slam_toolbox searches with coarse_angle_resolution 0.0349 rad = 2.0 deg, and /scan
    runs ~3.9 Hz. At 46 deg/s consecutive scans are ~12 deg apart -- six times the search window.
    The matcher cannot follow, the pose slides mid-turn, and the controller then steers against a
    stale estimate. That is the "constant rotating tricks slam into thinking it is in one
    orientation instead of another" the operator described.
  * Nav2's goal checker compares against that sliding estimate, so it hunts: it believes it has
    not arrived, turns further, drifts further.

So the turn is commanded open-loop at a rate the matcher CAN follow, and the pose is re-established
afterwards rather than trusted throughout. Default 0.15 rad/s = 8.6 deg/s, which at 3.9 Hz puts
~2.2 deg between scans -- just inside the search window. A 180 turn takes about 21 s.

Odometry closes the loop on HOW FAR it has turned, because wheel odometry is reliable over a single
short rotation even when the map-frame estimate is not -- that is exactly the split this exploits.

Publishes to /cmd_vel_teleop, which is the mux's highest-priority source and is not gated on the
deadman (config/safety.yaml requires_enable: false). The estop and, when enabled, the arm_stowed
gate still apply -- this cannot drive through an interlock.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

DEFAULT_RATE = 0.15          # rad/s. See the header: 2.2 deg between scans at 3.9 Hz.
SETTLE_S = 1.5               # stand still before measuring, so the matcher gets clean scans
TOL_DEG = 3.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("degrees", type=float, help="degrees to turn; positive = counter-clockwise")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE, help="rad/s (default 0.15)")
    ap.add_argument("--no-reloc", action="store_true", help="skip the relocalize afterwards")
    ap.add_argument("--topic", default="/cmd_vel_teleop")
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    rclpy.init()
    n = Node("utp_spin")
    pub = n.create_publisher(Twist, a.topic, 10)
    state = {"yaw": None, "first": None, "acc": 0.0}

    def on_odom(m: Odometry) -> None:
        q = m.pose.pose.orientation
        y = 2.0 * math.atan2(q.z, q.w)
        if state["yaw"] is not None:
            d = (y - state["yaw"] + math.pi) % (2 * math.pi) - math.pi
            state["acc"] += d
        state["yaw"] = y
        if state["first"] is None:
            state["first"] = y

    n.create_subscription(Odometry, "/odom", on_odom, qos_profile_sensor_data)

    t0 = time.time()
    while state["yaw"] is None and time.time() - t0 < 8:
        rclpy.spin_once(n, timeout_sec=0.05)
    if state["yaw"] is None:
        print("no /odom -- refusing to spin blind", file=sys.stderr)
        return 1

    target = math.radians(a.degrees)
    wz = math.copysign(abs(a.rate), target)
    print(f"  turning {a.degrees:+.0f} deg at {abs(wz):.2f} rad/s "
          f"({math.degrees(abs(wz)):.1f} deg/s, ~{abs(target)/abs(wz):.0f} s)", flush=True)

    tw = Twist()
    deadline = time.time() + abs(target) / abs(wz) + 15.0     # generous: odometry decides, not the clock
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(n, timeout_sec=0.02)
            remaining = target - state["acc"]
            if abs(remaining) <= math.radians(TOL_DEG):
                break
            # Ease off over the last 15 deg so it stops cleanly instead of overshooting and
            # hunting -- an overshoot correction is another fast rotation, which is the problem.
            scale = min(1.0, max(0.25, abs(remaining) / math.radians(15.0)))
            tw.angular.z = math.copysign(abs(wz) * scale, remaining)
            pub.publish(tw)
        tw.angular.z = 0.0
        for _ in range(10):
            pub.publish(tw)
            time.sleep(0.05)
    finally:
        tw.angular.z = 0.0
        for _ in range(10):
            pub.publish(tw)
            time.sleep(0.05)

    turned = math.degrees(state["acc"])
    print(f"  turned {turned:+.1f} deg by odometry (asked {a.degrees:+.0f})", flush=True)
    time.sleep(SETTLE_S)
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

    if a.no_reloc:
        return 0
    print("  re-localizing (the map-frame estimate is the thing a turn degrades)", flush=True)
    import subprocess
    from pathlib import Path
    repo = Path(__file__).resolve().parent.parent
    r = subprocess.run([sys.executable, str(repo / "bringup" / "relocalise.py")],
                       capture_output=True, text=True, timeout=420)
    for line in (r.stdout or "").strip().splitlines()[-3:]:
        print("   ", line.strip())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
