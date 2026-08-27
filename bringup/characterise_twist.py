#!/usr/bin/env python3
"""Does a commanded twist produce the motion we think it does? Measure it. CALIBRATION 5 and 6.

    python3 bringup/characterise_twist.py --check      # read-only: publishers, pose, no motion
    python3 bringup/characterise_twist.py --go         # COMMANDS MOTION, ~1 m of travel total

THE ROBOT MOVES with --go. Hand on the E-stop. Needs ~2 m of clear space ahead and around.

WHY THIS EXISTS, AND WHY IT SHOULD HAVE COME FIRST. On 2026-08-26 a waypoint controller was
written, watched to misbehave ("wheels just rotate and go crazy"), and then debugged DOWNSTREAM --
turn hysteresis, then a vendor-driver mode-latch bug. Both were real. Neither was established to
be THE problem, because nobody had ever checked the thing underneath them: that
`angular.z = +0.2` actually rotates the robot left at 0.2 rad/s.

If the yaw sign is inverted, a proportional heading controller drives the error AWAY from zero.
It turns harder, the error grows, and the symptom is indistinguishable from a mode-thrash bug or
a tuning problem. That is a five-minute measurement masquerading as a day of debugging.

docs/CALIBRATION.md has listed item 5 (odometry scale, 2%/3%) and item 6 (twist characterisation,
"table reproduces") as open the whole time.

WHAT IT MEASURES, open-loop, one axis at a time:
  * SIGN    -- does +vx go forward, does +wz go left (ROS: +z is counter-clockwise from above)
  * SCALE   -- commanded vs measured, as a ratio. 1.00 is perfect; 0.5 means odometry or the
               chassis is off by half and every distance in the stack is wrong by that factor.

STEP 0 IS NOT OPTIONAL. It counts /odom publishers. On 2026-08-26 two different odom values were
read seconds apart, which would invalidate every measurement below and every waypoint ever
recorded. If that count is not 1, nothing else here means anything.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402

CMD_TOPIC = "/cmd_vel_teleop"      # through the safety mux, never straight to /cmd_vel
RATE_HZ = 20.0
SETTLE_S = 1.5                     # 4WS wheels must physically re-steer before the body moves


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Meas(Node):
    def __init__(self) -> None:
        super().__init__("utp_characterise_twist")
        self.pose = None
        self.stamp = 0.0
        self.n_odom = 0
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)

    def _odom(self, m) -> None:
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.stamp = time.monotonic()
        self.n_odom += 1

    def spin(self, secs: float) -> None:
        end = time.monotonic() + secs
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_pose(self, t: float = 5.0) -> bool:
        end = time.monotonic() + t
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None:
                return True
        return False

    def stop(self) -> None:
        """An explicit zero is a COMMAND and stops the chassis now. Letting the firmware watchdog
        expire instead costs 1.26 s of coasting, about 18 cm."""
        for _ in range(6):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def run_twist(self, vx: float, wz: float, secs: float) -> tuple:
        """Command one twist open-loop and return (start_pose, end_pose)."""
        self.spin(0.3)
        start = self.pose
        t_end = time.monotonic() + secs
        while rclpy.ok() and time.monotonic() < t_end:
            t = Twist(); t.linear.x = vx; t.angular.z = wz
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=1.0/RATE_HZ)
        self.stop()
        self.spin(SETTLE_S)          # let it coast to a stop before reading
        return start, self.pose


def report(label, cmd, expected, measured, unit):
    sign_ok = (expected == 0) or (measured * expected > 0)
    scale = (measured / expected) if abs(expected) > 1e-9 else float("nan")
    print(f"  {label:<22} commanded {cmd:+6.2f}  expected {expected:+7.3f} {unit}  "
          f"measured {measured:+7.3f} {unit}")
    print(f"  {'':<22} sign {'OK' if sign_ok else '*** INVERTED ***':<18} "
          f"scale {scale:5.2f}  {'(1.00 = perfect)' if sign_ok else ''}")
    return sign_ok, scale


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="read-only; no motion")
    ap.add_argument("--go", action="store_true", help="COMMAND MOTION")
    ap.add_argument("--vx", type=float, default=0.10, help="m/s for the linear test")
    ap.add_argument("--wz", type=float, default=0.20, help="rad/s for the angular test")
    ap.add_argument("--secs", type=float, default=3.0, help="seconds per test")
    a = ap.parse_args()

    rclpy.init()
    n = Meas()
    try:
        # ---- step 0: is odometry even trustworthy -------------------------------------------
        import subprocess
        r = subprocess.run(["ros2", "topic", "info", "/odom"], capture_output=True, text=True,
                           timeout=25)
        pubs = [l for l in r.stdout.splitlines() if "Publisher count" in l]
        print("STEP 0  odometry sanity")
        print(f"  {pubs[0].strip() if pubs else 'Publisher count: unknown'}")
        count = int(pubs[0].split(":")[1]) if pubs else -1
        if count != 1:
            print(f"  *** {count} publishers on /odom. Every measurement below would be a blend of")
            print("      two sources, and every waypoint ever recorded is suspect. Fix this first.")
            return 2
        if not n.wait_pose():
            print("  no /odom -- is ranger_bringup running?", file=sys.stderr)
            return 1
        n.spin(2.0)
        print(f"  pose now: x={n.pose[0]:+.3f} y={n.pose[1]:+.3f} "
              f"yaw={math.degrees(n.pose[2]):+.1f} deg")
        print(f"  /odom rate: ~{n.n_odom/2.3:.0f} Hz")
        if a.check or not a.go:
            print("\nread-only. Add --go to measure sign and scale (the robot will move ~1 m).")
            return 0

        # ---- step 1: linear ------------------------------------------------------------------
        print(f"\nSTEP 1  linear: vx={a.vx:+.2f} m/s for {a.secs:.0f}s")
        s, e = n.run_twist(a.vx, 0.0, a.secs)
        dx, dy = e[0]-s[0], e[1]-s[1]
        # forward component in the STARTING body frame
        fwd = dx*math.cos(s[2]) + dy*math.sin(s[2])
        lat = -dx*math.sin(s[2]) + dy*math.cos(s[2])
        lin_ok, lin_scale = report("linear.x -> forward", a.vx, a.vx*a.secs, fwd, "m")
        print(f"  {'':<22} lateral drift {lat:+.3f} m, net yaw change "
              f"{math.degrees(wrap(e[2]-s[2])):+.1f} deg")

        # ---- step 2: angular -----------------------------------------------------------------
        print(f"\nSTEP 2  angular: wz={a.wz:+.2f} rad/s for {a.secs:.0f}s")
        s, e = n.run_twist(0.0, a.wz, a.secs)
        dyaw = wrap(e[2]-s[2])
        ang_ok, ang_scale = report("angular.z -> yaw (+ = LEFT)", a.wz,
                                   math.degrees(a.wz*a.secs), math.degrees(dyaw), "deg")
        print(f"  {'':<22} position drift "
              f"{math.hypot(e[0]-s[0], e[1]-s[1]):.3f} m (should be ~0 for a spin)")

        # ---- verdict -------------------------------------------------------------------------
        print("\nVERDICT")
        if not lin_ok:
            print("  linear.x is INVERTED -- forward commands drive backward.")
        if not ang_ok:
            print("  angular.z is INVERTED. A proportional heading controller will drive the")
            print("  error AWAY from zero: it turns harder, the error grows, and it looks")
            print("  exactly like 'the wheels just rotate and go crazy'. THIS IS THE BUG.")
        if lin_ok and ang_ok:
            print("  signs are correct. Scales above feed CALIBRATION items 5 and 6;")
            print("  anything outside 0.97-1.03 means every distance in the stack is off by it.")
        return 0
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 1
    finally:
        n.stop()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
