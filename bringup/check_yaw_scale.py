#!/usr/bin/env python3
"""Did the robot actually turn, or does odometry only think so? Ask the lidar.

    python3 bringup/check_yaw_scale.py            # read-only: is the scene matchable?
    python3 bringup/check_yaw_scale.py --go       # THE ROBOT SPINS IN PLACE ~35 deg

THE QUESTION THIS SETTLES. characterise_twist measured commanded wz against ODOMETRY and found
they disagree by 41% (0.20 rad/s for 3 s: expected 34.4 deg, odom said 20.3). That single number
has two opposite causes and the fix for one is the ruin of the other:

    chassis under-rotates   the robot really turned 20 deg. Odometry is honest, the controller is
                            being lied to about its authority, and raising the gain is correct.
    odometry under-reports  the robot really turned 34 deg and claims 20. Raising the gain then
                            makes it rotate 1.7x too far, AND every recorded waypoint yaw is
                            already wrong by the same factor.

Guessing turns a 41% error into a 70% one.

WHY THE LIDAR ANSWERS IT, with no floor marks and no human. The scan is a measurement of the
STATIC WORLD in the robot's frame. Rotate the robot by theta and every feature in that scan moves
by exactly -theta, whatever the wheels or the encoders believe. So the angular shift that best
re-aligns a before/after pair IS the true rotation, from a sensor with no stake in the argument.

It is a rotation-only comparison, which is why this spins in place and never translates: a
translated scan does not align under any pure rotation and the match would be meaningless.

WHAT MAKES IT FAIL, and it says so rather than guessing:
  * a featureless scene (a bare corridor looks the same at every angle) -- reported as a weak or
    ambiguous match, not as an answer
  * this A1M8 returns on a minority of beams, so only beams valid in BOTH scans are compared
  * anything that moved between the two scans (a person walking past) is noise in the match
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist, Vector3  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

CMD_TOPIC = "/cmd_vel_teleop"
WZ = 0.20
SECS = 3.0
SETTLE_S = 1.5      # 4WS wheels must physically re-steer before the body moves


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def valid(r: float) -> bool:
    return r == r and abs(r) != float("inf") and r > 0.0


def match_rotation(a: list, b: list, ainc: float, span_deg: float = 90.0):
    """Circular shift (radians) that best aligns scan b onto scan a, plus a quality score.

    Sum of absolute range differences over beams valid in BOTH, normalised by the count, so a
    sparse scan is not silently rewarded for overlapping in fewer places.
    """
    n = len(a)
    max_shift = int(math.radians(span_deg) / ainc)
    best = None
    curve = []
    for s in range(-max_shift, max_shift + 1):
        tot = 0.0
        cnt = 0
        for i in range(n):
            ra = a[i]
            rb = b[(i + s) % n]
            if valid(ra) and valid(rb):
                tot += abs(ra - rb)
                cnt += 1
        if cnt < 20:
            continue
        score = tot / cnt
        curve.append((s * ainc, score, cnt))
        if best is None or score < best[1]:
            best = (s * ainc, score, cnt)
    return best, curve


class Spin(Node):
    def __init__(self) -> None:
        super().__init__("utp_yaw_scale")
        self.scan = None
        self.pose = None
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)

    def _scan(self, m) -> None:
        self.scan = m

    def _odom(self, m) -> None:
        p = m.pose.pose
        self.pose = math.atan2(2*(p.orientation.w*p.orientation.z),
                               1 - 2*(p.orientation.z*p.orientation.z))

    def wait(self, t: float = 6.0) -> bool:
        end = time.monotonic() + t
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.scan is not None and self.pose is not None:
                return True
        return False

    def settle(self, secs: float) -> None:
        end = time.monotonic() + secs
        while rclpy.ok() and time.monotonic() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_for(self, wz: float, secs: float) -> None:
        end = time.monotonic() + secs
        while rclpy.ok() and time.monotonic() < end:
            self.pub.publish(Twist(angular=Vector3(z=wz)))
            rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(6):
            self.pub.publish(Twist())
            time.sleep(0.02)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go", action="store_true", help="spin the robot in place")
    ap.add_argument("--wz", type=float, default=WZ)
    ap.add_argument("--secs", type=float, default=SECS)
    a = ap.parse_args()

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = Spin()
    try:
        if not n.wait():
            print("no /scan or no /odom", file=sys.stderr)
            return 1
        s0 = n.scan
        v0 = sum(1 for r in s0.ranges if valid(r))
        print(f"scan: {len(s0.ranges)} beams, {v0} valid ({100*v0/len(s0.ranges):.0f}%)")
        if v0 < 40:
            print("too few returns to match a rotation against. Move somewhere with structure.",
                  file=sys.stderr)
            return 1
        if not a.go:
            # Prove the matcher finds ZERO shift between two stationary scans. If it cannot do
            # that, nothing it says about a real rotation is worth reading.
            n.settle(1.0)
            for _ in range(20):
                rclpy.spin_once(n, timeout_sec=0.05)
            best, _ = match_rotation(list(s0.ranges), list(n.scan.ranges), s0.angle_increment)
            if best:
                print(f"stationary self-check: best shift {math.degrees(best[0]):+.1f} deg "
                      f"(want ~0), residual {best[1]:.3f} m over {best[2]} beams")
            print("\nread-only. Add --go to spin and measure.")
            return 0

        before = list(s0.ranges)
        yaw0 = n.pose
        print(f"\nspinning wz={a.wz:+.2f} rad/s for {a.secs:.1f}s "
              f"(commanded {math.degrees(a.wz*a.secs):+.1f} deg)")
        n.settle(SETTLE_S)
        n.spin_for(a.wz, a.secs)
        n.settle(SETTLE_S)
        for _ in range(20):
            rclpy.spin_once(n, timeout_sec=0.05)
        after = list(n.scan.ranges)
        d_odom = wrap(n.pose - yaw0)

        best, curve = match_rotation(before, after, s0.angle_increment)
        if best is None:
            print("no usable overlap between the two scans", file=sys.stderr)
            return 1
        shift, resid, cnt = best
        # The scan rotates OPPOSITE to the robot: a feature at bearing b moves to b - theta.
        d_lidar = -shift

        ranked = sorted(curve, key=lambda c: c[1])
        margin = (ranked[1][1] - ranked[0][1]) if len(ranked) > 1 else 0.0

        cmd = a.wz * a.secs
        print(f"\n  commanded      {math.degrees(cmd):+7.2f} deg")
        print(f"  odometry says  {math.degrees(d_odom):+7.2f} deg   scale {d_odom/cmd:.2f}")
        print(f"  LIDAR says     {math.degrees(d_lidar):+7.2f} deg   scale {d_lidar/cmd:.2f}"
              f"   (residual {resid:.3f} m over {cnt} beams)")

        if abs(d_lidar) < math.radians(2.0):
            print("\n  the lidar sees almost no rotation -- suspect the match, or the robot "
                  "genuinely did not turn.")
            return 0
        r = d_odom / d_lidar if d_lidar else float("nan")
        print(f"\n  odometry / lidar = {r:.2f}")
        if 0.9 <= r <= 1.1:
            print("  VERDICT: odometry AGREES with the world. The CHASSIS under-rotates -- it\n"
                  "           really did turn less than commanded. A gain correction is the\n"
                  "           right fix, and recorded waypoint yaws are sound.")
        elif r < 0.9:
            print("  VERDICT: odometry UNDER-REPORTS. The robot turned further than it claims.\n"
                  "           Do NOT raise the controller gain -- it would over-rotate. Every\n"
                  "           recorded waypoint yaw is wrong by this factor and the odom yaw\n"
                  "           scale is what needs fixing.")
        else:
            print("  VERDICT: odometry OVER-REPORTS -- it claims more rotation than happened.")
        print(f"  match margin over the runner-up: {margin:.3f} m "
              f"({'clear' if margin > 0.05 else 'WEAK -- treat with suspicion'})")
        return 0
    finally:
        try:
            for _ in range(5):
                n.pub.publish(Twist())
                time.sleep(0.02)
        except Exception:
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
