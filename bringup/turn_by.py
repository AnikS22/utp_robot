#!/usr/bin/env python3
"""Turn in place by a fixed number of degrees, closed-loop on odometry.

    python3 bringup/turn_by.py --deg +35
    python3 bringup/turn_by.py --deg -70 --dry-run

THE ROBOT ROTATES unless --dry-run. It does not translate.

WHY A FIXED GOAL, LATCHED. The command is computed ONCE from the odom yaw at entry and never
re-derived. Re-issuing a slightly different angular command at 20 Hz is a mode change the 4WS
firmware answers by physically re-steering all four wheels, so the wheels re-orient continuously
and the body never commits -- the 90-second livelock of 2026-08-29, heading wobbling +-4 deg.

WHY CLOSED-LOOP. Commanded rotation delivers 0.59-0.80 of what is asked, INCONSISTENTLY -- a fixed
re-steer startup cost, not a gain error, so no scale factor fixes it. Odometry itself is honest
(verified against lidar scan-matching, odom/lidar = 1.02), so the loop closes on /odom yaw.
"""
from __future__ import annotations

import argparse
import json
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
from std_msgs.msg import String  # noqa: E402

from safety.mux_watch import MuxWatch  # noqa: E402
from safety.waypoint_drive import Limits, wrap  # noqa: E402

CMD_TOPIC = "/cmd_vel_teleop"
RATE_HZ = 20.0
YAW_TOL = math.radians(4.0)
W_MIN = 0.12          # below this the chassis stalls rather than creeps


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Turner(Node):
    def __init__(self) -> None:
        super().__init__("utp_turn_by")
        self.yaw = None
        self.create_subscription(Odometry, "/odom",
                                 lambda m: setattr(self, "yaw", yaw_of(m.pose.pose.orientation)), 10)
        self.create_subscription(String, "/safety/status", self._safety, 10)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.mux = MuxWatch(time.monotonic())

    def _safety(self, m) -> None:
        try:
            self.mux.note_status(json.loads(m.data).get("blocked_by"), time.monotonic())
        except (ValueError, TypeError):
            pass

    def wait(self, t: float = 8.0) -> bool:
        end = time.monotonic() + t
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.yaw is not None:
                return True
        return False

    def stop(self) -> None:
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def run(self, deg: float, timeout: float = 60.0) -> tuple[bool, str]:
        lim = Limits()
        self.mux.resume(time.monotonic())
        start = self.yaw
        goal = wrap(start + math.radians(deg))     # FIXED, computed once
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=1.0/RATE_HZ)
            err = wrap(goal - self.yaw)
            if abs(err) <= YAW_TOL:
                self.stop()
                return True, (f"turned {math.degrees(wrap(self.yaw - start)):+.1f} deg "
                              f"(asked {deg:+.1f})")
            w = max(-lim.w_max, min(lim.w_max, 1.2 * err))
            if abs(w) < W_MIN:
                w = math.copysign(W_MIN, w)
            self.pub.publish(Twist(angular=Vector3(z=w)))
            now = time.monotonic()
            self.mux.note_command(True, now)
            v = self.mux.verdict(now)
            if not v.ok:
                self.stop()
                return False, v.reason
        self.stop()
        return False, f"timed out {math.degrees(wrap(goal - self.yaw)):+.1f} deg short"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--deg", type=float, required=True, help="degrees to turn (+ = left)")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    if abs(a.deg) > 180.0:
        print("--deg must be within +-180", file=sys.stderr)
        return 2

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = Turner()
    try:
        if not n.wait():
            print("no /odom", file=sys.stderr)
            return 1
        if a.dry_run:
            print(f"DRY RUN: would turn {a.deg:+.1f} deg from "
                  f"{math.degrees(n.yaw):+.1f} deg")
            return 0
        ok, why = n.run(a.deg)
        print(f"turn_by: {why}")
        return 0 if ok else 1
    finally:
        try:
            n.stop()
        except Exception:
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
