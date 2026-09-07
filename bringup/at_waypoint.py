#!/usr/bin/env python3
"""Is the robot ALREADY at this waypoint? Exit 0 if yes, 1 if no, 2 if it cannot tell.

WHY THIS EXISTS. 2026-09-07, the best run of the night: the robot pressed the call button, the
doors opened, it drove into the lift car -- and the next leg, to the pose inside the car, came back
`blocked` after 47 milliseconds. Nothing was wrong. The entry drive had carried it all the way in,
so it was standing 4 cm and 0.5 deg from the goal, well inside the 14 cm tolerance, and Nav2
aborted rather than report an arrival it had nothing to do for. mission.sh treats a non-arrival as
fatal, correctly, so a run that had done everything right died one step from the floor button.

A goal you are already standing on is an ARRIVAL, and that has to be decided before the request is
sent, because Nav2's answer to it is not one the caller can distinguish from a real failure.

TOLERANCES are deliberately the goal checker's, not looser: this must never turn a leg the robot
genuinely has to drive into a skipped one. Pass them in.
"""
from __future__ import annotations
import argparse, math, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO)); sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy, tf2_ros, yaml  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("waypoint")
    ap.add_argument("--tol-m", type=float, default=0.14)
    ap.add_argument("--tol-deg", type=float, default=11.5)
    ap.add_argument("--settle", type=float, default=12.0)
    a = ap.parse_args()

    wps = yaml.safe_load((REPO / "maps" / "waypoints.yaml").read_text()) or {}
    w = wps.get(a.waypoint)
    if not w:
        print(f"no waypoint '{a.waypoint}'", file=sys.stderr)
        return 2
    if (w.get("frame") or "map") != "map":
        print(f"'{a.waypoint}' is not a map-frame waypoint", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    n = Node("utp_at_waypoint")
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
    end = time.time() + a.settle
    tf = None
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if buf.can_transform("map", "base_link", Time()):
            tf = buf.lookup_transform("map", "base_link", Time())
            break
    if tf is None:
        rclpy.shutdown()
        print("no map -> base_link transform", file=sys.stderr)
        return 2

    t, q = tf.transform.translation, tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    d = math.hypot(float(w["x"]) - t.x, float(w["y"]) - t.y)
    dyaw = abs((math.degrees(yaw - float(w["yaw"])) + 180.0) % 360.0 - 180.0)
    rclpy.shutdown()
    print(f"{d*100:.0f} cm and {dyaw:.1f} deg from '{a.waypoint}'")
    return 0 if (d <= a.tol_m and dyaw <= a.tol_deg) else 1


if __name__ == "__main__":
    sys.exit(main())
