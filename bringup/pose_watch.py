#!/usr/bin/env python3
"""Catch a SLAM pose that jumped without the robot moving. Nothing here moves anything.

WHY. 2026-09-07, at the floor-2 lift: the robot was standing still at the door pose, map ->
base_link read (5.990, 2.427), the doors opened, and the next reading was (2.525, 1.470) -- 3.6 m
back across the lobby, with the wheels stationary. slam_toolbox had re-matched the scan somewhere
else, which is exactly what an opening lift door invites: the geometry in front of the robot goes
from a flat surface a metre away to a 5 m corridor into the car, and a scan matcher offered that
much new structure can find a better-scoring pose in the wrong place.

Nav2 then planned the entry leg from a pose the robot was not standing on, and the drive it wanted
was 5.6 m across the lobby, away from the lift. mission.sh's drive gate could not see it: that gate
asks whether find_self ever agreed with itself in THIS map, which it had, twenty seconds earlier
and correctly.

THE TEST. Odometry and the map must move by the same amount. Wheel odometry drifts slowly and is
honest about short intervals -- it is what the localizer is correcting, not a competitor to it --
so over one leg the two displacements agree to a few centimetres. A localization jump breaks that
immediately: the map pose moves metres while odom reports the robot stood still.

This does NOT judge whether the pose is CORRECT. A wrong pose that has been wrong all along passes,
because nothing about it jumped. It catches the pose that WAS right and stopped being right, which
is the failure that puts a robot on a trajectory nobody planned.
"""
from __future__ import annotations
import argparse, json, math, sys, time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO)); sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy, tf2_ros  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.time import Time  # noqa: E402


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_pair(settle: float = 10.0):
    """(map_xy, odom_xy) right now, or None if either is unavailable."""
    rclpy.init(args=None)
    n = Node("utp_pose_watch")
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
    odom = {}
    n.create_subscription(Odometry, "/odom", lambda m: odom.__setitem__("p", m.pose.pose), 10)
    end = time.time() + settle
    tf = None
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if tf is None and buf.can_transform("map", "base_link", Time()):
            tf = buf.lookup_transform("map", "base_link", Time())
        if tf is not None and "p" in odom:
            break
    rclpy.shutdown()
    if tf is None or "p" not in odom:
        return None
    t = tf.transform.translation
    o = odom["p"].position
    return {"map": [t.x, t.y], "odom": [o.x, o.y], "t": time.time()}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--sample", action="store_true", help="record the current pair")
    g.add_argument("--check", action="store_true", help="compare against the recorded pair")
    ap.add_argument("--tol", type=float, default=0.50,
                    help="metres of disagreement between the map and odom displacements that counts "
                         "as a jump (default 0.50)")
    a = ap.parse_args()
    f = Path(a.file)

    now = read_pair()
    if now is None:
        print("pose_watch: no map -> base_link or no /odom; cannot judge", file=sys.stderr)
        return 2

    if a.sample:
        f.write_text(json.dumps(now))
        return 0

    if not f.exists():
        return 0                       # nothing to compare against yet: not a failure
    was = json.loads(f.read_text())
    dmap = math.dist(now["map"], was["map"])
    dodom = math.dist(now["odom"], was["odom"])
    gap = abs(dmap - dodom)
    print(f"since the last leg: map moved {dmap:.2f} m, odom moved {dodom:.2f} m")
    if gap > a.tol:
        print(f"POSE JUMPED: they disagree by {gap:.2f} m, over the {a.tol:.2f} m tolerance.\n"
              f"  The map pose moved without the wheels agreeing, so SLAM re-matched the scan\n"
              f"  somewhere else. Driving now plans from a place the robot is not standing on.",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
