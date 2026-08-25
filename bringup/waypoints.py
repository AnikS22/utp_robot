#!/usr/bin/env python3
"""Record places by driving to them, then drive back to them on odometry alone.

    python3 bringup/waypoints.py record door_a     # stand the robot here, save this spot
    python3 bringup/waypoints.py list
    python3 bringup/waypoints.py where             # where am I relative to each waypoint
    python3 bringup/waypoints.py goto door_a       # DRY RUN: prints what it would do
    python3 bringup/waypoints.py goto door_a --go  # THE ROBOT MOVES

WHY ODOMETRY AND NOT A MAP. On 2026-08-25 slam_toolbox could not hold a pose in this building --
in a corridor a ~100-point scan matches almost equally well at several positions and the estimate
flips between them. The experiment does not need a map. It needs the base to arrive with the ADA
plate in the camera frame; approach_target.py servos visually from there and hand-eye is good to
2.96 mm RMS. Odometry drift over a 15-20 m leg is well inside what the servo absorbs.

WHAT THAT COSTS YOU, STATED PLAINLY. Odometry is dead reckoning. It drifts, it never recovers, and
NOTHING here detects that it has. A waypoint recorded at the far end of a long drive is only as
good as the odometry that got you there. Record from a KNOWN START, keep legs short, and treat
every arrival as approximate until the camera confirms it.

  * `record` is passive -- it reads /odom and writes a file. It cannot move anything.
  * `goto` publishes to /cmd_vel_teleop, so it goes through the safety mux like everything else
    and is subject to every gate. It is NOT a bypass. The mux must be running.
  * A lidar corridor check vetoes motion; see safety/waypoint_drive.corridor_blocked.
  * A watchdog stops the robot if this process dies -- the chassis coasts 1.26 s on a lost
    commander (EXPERIMENT_LOG 2026-08-21d), so the parting zero matters.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))
from _ros_env import require_ros
require_ros()

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from safety.waypoint_drive import Limits, corridor_blocked, plan_step, to_goal, wrap  # noqa: E402

STORE = REPO / "maps" / "waypoints.yaml"
CMD_TOPIC = "/cmd_vel_teleop"
RATE_HZ = 20.0
ODOM_STALE_S = 0.5


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def load() -> dict:
    if not STORE.exists():
        return {}
    return yaml.safe_load(STORE.read_text()) or {}


def save(d: dict) -> None:
    STORE.parent.mkdir(parents=True, exist_ok=True)
    STORE.write_text(
        "# Waypoints in the ODOM frame. Only meaningful within one continuous run of the ranger\n"
        "# driver: restarting it re-zeros odom and silently invalidates every entry here.\n"
        "# The 'odom_epoch' field is how you tell. See bringup/waypoints.py.\n"
        + yaml.safe_dump(d, sort_keys=True))


class Pose(Node):
    """Just enough node to read one odom sample."""

    def __init__(self, name: str = "utp_waypoints"):
        super().__init__(name)
        self.pose = None
        self.stamp = 0.0
        self.scan = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self._scan, qos_profile_sensor_data)

    def _odom(self, m: Odometry) -> None:
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.stamp = time.monotonic()

    def _scan(self, m: LaserScan) -> None:
        self.scan = m

    def wait_for_pose(self, timeout: float = 5.0) -> bool:
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None:
                return True
        return False

    def fresh(self) -> bool:
        return self.pose is not None and (time.monotonic() - self.stamp) <= ODOM_STALE_S


def cmd_record(n: Pose, a) -> int:
    if not n.wait_for_pose():
        print("no /odom -- is ranger_bringup running?", file=sys.stderr)
        return 1
    x, y, th = n.pose
    d = load()
    d[a.name] = {"x": round(x, 4), "y": round(y, 4), "yaw": round(th, 4),
                 "odom_epoch": round(x*0 + time.time())}
    save(d)
    print(f"recorded '{a.name}': x={x:+.3f} y={y:+.3f} yaw={math.degrees(th):+.1f} deg")
    print(f"  -> {STORE}")
    return 0


def cmd_list(n: Pose, a) -> int:
    d = load()
    if not d:
        print("no waypoints yet")
        return 0
    for k, v in sorted(d.items()):
        print(f"  {k:<20} x={v['x']:+8.3f} y={v['y']:+8.3f} yaw={math.degrees(v['yaw']):+7.1f} deg")
    return 0


def cmd_where(n: Pose, a) -> int:
    if not n.wait_for_pose():
        print("no /odom", file=sys.stderr)
        return 1
    x, y, th = n.pose
    print(f"now: x={x:+.3f} y={y:+.3f} yaw={math.degrees(th):+.1f} deg")
    for k, v in sorted(load().items()):
        dist, bear = to_goal(x, y, th, v["x"], v["y"])
        print(f"  {k:<20} {dist:6.2f} m away, bearing {math.degrees(bear):+7.1f} deg")
    return 0


def cmd_goto(n: Pose, a) -> int:
    d = load()
    if a.name not in d:
        print(f"unknown waypoint '{a.name}'. Known: {sorted(d) or 'none'}", file=sys.stderr)
        return 2
    goal = d[a.name]
    if not n.wait_for_pose():
        print("no /odom -- is ranger_bringup running?", file=sys.stderr)
        return 1

    lim = Limits()
    pub = n.create_publisher(Twist, CMD_TOPIC, 10)
    if not a.go:
        x, y, th = n.pose
        dist, bear = to_goal(x, y, th, goal["x"], goal["y"])
        step = plan_step(dist, bear, wrap(goal["yaw"] - th), False, lim)
        print(f"DRY RUN. {dist:.2f} m away, bearing {math.degrees(bear):+.1f} deg")
        print(f"  first action: {step.state}  vx={step.twist.vx:.3f} wz={step.twist.wz:.3f}")
        print(f"  would publish to {CMD_TOPIC} at {RATE_HZ:.0f} Hz. Re-run with --go to move.")
        return 0

    print(f"DRIVING to '{a.name}'. Ctrl-C stops. E-stop is faster.")
    deadline = time.monotonic() + a.timeout
    last_state = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=1.0/RATE_HZ)
            if not n.fresh():
                pub.publish(Twist())
                continue
            x, y, th = n.pose
            dist, bear = to_goal(x, y, th, goal["x"], goal["y"])
            blocked = False
            if n.scan is not None:
                blocked = corridor_blocked(n.scan.ranges, n.scan.angle_min,
                                           n.scan.angle_increment)
            step = plan_step(dist, bear, wrap(goal["yaw"] - th), blocked, lim)
            t = Twist(); t.linear.x = step.twist.vx; t.angular.z = step.twist.wz
            pub.publish(t)
            if step.state != last_state:
                print(f"  [{step.state}] {dist:5.2f} m, bearing {math.degrees(bear):+6.1f} deg")
                last_state = step.state
            if step.state == "arrived":
                break
        else:
            print("TIMEOUT -- stopping", file=sys.stderr)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # The chassis coasts 1.26 s on a lost commander. An explicit zero is a COMMAND and stops
        # it now; letting the watchdog expire is 18 cm of uncommanded travel.
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.02)
        print("stopped (zero published)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record"); r.add_argument("name"); r.set_defaults(fn=cmd_record)
    sub.add_parser("list").set_defaults(fn=cmd_list)
    sub.add_parser("where").set_defaults(fn=cmd_where)
    g = sub.add_parser("goto"); g.add_argument("name")
    g.add_argument("--go", action="store_true", help="actually move the robot")
    g.add_argument("--timeout", type=float, default=120.0)
    g.set_defaults(fn=cmd_goto)
    a = ap.parse_args()

    rclpy.init()
    n = Pose()
    try:
        return a.fn(n, a)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
