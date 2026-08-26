#!/usr/bin/env python3
"""Run a mission: waypoints in order, with visual actions at the ones that matter.

    python3 bringup/route_run.py --list
    python3 bringup/route_run.py atrium_door                  # DRY RUN: validate + plan, no motion
    python3 bringup/route_run.py atrium_door --go             # THE ROBOT DRIVES

THE ROBOT DRIVES unless it is a dry run. Hand on the E-stop, and read docs/NAV2.md for why this
exists instead of Nav2.

WHY THIS AND NOT NAV2. Nav2 needs a map and a pose in it. On 2026-08-25 slam_toolbox could not
hold a pose in this building: a ~100-point scan matches almost equally well at many positions
along a corridor, and map->odom correction went from 0.1 cm to 13.7 cm per half-second as soon as
the base turned faster than 0.4 rad/s (the A1M8 sweeps for 145 ms and slam_toolbox treats that as
instantaneous). Nav2 was never the problem -- it never ran. The problem was upstream of it.

So: drive on ODOMETRY, and let VISION close every action. A leg only has to park the robot with
the target in the camera frame; the grounder and the visual servo do the rest, and they repeated
to 3 mm across four runs. Odometry drift over a 15-20 m leg is well inside that budget. This is a
deliberate trade, not a workaround: it moves the accuracy requirement to the one place we have
measured accuracy.

WHAT IT WILL NOT DO. It will not plan around an obstacle -- that needs a costmap, which needs the
localisation we just said we do not have. A blocked corridor STOPS the route and says so. Halting
on an unexpected obstruction is a defensible behaviour; improvising a detour on a pose estimate
we do not trust is not.
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy  # noqa: E402
import yaml  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from safety.route_plan import ACTION, GOTO, WAIT, RouteState, parse_route, validate_route  # noqa: E402
from safety.waypoint_drive import Limits, corridor_blocked, plan_step, to_goal, wrap  # noqa: E402

WAYPOINTS = REPO / "maps" / "waypoints.yaml"
ROUTES = REPO / "config" / "routes.yaml"
CMD_TOPIC = "/cmd_vel_teleop"
RATE_HZ = 20.0
ODOM_STALE_S = 0.5
LEG_TIMEOUT_S = 180.0

# Actions are shell-outs on purpose: the grounder needs torch (the pipeline venv) and this node
# needs rclpy. Different interpreters, so the boundary is a process, not an import.
ACTIONS = {
    "press_button": [str(REPO / "bringup" / "press_run.sh")],
}


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Runner(Node):
    def __init__(self) -> None:
        super().__init__("utp_route_run")
        self.pose = None
        self.stamp = 0.0
        self.scan = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self._scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)

    def _odom(self, m) -> None:
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.stamp = time.monotonic()

    def _scan(self, m) -> None:
        self.scan = m

    def fresh(self) -> bool:
        return self.pose is not None and (time.monotonic() - self.stamp) <= ODOM_STALE_S

    def wait_for_odom(self, timeout: float = 5.0) -> bool:
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None:
                return True
        return False

    def stop(self) -> None:
        """An explicit zero is a COMMAND and stops the chassis now. Letting the firmware watchdog
        expire instead costs 1.26 s of coasting -- about 18 cm (EXPERIMENT_LOG 2026-08-21d)."""
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def drive_leg(self, goal: dict, lim: Limits) -> tuple[bool, str]:
        deadline = time.monotonic() + LEG_TIMEOUT_S
        last = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=1.0/RATE_HZ)
            if not self.fresh():
                self.pub.publish(Twist())
                continue
            x, y, th = self.pose
            dist, bear = to_goal(x, y, th, goal["x"], goal["y"])
            blocked = (self.scan is not None and
                       corridor_blocked(self.scan.ranges, self.scan.angle_min,
                                        self.scan.angle_increment))
            step = plan_step(dist, bear, wrap(goal["yaw"] - th), blocked, lim)
            t = Twist(); t.linear.x = step.twist.vx; t.angular.z = step.twist.wz
            self.pub.publish(t)
            if step.state != last:
                print(f"      [{step.state}] {dist:5.2f} m, bearing {math.degrees(bear):+6.1f} deg")
                last = step.state
            if step.state == "arrived":
                self.stop()
                return True, ""
            if step.state == "blocked":
                # Hold, do not improvise. See the module docstring.
                self.stop()
                return False, "corridor blocked"
        self.stop()
        return False, f"leg timed out after {LEG_TIMEOUT_S:.0f}s"


def run_action(step, dry: bool) -> tuple[bool, str]:
    cmd = list(ACTIONS[step.name])
    if step.params.get("query"):
        cmd += ["--query", str(step.params["query"])]
    if step.params.get("standoff"):
        cmd += ["--standoff", str(step.params["standoff"])]
    if dry:
        cmd += ["--dry-run"]
    print(f"      $ {' '.join(cmd)}")
    r = subprocess.run(cmd)
    return (r.returncode == 0), ("" if r.returncode == 0 else f"exit {r.returncode}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("route", nargs="?", help="route name from config/routes.yaml")
    ap.add_argument("--list", action="store_true", help="list routes and waypoints, then exit")
    ap.add_argument("--go", action="store_true", help="actually drive")
    a = ap.parse_args()

    wps = yaml.safe_load(WAYPOINTS.read_text()) if WAYPOINTS.exists() else {}
    routes = (yaml.safe_load(ROUTES.read_text()) or {}).get("routes", {}) if ROUTES.exists() else {}

    if a.list or not a.route:
        print(f"waypoints ({len(wps)}): {sorted(wps) or 'none -- record some first'}")
        print(f"actions        : {sorted(ACTIONS)}")
        print(f"routes ({len(routes)}):")
        for name, spec in sorted(routes.items()):
            print(f"  {name}")
            for s in parse_route(spec):
                print(f"      {s.describe()}")
        return 0 if a.list else 2

    if a.route not in routes:
        print(f"unknown route '{a.route}'. Known: {sorted(routes)}", file=sys.stderr)
        return 2

    steps = parse_route(routes[a.route])
    errs = validate_route(steps, set(wps), set(ACTIONS))
    if errs:
        # Before anything moves. This is the whole point of validating a route as pure data.
        print(f"ROUTE '{a.route}' WILL NOT RUN -- {len(errs)} problem(s):", file=sys.stderr)
        for e in errs:
            print(f"  - {e}", file=sys.stderr)
        return 3

    print(f"route '{a.route}': {len(steps)} steps, validated against "
          f"{len(wps)} waypoints and {len(ACTIONS)} actions")
    for i, s in enumerate(steps):
        print(f"  {i+1}. {s.describe()}")
    if not a.go:
        print("\nDRY RUN. Nothing moved. Add --go to drive.")
        return 0

    rclpy.init()
    n = Runner()
    st = RouteState(steps)
    lim = Limits()
    try:
        if not n.wait_for_odom():
            print("no /odom -- is ranger_bringup running?", file=sys.stderr)
            return 1
        print(f"\nSTART. odom {[round(v,2) for v in n.pose]}   Ctrl-C stops; E-stop is faster.\n")
        while not st.done and rclpy.ok():
            step = st.current
            print(f"  {st.progress()}")
            if step.kind == GOTO:
                ok, why = n.drive_leg(wps[step.name], lim)
            elif step.kind == ACTION:
                ok, why = run_action(step, dry=False)
            else:
                time.sleep(min(step.params.get("seconds", 0.0), 300.0))
                ok, why = True, ""
            if not ok:
                st.fail(why)
                break
            st.advance()
    except KeyboardInterrupt:
        st.fail("interrupted by operator")
    finally:
        n.stop()
        print(f"\n{st.progress()}")
        print("stopped (zero published)")
        n.destroy_node()
        rclpy.shutdown()
    return 0 if not st.failed_reason else 1


if __name__ == "__main__":
    sys.exit(main())
