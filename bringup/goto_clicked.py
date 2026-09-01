#!/usr/bin/env python3
"""Click a point in RViz ("Publish Point"), and the robot drives there with Nav2.

    python3 bringup/goto_clicked.py            # ARMED: every click sends the robot
    python3 bringup/goto_clicked.py --dry-run  # prints the coordinate, sends nothing

WHY A POINT AND NOT A POSE. RViz's "2D Goal Pose" already drives the robot, but it makes you drag
out a heading you usually do not care about, and a careless drag sets a final orientation that can
make an otherwise reachable goal unplannable. A point has no heading, so this fills one in: the
robot ends up facing the way it travelled, which is what you want when the next thing it does is
look at whatever is in front of it.

THE ROBOT MOVES ON A SINGLE CLICK once armed. That is the point of the tool, and it is why the
coordinate is echoed before the goal is sent -- if the number is not where you meant, hit the
E-stop, because the goal is already away.

WHAT THE OUTCOMES MEAN, and they are not all failures:
  arrived   the robot reached the goal
  blocked   Nav2 planned, tried, exhausted its recoveries and stopped. On this project that is a
            RESULT, not a fault -- a closed door is exactly what starts reason -> ground -> act.
  rejected  Nav2 refused before moving: the goal is outside the map, or inside an inflated cell.
            Turn on the global costmap in RViz and look at where you clicked.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _ros_env import require_ros  # noqa: E402

require_ros()

import rclpy                                              # noqa: E402
from rclpy.action import ActionClient                     # noqa: E402
from rclpy.node import Node                               # noqa: E402
from geometry_msgs.msg import PointStamped, PoseStamped   # noqa: E402
from nav2_msgs.action import NavigateToPose               # noqa: E402
import tf2_ros                                            # noqa: E402

MAP_FRAME = "map"


class ClickDriver(Node):
    def __init__(self, dry_run: bool):
        super().__init__("utp_goto_clicked")
        self.dry_run = dry_run
        self.busy = False
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(PointStamped, "/clicked_point", self.on_click, 1)
        self.get_logger().info(
            f"listening on /clicked_point -- {'DRY RUN, nothing will move' if dry_run else 'ARMED: a click DRIVES the robot'}")

    def robot_xy(self):
        try:
            t = self.buf.lookup_transform(MAP_FRAME, "base_link", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception as e:
            self.get_logger().error(f"no {MAP_FRAME}->base_link: {e}")
            return None

    def on_click(self, msg: PointStamped):
        gx, gy = msg.point.x, msg.point.y
        frame = msg.header.frame_id or "?"
        if frame != MAP_FRAME:
            # A point clicked while RViz's Fixed Frame is odom is NOT a map coordinate, and driving
            # to it would mean something different in every session.
            self.get_logger().error(
                f"clicked point is in frame '{frame}', not '{MAP_FRAME}'. "
                f"Set RViz Global Options -> Fixed Frame to 'map'.")
            return
        here = self.robot_xy()
        if here is None:
            return
        dist = math.dist(here, (gx, gy))
        yaw = math.atan2(gy - here[1], gx - here[0])   # face the way we travel
        print(f"\n  CLICKED  x={gx:+.2f}  y={gy:+.2f}  (map)   {dist:.2f} m away, "
              f"final heading {math.degrees(yaw):+.0f} deg")
        if self.dry_run:
            print("  dry run -- no goal sent")
            return
        if self.busy:
            print("  a goal is already running; ignoring this click")
            return
        if not self.client.wait_for_server(timeout_sec=5.0):
            print("  no navigate_to_pose action server -- is Nav2 running?")
            return
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = MAP_FRAME
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = gx
        g.pose.pose.position.y = gy
        g.pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.busy = True
        print("  sending goal ... THE ROBOT IS MOVING")
        self.client.send_goal_async(g).add_done_callback(self.on_accepted)

    def on_accepted(self, fut):
        h = fut.result()
        if not h.accepted:
            print("  Nav2 REJECTED the goal (outside the map, or in an inflated cell)")
            self.busy = False
            return
        print("  goal accepted")
        h.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        # status 4 == SUCCEEDED. Anything else is Nav2 giving up, which on this project is a
        # legitimate observation about the world rather than a bug.
        status = fut.result().status
        print("  arrived" if status == 4 else f"  blocked (status {status})")
        self.busy = False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = ClickDriver(a.dry_run)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
