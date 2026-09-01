#!/usr/bin/env python3
"""Drive to a MAP-frame waypoint with Nav2. The map-based twin of `waypoints.py goto`.

    python3 bringup/nav2_goto.py door            # DRY RUN: prints the goal, moves nothing
    python3 bringup/nav2_goto.py door --go       # THE ROBOT MOVES

WHY THIS EXISTS
---------------
`waypoints.py goto` drives on ODOM. That was correct when the only lidar was the A1M8 and
slam_toolbox could not hold a pose (route_run.py records the measurement: a ~100-point scan matches
almost equally well at many positions along a corridor). The OS0-128 removed that constraint on
2026-08-30 -- 977 valid beams over a full 360, a 666x779 @ 0.05 m map of the atrium, and Nav2
planning on it -- but navigate_to_goal was never updated. This closes that gap.

WHY A MAP MATTERS FOR *FIFTY* TRIALS SPECIFICALLY
-------------------------------------------------
Odom-frame waypoints have two failure modes that a 50-trial session hits and a 1-trial demo does
not: they drift continuously, and they die outright when `ranger_base` restarts. Both are fatal to
repeatability, and both are exactly what a SAVED, NAMED map fixes -- the coordinates stop being
session-scoped. safety/map_frame.py already enforces the distinction that makes this safe: a
fresh-SLAM `map` frame looks identical in the TF tree but its origin is wherever the robot booted,
so it refuses to treat a nameless recording as portable.

WHAT STAYS ON ODOM, DELIBERATELY
---------------------------------
Only the LEG runs on the map. docs/NAV2.md is explicit about why: "an AMCL correction mid-press
would move the target under the arm." So approach_blockage, the look-around ladder and the press
chain keep running in odom, where motion is smooth and continuous. Nav2 gets the robot to the
door; vision and odom close the last metre. That split is the design, not a compromise.

OUTPUT CONTRACT
---------------
RosWorld.navigate_to_goal parses stdout for the words `arrived` / `blocked`, exactly as it does
for waypoints.py. This script prints the same vocabulary so the two backends are interchangeable.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

GOAL_TIMEOUT_S = 180.0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="waypoint name (must have been recorded in the MAP frame)")
    ap.add_argument("--go", action="store_true", help="actually move; without it this is a dry run")
    ap.add_argument("--timeout", type=float, default=GOAL_TIMEOUT_S)
    ap.add_argument("--force", action="store_true",
                    help="drive even if the waypoint's map provenance cannot be confirmed")
    a = ap.parse_args()

    from waypoints import load as load_waypoints
    from safety.map_frame import FRAME_KEY, FRAME_MAP, MAP_NAME_KEY

    store = load_waypoints()
    if a.name not in store:
        print(f"unknown waypoint '{a.name}'. Known: {sorted(store) or 'none'}", file=sys.stderr)
        return 2
    wp = store[a.name]
    frame = wp.get(FRAME_KEY, "odom")
    if frame != FRAME_MAP and not a.force:
        print(f"waypoint '{a.name}' is in the '{frame}' frame, not '{FRAME_MAP}'. Nav2 needs a map "
              f"pose. Re-record it while localized in a NAMED map, or use waypoints.py goto.",
              file=sys.stderr)
        return 3
    if not wp.get(MAP_NAME_KEY) and not a.force:
        print(f"waypoint '{a.name}' carries no map name — it was recorded against a fresh SLAM "
              f"session whose origin is wherever the robot booted, so the coordinate is not "
              f"portable. Load a saved map, relocalize, and re-record. (--force overrides.)",
              file=sys.stderr)
        return 3

    x, y, yaw = float(wp["x"]), float(wp["y"]), float(wp.get("yaw", 0.0))
    print(f"goal '{a.name}' in {frame}: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):+.1f} deg "
          f"(map={wp.get(MAP_NAME_KEY)})")

    if not a.go:
        print("DRY RUN. Add --go to send the goal.")
        return 0

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped

    rclpy.init()
    node = Node("utp_nav2_goto")
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=10.0):
        # A silent bt_navigator is the classic half-failed Nav2 bringup: the lifecycle nodes come
        # up unconfigured and nothing says so.
        print("no navigate_to_pose action server after 10 s — is Nav2 up AND activated? "
              "(ros2 lifecycle get /bt_navigator)", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 4

    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    goal.pose = ps

    print(f"DRIVING to '{a.name}' via Nav2. Ctrl-C stops. E-stop is faster.")
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print("Nav2 REJECTED the goal — outside the map, or in an inflated cell?", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 5

    result_fut = handle.get_result_async()
    t0 = time.time()
    # EXIT CODES ARE A CONTRACT WITH RosWorld.navigate_to_goal:
    #   0      a real navigation outcome (arrived / blocked) -- parse stdout
    #   6      a real TIMEOUT
    #   2..5   this backend cannot serve the request -> caller falls back to odom waypoints
    #   1      anything else, i.e. we crashed -> caller must also fall back, NOT record a timeout
    # 1 previously meant timeout, which collided with an uncaught exception: a nav2_goto that died
    # on an import would have been recorded as a legitimate navigation timeout.
    rc = 6
    try:
        while rclpy.ok() and time.time() - t0 < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
            if result_fut.done():
                break
        if not result_fut.done():
            handle.cancel_goal_async()
            rclpy.spin_once(node, timeout_sec=2.0)
            print(f"TIMEOUT after {a.timeout:.0f} s — cancelled", file=sys.stderr)
            rc = 6
        else:
            status = result_fut.result().status
            # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus
            if status == 4:
                print(f"arrived at '{a.name}' in {time.time() - t0:.1f} s")
                rc = 0
            else:
                # Nav2 exhausting its recoveries in front of an obstruction is the same event the
                # odom backend reports as `blocked`, and the FSM treats it the same way: stop and
                # let reason -> ground -> act run from here.
                print(f"blocked: Nav2 finished with status {status} after "
                      f"{time.time() - t0:.1f} s (recoveries exhausted?)")
                rc = 0
    except KeyboardInterrupt:
        handle.cancel_goal_async()
        rclpy.spin_once(node, timeout_sec=2.0)
        print("\ninterrupted — goal cancelled")
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
