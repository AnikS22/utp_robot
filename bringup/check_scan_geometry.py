#!/usr/bin/env python3
"""Validate the lidar's scan DIRECTION and ZERO-ANGLE against physical reality.

    python3 bringup/check_scan_geometry.py            # live ASCII polar view
    python3 bringup/check_scan_geometry.py --tf       # also check base_link -> lidar_link exists

Why this is a separate gate from "/scan is publishing"
------------------------------------------------------
A lidar that is mirrored, or rotated 90 deg, or whose zero-angle points the wrong way, publishes a
scan that looks entirely healthy: right beam count, right ranges, plausible room shape. It then
builds a map that looks plausible and navigates catastrophically. No amount of staring at message
fields catches it. The only check that works is physical: put a known object at a known bearing and
confirm the scan agrees.

HOW TO USE, once the lidar is mounted on the rover:
  1. Clear a space around the robot.
  2. Stand (or put a box) about 1 m DIRECTLY IN FRONT of the robot -- the +x direction it drives.
  3. Run this. The nearest return should sit at ~0 deg, and the ASCII view should show the blob at
     the TOP (front).
  4. Repeat on the LEFT. ROS REP-103 is x forward, y left, yaw counter-clockwise, so left must read
     as a POSITIVE angle (~+90 deg). If left reads -90, the scan is mirrored -- fix it with the
     driver's `inverted` / `flip_x_axis` parameter, NOT by negating angles downstream.

The ASCII view is oriented as the robot sees the world: UP is +x (forward), LEFT is +y (robot left).
"""
from __future__ import annotations

import argparse
import math
import sys

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))
from _ros_env import require_ros
require_ros()

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

W, H = 61, 31          # odd, so there is an exact centre cell


def render(scan: LaserScan, max_r: float) -> str:
    grid = [[" "] * W for _ in range(H)]
    cx, cy = W // 2, H // 2
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r <= 0 or r > max_r:
            continue
        a = scan.angle_min + i * scan.angle_increment
        # robot frame: x forward, y left. Screen: up = +x, left = +y.
        x, y = r * math.cos(a), r * math.sin(a)
        col = int(round(cx - y / max_r * cx))
        row = int(round(cy - x / max_r * cy))
        if 0 <= row < H and 0 <= col < W:
            grid[row][col] = "#"
    grid[cy][cx] = "+"                       # the sensor
    for d in range(1, 5):                    # forward axis marker
        if cy - d >= 0 and grid[cy - d][cx] == " ":
            grid[cy - d][cx] = "|"
    return "\n".join("".join(r) for r in grid)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--max-range", type=float, default=4.0, help="metres shown in the ASCII view")
    ap.add_argument("--tf", action="store_true", help="also verify base_link -> lidar_link")
    args = ap.parse_args()

    rclpy.init()
    node = Node("check_scan_geometry")
    latest: list[LaserScan] = []
    node.create_subscription(LaserScan, args.topic,
                             lambda m: latest.append(m), qos_profile_sensor_data)

    tf_buffer = None
    if args.tf:
        from tf2_ros import Buffer, TransformListener
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)

    print(f"waiting for {args.topic} ...")
    import time
    t0 = time.time()
    while not latest and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not latest:
        print(f"NO DATA on {args.topic} — is bringup/lidar.sh running, and ROS_DOMAIN_ID the same?")
        sys.exit(1)

    if tf_buffer is not None:
        frame = latest[-1].header.frame_id
        for _ in range(25):
            rclpy.spin_once(node, timeout_sec=0.2)
        try:
            from rclpy.time import Time
            t = tf_buffer.lookup_transform("base_link", frame, Time())
            tr = t.transform.translation
            print(f"TF base_link -> {frame}: xyz=({tr.x:.3f}, {tr.y:.3f}, {tr.z:.3f})  OK")
        except Exception as e:
            print(f"TF base_link -> {frame} MISSING: {e}")
            print("  Nav2's costmap needs this or it is blind. Run bringup/lidar.sh (publishes it),")
            print("  or a URDF / robot_state_publisher that owns the transform.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if not latest:
                continue
            m = latest[-1]
            latest.clear()
            pts = [(r, m.angle_min + i * m.angle_increment)
                   for i, r in enumerate(m.ranges) if math.isfinite(r) and r > 0]
            print("\033[2J\033[H", end="")     # clear
            print(f"{args.topic}  frame={m.header.frame_id}  "
                  f"{len(pts)}/{len(m.ranges)} valid   view radius {args.max_range:.1f} m")
            print("UP = +x (robot FORWARD)   LEFT = +y (robot LEFT)   '+' = sensor\n")
            print(render(m, args.max_range))
            if pts:
                r, a = min(pts)
                print(f"\nnearest return : {r:.2f} m at {math.degrees(a):+.1f} deg")
                print("front object should read ~0 deg;  LEFT object should read ~+90 deg "
                      "(negative => scan is MIRRORED)")
            print("\nCtrl-C to exit")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl-C can leave the context already shut down; calling shutdown() again raises RCLError
        # and buries the tool's output in a traceback.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
