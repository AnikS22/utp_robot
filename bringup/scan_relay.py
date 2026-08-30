#!/usr/bin/env python3
"""Republish a BEST_EFFORT LaserScan as RELIABLE, so slam_toolbox can actually receive it.

    python3 bringup/scan_relay.py            # /scan_filtered (best effort) -> /scan (reliable)

WHY. pointcloud_to_laserscan publishes /scan_filtered with Reliability: BEST_EFFORT.
slam_toolbox subscribes to its scan topic with RELIABLE. Those are INCOMPATIBLE in DDS: a
best-effort publisher cannot satisfy a reliable subscriber, so not one message is delivered --
and nothing anywhere reports an error. slam_toolbox simply sits there having logged its stack
size, publishing no /map and no map->odom, looking exactly like a hung node.

Measured 2026-08-30: /scan_filtered at 9.2 Hz with a healthy odom->base_link TF, and
slam_toolbox silent from the moment it started.

This is the same silent-discard shape as the safety mux discarding commands and the deadman that
was never published: the system is working exactly as configured, and the configuration means
"deliver nothing".
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

from _ros_env import require_ros      # noqa: E402
require_ros()

import rclpy                          # noqa: E402
from rclpy.node import Node           # noqa: E402
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,   # noqa: E402
                       QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan  # noqa: E402

IN_TOPIC = "/scan_filtered"
OUT_TOPIC = "/scan"


class Relay(Node):
    def __init__(self) -> None:
        super().__init__("utp_scan_relay")
        out_qos = QoSProfile(depth=10,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, OUT_TOPIC, out_qos)
        self.create_subscription(LaserScan, IN_TOPIC, self._cb, qos_profile_sensor_data)
        self.n = 0

    def _cb(self, m: LaserScan) -> None:
        self.pub.publish(m)
        self.n += 1
        if self.n in (1, 50) or self.n % 500 == 0:
            self.get_logger().info(f"relayed {self.n} scans {IN_TOPIC} -> {OUT_TOPIC}")


def main() -> int:
    rclpy.init()
    n = Relay()
    print(f"\n  {IN_TOPIC} (BEST_EFFORT) -> {OUT_TOPIC} (RELIABLE)\n")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
