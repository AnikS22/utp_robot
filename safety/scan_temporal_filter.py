#!/usr/bin/env python3
"""Suppress flickering near-field LaserScan returns before they enter Nav2.

Fresh Ouster clouds sometimes alternate between the real surface and a 0.45--0.52 m return on
high downward-looking rings intersecting the robot/arm envelope. This is not a stale-message
filter: every message is current. A near obstacle must instead be geometrically consistent for
three consecutive scans. Far observations and clearing (+inf) pass immediately.

Default wiring: /scan (SLAM-safe geometric mask) -> /scan_nav (Nav2 only).
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))


@dataclass
class NearFieldConfirm:
    threshold_m: float = 0.70
    confirmations: int = 3
    # A real stationary surface moves only millimetres scan-to-scan. The observed artifact walked
    # 4--8 cm between adjacent frames; a loose 12 cm gate incorrectly promoted one such run.
    range_tolerance_m: float = 0.03
    neighbor_bins: int = 2
    rear_mask_min_deg: float = 74.0
    rear_mask_max_m: float = 1.30
    previous: list[float] = field(default_factory=list)
    streak: list[int] = field(default_factory=list)

    def filter(self, ranges: Sequence[float], angle_min: float = 0.0,
               angle_increment: float = 0.0) -> tuple[list[float], int]:
        cur = list(ranges); n = len(cur); out = list(cur); suppressed = 0
        if len(self.previous) != n:
            self.previous = [math.inf] * n; self.streak = [0] * n
        next_streak = [0] * n
        for i, value in enumerate(cur):
            # MEASURED 2026-09-05: a base-fixed phantom ring appears 0.85--1.20 m ASTERN in open
            # space. It is in fresh clouds (not stale DDS/costmap data), and temporal confirmation
            # can promote it when it persists. Remove only the affected rear arc/radius; real rear
            # walls beyond 1.30 m survive.
            a = angle_min + i * angle_increment
            deg = abs((math.degrees(a) + 180.0) % 360.0 - 180.0)
            if (math.isfinite(value) and value > 0 and deg >= self.rear_mask_min_deg
                    and value <= self.rear_mask_max_m):
                out[i] = math.inf; suppressed += 1
                continue
            if not math.isfinite(value) or value <= 0 or value >= self.threshold_m:
                # Clearing and ordinary-range observations are never delayed.
                continue
            best = 0
            lo, hi = max(0, i-self.neighbor_bins), min(n, i+self.neighbor_bins+1)
            for j in range(lo, hi):
                old = self.previous[j]
                if math.isfinite(old) and abs(old-value) <= self.range_tolerance_m:
                    best = max(best, self.streak[j])
            next_streak[i] = best + 1
            if next_streak[i] < self.confirmations:
                out[i] = math.inf; suppressed += 1
        self.previous = cur; self.streak = next_streak
        return out, suppressed


def main() -> int:
    from bringup._ros_env import require_ros
    require_ros()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import LaserScan

    input_topic = os.environ.get("UTP_SCAN_TEMPORAL_IN", "/scan")
    output_topic = os.environ.get("UTP_SCAN_TEMPORAL_OUT", "/scan_nav")
    confirm = NearFieldConfirm(
        threshold_m=float(os.environ.get("UTP_NEAR_CONFIRM_M", "0.70")),
        confirmations=int(os.environ.get("UTP_NEAR_CONFIRM_SCANS", "3")))
    rclpy.init(); node = Node("utp_scan_temporal_filter")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
    pub = node.create_publisher(LaserScan, output_topic, qos)
    counts = {"scans": 0, "suppressed": 0}

    def callback(msg):
        values, n = confirm.filter(msg.ranges, msg.angle_min, msg.angle_increment)
        msg.ranges = values; pub.publish(msg)
        counts["scans"] += 1; counts["suppressed"] += n
        if counts["scans"] % 50 == 0:
            node.get_logger().info(
                f"{input_topic}->{output_topic}: suppressed {counts['suppressed']} "
                f"unconfirmed near bins in {counts['scans']} scans")

    node.create_subscription(LaserScan, input_topic, callback, qos)
    node.get_logger().info(f"{input_topic} -> {output_topic}: require {confirm.confirmations} "
                           f"scans below {confirm.threshold_m:.2f} m")
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__": raise SystemExit(main())
