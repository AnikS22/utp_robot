#!/usr/bin/env python3
"""Live compass of what the lidar can and cannot see, in ROBOT directions.

    python3 bringup/scan_compass.py            # live, Ctrl-C to stop
    python3 bringup/scan_compass.py --secs 20  # accumulate then print once

WHY THIS EXISTS. On 2026-08-25 the A1M8 returned on only ~30% of its beams and the map was
unusable. It was not a weak laser: read over the RAW serial protocol, sectors that see anything
return 95-99.7% of the time with quality up to 44/63, and sectors that do not return EXACTLY 0.0%.
That is a blocked beam, not a failing sensor.

THE TRAP THIS TOOL EXISTS TO EXPOSE. range_min is 0.15 m. Anything CLOSER than that does not
report as a short range -- it reports as NO RETURN, identical to empty space. So an object almost
touching the lidar is invisible and totally blinding at the same time. Looking for dirt on the
window will not find it; you are looking for an OBJECT beside the window -- a cable, a bracket
edge, a zip tie, a mount lip.

So: run this, read off the blocked bearings, and go look at those bearings on the physical robot.

It also settles CALIBRATION item 4 (scan handedness), which nothing else here can. Hold something
about 1 m to the robot's LEFT. It must appear at about +90. If it appears at -90 the scan is
MIRRORED, and every map built from it will look plausible and be wrong everywhere.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))
from _ros_env import require_ros
require_ros()

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SECTORS = 24                       # 15 degrees each
DEAD_FRACTION = 0.02               # below this hit rate a sector counts as blocked


def label(deg: float) -> str:
    """ROS convention: +y is LEFT, so a positive bearing is to the robot's left."""
    d = (deg + 180.0) % 360.0 - 180.0
    if abs(d) <= 15: return "AHEAD"
    if abs(d) >= 165: return "BEHIND"
    side = "LEFT" if d > 0 else "RIGHT"
    a = abs(d)
    if a <= 75: return f"{side}-FRONT"
    if a <= 105: return f"{side}"
    return f"{side}-REAR"


class Compass(Node):
    def __init__(self, live: bool):
        super().__init__("utp_scan_compass")
        self.live = live
        self.hits = [0]*SECTORS
        self.total = [0]*SECTORS
        self.near = [float("inf")]*SECTORS
        self.scans = 0
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)
        self.last_draw = 0.0

    def cb(self, m: LaserScan) -> None:
        self.scans += 1
        for i, r in enumerate(m.ranges):
            deg = math.degrees(m.angle_min + i*m.angle_increment)
            deg = (deg + 180.0) % 360.0 - 180.0
            s = int((deg + 180.0) // (360.0/SECTORS)) % SECTORS
            self.total[s] += 1
            if r == r and math.isfinite(r) and m.range_min <= r <= m.range_max:
                self.hits[s] += 1
                self.near[s] = min(self.near[s], r)
        if self.live and time.monotonic() - self.last_draw > 0.5:
            self.draw(); self.last_draw = time.monotonic()

    def draw(self) -> None:
        w = 360.0/SECTORS
        if self.live:
            sys.stdout.write("\033[2J\033[H")
        print(f"  scans: {self.scans}    sectors of {w:.0f} deg    "
              f"+bearing = LEFT, -bearing = RIGHT, 0 = straight ahead\n")
        print(f"  {'bearing':>16}  {'direction':<12} {'hit%':>6} {'nearest':>9}")
        blocked = []
        for s in range(SECTORS):
            lo = -180.0 + s*w
            mid = lo + w/2
            t, h = self.total[s], self.hits[s]
            if t == 0:
                continue
            pct = h/t
            n = self.near[s]
            bar = "#"*int(round(20*pct))
            flag = ""
            if pct <= DEAD_FRACTION:
                flag = "  <== BLOCKED"
                blocked.append((lo, lo+w))
            print(f"  {lo:+6.0f}..{lo+w:+6.0f}  {label(mid):<12} {100*pct:5.1f}% "
                  f"{(f'{n:.2f} m' if math.isfinite(n) else '   --'):>9} {bar}{flag}")
        if blocked:
            merged = [list(blocked[0])]
            for a, b in blocked[1:]:
                if abs(a - merged[-1][1]) < 1e-6: merged[-1][1] = b
                else: merged.append([a, b])
            print("\n  BLOCKED ARCS -- go look at these bearings on the robot:")
            for a, b in merged:
                print(f"    {a:+.0f} .. {b:+.0f} deg   ({label((a+b)/2)}, {b-a:.0f} deg wide)")
            print("\n  Remember: an object CLOSER than 0.15 m reads as no-return, not as a short")
            print("  range. You are looking for something beside the window, not dirt on it.")
        else:
            print("\n  No fully blocked arc. ")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=0.0,
                    help="accumulate for N seconds then print once (default: live)")
    a = ap.parse_args()
    rclpy.init()
    n = Compass(live=a.secs <= 0)
    end = time.monotonic() + (a.secs if a.secs > 0 else 1e9)
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    if a.secs > 0:
        n.draw()
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
