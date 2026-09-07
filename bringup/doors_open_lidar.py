#!/usr/bin/env python3
"""Has the door in front of the robot opened? Ask the 3D lidar, not the camera.

    python3 bringup/doors_open_lidar.py                 # wait up to 10 s, exit 0 when it opens
    python3 bringup/doors_open_lidar.py --once          # single look
    python3 bringup/doors_open_lidar.py --clear-m 2.0   # how far ahead counts as open

EXIT CODES, because a script decides whether to drive on them:
    0  OPEN        -- the way ahead is clear, go now
    1  STILL SHUT
    2  COULD NOT TELL (no /scan) -- treat as shut

WHY THE LIDAR AND NOT THE CAMERA. bringup/doors_open.py asks a VLM, and its own header records the
measurement that condemns that choice: on 2026-09-01, from the door pose, the camera "looked
straight through them and reported an open walkway with pillars" while the doors were CLOSED --
and on the same scene "the lidar had 85 returns at 0.72 m where the camera saw nothing". The
sensor that was right is the one that was not being asked.

It is also the sensor that is fast. Measured 2026-09-06: the VLM version spent 26.5 s over five
looks and still said SHUT after a press that had been made, while an ADA opener holds for a
bounded time and then shuts. This reads /scan, which already runs at 10 Hz, and decides in the
time it takes to collect a few sweeps. On a task whose entire problem is getting through a closing
door, a check that costs half a minute is not a check, it is the reason you did not make it.

WHAT IT MEASURES. /scan is already in base_link with the rear masked, so a forward sector is just
an angular slice. A closed door is a wall of returns about a metre ahead; an open one is the
corridor beyond it. It reports the CLEAR DISTANCE -- the nearest thing in the sector, taken as a
low percentile rather than the single minimum so one stray return does not veto the drive.

WHAT IT CANNOT DO: tell a door that opened from a door someone walked through, or from the robot
having turned away. It answers "is the way ahead clear", which is what the drive needs, and it
does not pretend to answer "did the plate actuate".
"""
from __future__ import annotations

import argparse
import math
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--clear-m", type=float, default=2.0,
                    help="forward clear distance that counts as OPEN (default 2.0 m)")
    ap.add_argument("--sector-deg", type=float, default=30.0,
                    help="full width of the forward sector examined (default 30 deg)")
    ap.add_argument("--topic", default="/scan_nav")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    n = Node("utp_doors_lidar")
    latest = {}
    n.create_subscription(LaserScan, a.topic, lambda m: latest.__setitem__("s", m),
                          qos_profile_sensor_data)

    half = math.radians(a.sector_deg) / 2.0

    def clear_distance():
        s = latest.get("s")
        if s is None:
            return None
        r = np.asarray(s.ranges, dtype=float)
        ang = s.angle_min + np.arange(len(r)) * s.angle_increment
        # Wrap to [-pi, pi] so a scan starting at 0 rather than -pi is handled the same way.
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        sec = np.abs(ang) <= half
        v = r[sec]
        v = v[np.isfinite(v) & (v > s.range_min)]
        if v.size < 5:
            # No returns at all in the sector IS the open case: nothing within range ahead.
            return float("inf")
        # MEDIAN, NOT 10th PERCENTILE. The 10th percentile asks "is almost every forward beam
        # reaching past the threshold", which is only true when the robot is square in the opening.
        # Measured 2026-09-07 at the floor-2 lift with the doors DEMONSTRABLY OPEN: straight ahead
        # p10 1.10 m, median 3.15 m, max 4.90 m, 253 returns past 3 m -- and the check said SHUT,
        # because the robot had arrived 15 deg off the door waypoint and about a tenth of the
        # forward beams were landing on the door frame beside the opening. The operator watched it
        # sit in front of an open lift.
        #
        # The median asks the question that actually matters -- is the bulk of the forward sector
        # looking THROUGH something -- and it is just as robust to the stray close return the
        # percentile was introduced for, since a few near beams cannot move it. A shut door still
        # reads its own surface across the whole sector: measured 0.48 m closed against 3.15 m open.
        return float(np.median(v))

    # Discovery first. A node created a moment ago has not seen /scan yet, and reporting "no scan"
    # because we did not wait is the failure mode this repo keeps finding.
    end = time.time() + 5.0
    while rclpy.ok() and time.time() < end and "s" not in latest:
        rclpy.spin_once(n, timeout_sec=0.05)
    if "s" not in latest:
        print(f"COULD NOT TELL: nothing on {a.topic}", file=sys.stderr)
        rclpy.shutdown()
        return 2

    deadline = time.time() + (0.0 if a.once else a.timeout)
    best = 0.0
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        d = clear_distance()
        if d is not None:
            best = max(best, d if math.isfinite(d) else 99.9)
            if d >= a.clear_m:
                if not a.quiet:
                    shown = "inf" if math.isinf(d) else f"{d:.2f} m"
                    print(f"OPEN: {shown} clear ahead over {a.sector_deg:.0f} deg "
                          f"(threshold {a.clear_m:.2f} m)")
                rclpy.shutdown()
                return 0
        if time.time() >= deadline:
            break
    if not a.quiet:
        print(f"STILL SHUT: best {best:.2f} m clear ahead, needed {a.clear_m:.2f} m")
    rclpy.shutdown()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
