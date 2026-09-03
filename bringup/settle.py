#!/usr/bin/env python3
"""Hold until the map->base_link pose stops moving, or the budget runs out.

    python3 bringup/settle.py <label> [max_seconds]

WHY. slam_toolbox searches with coarse_angle_resolution 0.0349 rad (2.0 deg) and /scan runs about
2 Hz on this stack, so a brisk turn puts far more than 2 deg between consecutive scans and the pose
estimate slides during the rotation. The controller then drives against a stale estimate. On
2026-09-03 that presented as stuttering and a light wall contact while reversing into the lift car,
with the operator confirming the robot had never actually been in a wall.

Capping wz_max would slow every leg, including the straight ones, to fix a problem that only
happens while turning. Standing still afterwards costs nothing and is the one condition under which
a slow scan rate does not hurt: no motion, so no drift, and the matcher gets as many scans as it
needs to converge.

Exit is always 0. A leg that never settles is REPORTED, not fatal -- the residual per-sample motion
of a robot that is supposed to be stopped is drift, and it is worth seeing in the log next to the
leg that produced it rather than aborting a run over it.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
import tf2_ros

STILL_M = 0.01                 # 1 cm between samples
STILL_RAD = math.radians(1.0)  # 1 degree between samples
STILL_FOR_S = 1.0              # must hold that quiet for this long


def main() -> int:
    label = sys.argv[1] if len(sys.argv) > 1 else "?"
    budget = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    rclpy.init()
    n = Node("utp_settle")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)

    def pose():
        t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        q, v = t.transform.rotation, t.transform.translation
        return v.x, v.y, 2.0 * math.atan2(q.z, q.w)

    t0 = time.time()
    prev = None
    still = 0.0
    worst = (0.0, 0.0)
    ok = False
    while time.time() - t0 < budget:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            cur = pose()
        except Exception:
            continue
        if prev is not None:
            d = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            dth = abs((cur[2] - prev[2] + math.pi) % (2 * math.pi) - math.pi)
            worst = (max(worst[0], d), max(worst[1], dth))
            if d < STILL_M and dth < STILL_RAD:
                still += 0.2
                if still >= STILL_FOR_S:
                    print(f"  settled after {time.time() - t0:.1f}s")
                    ok = True
                    break
            else:
                still = 0.0
        prev = cur

    if not ok:
        print(f"  WARNING: '{label}' never settled in {budget:.0f}s -- worst step "
              f"{worst[0]*100:.1f} cm / {math.degrees(worst[1]):.1f} deg between samples. "
              f"The robot is stopped, so that is POSE DRIFT, not motion.")
    n.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
