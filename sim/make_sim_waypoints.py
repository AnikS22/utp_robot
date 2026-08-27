#!/usr/bin/env python3
"""Write the sim benchmark waypoints, in the SIM's odom frame, from the scene's known geometry.

    ROS_DOMAIN_ID=42 UTP_WAYPOINTS=maps/waypoints_sim.yaml python3 sim/make_sim_waypoints.py

On hardware waypoints are recorded by a human piloting the robot; the sim equivalent of that
human knowledge is the procedural scene's layout constants (scene_gen/scenes.py):
    wall front face x=1.4075, sliding double door centred y=0 (opening +-0.8),
    button at (1.3875, 1.55) facing -X, robot spawn (-0.9, 0) facing +X, goal (3, 0).

The odom frame need not equal the world frame (the bridge zeroes odom at spawn), so this reads
one synchronized (/odom, /scene/state robot_pose) pair and rigidly maps world->odom -- same math
as `waypoints.py rebase`.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy  # noqa: E402
import yaml  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402

# world-frame benchmark poses (x, y, yaw)
WORLD_WPS = {
    "start":   (-0.9, 0.0, 0.0),
    # 1.36 m off the wall face. NOT closer: the lidar rides 0.25 m ahead of base_link and the
    # corridor veto looks 0.90 m ahead, so a goal nearer than ~1.15 m to the door surface is
    # unreachable with the doors closed -- measured in sim 2026-08-27 (veto tripped at 0.19 m
    # short of a 1.06 m goal). The same margin applies to the REAL door waypoint.
    "door":    (0.05, 0.0, 0.0),
    "button":  (0.84, 1.55, 0.0),   # ~0.55 m standoff square to the button, camera on it
    "outside": (3.0, 0.0, 0.0),     # the scene goal
}


class Sync(Node):
    def __init__(self):
        super().__init__("utp_sim_waypoints")
        self.odom = None
        self.world = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(String, "/scene/state", self._scene, 10)

    def _odom(self, m):
        p = m.pose.pose
        yaw = math.atan2(2*(p.orientation.w*p.orientation.z + p.orientation.x*p.orientation.y),
                         1 - 2*(p.orientation.y**2 + p.orientation.z**2))
        self.odom = (p.position.x, p.position.y, yaw)

    def _scene(self, m):
        try:
            d = json.loads(m.data)
            rp = d.get("robot_pose")
            if rp:
                self.world = (rp["xy"][0], rp["xy"][1], rp["yaw"])
        except Exception:
            pass


def main() -> int:
    store = os.environ.get("UTP_WAYPOINTS")
    if not store:
        print("refusing: set UTP_WAYPOINTS -- this must never write the hardware waypoint file",
              file=sys.stderr)
        return 2
    rclpy.init()
    n = Sync()
    end = time.monotonic() + 15.0
    while time.monotonic() < end and not (n.odom and n.world):
        rclpy.spin_once(n, timeout_sec=0.2)
    if not (n.odom and n.world):
        print(f"no data: odom={n.odom} world={n.world} -- trial server up? scene built?",
              file=sys.stderr)
        n.destroy_node(); rclpy.shutdown()
        return 1
    xo, yo, ao = n.odom
    xw, yw, aw = n.world
    n.destroy_node(); rclpy.shutdown()

    dyaw = ao - aw
    c, s = math.cos(dyaw), math.sin(dyaw)
    tx = xo - (c * xw - s * yw)
    ty = yo - (s * xw + c * yw)

    out = {}
    for name, (x, y, yaw) in WORLD_WPS.items():
        out[name] = {"x": round(c * x - s * y + tx, 4),
                     "y": round(s * x + c * y + ty, 4),
                     "yaw": round(math.atan2(math.sin(yaw + dyaw), math.cos(yaw + dyaw)), 4),
                     "odom_epoch": round(time.time())}
    Path(store).parent.mkdir(parents=True, exist_ok=True)
    yaml.safe_dump(out, open(store, "w"), sort_keys=True)
    print(f"world->odom: dyaw={math.degrees(dyaw):+.1f} deg  t=({tx:+.3f},{ty:+.3f})")
    for k, v in sorted(out.items()):
        print(f"  {k:<8} x={v['x']:+7.3f} y={v['y']:+7.3f} yaw={math.degrees(v['yaw']):+6.1f} deg")
    print(f"-> {store}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
