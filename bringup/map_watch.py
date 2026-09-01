#!/usr/bin/env python3
"""Live: is the mapping drive actually being recorded? Read-only, moves nothing.

    python3 bringup/map_watch.py

Run it in a terminal while you drive. Once a second it shows the two INDEPENDENT answers to "is
this drive being recorded" -- the wheels, and the map -- because they fail differently and the
difference is the diagnosis:

  wheels move, map does not grow   slam_toolbox is not matching. Featureless space, too-fast
                                   motion, or /scan stopped arriving. Nothing is being recorded.
  map grows, wheels do not         the robot is being pushed or carried, or the chassis encoders
                                   are dead. The map is still fine.
  neither moves                    usually the RC: with SWB in the wrong position the sticks do
                                   nothing, and odom, slam_toolbox and every node look perfectly
                                   healthy while the robot sits still.
  both move                        it is working.

WHY OCCUPIED CELLS, NOT MAP DIMENSIONS. /map's width x height is the bounding box of everything
explored, and it JUMPS the moment a single stray beam lands far away -- so it grows convincingly
while the robot is parked. Occupied cells only rise when new wall is actually observed and
matched into the graph, which is what "recorded" means.

THIS WATCHES slam_toolbox. It used to watch MOLA (/lidar_odometry/pose, /lidar_odometry/
localmap_points), which nothing in this stack publishes -- so the lidar column read zero all the
way through a perfectly good drive and the verdict line said "map is being corrupted, STOP".
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy                          # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry   # noqa: E402
from rclpy.node import Node           # noqa: E402
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,  # noqa: E402
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan             # noqa: E402

MOVING_M = 0.05          # below this the mover is noise, not a drive
OCC_THRESHOLD = 50       # OccupancyGrid: >=50 is occupied, -1 unknown, 0 free

# /map is latched (transient local) and RELIABLE. Subscribing with the default profile gets you
# nothing until the next update, which on a 5 s map_update_interval looks like a dead topic.
MAP_QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class Watch(Node):
    def __init__(self) -> None:
        super().__init__("utp_map_watch")
        self.odom0 = self.odom = self._prev = None
        self.odom_path = 0.0
        self.scan_n = 0
        self.scan_valid = 0
        self.occ = 0
        self.occ0 = None
        self.grid = ""
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self._map, MAP_QOS)
        self.create_timer(1.0, self._print)
        self.t0 = time.monotonic()

    def _odom(self, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.odom0 is None:
            self.odom0 = p
        if self._prev is not None:
            self.odom_path += math.dist(p, self._prev)
        self._prev = self.odom = p

    def _scan(self, m):
        self.scan_n += 1
        # A LaserScan arriving is not a LaserScan with data in it: a lidar streaming to the wrong
        # udp_dest still publishes, at the right rate, entirely full of inf.
        self.scan_valid = sum(1 for r in m.ranges
                              if m.range_min <= r <= m.range_max and r == r)

    def _map(self, m):
        self.occ = sum(1 for v in m.data if v >= OCC_THRESHOLD)
        if self.occ0 is None:
            self.occ0 = self.occ
        self.grid = f"{m.info.width}x{m.info.height}"

    def _print(self):
        od = math.dist(self.odom, self.odom0) if self.odom and self.odom0 else 0.0
        grew = self.occ - self.occ0 if self.occ0 is not None else 0
        moving = self.odom_path > MOVING_M
        mapping = grew > 0
        if self.scan_valid == 0 and self.scan_n:
            verdict = "/scan is ARRIVING BUT EMPTY -- lidar streaming to the wrong host? STOP"
        elif not self.scan_n:
            verdict = "NO /scan AT ALL -- the relay is not running (bringup/scan_relay.py)"
        elif self.occ0 is None:
            verdict = "no /map yet -- slam_toolbox not activated? ros2 lifecycle get /slam_toolbox"
        elif moving and mapping:
            verdict = "RECORDING -- wheels and map agree"
        elif mapping and not moving:
            verdict = "map growing, WHEELS NOT -- pushed/carried, or encoders dead (map is fine)"
        elif moving and not mapping:
            verdict = "wheels moving, MAP NOT GROWING -- nothing is being recorded, STOP"
        else:
            verdict = "NOTHING MOVING -- check SWB on the RC; nothing is being recorded"
        print(f"[{time.monotonic()-self.t0:6.0f}s] "
              f"odom {od:6.2f} m (path {self.odom_path:6.2f})  "
              f"scan {self.scan_n:5d} msgs / {self.scan_valid:5d} valid beams  "
              f"map {self.grid or '-':>10s} +{grew:6d} occupied\n"
              f"          {verdict}", flush=True)


def main() -> int:
    rclpy.init()
    n = Watch()
    print("watching -- Ctrl-C to stop. Drive the robot.\n"
          "Occupied cells are the signal: they rise only when new wall is matched into the graph.\n"
          "Map DIMENSIONS grow on a single stray beam, so ignore them.\n"
          "CLOSE THE LOOP -- drive back past where you started, or the map comes out bent.\n")
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
