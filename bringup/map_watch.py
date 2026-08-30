#!/usr/bin/env python3
"""Live: is the mapping drive actually being recorded? Read-only, moves nothing.

    python3 bringup/map_watch.py

Run it in a terminal while you drive. It shows, once a second, the two INDEPENDENT answers to
"did the robot move" -- the wheels, and the lidar -- plus whether MOLA is laying down keyframes.

WHY BOTH. They fail differently and the difference is the diagnosis:

  wheels move, lidar does not   MOLA is not tracking. Featureless space, too-fast motion, or the
                                cloud stopped arriving. The map is being corrupted; stop.
  lidar moves, wheels do not    the robot is being pushed or carried, or the chassis encoders are
                                dead. The map is still fine -- MOLA needs no odometry.
  neither moves                 nothing is being recorded. Usually the RC: with SWB in the wrong
                                position the sticks do nothing, and odom, MOLA and every node look
                                perfectly healthy while the robot sits still.
  both move, keyframes climb    it is working.

KEYFRAMES ARE THE THING TO WATCH, not the local map size. /lidar_odometry/localmap_points is a
SLIDING LOCAL map with a bounded point count -- it plateaus around 20-30k and stays there no
matter how far you drive, which looks like a stall and is not one. The map that gets saved is the
keyframe set, and keyframes are only laid down when the robot has MOVED far enough.
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

from _ros_env import require_ros      # noqa: E402
require_ros()

import rclpy                          # noqa: E402
from nav_msgs.msg import Odometry     # noqa: E402
from rclpy.node import Node           # noqa: E402
from rclpy.qos import qos_profile_sensor_data   # noqa: E402
from sensor_msgs.msg import PointCloud2         # noqa: E402
from std_msgs.msg import Float32                # noqa: E402

KEYFRAME_MIN_TRAVEL_M = 0.05     # below this the mover is noise, not a drive


class Watch(Node):
    def __init__(self) -> None:
        super().__init__("utp_map_watch")
        self.odom0 = None
        self.odom = None
        self.mola0 = None
        self.mola = None
        self.quality = None
        self.cloud_n = 0
        self.map_pts = 0
        self.odom_path = 0.0
        self.mola_path = 0.0
        self._po = None
        self._pm = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(Odometry, "/lidar_odometry/pose", self._mola, 10)
        self.create_subscription(Float32, "/lidar_odometry/pose_quality", self._q, 10)
        self.create_subscription(PointCloud2, "/ouster/points", self._cloud,
                                 qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/lidar_odometry/localmap_points", self._map,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self._print)
        self.t0 = time.monotonic()

    def _xy(self, m):
        p = m.pose.pose.position
        return (p.x, p.y)

    def _odom(self, m):
        p = self._xy(m)
        if self.odom0 is None:
            self.odom0 = p
        if self._po is not None:
            self.odom_path += math.dist(p, self._po)
        self._po = p
        self.odom = p

    def _mola(self, m):
        p = self._xy(m)
        if self.mola0 is None:
            self.mola0 = p
        if self._pm is not None:
            self.mola_path += math.dist(p, self._pm)
        self._pm = p
        self.mola = p

    def _q(self, m):
        self.quality = float(m.data)

    def _cloud(self, m):
        self.cloud_n += 1

    def _map(self, m):
        self.map_pts = m.width * m.height

    def _print(self):
        od = math.dist(self.odom, self.odom0) if self.odom and self.odom0 else 0.0
        md = math.dist(self.mola, self.mola0) if self.mola and self.mola0 else 0.0
        moving_o = self.odom_path > KEYFRAME_MIN_TRAVEL_M
        moving_m = self.mola_path > KEYFRAME_MIN_TRAVEL_M
        if moving_o and moving_m:
            verdict = "RECORDING -- both agree the robot is moving"
        elif moving_m and not moving_o:
            verdict = "lidar moving, WHEELS NOT -- pushed/carried, or encoders dead (map is fine)"
        elif moving_o and not moving_m:
            verdict = "wheels moving, LIDAR NOT TRACKING -- map is being corrupted, STOP"
        else:
            verdict = "NOTHING MOVING -- check SWB on the RC; nothing is being recorded"
        q = f"{self.quality:.2f}" if self.quality is not None else "?"
        print(f"[{time.monotonic()-self.t0:6.0f}s] "
              f"odom {od:6.2f} m (path {self.odom_path:6.2f})  "
              f"lidar {md:6.2f} m (path {self.mola_path:6.2f})  "
              f"q {q}  cloud {self.cloud_n:5d}  localmap {self.map_pts:6d}\n"
              f"          {verdict}", flush=True)


def main() -> int:
    rclpy.init()
    n = Watch()
    print("watching -- Ctrl-C to stop. Drive the robot.\n"
          "NOTE: localmap plateaus around 20-30k by design (it is a SLIDING local map).\n"
          "      The saved map is the keyframe set; distance travelled is what matters.\n")
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
