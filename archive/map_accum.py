#!/usr/bin/env python3
"""Accumulate the OS0 sweeps into a growing map you can watch fill in, in RViz.

    python3 bringup/map_accum.py            # publishes /map_accum (PointCloud2, frame: map)

WHY THIS EXISTS. Neither MOLA topic shows a map GROWING:
  /lidar_odometry/localmap_points   a SLIDING local map with a bounded point count. It plateaus
                                    around 20-30k and travels with the robot, so driving further
                                    does not make it bigger -- it just moves.
  /ouster/points                    one 10 Hz sweep, replaced every 100 ms.
The map that actually accumulates is MOLA's keyframe set, and MOLA does not publish it live.

So this does the obvious thing: take each sweep, put it in the MAP frame using the TF MOLA is
already publishing, drop it into a voxel grid, and republish the union. Watching that fill in as
you drive is the direct answer to "is it recording" -- and, unlike a number in a terminal, it also
shows you HOW WELL: a good drive thickens walls into clean surfaces, while a drive whose odometry
is slipping smears them into doubled, blurred ghosts. That is worth stopping for.

THIS IS A VIEWER, NOT THE MAP. Nothing here is saved and nothing feeds MOLA. The real map is the
keyframe set written by bringup/map_persist.sh. If this looks good, the saved map is good.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

from _ros_env import require_ros          # noqa: E402
require_ros()

import rclpy                              # noqa: E402
from rclpy.node import Node               # noqa: E402
from rclpy.qos import qos_profile_sensor_data   # noqa: E402
from rclpy.duration import Duration       # noqa: E402
from rclpy.time import Time               # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField   # noqa: E402
from sensor_msgs_py import point_cloud2 as pc2        # noqa: E402
from std_msgs.msg import Header           # noqa: E402
from tf2_ros import (ConnectivityException, ExtrapolationException,  # noqa: E402
                     LookupException)
from tf2_ros.buffer import Buffer                     # noqa: E402
from tf2_ros.transform_listener import TransformListener   # noqa: E402

MAP_FRAME = "map"


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]])


class Accum(Node):
    def __init__(self, voxel: float, stride: int, max_pts: int, rmin: float, rmax: float):
        super().__init__("utp_map_accum")
        self.voxel, self.stride, self.max_pts = voxel, stride, max_pts
        self.rmin, self.rmax = rmin, rmax
        self.grid: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(PointCloud2, "/map_accum", 1)
        self.create_subscription(PointCloud2, "/ouster/points", self._cloud,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self._publish)
        self.n_in = 0
        self.full = False

    def _cloud(self, m: PointCloud2) -> None:
        if self.full:
            return
        try:
            t = self.buf.lookup_transform(MAP_FRAME, m.header.frame_id, Time(),
                                          timeout=Duration(seconds=0.0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        pts = pc2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=False)
        p = np.asarray(pts, dtype=np.float32).reshape(-1, 3)[::self.stride]
        r = np.linalg.norm(p, axis=1)
        p = p[np.isfinite(r) & (r > self.rmin) & (r < self.rmax)]
        if not len(p):
            return
        tr, q = t.transform.translation, t.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        w = p @ R.T + np.array([tr.x, tr.y, tr.z], dtype=np.float32)
        keys = np.floor(w / self.voxel).astype(np.int32)
        for k, pt in zip(map(tuple, keys), w):
            if k not in self.grid:
                self.grid[k] = (float(pt[0]), float(pt[1]), float(pt[2]))
        self.n_in += 1
        if len(self.grid) >= self.max_pts and not self.full:
            self.full = True
            self.get_logger().warn(
                f"accumulator full at {len(self.grid)} voxels -- it has STOPPED adding. This is a "
                f"viewer limit only; MOLA's own map is unaffected. Raise --max-points or --voxel.")

    def _publish(self) -> None:
        if not self.grid:
            return
        arr = np.array(list(self.grid.values()), dtype=np.float32)
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = MAP_FRAME
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
        self.pub.publish(pc2.create_cloud(h, fields, arr))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--voxel", type=float, default=0.10, help="metres, default 0.10")
    ap.add_argument("--stride", type=int, default=3,
                    help="keep 1 point in N from each 131k sweep (default 3)")
    ap.add_argument("--max-points", type=int, default=1_500_000)
    ap.add_argument("--range-min", type=float, default=0.6,
                    help="drop returns from the robot's own mast and arm")
    ap.add_argument("--range-max", type=float, default=25.0)
    a = ap.parse_args()

    rclpy.init()
    n = Accum(a.voxel, a.stride, a.max_points, a.range_min, a.range_max)
    print(f"\n  accumulating /ouster/points -> /map_accum  (voxel {a.voxel} m, stride {a.stride})")
    print("  Add a PointCloud2 display on /map_accum in RViz, fixed frame 'map'.\n")
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
