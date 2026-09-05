#!/usr/bin/env python3
"""Drop the OS0's near-field crosstalk from the cloud, before anything projects it to a scan.

    python3 safety/cloud_artifact_filter.py
    UTP_ART_IN=/ouster/points UTP_ART_OUT=/ouster/points_clean \
    UTP_ART_MAX_RANGE=1.4 UTP_ART_MAX_REFL=1 python3 safety/cloud_artifact_filter.py

WHAT THE ARTIFACT IS. Measured on hardware 2026-09-05, in an empty room:

    rear-arc points at 0.85-1.35 m horizontal
        reflectivity   min 1, median 1, max 1     -- EVERY point at the sensor's floor value
        rings          15 of 128, all downward-looking
        height         0.51-0.89 m above the ground
        persistence    present in every single scan

A real surface a metre away returns strongly and with variation. A constant reflectivity of 1
across every point is stray light -- the OS0's own emission bleeding into adjacent channels in the
near field. It is not the robot's mast, which is what it was mistaken for: the giveaway is that
reflectivity never varies.

WHY THE TWO EARLIER FIXES DID NOT WORK, because both were reasonable and both failed:

  A TEMPORAL FILTER (safety/scan_temporal_filter.py) keeps returns that persist across frames.
  This artifact is in EVERY frame, so it passed straight through -- measured, /scan_nav came out
  byte-identical to /scan while Nav2 had already been switched to consume it. A temporal filter
  cannot remove a persistent artifact.

  A WIDER GEOMETRIC MASK (scan_relay's UTP_MASK_MAX_M, pushed 0.90 -> 1.30 m) does remove what it
  covers, but the artifact extends past the boundary: with the mask at 1.30 m the nearest returns
  land at exactly 1.30-1.31 m, piled against the edge. Chasing it outward only blinds the robot
  further astern, and rear vision is what lift entry needs.

WHY RANGE ALONE OR REFLECTIVITY ALONE WILL NOT DO IT EITHER. Measured over a full cloud:
  reflectivity <= 2 covers 44.7% of near returns but also 26.1% of FAR ones, so a global
  reflectivity threshold eats real walls. Range alone is the geometric mask above. The two
  together are clean, because the artifact exists ONLY in the near field:

    inside the 0.20-1.20 m projection band (14,236 pts)
      near (<1.4 m) reflectivity <=1 :   690    <- dropped here
      near (<1.4 m) reflectivity  >1 :  1151    <- kept
      far  (>=1.4 m) reflectivity <=1 :  2559   <- untouched, the rule is near-only

This runs BEFORE pointcloud_to_laserscan so both /scan and /scan_nav inherit the fix, and the
geometric mask can go back to covering only what it was written for (the chassis at 0.39-0.85 m).

Fails safe: any point whose fields cannot be read is KEPT. Dropping a real obstacle is the one
error this must not make.
"""
from __future__ import annotations

import os
import struct
import sys

IN_TOPIC = os.environ.get("UTP_ART_IN", "/ouster/points")
OUT_TOPIC = os.environ.get("UTP_ART_OUT", "/ouster/points_clean")
MAX_RANGE = float(os.environ.get("UTP_ART_MAX_RANGE", 1.4))
MAX_REFL = int(os.environ.get("UTP_ART_MAX_REFL", 1))
LOG_PERIOD_S = 5.0


def filter_cloud(data: bytes, point_step: int, n_points: int, off_x: int, off_refl: int,
                 max_range: float, max_refl: int) -> tuple[bytearray, int]:
    """Return (kept_bytes, n_dropped). Pure bytes in, pure bytes out -- testable without ROS.

    VECTORISED ON PURPOSE. The first version looped in Python over every point. The OS0 sends
    512 x 128 = 65,536 points per cloud at 10 Hz, and that loop ran the whole chain down to
    0.50 Hz on /ouster/points_clean against 3.74 Hz in -- the filter became a worse problem than
    the artifact it removes. numpy reads the fields as strided views over the same buffer, so the
    per-cloud work is three comparisons over an array instead of 65,536 interpreted iterations.
    """
    import numpy as np
    n = min(n_points, len(data) // point_step) if point_step else 0
    if n <= 0:
        return bytearray(data), 0
    buf = np.frombuffer(data, dtype=np.uint8, count=n * point_step).reshape(n, point_step)
    # x,y are float32 at off_x; reflectivity is uint8 at off_refl. Copy only those columns.
    xy = np.ascontiguousarray(buf[:, off_x:off_x + 8]).view(np.float32).reshape(n, 2)
    refl = buf[:, off_refl]
    with np.errstate(invalid="ignore"):
        near = (xy[:, 0] * xy[:, 0] + xy[:, 1] * xy[:, 1]) < (max_range * max_range)
    finite = xy[:, 0] == xy[:, 0]          # NaN x means no return; leave those to the projection
    drop = near & finite & (refl <= max_refl)
    if not drop.any():
        return bytearray(data[:n * point_step]), 0
    kept = buf[~drop]
    return bytearray(kept.tobytes()), int(drop.sum())


def main() -> int:
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = Node("utp_cloud_artifact_filter")
    pub = node.create_publisher(PointCloud2, OUT_TOPIC, qos_profile_sensor_data)
    state = {"n": 0, "dropped": 0, "last": 0.0, "warned": False}

    def on_cloud(msg: PointCloud2) -> None:
        off = {f.name: f.offset for f in msg.fields}
        if "x" not in off or "reflectivity" not in off:
            if not state["warned"]:
                node.get_logger().warn(
                    f"{IN_TOPIC} has no 'reflectivity' field (fields: {list(off)}); "
                    f"passing the cloud through UNFILTERED rather than guessing.")
                state["warned"] = True
            pub.publish(msg)
            return
        n_pts = msg.width * msg.height
        kept, dropped = filter_cloud(bytes(msg.data), msg.point_step, n_pts,
                                     off["x"], off["reflectivity"], MAX_RANGE, MAX_REFL)
        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.is_dense = msg.is_dense
        # An unorganised cloud: the projection does not care about the 512x128 grid, and rebuilding
        # it after removing scattered points would mean padding with NaNs for no benefit.
        out.height = 1
        out.width = len(kept) // msg.point_step if msg.point_step else 0
        out.row_step = len(kept)
        out.data = bytes(kept)
        pub.publish(out)

        state["n"] += 1
        state["dropped"] += dropped
        now = time.time()
        if now - state["last"] >= LOG_PERIOD_S:
            state["last"] = now
            node.get_logger().info(
                f"{IN_TOPIC} -> {OUT_TOPIC}: {state['n']} clouds, dropped {dropped} pts this cloud "
                f"({state['dropped']} total) with range < {MAX_RANGE} m and reflectivity <= {MAX_REFL}")

    node.create_subscription(PointCloud2, IN_TOPIC, on_cloud, qos_profile_sensor_data)
    print(f"  {IN_TOPIC} -> {OUT_TOPIC}: dropping range < {MAX_RANGE} m AND reflectivity <= {MAX_REFL}",
          flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
