#!/usr/bin/env python3
"""Re-express a target grounded EARLIER into the camera frame of the pose the robot holds NOW.

    python3 bringup/reproject_target.py captures/reach_XXXX          # reads target_odom.json
    python3 bringup/reproject_target.py --odom 16.142 15.208 --z 1.183 --cam captures/reach_XXXX

Writes captures/press_target.json, which press_run.sh prefers over grounding when it is fresh.
Runs under ROS python (needs /odom and /tf). Moves nothing.

WHY. At the press standoff the plate sits dead ahead at ~0.7 m -- exactly where the mast
camera's view is blocked by the robot's own stowed arm. Measured 2026-08-29: positioned
0.68 m / 0.0 deg, the re-ground from that pose returned the FIRE alarm (0.510) because the plate
was behind the housing; the veto refused it and the run ended one step from the press.
Re-grounding at the press pose is structurally blind here.

isaac_world does the right thing and says why: "lift the target with the OBSERVATION pose". The
plate was grounded and vetoed at 1.66 m; the base then moved ~1 m on odometry, which is accurate
to centimetres over that distance; the same world point is re-expressed where the arm now is.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Reproj(Node):
    def __init__(self) -> None:
        super().__init__("utp_reproject")
        self.pose = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        from tf2_ros import Buffer, TransformListener
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self, spin_thread=False)

    def _odom(self, m) -> None:
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def ready(self, frame: str, t: float = 8.0) -> bool:
        end = time.monotonic() + t
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.tfb.can_transform(frame, "base_link",
                                                                 rclpy.time.Time()):
                return True
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", nargs="?", type=Path,
                    help="capture dir holding target_odom.json (written by face_target)")
    ap.add_argument("--odom", nargs=2, type=float, metavar=("X", "Y"),
                    help="target in the odom frame (override / bootstrap)")
    ap.add_argument("--z", type=float, help="target height above base_link, metres")
    ap.add_argument("--cam", type=Path, help="capture dir with cam.json for K and the frame name")
    a = ap.parse_args()

    src = None
    if a.capture and (a.capture / "target_odom.json").exists():
        src = json.loads((a.capture / "target_odom.json").read_text())
        cam_dir = a.capture
    elif a.odom and a.z is not None and a.cam:
        src = {"odom_xy": a.odom, "z_base": a.z, "score": 0.0, "query": "(bootstrap)"}
        cam_dir = a.cam
    else:
        print("need a capture with target_odom.json, or --odom X Y --z Z --cam DIR",
              file=sys.stderr)
        return 2
    cam = json.loads((cam_dir / "cam.json").read_text())
    frame = cam.get("frame") or "mast_cam_color_optical_frame"
    K = cam["K"]
    fx, fy, cx, cy = K[0][0], K[1][1], K[0][2], K[1][2]

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = Reproj()
    try:
        if not n.ready(frame):
            print("no /odom or no TF base_link <- camera", file=sys.stderr)
            return 1
        rx, ry, rth = n.pose
        tx, ty = float(src["odom_xy"][0]), float(src["odom_xy"][1])
        c, s = math.cos(rth), math.sin(rth)
        bx = c * (tx - rx) + s * (ty - ry)
        by = -s * (tx - rx) + c * (ty - ry)
        bz = float(src["z_base"])

        t = n.tfb.lookup_transform(frame, "base_link", rclpy.time.Time())
        q, v = t.transform.rotation, t.transform.translation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
                      [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                      [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]])
        pc = R @ np.array([bx, by, bz]) + np.array([v.x, v.y, v.z])
        u = cx + fx * pc[0] / pc[2]
        vv = cy + fy * pc[1] / pc[2]
        half = 0.5 * fx * 0.12 / pc[2]        # a ~12 cm plate, in pixels at this range

        out = {"query": src.get("query"), "backend": "reprojected", "frame": frame,
               "point3d_cam_m": [float(x) for x in pc],
               "bbox_px": [float(u-half), float(vv-half), float(u+half), float(vv+half)],
               "score": float(src.get("score", 0.0)),
               "source": f"target ({tx:+.3f}, {ty:+.3f}) odom, z {bz:+.3f}, re-expressed at "
                         f"robot pose ({rx:+.3f}, {ry:+.3f}, {math.degrees(rth):+.1f} deg)",
               "base_link": [bx, by, bz], "written_at": time.time()}
        dst = REPO / "captures" / "press_target.json"
        dst.write_text(json.dumps(out, indent=2))
        rng = math.hypot(bx, by)
        print(f"target in base_link now: ({bx:+.3f}, {by:+.3f}, {bz:+.3f})  "
              f"range {rng:.2f} m, bearing {math.degrees(math.atan2(by, bx)):+.1f} deg")
        print(f"camera frame           : ({pc[0]:+.3f}, {pc[1]:+.3f}, {pc[2]:+.3f})  "
              f"pixel ({u:.0f}, {vv:.0f})")
        print(f"-> {dst}")
        if rng > 0.88:
            print(f"WARNING: {rng:.2f} m is outside the 0.88 m arm envelope", file=sys.stderr)
            return 1
        return 0
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
