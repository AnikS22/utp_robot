#!/usr/bin/env python3
"""Capture one aligned RGB-D frame from the D435 to disk, for the grounder to work on.

    python3 bringup/grab_frame.py                    # -> captures/frame_<n>/{rgb.png,depth.npy,cam.json}
    python3 bringup/grab_frame.py --name ada_door_1

Runs under ROS's python (needs rclpy/cv_bridge). The grounder runs under the pipeline venv
(needs torch) -- those are different interpreters and cannot share a process, so perception is
split into grab-then-detect with files in between. That split is a feature for this experiment:
the exact frame the detector saw is kept, so a result can be re-examined, re-run against another
backend, or put in the paper.

ALIGNED depth, not raw. /mast_cam/aligned_depth_to_color/image_raw is depth resampled into the
COLOR camera's frame, so depth[y, x] corresponds to rgb[y, x]. The raw depth stream
(/mast_cam/depth/image_rect_raw) is 848x480 in the DEPTH sensor's frame and is offset from the
colour image by the stereo baseline -- indexing it with colour pixel coordinates silently reads
the wrong part of the scene, and on a small target like a button that is the whole target.

Depth is uint16 millimetres on the wire and float32 metres in the .npy, because the rest of the
stack is metres (HARDWARE_SPECS: the xArm SDK is the only millimetre thing here and it is
converted at its own boundary). Zero means NO RETURN, not zero distance -- it is preserved as NaN
so that averaging cannot quietly pull a measurement toward the camera.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))
from _ros_env import require_ros
require_ros()

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_NS = os.environ.get("UTP_CAMERA_NS", "mast_cam")


def _decode(msg: Image) -> np.ndarray:
    """Image -> ndarray without cv_bridge (one less thing that must be installed and matching)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        a = buf.reshape(msg.height, msg.step // 1)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return a[:, :, ::-1] if msg.encoding == "bgr8" else a
    if msg.encoding in ("16UC1", "mono16"):
        a = buf.view(np.uint16).reshape(msg.height, msg.step // 2)[:, : msg.width]
        return a
    if msg.encoding == "32FC1":
        # Isaac's bridge publishes depth as float32 METRES (distance_to_image_plane on the color
        # render product) -- already aligned to color, no /1000 needed downstream.
        a = buf.view(np.float32).reshape(msg.height, msg.step // 4)[:, : msg.width]
        return a
    raise ValueError(f"unhandled encoding {msg.encoding}")


class Grabber(Node):
    def __init__(self, ns: str, settle: int):
        super().__init__("utp_grab_frame")
        self.rgb = None
        self.depth = None
        self.info = None
        self.n_rgb = self.n_depth = 0
        self.settle = settle
        self.create_subscription(Image, f"/{ns}/color/image_raw", self._on_rgb, qos_profile_sensor_data)
        # Hardware: realsense publishes depth resampled into the color frame on this topic.
        # Sim: the bridge renders depth ON the color camera (already aligned) under a different
        # name -- override with UTP_DEPTH_TOPIC=/mast_cam/depth/image_rect_raw for Isaac.
        depth_topic = os.environ.get("UTP_DEPTH_TOPIC",
                                     f"/{ns}/aligned_depth_to_color/image_raw")
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, f"/{ns}/color/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        # THE LIDAR RIDES ALONG WITH EVERY FRAME. Saved beside rgb/depth together with the
        # camera<-base<-lidar transforms, so a detector can lift a box to 3D from the lidar range
        # along that pixel's ray when depth has nothing: on glass (depth is garbage, the lidar sees
        # the tape) and in the Isaac sim on this laptop (depth publishes 100% inf on every frame,
        # measured 2026-08-29, while the RTX lidar is fine). Not a substitute for depth where depth
        # works; detect_frame uses it only when the depth lift fails, and says so.
        self.scan = None
        from sensor_msgs.msg import LaserScan
        self.create_subscription(LaserScan, os.environ.get("UTP_SCAN_TOPIC", "/scan_filtered"),
                                 lambda m: setattr(self, "scan", m), qos_profile_sensor_data)
        try:
            from tf2_ros import Buffer, TransformListener
            self.tfb = Buffer()
            self.tfl = TransformListener(self.tfb, self, spin_thread=False)
        except Exception:
            self.tfb = None

    def transform_matrix(self, target: str, source: str):
        """4x4 T such that p_target = T @ p_source, or None."""
        if self.tfb is None:
            return None
        try:
            import rclpy.time
            if not self.tfb.can_transform(target, source, rclpy.time.Time()):
                return None
            t = self.tfb.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None
        q, v = t.transform.rotation, t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
                      [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [v.x, v.y, v.z]
        return T

    def _on_rgb(self, msg):
        self.n_rgb += 1
        self.rgb = msg          # keep the LATEST; auto-exposure needs a few frames to settle

    def _on_depth(self, msg):
        self.n_depth += 1
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def ready(self) -> bool:
        # Wait for `settle` frames, not just one: the D435's auto-exposure and auto-white-balance
        # take roughly half a second to converge after a subscriber attaches, and a detector fed a
        # dark or colour-cast first frame fails for a reason that has nothing to do with the scene.
        return (self.rgb is not None and self.depth is not None and self.info is not None
                and self.n_rgb >= self.settle and self.n_depth >= self.settle)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ns", default=DEFAULT_NS)
    ap.add_argument("--name", default=None, help="output dir name under captures/")
    ap.add_argument("--settle", type=int, default=15, help="frames to discard before capturing")
    ap.add_argument("--timeout", type=float, default=20.0)
    a = ap.parse_args()

    rclpy.init()
    node = Grabber(a.ns, a.settle)
    deadline = time.monotonic() + a.timeout
    while time.monotonic() < deadline and not node.ready():
        rclpy.spin_once(node, timeout_sec=0.2)

    if not node.ready():
        print(f"timed out. rgb={node.n_rgb} depth={node.n_depth} "
              f"info={'yes' if node.info else 'no'}", file=sys.stderr)
        print(f"  Is the camera running? Expected topics under /{a.ns}/", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        return 1

    rgb = _decode(node.rgb).copy()
    depth_raw = _decode(node.depth)
    if depth_raw.dtype == np.float32:
        # 32FC1 (sim): already metres. Non-positive / non-finite = no return.
        depth_m = np.where(np.isfinite(depth_raw) & (depth_raw > 0), depth_raw,
                           np.nan).astype(np.float32)
    else:
        depth_mm = depth_raw.astype(np.float32)
        # 0 = NO RETURN. Keeping it as 0.0 metres would place the target AT the lens, and any
        # mean over a patch would be dragged toward the camera by exactly the pixels that failed.
        depth_m = np.where(depth_mm > 0, depth_mm / 1000.0, np.nan).astype(np.float32)

    K = np.array(node.info.k, dtype=np.float64).reshape(3, 3)
    name = a.name or f"frame_{int(time.time())}"
    out = os.path.join(REPO, "captures", name)
    os.makedirs(out, exist_ok=True)

    try:
        from PIL import Image as PILImage
        PILImage.fromarray(rgb).save(os.path.join(out, "rgb.png"))
    except ImportError:
        np.save(os.path.join(out, "rgb.npy"), rgb)

    np.save(os.path.join(out, "depth.npy"), depth_m)
    cam_frame = node.info.header.frame_id
    meta = {"K": K.tolist(), "frame": cam_frame,
            "width": int(node.info.width), "height": int(node.info.height),
            "rgb_shape": list(rgb.shape), "depth_shape": list(depth_m.shape)}
    # A few spins so TF and the scan have arrived, then save what a lidar lift needs.
    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.scan is not None:
        sc = node.scan
        with open(os.path.join(out, "scan.json"), "w") as f:
            json.dump({"frame": sc.header.frame_id, "angle_min": float(sc.angle_min),
                       "angle_increment": float(sc.angle_increment),
                       "ranges": [float(r) for r in sc.ranges]}, f)
        T_cb = node.transform_matrix(cam_frame, "base_link")
        T_bl = node.transform_matrix("base_link", sc.header.frame_id)
        if T_cb is not None and T_bl is not None:
            meta["T_cam_base"] = T_cb.tolist()
            meta["T_base_lidar"] = T_bl.tolist()
            meta["lidar_frame"] = sc.header.frame_id
    with open(os.path.join(out, "cam.json"), "w") as f:
        json.dump(meta, f, indent=2)

    valid = np.isfinite(depth_m)
    print(f"captured -> {out}")
    print(f"  rgb   {rgb.shape} {rgb.dtype}")
    print(f"  depth {depth_m.shape}  {100*valid.mean():.1f}% valid, "
          f"range {np.nanmin(depth_m):.2f}-{np.nanmax(depth_m):.2f} m")
    print(f"  frame {node.info.header.frame_id}  fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
    if rgb.shape[:2] != depth_m.shape[:2]:
        print("  WARNING: rgb and depth differ in size -- depth is NOT aligned to colour, so "
              "depth[y,x] does not correspond to rgb[y,x].", file=sys.stderr)

    node.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
