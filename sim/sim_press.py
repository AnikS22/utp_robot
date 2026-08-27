#!/usr/bin/env python3
"""The press action, SIM edition: same perception, different arm.

    UTP_SIM=1 route_run.py ... -> this script (see ACTIONS in route_run.py)

What is identical to hardware: grab_frame captures the frame, detect_frame (GDINO, pipeline
venv) grounds the query and writes detection.json, and the action REFUSES to move without a
grounded 3D point. What differs: the reach itself goes to the Isaac trial server's
/arm_reach/goal (PointStamped in base_link) and the server's IK does approach/press/retreat,
instead of the xArm SDK. The server physically depresses the button and opens the door when the
press lands -- so a wrong 3D point FAILS the mission, exactly like hardware.

Flags mirror press_run.sh so route_run can pass the same params. --standoff is accepted but the
server owns the standoff in sim (6 cm, REACH_STANDOFF); we note when they differ.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy  # noqa: E402
from geometry_msgs.msg import PointStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

VENV = Path.home() / "unlocking-the-path" / "env" / ".venv" / "bin" / "python"
RESULT_TIMEOUT_S = 150.0     # arm phases run in SIM time; headless RTF can be well under 1


def quat_to_R(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


class Press(Node):
    def __init__(self):
        super().__init__("utp_sim_press")
        from tf2_ros import Buffer, TransformListener
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self, spin_thread=False)
        self.result = None
        self.scene = None
        self.pub = self.create_publisher(PointStamped, "/arm_reach/goal", 10)
        self.create_subscription(Bool, "/arm_reach/result", self._on_result, 10)
        self.create_subscription(String, "/scene/state", self._on_scene, 10)

    def _on_result(self, m):
        self.result = bool(m.data)

    def _on_scene(self, m):
        try:
            self.scene = json.loads(m.data)
        except Exception:
            pass

    def to_base(self, frame: str, p_cam):
        """point in camera frame -> base_link, via the bridge's TF."""
        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tfb.can_transform("base_link", frame, rclpy.time.Time()):
                break
        else:
            return None
        t = self.tfb.lookup_transform("base_link", frame, rclpy.time.Time())
        q, v = t.transform.rotation, t.transform.translation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        return [sum(R[i][j] * p_cam[j] for j in range(3)) + [v.x, v.y, v.z][i]
                for i in range(3)]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--query", default="the door release button on the wall")
    ap.add_argument("--standoff", type=float, default=60.0)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    name = f"sim_press_{int(time.time())}"
    cap = REPO / "captures" / name

    env = dict(os.environ, UTP_DEPTH_TOPIC="/mast_cam/depth/image_rect_raw")
    r = subprocess.run([sys.executable, str(REPO / "bringup" / "grab_frame.py"),
                        "--name", name, "--timeout", "45"], env=env)
    if r.returncode != 0:
        print("sim_press: no frame from the sim camera", file=sys.stderr)
        return 2

    r = subprocess.run([str(VENV), str(REPO / "bringup" / "detect_frame.py"),
                        str(cap), "--query", a.query])
    det_file = cap / "detection.json"
    if r.returncode != 0 or not det_file.exists():
        print("sim_press: grounding failed -- no detection.json, refusing to reach",
              file=sys.stderr)
        return 3
    det = json.loads(det_file.read_text())
    p_cam = det["point3d_cam_m"]
    cam_frame = json.loads((cap / "cam.json").read_text()).get("frame", "mast_cam_optical")
    print(f"sim_press: grounded {a.query!r} at cam {['%.3f' % v for v in p_cam]} "
          f"(score {det.get('score')})")
    if abs(a.standoff - 60.0) > 1e-6:
        print(f"  note: --standoff {a.standoff} accepted but the trial server owns the sim "
              f"standoff (6 cm)")
    if a.dry_run:
        print("sim_press: DRY RUN, not reaching")
        return 0

    rclpy.init()
    n = Press()
    try:
        p_base = n.to_base(cam_frame, p_cam)
        if p_base is None:
            print(f"sim_press: no TF base_link <- {cam_frame}", file=sys.stderr)
            return 4
        print(f"sim_press: target in base_link {['%.3f' % v for v in p_base]}")
        door_before = (n.scene or {}).get("door_open")

        msg = PointStamped()
        msg.header.frame_id = "base_link"
        msg.point.x, msg.point.y, msg.point.z = p_base
        n.pub.publish(msg)
        print("sim_press: goal published, waiting for /arm_reach/result ...")

        end = time.monotonic() + RESULT_TIMEOUT_S
        while time.monotonic() < end and n.result is None:
            rclpy.spin_once(n, timeout_sec=0.2)
        if n.result is None:
            print("sim_press: no result before timeout", file=sys.stderr)
            return 5
        # let a couple of /scene/state ticks land so door_open reflects the press
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.2)
        door_after = (n.scene or {}).get("door_open")
        print(f"sim_press: result={n.result}  door_open {door_before} -> {door_after}")
        return 0 if n.result else 6
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
