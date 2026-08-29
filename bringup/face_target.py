#!/usr/bin/env python3
"""Put a grounded control within arm reach: yaw to face it, step in to the press standoff.

    python3 bringup/face_target.py captures/button_probe --dry-run
    python3 bringup/face_target.py captures/button_probe --standoff 0.55

THE ROBOT DRIVES unless --dry-run. It steps toward a wall and stops at the standoff.

THE GAP THIS FILLS. approach_blockage stops the base ~0.55 m from the OBSTRUCTION -- the door.
The control that opens it is on the wall BESIDE the door, so from a door-facing pose it is
routinely outside the arm's 0.88 m envelope, and the grounder having found it perfectly changes
nothing. isaac_world.act() has solved this since it was written ("POSITION the base within arm
reach of the target, then let the arm IK make the final reach"; _approach_press_pose faces the
grounded button and steps in). ros_world.act() went straight from detection to the arm, so on
hardware the base never repositioned at all.

RE-GROUND AFTERWARDS, ALWAYS. The 3D point is expressed in the camera frame AT THE MOMENT OF
CAPTURE. Move the base and it is stale, and staleness here is silent: isaac_world records a case
where the base yawed 0.69 rad between observing and pressing, which swings a target 1 m out by
about half a metre -- the arm reached confidently at blank wall and the trial was booked as a
GROUNDING failure although the detector and the reasoner had both been right. The sim solves that
by lifting the point with the OBSERVATION pose. Hardware can do better: take a fresh frame from
the new pose and ground again, so the number the arm aims at was measured from where the arm
actually is. This tool therefore MOVES ONLY. It never hands a transformed point to the arm.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist, Vector3  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from safety.mux_watch import MuxWatch  # noqa: E402
from safety.reach_envelope import (APPROACH_BUDGET_S, ARM_REACH_M as ENVELOPE_M,  # noqa: E402
                                   MIN_LIDAR_RANGE_M, PRESS_STANDOFF_M, check_before_reach,
                                   press_pose_ok, stalled)
from safety.waypoint_drive import Limits, corridor_blocked, wrap  # noqa: E402

CMD_TOPIC = "/cmd_vel_teleop"
RATE_HZ = 20.0
ARM_REACH_M = 0.88          # xArm6 envelope with the riser fitted (HARDWARE_SPECS)
PRESS_STANDOFF_M = 0.55     # the press pose proven on 2026-08-25
YAW_TOL = math.radians(4.0)
POS_TOL_M = 0.06


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Facer(Node):
    def __init__(self) -> None:
        super().__init__("utp_face_target")
        self.pose = None
        self.stamp = 0.0
        self.scan = None
        self.tfb = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self._scan, qos_profile_sensor_data)
        self.create_subscription(String, "/safety/status", self._safety, 10)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.mux = MuxWatch(time.monotonic())
        from tf2_ros import Buffer, TransformListener
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self, spin_thread=False)

    def _odom(self, m) -> None:
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.stamp = time.monotonic()

    def _scan(self, m) -> None:
        self.scan = m

    def _safety(self, m) -> None:
        try:
            self.mux.note_status(json.loads(m.data).get("blocked_by"), time.monotonic())
        except (ValueError, TypeError):
            pass

    def wait(self, t: float = 8.0) -> bool:
        end = time.monotonic() + t
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None:
                return True
        return False

    def to_base(self, p_cam, frame: str, timeout: float = 5.0):
        """Camera optical frame -> base_link, via /tf. None if the transform never arrives."""
        import numpy as np
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.tfb.can_transform("base_link", frame, rclpy.time.Time()):
                break
        else:
            return None
        if not self.tfb.can_transform("base_link", frame, rclpy.time.Time()):
            return None
        t = self.tfb.lookup_transform("base_link", frame, rclpy.time.Time())
        q, v = t.transform.rotation, t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
        return (R @ np.asarray(p_cam, dtype=float)) + np.array([v.x, v.y, v.z])

    def stop(self) -> None:
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def nearest_ahead(self, half_deg: float = 15.0) -> float | None:
        """Nearest lidar return within +-half_deg of straight ahead, or None.

        Used ONLY as the hard floor in servo_to_press_pose. It was referenced there and never
        defined on this class -- the method lives on approach_blockage.Approach -- so the first
        live reposition crashed with AttributeError one line before it would have driven, with
        the plate grounded at 0.521 and the veto passed. 2026-08-29."""
        if self.scan is None:
            return None
        s = self.scan
        best = None
        for i, r in enumerate(s.ranges):
            if r != r or abs(r) == float("inf") or r <= 0:
                continue
            a = math.atan2(math.sin(s.angle_min + i * s.angle_increment),
                           math.cos(s.angle_min + i * s.angle_increment))
            if abs(math.degrees(a)) <= half_deg:
                best = r if best is None else min(best, r)
        return best

    def _drive(self, v: float, w: float) -> bool:
        self.pub.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=w)))
        now = time.monotonic()
        self.mux.note_command(not (v == 0.0 and w == 0.0), now)
        return self.mux.verdict(now).ok

    def servo_to_press_pose(self, tx: float, ty: float, lim: Limits) -> tuple[bool, str]:
        """Face the grounded target and step to the press standoff. Target-relative, on odometry.

        THIS IS isaac_world._approach_press_pose, PORTED. Its own comment says why it must not use
        the lidar: "Two interleaved controls, target-relative (NOT lidar -- the self-hit filter
        can't guard the < 0.30 m close range; PRESS_STANDOFF_X keeps the chassis clear of the
        door/wall)". My first hardware version serviced the final approach on lidar range, and
        corridor_blocked's 0.90 m look-ahead is measured from a sensor 0.318 m FORWARD of
        base_link -- so it halted the chassis 1.23 m from the plate, outside the 0.88 m arm, and
        the press faulted with ControllerError 21. The lidar is the wrong sensor for this job.

        (tx, ty) is the target in the ODOM frame, fixed at entry. It does not move as the robot
        does, which is the whole point -- a target re-derived from a fresh frame each tick is the
        4WS mode thrash that produced the 90-second livelock.

        Also ported: the honest exits. Latency here is a data-integrity concern in the sim's
        words -- "a doomed approach that eats the trial budget turns a would-be
        report_unreachable into a scored timeout" -- so it stops on evidence of not converging
        rather than running the clock out.
        """
        self.mux.resume(time.monotonic())
        t0 = time.monotonic()
        hist: list = []
        while rclpy.ok() and time.monotonic() - t0 < APPROACH_BUDGET_S:
            rclpy.spin_once(self, timeout_sec=1.0/RATE_HZ)
            if self.pose is None or (time.monotonic() - self.stamp) > 0.5:
                self.pub.publish(Twist())
                continue
            x, y, th = self.pose
            dist = math.hypot(tx - x, ty - y)
            yaw_err = wrap(math.atan2(ty - y, tx - x) - th)

            if press_pose_ok(dist, yaw_err):
                self.stop()
                return True, (f"positioned: {dist:.2f} m from the target, "
                              f"{math.degrees(yaw_err):+.1f} deg off the press axis")

            now = time.monotonic()
            hist.append((now, abs(dist), abs(yaw_err)))
            while len(hist) > 2 and now - hist[1][0] >= 4.0:
                hist.pop(0)
            if stalled(hist, now):
                self.stop()
                return False, (f"approach STALLED at {dist:.2f} m, "
                               f"{math.degrees(yaw_err):+.1f} deg -- neither error improved over "
                               f"4 s. The base is against something, or that point is not "
                               f"drivable-to. Driving longer cannot help.")

            # HARD FLOOR ONLY. The lidar no longer decides when to stop; it just refuses to let
            # the chassis close on anything inside MIN_LIDAR_RANGE_M.
            near = self.nearest_ahead()
            if near is not None and near < MIN_LIDAR_RANGE_M:
                self.stop()
                return False, (f"hard floor: something is {near:.2f} m ahead, closer than the "
                               f"{MIN_LIDAR_RANGE_M:.2f} m minimum. Not driving into it.")

            # Yaw first while badly off axis, then drive and steer together.
            w = max(-lim.w_max, min(lim.w_max, 1.2 * yaw_err))
            if abs(w) < 0.12 and abs(yaw_err) > YAW_TOL:
                w = math.copysign(0.12, w)
            v = 0.0 if abs(yaw_err) > 0.6 else min(0.10, max(0.0, dist - PRESS_STANDOFF_M))
            if 0.0 < v < 0.05:
                v = 0.05
            self.pub.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=w)))
            self.mux.note_command(not (v == 0.0 and w == 0.0), now)
            vd = self.mux.verdict(now)
            if not vd.ok:
                self.stop()
                return False, vd.reason
        self.stop()
        return False, f"approach timed out after {APPROACH_BUDGET_S:.0f}s"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", type=Path, help="capture dir holding detection.json")
    ap.add_argument("--standoff", type=float, default=PRESS_STANDOFF_M)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    det_file = a.capture / "detection.json"
    if not det_file.exists():
        print(f"no {det_file} -- ground the target first (detect_frame.py)", file=sys.stderr)
        return 2
    det = json.loads(det_file.read_text())
    p_cam = det.get("point3d_cam_m")
    if not p_cam:
        print("detection has no 3D point; refusing to move", file=sys.stderr)
        return 2
    frame = det.get("frame") or det.get("cam_frame") or "mast_cam_color_optical_frame"

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = Facer()
    lim = Limits()
    try:
        if not n.wait():
            print("no /odom", file=sys.stderr)
            return 1
        p_base = n.to_base(p_cam, frame)
        if p_base is None:
            print(f"no TF base_link <- {frame}", file=sys.stderr)
            return 1
        tx_b, ty_b = float(p_base[0]), float(p_base[1])
        dist = math.hypot(tx_b, ty_b)
        bear = math.atan2(ty_b, tx_b)
        step_in = dist - a.standoff

        print(f"  target in base_link: x={tx_b:+.3f} y={ty_b:+.3f} z={float(p_base[2]):+.3f}")
        print(f"  range {dist:.2f} m, bearing {math.degrees(bear):+.1f} deg")
        print(f"  arm reach is {ARM_REACH_M:.2f} m -> "
              + ("ALREADY IN REACH; no base move needed"
                 if dist <= ARM_REACH_M else
                 f"OUT OF REACH by {dist-ARM_REACH_M:.2f} m"))
        print(f"  plan: turn {math.degrees(bear):+.1f} deg, then advance {step_in:+.2f} m "
              f"to a {a.standoff:.2f} m standoff")
        if dist <= ARM_REACH_M and abs(bear) <= YAW_TOL:
            print("  nothing to do.")
            return 0
        if a.dry_run:
            print("\n  DRY RUN. Nothing moved.")
            return 0
        if step_in < -0.05:
            print("  target is CLOSER than the standoff; backing off is not implemented here",
                  file=sys.stderr)
            return 1

        # Lift the target into ODOM once, from the pose the observation was taken at. The
        # camera->base transform is rigid, so with the base stationary this is exact -- and
        # fixing it here is what lets the servo run without re-deriving a moving goal.
        rx, ry, rth = n.pose
        c, sn = math.cos(rth), math.sin(rth)
        tx = rx + c*tx_b - sn*ty_b
        ty = ry + sn*tx_b + c*ty_b
        print(f"  target in odom: ({tx:+.3f}, {ty:+.3f})   press standoff {PRESS_STANDOFF_M:.2f} m")
        # Persist it. The re-ground at the press pose is structurally blind (the plate sits
        # behind the robot's own stowed arm at 0.7 m dead ahead -- measured 2026-08-29, the
        # grounder returned the fire alarm and the veto refused it), so the ARM's target is this
        # world point re-expressed after positioning: bringup/reproject_target.py.
        (a.capture / "target_odom.json").write_text(json.dumps(
            {"odom_xy": [tx, ty], "z_base": float(p_base[2]), "frame": frame,
             "score": float(det.get("score", 0.0)), "query": det.get("query")}, indent=2))

        ok, why = n.servo_to_press_pose(tx, ty, lim)
        print(f"  approach: {'ok' if ok else 'FAILED'} {why}")
        if not ok:
            return 1

        rp = __import__("subprocess").run(
            [sys.executable, str(REPO / "bringup" / "reproject_target.py"), str(a.capture)],
            capture_output=True, text=True)
        for ln in (rp.stdout or rp.stderr or "").strip().splitlines():
            print(f"  {ln}")
        print("\n  positioned. RE-GROUND from here before reaching -- the old 3D point was "
              "measured from the pose you just left.")
        return 0
    finally:
        try:
            n.stop()
        except Exception:
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
