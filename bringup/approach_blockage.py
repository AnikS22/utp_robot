#!/usr/bin/env python3
"""Drive up to whatever is blocking the way, so perception happens at a usable range.

    python3 bringup/approach_blockage.py --dry-run
    python3 bringup/approach_blockage.py --stop-at 0.55

THE ROBOT DRIVES FORWARD unless --dry-run. It stops at the lidar veto and never commands contact.

WHY THIS EXISTS. utp/pipeline/fsm.py:421 calls world.approach_blockage() when the world provides
it. isaac_world does; ros_world did not, so the hasattr check silently skipped it on hardware and
perception ran from wherever navigation happened to stop. The sim version says exactly what that
costs:

    "Depth is invalid/noisy at 8-10 m -- grounding from there lifts the target to a garbage 3D
     point and the arm IKs out of reach."

Measured on this robot 2026-08-29, standing ~9 m off the doors: the camera saw "closed double
glass doors" correctly, and the VLM then abstained -- "I cannot see a button or a card reader on
or near the glass double doors." It was right. From 9 m the ADA plate is a few pixels and the
depth behind it is noise.

WHAT THIS REPLACES, and it matters for the experiment. Without it, getting the robot within arm
reach needs a PRE-RECORDED `button` waypoint -- the operator telling the robot where the control
is. That hollows out the grounding claim: the robot would not be finding the plate, it would be
driven to it. Approaching the blockage geometrically -- no map, no waypoint, just "close the gap
until the lidar says stop" -- gets the robot to a depth-valid, arm-reachable distance without
anyone naming where the button is.

It approaches the BLOCKAGE, not the button. It has no idea what a button is, and it must not:
the whole point is that finding the control is the grounder's job, from a frame taken here.
"""
from __future__ import annotations

import argparse
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
from safety.reach_envelope import MIN_LIDAR_RANGE_M  # noqa: E402
from safety.waypoint_drive import corridor_blocked  # noqa: E402

CMD_TOPIC = "/cmd_vel_teleop"
RATE_HZ = 20.0
V_APPROACH = 0.12          # slow: this drives deliberately AT something
MAX_ADVANCE_M = 6.0        # never close more than this without a fresh decision


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Approach(Node):
    def __init__(self) -> None:
        super().__init__("utp_approach_blockage")
        self.pose = None
        self.stamp = 0.0
        self.scan = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self._scan, qos_profile_sensor_data)
        self.create_subscription(String, "/safety/status", self._safety, 10)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.mux = MuxWatch(time.monotonic())

    def _odom(self, m) -> None:
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.stamp = time.monotonic()

    def _scan(self, m) -> None:
        self.scan = m

    def _safety(self, m) -> None:
        import json
        try:
            self.mux.note_status(json.loads(m.data).get("blocked_by"), time.monotonic())
        except (ValueError, TypeError):
            pass

    def wait_ready(self, timeout: float = 8.0) -> bool:
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None and self.scan is not None:
                return True
        return False

    def stop(self) -> None:
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def nearest_ahead(self, half_deg: float = 15.0) -> float | None:
        if self.scan is None:
            return None
        best = None
        s = self.scan
        for i, r in enumerate(s.ranges):
            if r != r or abs(r) == float("inf") or r <= 0:
                continue
            a = s.angle_min + i * s.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(math.degrees(a)) <= half_deg:
                best = r if best is None else min(best, r)
        return best

    def reverse(self, dist_m: float, dry: bool) -> tuple[bool, str]:
        """Back straight up by a fixed distance. True only if the base actually moved.

        WHY BACKING OFF IS A REAL RECOVERY. The reasoner's prompt forbids naming a control it
        cannot see, and an ADA plate sits BESIDE the door -- so the closer the robot gets to the
        DOOR, the further off-axis the plate swings. Measured 2026-08-29: from 0.54 m the plate
        was at the extreme left edge, half cut off, and the VLM and the grounder both missed it.
        From further back the same plate was 81x88 px and grounded at 0.489. When the robot is
        already too close, closing further cannot help; only reversing can.

        NO LIDAR BEHIND. The A1M8's rear sector is filtered out because it is the robot seeing
        ITSELF (measured self-hits at 0.17 m), so there is no obstacle check astern -- which is
        why this is bounded, slow, and deliberate, and never a general-purpose reverse.
        """
        if not (0.05 <= dist_m <= 1.0):
            return False, f"refusing to reverse {dist_m:.2f} m; bounded to 0.05-1.00 m"
        if dry:
            return True, f"DRY RUN: would reverse {dist_m:.2f} m"
        x0, y0, _ = self.pose
        self.mux.resume(time.monotonic())
        end = time.monotonic() + 40.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=1.0/RATE_HZ)
            if self.pose is None or (time.monotonic() - self.stamp) > 0.5:
                self.pub.publish(Twist())
                continue
            gone = math.hypot(self.pose[0]-x0, self.pose[1]-y0)
            if gone >= dist_m:
                self.stop()
                return True, f"reversed {gone:.2f} m"
            self.pub.publish(Twist(linear=Vector3(x=-V_APPROACH)))
            now = time.monotonic()
            self.mux.note_command(True, now)
            v = self.mux.verdict(now)
            if not v.ok:
                self.stop()
                return False, v.reason
        self.stop()
        gone = math.hypot(self.pose[0]-x0, self.pose[1]-y0)
        return (gone > 0.05), f"reverse timed out after {gone:.2f} m"

    def run(self, stop_at: float, dry: bool) -> tuple[bool, str]:
        x0, y0, _ = self.pose
        self.mux.resume(time.monotonic())
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)
            if self.pose is None or (time.monotonic() - self.stamp) > 0.5:
                self.pub.publish(Twist())
                continue
            x, y, _ = self.pose
            advanced = math.hypot(x - x0, y - y0)
            near = self.nearest_ahead()
            blocked = (self.scan is not None and
                       corridor_blocked(self.scan.ranges, self.scan.angle_min,
                                        self.scan.angle_increment))

            # STOP ON RANGE STRAIGHT AHEAD, NOT ON THE DRIVING VETO.
            #
            # corridor_blocked watches a 0.90 x 0.80 m BOX, so it latches on anything within
            # ~24 deg -- a bench, a pillar, the flanking wall -- and stops the robot before it
            # has closed on the thing it is actually approaching. Measured 2026-08-29: the FSM
            # logged "stopped 0.98 m from what is ahead after 0.00 m" and never moved, while the
            # doors it was approaching were 2.65 m away and the ADA plate 1.83 m. The reasoner was
            # then asked to identify a ~50 px disc from 1.83 m and, correctly, could not. From the
            # operator's side it "looked like it was going to the doors but really did nothing".
            #
            # This is the same mistake as letting the veto govern the press approach, one stage
            # earlier: a rule for not hitting unknown obstacles while driving is the wrong rule
            # for deliberately closing on a known one. nearest_ahead() uses a narrow +-15 deg, so
            # it tracks what is actually in front. The veto survives as a hard floor below.
            if near is not None and near <= stop_at:
                self.stop()
                return True, f"stopped {near:.2f} m from what is ahead after {advanced:.2f} m"
            if blocked and (near is None or near <= MIN_LIDAR_RANGE_M):
                # Hard floor: the veto is latched AND nothing usable is resolvable straight
                # ahead, so we are wedged against something. Stop regardless of the range test.
                self.stop()
                return True, (f"stopped on the corridor veto after {advanced:.2f} m "
                              f"(nothing resolvable straight ahead)")
            if advanced >= MAX_ADVANCE_M:
                self.stop()
                return False, (f"advanced {advanced:.2f} m without meeting anything -- the way "
                               f"ahead is open, so there is nothing here to approach")
            if dry:
                self.stop()
                return True, (f"DRY RUN: would advance at {V_APPROACH} m/s until "
                              f"{stop_at:.2f} m out; nearest ahead now "
                              f"{('%.2f m' % near) if near else 'nothing in range'}")
            t = Twist(); t.linear.x = V_APPROACH
            self.pub.publish(t)

            now = time.monotonic()
            self.mux.note_command(True, now)
            v = self.mux.verdict(now)
            if not v.ok:
                self.stop()
                return False, v.reason
        self.stop()
        return False, "interrupted"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stop-at", type=float, default=0.55,
                    help="metres from the obstruction to stop at (default 0.55, the press pose)")
    # THE ROBOT DRIVES UNLESS --dry-run, matching press_run.sh and approach_target.py. It is the
    # convention route_run's run_action() relies on: it appends --dry-run for a dry run and
    # nothing for a live one, so an action that needed an explicit --go would silently no-op in
    # the middle of a real trial and look like a robot that decided not to move.
    ap.add_argument("--back", type=float, default=0.0,
                    help="REVERSE this many metres instead of approaching. Used by widen_view: "
                         "when the robot is already too close to survey, closing further cannot "
                         "help and only backing off gets the blockage in frame.")
    ap.add_argument("--dry-run", action="store_true", help="plan and report; move nothing")
    ap.add_argument("--go", action="store_true", help="accepted for symmetry; motion is the "
                                                      "default and --dry-run is what stops it")
    a = ap.parse_args()
    if not (0.3 <= a.stop_at <= 2.0):
        print("--stop-at must be 0.3-2.0 m", file=sys.stderr)
        return 2

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = Approach()
    try:
        if not n.wait_ready():
            print("no /odom or no /scan_filtered -- is the stack up?", file=sys.stderr)
            return 1
        if a.back > 0.0:
            ok, why = n.reverse(a.back, dry=a.dry_run)
        else:
            ok, why = n.run(a.stop_at, dry=a.dry_run)
        print(f"approach_blockage: {why}")
        return 0 if ok else 1
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
