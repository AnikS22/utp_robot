#!/usr/bin/env python3
"""The fastest honest test of waypoint navigation: drive there, drive back, measure the error.

    python3 bringup/twopoint.py                 # DRY RUN: show the two points and the first move
    python3 bringup/twopoint.py --go            # one lap
    python3 bringup/twopoint.py --go --laps 10  # repeatability, with statistics

WHY THE POINTS ARE NOT WRITTEN DOWN. Waypoints live in the odom frame and odom re-zeroes on every
ranger_base restart, so any coordinate committed to a file is valid only until the next restart --
that is what made maps/waypoints.yaml silently wrong for three days. This test defines both points
at RUN TIME:

    P1 = wherever the robot is standing when you press go
    P2 = P1 projected --dist metres straight ahead of its current heading

Nothing is stored, so nothing can go stale, and the test is identical on the hundredth run as on
the first. No recording step, one command.

WHAT THE RETURN ERROR MEANS, AND WHAT IT DOES NOT. Returning to P1 closes the loop in ODOMETRY, so
the number printed is CONTROLLER error -- overshoot, turn hysteresis, arrival tolerance. It cannot
see odometry error itself: if the wheels under-report, P1 drifts with the robot and odom will
happily report a 2 cm return while the robot is 30 cm from where it started.

So: put a mark on the floor under the robot before the first lap. Odom error is the gap between
what this prints and what you measure against that mark, and on a 4WS chassis it accumulates in
TURNS, which is why the lap count matters more than the distance.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from safety.mux_watch import MuxWatch  # noqa: E402
from safety.waypoint_drive import Limits, corridor_blocked, plan_step, to_goal, wrap  # noqa: E402

CMD_TOPIC = "/cmd_vel_teleop"
RATE_HZ = 20.0
LEG_TIMEOUT_S = 120.0


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Driver(Node):
    def __init__(self) -> None:
        super().__init__("utp_twopoint")
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

    def wait_for_pose(self, timeout: float = 5.0) -> bool:
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None:
                return True
        return False

    def stop(self) -> None:
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def drive_to(self, gx: float, gy: float, gyaw: float, lim: Limits) -> tuple[bool, str]:
        deadline = time.monotonic() + LEG_TIMEOUT_S
        # Anything before this leg may have blocked the thread for seconds without spinning
        # (a --confirm prompt, an action subprocess, a wait). Staleness cannot tell "the mux
        # went quiet" from "we stopped listening", so the clocks start fresh here.
        self.mux.resume(time.monotonic())
        last = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=1.0/RATE_HZ)
            if self.pose is None or (time.monotonic() - self.stamp) > 0.5:
                self.pub.publish(Twist())
                continue
            x, y, th = self.pose
            dist, bear = to_goal(x, y, th, gx, gy)
            blocked = (self.scan is not None and
                       corridor_blocked(self.scan.ranges, self.scan.angle_min,
                                        self.scan.angle_increment))
            step = plan_step(dist, bear, wrap(gyaw - th), blocked, lim, prev_state=last or "")
            t = Twist(); t.linear.x = step.twist.vx; t.angular.z = step.twist.wz
            self.pub.publish(t)

            now = time.monotonic()
            self.mux.note_command(not (step.twist.vx == 0.0 and step.twist.wz == 0.0), now)
            v = self.mux.verdict(now)
            if not v.ok:
                self.stop()
                return False, v.reason

            if step.state != last:
                print(f"        [{step.state}] {dist:5.2f} m  bearing {math.degrees(bear):+6.1f} deg")
                last = step.state
            if step.state == "arrived":
                self.stop()
                return True, ""
            if step.state == "blocked":
                self.stop()
                return False, ("corridor blocked -- something is inside the 0.90 x 0.80 m box "
                               "ahead. If the way is visibly clear, suspect the lidar mount "
                               "height in config/lidar.yaml (CAD and config disagree by 34 cm).")
        self.stop()
        return False, f"leg timed out after {LEG_TIMEOUT_S:.0f}s"


def endpoint(n, label: str, gx: float, gy: float, gyaw: float, a) -> bool:
    """Report where odom thinks we are, and optionally hold so the operator can measure.

    ODOM CANNOT MEASURE ITS OWN ERROR. The numbers printed here are what the wheels believe. The
    measurement that matters is the tape from the robot to the floor mark, and it only exists if
    the robot STOPS at the endpoint long enough to take it -- which is why --pause exists.

    Returns False if the operator asked to stop.
    """
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)
    x, y, th = n.pose
    print(f"      at {label}: odom says x={x:+.3f} y={y:+.3f} yaw={math.degrees(th):+.1f} deg "
          f"| off target by {math.hypot(x-gx, y-gy)*100:.1f} cm, "
          f"{abs(math.degrees(wrap(th-gyaw))):.1f} deg")
    if a.pause:
        print(f"      MEASURE {label} against the floor mark now.")
        try:
            ans = input("      Enter to continue, q+Enter to stop: ")
        except EOFError:
            return False
        if ans.strip().lower().startswith("q"):
            return False
    elif a.dwell > 0:
        t_end = time.monotonic() + a.dwell
        while time.monotonic() < t_end and rclpy.ok():
            n.pub.publish(Twist())
            rclpy.spin_once(n, timeout_sec=0.05)
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dist", type=float, default=2.0, help="metres from P1 to P2 (default 2.0)")
    ap.add_argument("--laps", type=int, default=1, help="out-and-back cycles")
    ap.add_argument("--tol", type=float, default=None,
                    help="arrival radius in metres (default %.2f, the route value). The test "
                         "cannot resolve better than this: the controller stops the moment it "
                         "enters the circle, so every lap reports an error of about --tol until "
                         "real drift exceeds it. Use 0.05 to measure repeatability."
                         % Limits().pos_tol_m)
    ap.add_argument("--pause", action="store_true",
                    help="stop at EACH endpoint and wait for Enter, so you can measure the "
                         "robot against a floor mark before it moves on")
    ap.add_argument("--dwell", type=float, default=0.0,
                    help="seconds to hold at each endpoint (use --pause instead to measure)")
    ap.add_argument("--no-veto", action="store_true",
                    help="drive with NO obstacle check (requires /scan_filtered otherwise)")
    ap.add_argument("--go", action="store_true", help="actually drive")
    a = ap.parse_args()

    if not (0.3 <= a.dist <= 10.0):
        print("--dist must be 0.3-10.0 m", file=sys.stderr)
        return 2

    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = {"v": False}
    import signal as _sig
    _sig.signal(_sig.SIGINT, lambda *_: stop.__setitem__("v", True))
    _sig.signal(_sig.SIGTERM, lambda *_: stop.__setitem__("v", True))

    n = Driver()
    # A tighter arrival radius is safe HERE because the hysteresis fix (1.6x pos_tol on re-entry)
    # is what stopped the goal-edge chatter, and it scales with the tolerance rather than being
    # an absolute band. Tightening it in a route would be a different question: there the 15 cm
    # exists because the visual servo closes the rest, and it does not need better.
    lim = Limits() if a.tol is None else Limits(pos_tol_m=a.tol)
    rc = 0
    try:
        if not n.wait_for_pose():
            print("no /odom -- is ranger_bringup running?", file=sys.stderr)
            return 1

        x1, y1, th1 = n.pose
        x2, y2 = x1 + a.dist*math.cos(th1), y1 + a.dist*math.sin(th1)
        print(f"\n  P1  x={x1:+.3f} y={y1:+.3f} yaw={math.degrees(th1):+.1f} deg   <- where you are now")
        print(f"  P2  x={x2:+.3f} y={y2:+.3f} yaw={math.degrees(th1):+.1f} deg   "
              f"<- {a.dist:.2f} m straight ahead")
        print(f"\n  Arrival tolerance is {lim.pos_tol_m*100:.0f} cm, so 'arrived' means within "
              f"{lim.pos_tol_m*100:.0f} cm, not on the spot.")
        if not a.go:
            dist, bear = to_goal(x1, y1, th1, x2, y2)
            step = plan_step(dist, bear, 0.0, False, lim)
            print(f"\n  DRY RUN. First action: {step.state}  "
                  f"vx={step.twist.vx:.3f} wz={step.twist.wz:.3f}")
            print(f"  Would publish to {CMD_TOPIC} at {RATE_HZ:.0f} Hz. Add --go to drive.\n")
            return 0

        # Will the chassis even obey? With SWB down it is in RC mode and DISCARDS every
        # command, while odom flows at 50 Hz and the mux reports "permitted". Nothing in ROS
        # can see that, so it is checked here on the bus itself.
        try:
            from chassis_mode import ADVICE, GOOD, chassis_mode
            chassis = chassis_mode()      # never `st`: too easy to shadow
            if chassis is not None and chassis[1] != GOOD:
                print(f"\nNOT DRIVING: chassis control_mode={chassis[1]} -- "
                      f"{ADVICE.get(chassis[1], '')}", file=sys.stderr)
                return 1
        except Exception:
            pass    # no CAN access is not itself a reason to refuse; the mux watch still applies

        # No scan means the corridor veto silently does NOTHING: the guard is
        # `blocked = scan is not None and corridor_blocked(...)`, so a missing topic reads as
        # "clear" rather than "unknown". Failing open is the wrong direction for an obstacle
        # check, so it is refused here instead of discovered by driving into something.
        for _ in range(40):
            rclpy.spin_once(n, timeout_sec=0.1)
            if n.scan is not None:
                break
        if n.scan is None and not a.no_veto:
            print("\nNOT DRIVING: no /scan_filtered, so the corridor veto would be inactive and "
                  "the robot would drive with NO obstacle check.\n"
                  "  Start it with bringup/lidar.sh, or pass --no-veto to drive blind "
                  "deliberately.", file=sys.stderr)
            return 1
        if n.scan is None:
            print("\n  WARNING: --no-veto, driving with NO obstacle check.")

        print(f"""
  MARK THE FLOOR BEFORE DRIVING -- odom cannot measure its own error.

    P1  tape a cross under TWO fixed points on the chassis (the two front wheel contact
        patches work well). Two marks, not one: one gives you position error only, two also
        give you HEADING error, and on a 4WS base heading is what accumulates.
    P2  measure {a.dist:.2f} m forward along the robot's current heading and mark it the same way.
        This is the mark that matters for the benchmark -- it is where the camera has to see
        the plate, so its repeatability is what decides whether the press works.

  Then run with --pause: the robot stops at each endpoint and waits while you measure.
""")
        print(f"  {a.laps} lap(s). Ctrl-C stops; E-stop is faster.\n")

        errs, yaw_errs = [], []
        for lap in range(1, a.laps + 1):
            if stop["v"]:
                break
            print(f"  lap {lap}/{a.laps}")
            print("    out ->")
            ok, why = n.drive_to(x2, y2, th1, lim)
            if not ok:
                print(f"    FAILED on the outbound leg: {why}", file=sys.stderr)
                rc = 1
                break
            if stop["v"]:
                break
            if not endpoint(n, "P2", x2, y2, th1, a):
                break
            print("    back ->")
            ok, why = n.drive_to(x1, y1, th1, lim)
            if not ok:
                print(f"    FAILED on the return leg: {why}", file=sys.stderr)
                rc = 1
                break

            for _ in range(10):
                rclpy.spin_once(n, timeout_sec=0.05)
            xe, ye, the = n.pose
            endpoint(n, "P1", x1, y1, th1, a)
            err = math.hypot(xe - x1, ye - y1)
            yerr = abs(math.degrees(wrap(the - th1)))
            errs.append(err)
            yaw_errs.append(yerr)
            print(f"    return error {err*100:5.1f} cm, heading {yerr:4.1f} deg   "
                  f"(odom-measured -- check the floor mark)\n")

        if errs:
            print(f"  {len(errs)} lap(s): return error median {statistics.median(errs)*100:.1f} cm, "
                  f"worst {max(errs)*100:.1f} cm")
            print(f"                 heading  median {statistics.median(yaw_errs):.1f} deg, "
                  f"worst {max(yaw_errs):.1f} deg")
            if len(errs) >= 3:
                drift = errs[-1] - errs[0]
                print(f"  first->last change {drift*100:+.1f} cm -- "
                      + ("growing, so error is ACCUMULATING, not just noisy"
                         if drift > 0.05 else "no clear accumulation over these laps"))
            print("\n  Now measure against the floor mark. The gap between that and the number "
                  "above\n  is odometry error, and it is the one this test cannot see.\n")
    finally:
        try:
            n.stop()
        except Exception:
            pass
        n.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
