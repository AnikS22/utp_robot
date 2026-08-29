#!/usr/bin/env python3
"""Run a mission: waypoints in order, with visual actions at the ones that matter.

    python3 bringup/route_run.py --list
    python3 bringup/route_run.py atrium_door                  # DRY RUN: validate + plan, no motion
    python3 bringup/route_run.py atrium_door --go             # THE ROBOT DRIVES

THE ROBOT DRIVES unless it is a dry run. Hand on the E-stop, and read docs/NAV2.md for why this
exists instead of Nav2.

WHY THIS AND NOT NAV2. Nav2 needs a map and a pose in it. On 2026-08-25 slam_toolbox could not
hold a pose in this building: a ~100-point scan matches almost equally well at many positions
along a corridor, and map->odom correction went from 0.1 cm to 13.7 cm per half-second as soon as
the base turned faster than 0.4 rad/s (the A1M8 sweeps for 145 ms and slam_toolbox treats that as
instantaneous). Nav2 was never the problem -- it never ran. The problem was upstream of it.

So: drive on ODOMETRY, and let VISION close every action. A leg only has to park the robot with
the target in the camera frame; the grounder and the visual servo do the rest, and they repeated
to 3 mm across four runs. Odometry drift over a 15-20 m leg is well inside that budget. This is a
deliberate trade, not a workaround: it moves the accuracy requirement to the one place we have
measured accuracy.

WHAT IT WILL NOT DO. It will not plan around an obstacle -- that needs a costmap, which needs the
localisation we just said we do not have. A blocked corridor STOPS the route and says so. Halting
on an unexpected obstruction is a defensible behaviour; improvising a detour on a pose estimate
we do not trust is not.

THE ONE DECISION IT DOES MAKE: a `check: blockage` step captures a frame, asks the VLM what is in
front of the robot, and if the way is blocked splices a pre-written sub-route (drive to the
button, press it, wait) into the run. The VLM only ever chooses BETWEEN TWO REVIEWED PLANS --
continue, or run the named branch. It never invents motion. If the VLM cannot be reached or does
not answer cleanly, the route FAILS CLOSED and the robot holds position: acting on a guess in
front of a glass door is the wrong way to be wrong.
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
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))
from _ros_env import require_ros  # noqa: E402
require_ros()

import rclpy  # noqa: E402
import yaml  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from odom_session import odom_session_id  # noqa: E402
from safety.mux_watch import MuxWatch  # noqa: E402
from safety.waypoint_frame import check_session  # noqa: E402
from safety.route_plan import ACTION, CHECK, GOTO, WAIT, RouteState, parse_route, validate_route  # noqa: E402
from safety.waypoint_drive import Limits, corridor_blocked, plan_step, to_goal, wrap  # noqa: E402

WAYPOINTS = Path(os.environ.get("UTP_WAYPOINTS", "")) if os.environ.get("UTP_WAYPOINTS") \
    else REPO / "maps" / "waypoints.yaml"
ROUTES = REPO / "config" / "routes.yaml"
CMD_TOPIC = "/cmd_vel_teleop"
SAFETY_STATUS_TOPIC = "/safety/status"
RATE_HZ = 20.0
ODOM_STALE_S = 0.5
LEG_TIMEOUT_S = 180
_INTERRUPT = {"v": False}   # set by the SIGINT handler in main(); read inside the leg loop.0

# Actions are shell-outs on purpose: the grounder needs torch (the pipeline venv) and this node
# needs rclpy. Different interpreters, so the boundary is a process, not an import.
ACTIONS = {
    "press_button": [str(REPO / "bringup" / "press_run.sh")],
}
# UTP_SIM=1: same route, same step names, but the press goes to the Isaac trial server's
# /arm_reach action instead of the real xArm SDK. Everything upstream of the action --
# waypoint legs, corridor veto, blockage check, grounding -- is IDENTICAL code.
if os.environ.get("UTP_SIM") == "1":
    ACTIONS["press_button"] = [sys.executable, str(REPO / "sim" / "sim_press.py")]
    # every child that captures a frame (blockage check, sim press) needs the sim's depth topic
    os.environ.setdefault("UTP_DEPTH_TOPIC", "/mast_cam/depth/image_rect_raw")
PIPELINE_VENV = Path.home() / "unlocking-the-path" / "env" / ".venv" / "bin" / "python"


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Runner(Node):
    def __init__(self) -> None:
        super().__init__("utp_route_run")
        self.pose = None
        self.stamp = 0.0
        self.scan = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self._scan, qos_profile_sensor_data)
        self.create_subscription(String, SAFETY_STATUS_TOPIC, self._safety, 10)
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        # Watches whether the mux is actually passing our commands through. Without it a closed
        # gate is indistinguishable from a robot that will not drive. See safety/mux_watch.py.
        self.mux = MuxWatch(time.monotonic())

    def _odom(self, m) -> None:
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.stamp = time.monotonic()

    def _scan(self, m) -> None:
        self.scan = m

    def _safety(self, m) -> None:
        try:
            st = json.loads(m.data)
        except (ValueError, TypeError):
            return          # malformed status is no status; MuxWatch will call it stale
        self.mux.note_status(st.get("blocked_by"), time.monotonic())

    def fresh(self) -> bool:
        return self.pose is not None and (time.monotonic() - self.stamp) <= ODOM_STALE_S

    def wait_for_odom(self, timeout: float = 5.0) -> bool:
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None:
                return True
        return False

    def wait_for_permission(self, timeout: float = 8.0) -> tuple[bool, str]:
        """Wait for the safety mux to report, and to report that it is NOT blocking.

        Note what this does not do: it never overrides a gate. If the arm is not stowed the right
        answer is to stow the arm, not to start driving and hope. All this buys is that the
        refusal arrives in one line at t=0 instead of as an unexplained timeout at t=180."""
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.mux.seen_status and self.mux.blocked_by in (None, "no_source"):
                # no_source is expected here: we have not commanded anything yet.
                return True, ""
        if not self.mux.seen_status:
            return False, ("no /safety/status after %.0fs -- the safety mux is not running, so "
                           "nothing forwards %s to /cmd_vel. Start the safety stack, and check "
                           "ROS_DOMAIN_ID (9 = hardware, 42 = sim)." % (timeout, CMD_TOPIC))
        from safety.mux_watch import HINTS
        why = self.mux.blocked_by or "?"
        return False, f"safety mux is blocking before we even start: {why}" + \
            (" -- " + HINTS[why] if why in HINTS else "")

    def stop(self) -> None:
        """An explicit zero is a COMMAND and stops the chassis now. Letting the firmware watchdog
        expire instead costs 1.26 s of coasting -- about 18 cm (EXPERIMENT_LOG 2026-08-21d)."""
        for _ in range(5):
            self.pub.publish(Twist())
            time.sleep(0.02)

    def drive_leg(self, goal: dict, lim: Limits) -> tuple[bool, str]:
        deadline = time.monotonic() + LEG_TIMEOUT_S
        last = None
        while rclpy.ok() and time.monotonic() < deadline:
            if _INTERRUPT["v"]:
                self.stop()
                return False, "interrupted by operator"
            rclpy.spin_once(self, timeout_sec=1.0/RATE_HZ)
            if not self.fresh():
                self.pub.publish(Twist())
                continue
            x, y, th = self.pose
            dist, bear = to_goal(x, y, th, goal["x"], goal["y"])
            blocked = (self.scan is not None and
                       corridor_blocked(self.scan.ranges, self.scan.angle_min,
                                        self.scan.angle_increment))
            step = plan_step(dist, bear, wrap(goal["yaw"] - th), blocked, lim,
                             prev_state=last or "")
            t = Twist(); t.linear.x = step.twist.vx; t.angular.z = step.twist.wz
            self.pub.publish(t)

            # Did any of that reach the wheels? A fail-closed gate discards our commands
            # silently, and without this the leg burns its whole 180 s timeout and then reports
            # "timed out" -- naming the symptom, not the cause. Abort and name the gate instead.
            now = time.monotonic()
            self.mux.note_command(not (step.twist.vx == 0.0 and step.twist.wz == 0.0), now)
            v = self.mux.verdict(now)
            if not v.ok:
                self.stop()
                return False, v.reason

            if step.state != last:
                print(f"      [{step.state}] {dist:5.2f} m, bearing {math.degrees(bear):+6.1f} deg")
                last = step.state
            if step.state == "arrived":
                self.stop()
                return True, ""
            if step.state == "blocked":
                # Hold, do not improvise. See the module docstring.
                self.stop()
                return False, "corridor blocked"
        self.stop()
        return False, f"leg timed out after {LEG_TIMEOUT_S:.0f}s"


def run_blockage_check() -> dict:
    """Capture a frame, ask the VLM what is in the robot's way. Never raises; fails closed.

    Two subprocesses because two interpreters: grab_frame needs rclpy (ROS python),
    ask_blockage needs openai (pipeline venv). The frame is kept in captures/ so the exact
    image behind every branch decision can be re-examined after the run.
    """
    import json as _json
    name = f"blockage_{int(time.time())}"
    cap = REPO / "captures" / name
    r = subprocess.run([sys.executable, str(REPO / "bringup" / "grab_frame.py"),
                        "--name", name, "--timeout", "45"])
    if r.returncode != 0 or not (cap / "rgb.png").exists():
        return {"blocked": True, "kind": "", "note": "no frame",
                "description": f"grab_frame failed (exit {r.returncode}) -- is the camera up?"}
    if not PIPELINE_VENV.exists():
        return {"blocked": True, "kind": "", "note": "no venv",
                "description": f"pipeline venv missing at {PIPELINE_VENV}"}
    r = subprocess.run([str(PIPELINE_VENV), str(REPO / "bringup" / "ask_blockage.py"),
                        str(cap), "--json"], capture_output=True, text=True)
    try:
        out = _json.loads(r.stdout.strip().splitlines()[-1])
        out["capture"] = str(cap)
        return out
    except Exception:
        return {"blocked": True, "kind": "", "note": "bad output",
                "description": f"ask_blockage said: {(r.stdout or r.stderr).strip()[:200]}"}


def run_action(step, dry: bool) -> tuple[bool, str]:
    cmd = list(ACTIONS[step.name])
    if step.params.get("query"):
        cmd += ["--query", str(step.params["query"])]
    if step.params.get("standoff"):
        cmd += ["--standoff", str(step.params["standoff"])]
    if dry:
        cmd += ["--dry-run"]
    print(f"      $ {' '.join(cmd)}")
    r = subprocess.run(cmd)
    return (r.returncode == 0), ("" if r.returncode == 0 else f"exit {r.returncode}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("route", nargs="?", help="route name from config/routes.yaml")
    ap.add_argument("--list", action="store_true", help="list routes and waypoints, then exit")
    ap.add_argument("--go", action="store_true", help="actually drive")
    ap.add_argument("--confirm", action="store_true",
                    help="pause before EVERY step: Enter runs it, q stops the route")
    a = ap.parse_args()

    wps = yaml.safe_load(WAYPOINTS.read_text()) if WAYPOINTS.exists() else {}
    routes = (yaml.safe_load(ROUTES.read_text()) or {}).get("routes", {}) if ROUTES.exists() else {}

    if a.list or not a.route:
        print(f"waypoints ({len(wps)}): {sorted(wps) or 'none -- record some first'}")
        print(f"actions        : {sorted(ACTIONS)}")
        print(f"routes ({len(routes)}):")
        for name, spec in sorted(routes.items()):
            print(f"  {name}")
            for s in parse_route(spec):
                print(f"      {s.describe()}")
        return 0 if a.list else 2

    if a.route not in routes:
        print(f"unknown route '{a.route}'. Known: {sorted(routes)}", file=sys.stderr)
        return 2

    steps = parse_route(routes[a.route])
    # Every OTHER route is a candidate branch target for a `check` step in this one.
    subroutes = {}
    for rname, rspec in routes.items():
        if rname == a.route:
            continue
        try:
            subroutes[rname] = parse_route(rspec)
        except ValueError:
            pass    # its own validation will complain when someone tries to run it
    errs = validate_route(steps, set(wps), set(ACTIONS), subroutes)
    if errs:
        # Before anything moves. This is the whole point of validating a route as pure data.
        print(f"ROUTE '{a.route}' WILL NOT RUN -- {len(errs)} problem(s):", file=sys.stderr)
        for e in errs:
            print(f"  - {e}", file=sys.stderr)
        return 3

    print(f"route '{a.route}': {len(steps)} steps, validated against "
          f"{len(wps)} waypoints and {len(ACTIONS)} actions")
    for i, s in enumerate(steps):
        print(f"  {i+1}. {s.describe()}")
    if not a.go:
        print("\nDRY RUN. Nothing moved. Add --go to drive.")
        return 0

    # Take SIGINT OURSELVES. rclpy's default handler tears the context down before `finally`
    # runs, so the stopping zero in Runner.stop() raises "publisher's context is invalid" and is
    # never sent -- measured 2026-08-26, on a Ctrl-C during a real leg. The base then coasts on
    # the chassis watchdog instead: 1.26 s, about 18 cm (EXPERIMENT_LOG 2026-08-21d). An explicit
    # zero is a COMMAND and stops it now; letting the watchdog expire is the difference.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    _interrupted = {"v": False}

    def _on_sigint(signum, frame):
        _interrupted["v"] = True
        _INTERRUPT["v"] = True
    import signal as _signal
    _signal.signal(_signal.SIGINT, _on_sigint)
    _signal.signal(_signal.SIGTERM, _on_sigint)

    n = Runner()
    st = RouteState(steps)
    lim = Limits()
    try:
        if not n.wait_for_odom():
            print("no /odom -- is ranger_bringup running?", file=sys.stderr)
            return 1
        # Prove the mux is alive and permitting BEFORE the first leg. Every gate is fail-closed,
        # so the default state of this robot is "will not move"; starting a route without
        # checking means discovering that fact one 180 s leg timeout at a time.
        # Are these coordinates even in the frame the robot is living in? Odom re-zeroes on
        # every ranger_base restart. Checked for every waypoint the route VISITS, including the
        # ones inside branches -- a branch that fires mid-run must not be the thing that
        # discovers its waypoints are three days old.
        visited = {st_.name for st_ in steps if st_.kind == GOTO}
        # Only branches THIS route can actually reach. Checking every route in the file would
        # refuse a run because some unrelated route mentions a stale waypoint.
        for st_ in steps:
            if st_.kind != CHECK:
                continue
            for raw in (routes.get(st_.params.get("if_blocked")) or []):
                if isinstance(raw, dict) and raw.get("goto"):
                    visited.add(raw["goto"])
        ok, why = check_session(wps, odom_session_id(n), names=visited)
        if not ok:
            print(f"\nSTALE WAYPOINTS -- not driving.\n  {why}", file=sys.stderr)
            return 1

        ok, why = n.wait_for_permission()
        if not ok:
            print(f"\nNOT DRIVING: {why}", file=sys.stderr)
            return 1
        print(f"\nSTART. odom {[round(v,2) for v in n.pose]}   Ctrl-C stops; E-stop is faster.\n")
        while not st.done and rclpy.ok():
            if _interrupted["v"]:
                st.fail("interrupted by operator")
                break
            step = st.current
            print(f"  {st.progress()}")
            if a.confirm:
                # The robot is stationary here: nothing publishes on /cmd_vel_teleop while we
                # wait, and the firmware watchdog holds zero. Operator paces the run.
                try:
                    ans = input("      [confirm] Enter to run this step, q+Enter to stop: ")
                except EOFError:
                    ans = "q"
                if _interrupted["v"] or ans.strip().lower().startswith("q"):
                    st.fail("stopped by operator at confirm prompt")
                    break
            if step.kind == GOTO:
                ok, why = n.drive_leg(wps[step.name], lim)
            elif step.kind == ACTION:
                ok, why = run_action(step, dry=False)
            elif step.kind == CHECK:
                v = run_blockage_check()
                if v.get("note"):
                    ok, why = False, f"blockage check failed closed ({v['note']}): {v['description']}"
                elif v.get("blocked"):
                    sub = step.params["if_blocked"]
                    print(f"      BLOCKED: {v['description']!r} (kind: {v['kind'] or 'unclassified'})")
                    print(f"      -> splicing route '{sub}' ({len(subroutes[sub])} steps), then continuing")
                    st.splice(subroutes[sub])
                    ok, why = True, ""
                else:
                    print(f"      CLEAR: {v['description']!r} -- passable, continuing")
                    ok, why = True, ""
            else:
                time.sleep(min(step.params.get("seconds", 0.0), 300.0))
                ok, why = True, ""
            if not ok:
                st.fail(why)
                break
            st.advance()
    except KeyboardInterrupt:
        st.fail("interrupted by operator")
    finally:
        n.stop()
        print(f"\n{st.progress()}")
        print("stopped (zero published)")
        n.destroy_node()
        rclpy.shutdown()
    return 0 if not st.failed_reason else 1


if __name__ == "__main__":
    sys.exit(main())
