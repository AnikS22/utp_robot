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
from safety.local_avoid import choose_heading  # noqa: E402
from safety.mux_watch import MuxWatch  # noqa: E402
from safety.waypoint_frame import check_session  # noqa: E402
from safety.route_plan import ACTION, CHECK, GOTO, WAIT, RouteState, parse_route, validate_route  # noqa: E402
from safety.waypoint_drive import Limits, corridor_blocked, plan_step, to_goal, wrap  # noqa: E402

WAYPOINTS = Path(os.environ.get("UTP_WAYPOINTS", "")) if os.environ.get("UTP_WAYPOINTS") \
    else REPO / "maps" / "waypoints.yaml"
ROUTES = REPO / "config" / "routes.yaml"
CMD_TOPIC = "/cmd_vel_teleop"
# Marks a leg that failed because THE WAY IS BLOCKED, as opposed to a timeout, an interrupt or a
# safety gate. This is the distinction the escalation turns on: geometry having no answer is
# exactly the evidence that the problem is semantic -- a door to be opened, a lift to be called,
# a person to be asked -- and that is what the pipeline is for. Every other failure is a FAULT,
# and putting a fault to a VLM would just be guessing with extra steps.
STUCK = "STUCK: "
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
        self.avoid = False          # set by --avoid; off by default
        self._prev_avoid = None     # last steered bearing, for hysteresis

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
        # Anything before this leg may have blocked the thread for seconds without spinning
        # (a --confirm prompt, an action subprocess, a wait). Staleness cannot tell "the mux
        # went quiet" from "we stopped listening", so the clocks start fresh here.
        self.mux.resume(time.monotonic())
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
            # Reactive avoidance. Only while still TRAVELLING: inside the arrival zone the
            # bearing to the goal swings wildly on millimetre drift, and the thing the scan sees
            # may well be the goal's own surroundings.
            avoiding = ""
            if self.avoid and blocked and dist > 2.0 * lim.pos_tol_m:
                ch = choose_heading(self.scan.ranges, self.scan.angle_min,
                                    self.scan.angle_increment, bear,
                                    prev_bearing_rad=self._prev_avoid)
                if ch.bearing_rad is None:
                    self.stop()
                    return False, STUCK + f"no way around -- {ch.reason}"
                # Steer down the gap instead of at the goal. dist is unchanged, so arrival
                # still needs real progress -- this redirects the robot, it does not fake it.
                bear = ch.bearing_rad
                self._prev_avoid = ch.bearing_rad
                blocked = False
                avoiding = f"  [avoid] {ch.reason}"
            elif not blocked:
                self._prev_avoid = None

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

            if step.state != last or avoiding:
                print(f"      [{step.state}] {dist:5.2f} m, bearing "
                      f"{math.degrees(bear):+6.1f} deg{avoiding}")
                last = step.state
            if step.state == "arrived":
                self.stop()
                return True, ""
            if step.state == "blocked":
                # Hold, do not improvise. See the module docstring.
                self.stop()
                return False, STUCK + "corridor blocked"
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


def escalate(st, step, why: str, sub_name: str, subroutes: dict, budget: dict,
             quiet_if_clear: bool = False) -> tuple[bool, str]:
    """Geometry has run out. Ask what this thing IS, and act if it is something we can act on.

    The POLICY -- when it is safe to act on the answer -- lives in safety/escalation.py, pure and
    tested. This is only the plumbing: spend a unit of budget, capture and ask, then do what the
    decision says. See that module for why every ambiguous case stops.
    """
    from safety.escalation import ACT, decide

    check = None
    if budget["left"] > 0:
        budget["left"] -= 1
        print(f"      {why}")
        print("      geometry cannot solve this. Asking what it is...")
        check = run_blockage_check()

    d = decide(check, budget["left"] + 1 if check is not None else budget["left"],
               budget["max"], why)
    if d.action != ACT:
        # A proactive look that finds the way CLEAR is the expected case, not a failure: the
        # camera agreed with the lidar and we simply drive. Every other refusal still stops.
        if quiet_if_clear and check is not None and not check.get("note") \
                and not check.get("blocked"):
            print(f"      looked first: clear ({check.get('description', '')!r})")
            budget["left"] += 1        # a clear look costs nothing
            return True, ""
        return False, d.message

    print(f"      {d.message}")
    print(f"      -> running '{sub_name}' ({len(subroutes[sub_name])} steps), then retrying "
          f"this leg  [{budget['left']} escalation(s) left]")
    # The branch, and then THIS SAME LEG again. Retrying is the point: pressing the button is
    # only useful if we then go through the door it opened.
    st.splice(list(subroutes[sub_name]) + [step])
    return True, ""


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
    ap.add_argument("--goto", metavar="A,B,C",
                    help="ad-hoc route: drive these waypoints in order, no YAML editing. "
                         "For a plain navigation test -- record the poses with "
                         "`waypoints.py record <name>`, then drive them.")
    ap.add_argument("--list", action="store_true", help="list routes and waypoints, then exit")
    ap.add_argument("--go", action="store_true", help="actually drive")
    ap.add_argument("--avoid", action="store_true",
                    help="steer around obstacles instead of stopping at them (reactive, uses "
                         "the live scan only -- no map). It has NO MEMORY: a U-shape or dead end "
                         "will trap it, and it stops rather than escaping.")
    ap.add_argument("--on-blocked", metavar="ROUTE",
                    help="when the way is blocked and there is no way around it, ASK: capture a "
                         "frame, put it to the VLM, and if it is a blockage this route can act "
                         "on, run that route and retry the leg. Fails closed on any doubt.")
    ap.add_argument("--look-first", action="store_true",
                    help="ASK BEFORE EACH LEG, not only when geometry fails. Required for GLASS: "
                         "a 2D lidar cannot see it, so the veto never fires and --on-blocked is "
                         "never reached. Costs one VLM call (~6 s) per leg.")
    ap.add_argument("--escalations", type=int, default=2,
                    help="how many times a route may escalate before giving up (default 2)")
    ap.add_argument("--loops", type=int, default=1,
                    help="repeat the whole route N times. With --goto start,finish this drives "
                         "start->finish->start->finish..., re-squaring on the recorded start "
                         "pose every lap, which is what makes run 1 and run 100 comparable.")
    ap.add_argument("--confirm", action="store_true",
                    help="pause before EVERY step: Enter runs it, q stops the route")
    a = ap.parse_args()

    wps = yaml.safe_load(WAYPOINTS.read_text()) if WAYPOINTS.exists() else {}
    routes = (yaml.safe_load(ROUTES.read_text()) or {}).get("routes", {}) if ROUTES.exists() else {}

    # An ad-hoc path is a real route built in memory: it goes through the SAME parse,
    # the SAME validation, and the SAME stale-waypoint check as anything in routes.yaml.
    # Convenience must not mean a second, less-checked way to move the robot.
    if a.goto:
        names = [w.strip() for w in a.goto.split(",") if w.strip()]
        if not names:
            print("--goto needs at least one waypoint name", file=sys.stderr)
            return 2
        routes = dict(routes)
        routes["--goto"] = [{"goto": w} for w in names]
        a.route = "--goto"

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
    if a.loops < 1:
        print("--loops must be >= 1", file=sys.stderr)
        return 2
    if a.loops > 1:
        # Repeat the PARSED steps, so every lap is the same validated sequence. Done before
        # validation so a bad route is still caught once, not N times.
        steps = [st_ for _ in range(a.loops) for st_ in steps]
    # Every OTHER route is a candidate branch target for a `check` step in this one.
    subroutes = {}
    for rname, rspec in routes.items():
        if rname == a.route:
            continue
        try:
            subroutes[rname] = parse_route(rspec)
        except ValueError:
            pass    # its own validation will complain when someone tries to run it
    # --on-blocked names a route that may run MID-DRIVE. Resolve and validate it now: a typo
    # discovered by a KeyError while the robot is stopped in a doorway is not a good time.
    if a.on_blocked:
        if a.on_blocked not in subroutes:
            print(f"--on-blocked route '{a.on_blocked}' not found. Known: "
                  f"{sorted(subroutes)}", file=sys.stderr)
            return 2
        errs_b = validate_route(subroutes[a.on_blocked], set(wps), set(ACTIONS))
        if errs_b:
            print(f"--on-blocked route '{a.on_blocked}' WILL NOT RUN -- "
                  f"{len(errs_b)} problem(s):", file=sys.stderr)
            for e in errs_b:
                print(f"  - {e}", file=sys.stderr)
            return 3

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
    n.avoid = a.avoid
    budget = {"left": max(0, a.escalations), "max": max(0, a.escalations)}
    started = {"v": False}    # did we get past the preflights and actually begin driving?
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
        # The escalation route's waypoints too -- it can fire on any leg, so it is not optional.
        if a.on_blocked:
            for st_ in subroutes[a.on_blocked]:
                if st_.kind == GOTO:
                    visited.add(st_.name)
        ok, why = check_session(wps, odom_session_id(n), names=visited)
        if not ok:
            print(f"\nSTALE WAYPOINTS -- not driving.\n  {why}", file=sys.stderr)
            return 1

        # Will the chassis even obey? With SWB down it is in RC mode and DISCARDS every
        # command, while odom flows at 50 Hz and the mux reports "permitted". Nothing in ROS
        # can see that, so it is checked here on the bus itself.
        try:
            from chassis_mode import ADVICE, GOOD, chassis_mode
            chassis = chassis_mode()      # NOT `st` -- that is the RouteState
            if chassis is not None and chassis[1] != GOOD:
                print(f"\nNOT DRIVING: chassis control_mode={chassis[1]} -- "
                      f"{ADVICE.get(chassis[1], '')}", file=sys.stderr)
                return 1
        except Exception:
            pass    # no CAN access is not itself a reason to refuse; the mux watch still applies

        ok, why = n.wait_for_permission()
        if not ok:
            print(f"\nNOT DRIVING: {why}", file=sys.stderr)
            return 1
        started["v"] = True
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
                # LOOK BEFORE DRIVING. Measured on this robot 2026-08-29, parked in front of
                # closed glass double doors: the lidar's nearest return within +-15 deg was
                # 7.79 m, corridor_blocked was False, and local_avoid reported "clear toward the
                # goal". The camera, asked at the same instant, returned
                # {"kind": "door", "description": "closed glass double doors", "blocked": true}.
                #
                # So escalation ON GEOMETRIC FAILURE alone cannot protect against glass: geometry
                # never fails, it reports a clear path and the robot drives into the door at
                # 0.25 m/s. The only sensor that sees glass here is the camera, so on legs where
                # glass is possible the camera has to be asked BEFORE moving, not after failing.
                if a.look_first and a.on_blocked:
                    ok, why = escalate(st, step, STUCK + "looking before driving",
                                       a.on_blocked, subroutes, budget, quiet_if_clear=True)
                    if not ok:
                        st.fail(why)
                        break
                    if st.current is not step:
                        continue      # a branch was spliced in; run it before this leg
                ok, why = n.drive_leg(wps[step.name], lim)
                if not ok and why.startswith(STUCK) and a.on_blocked:
                    ok, why = escalate(st, step, why, a.on_blocked, subroutes, budget)
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
        # Only report progress if we actually got as far as driving. A preflight refusal that
        # then prints "step 1/2: drive to 'start'" and "stopped (zero published)" reads exactly
        # like a run that started and failed -- which is the opposite of what happened, and the
        # wrong thing to be told when you are deciding whether the robot moved.
        if started["v"]:
            print(f"\n{st.progress()}")
            print("stopped (zero published)")
        n.destroy_node()
        rclpy.shutdown()
    return 0 if not st.failed_reason else 1


if __name__ == "__main__":
    sys.exit(main())
