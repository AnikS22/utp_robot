#!/usr/bin/env python3
"""Drive to a MAP-frame waypoint with Nav2. The map-based twin of `waypoints.py goto`.

    python3 bringup/nav2_goto.py door            # DRY RUN: prints the goal, moves nothing
    python3 bringup/nav2_goto.py door --go       # THE ROBOT MOVES

WHY THIS EXISTS
---------------
`waypoints.py goto` drives on ODOM. That was correct when the only lidar was the A1M8 and
slam_toolbox could not hold a pose (route_run.py records the measurement: a ~100-point scan matches
almost equally well at many positions along a corridor). The OS0-128 removed that constraint on
2026-08-30 -- 977 valid beams over a full 360, a 666x779 @ 0.05 m map of the atrium, and Nav2
planning on it -- but navigate_to_goal was never updated. This closes that gap.

WHY A MAP MATTERS FOR *FIFTY* TRIALS SPECIFICALLY
-------------------------------------------------
Odom-frame waypoints have two failure modes that a 50-trial session hits and a 1-trial demo does
not: they drift continuously, and they die outright when `ranger_base` restarts. Both are fatal to
repeatability, and both are exactly what a SAVED, NAMED map fixes -- the coordinates stop being
session-scoped. safety/map_frame.py already enforces the distinction that makes this safe: a
fresh-SLAM `map` frame looks identical in the TF tree but its origin is wherever the robot booted,
so it refuses to treat a nameless recording as portable.

WHAT STAYS ON ODOM, DELIBERATELY
---------------------------------
Only the LEG runs on the map. docs/NAV2.md is explicit about why: "an AMCL correction mid-press
would move the target under the arm." So approach_blockage, the look-around ladder and the press
chain keep running in odom, where motion is smooth and continuous. Nav2 gets the robot to the
door; vision and odom close the last metre. That split is the design, not a compromise.

OUTPUT CONTRACT -- THE LAST STDOUT LINE IS THE CONTRACT
------------------------------------------------------
    RESULT {"status": "...", "waypoint": "...", "elapsed_s": 0.0, "detail": "..."}

`status` is the closed enum in STATUS_EXIT below. `blocked` means Nav2 STATUS_ABORTED and nothing
else, because a `blocked` verdict is what starts reason -> ground -> press and can put an arm at a
wall; every other non-arrival is a distinct status.

THE HUMAN LINES ABOVE IT ARE LEGACY, KEPT ONLY UNTIL RosWorld MIGRATES. navigate_to_goal still
substring-matches stdout for `arrived` / `blocked` and tests `arrived` FIRST, so any stdout line
containing that substring reports success -- a diagnostic reading "not arrived" would do it, and so
would a waypoint named `blocked`. That is the fragility RESULT removes. `arrived at '<name>' ...`
and `blocked: Nav2 ABORTED ...` therefore still print, unchanged. WHEN THE TWO DISAGREE, RESULT
WINS: whoever migrates ros_world.py should parse the last stdout line and stop grepping the rest.

A dry run (no --go) prints NO RESULT line: it attempted no navigation, so it has no outcome, and
forcing one out of a closed outcome enum is the same error as calling a cancelled goal `blocked`.
No RESULT line is never an arrival.

Exit codes are a SEPARATE contract with RosWorld and are unchanged -- see STATUS_EXIT.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

GOAL_TIMEOUT_S = 180.0
CANCEL_TIMEOUT_S = 5.0

# THE CLOSED STATUS ENUM, and the exit code(s) each may be reported with. The two contracts are
# deliberately different resolutions: exit codes only have to tell RosWorld "real outcome / timeout
# / fall back", so several statuses share one code. Exit codes are unchanged from before RESULT
# existed -- 0 a real navigation outcome, 6 a real timeout, 2-5 this backend cannot serve the
# request, 1 crashed (and a crash prints no RESULT line at all).
STATUS_EXIT = {
    "arrived":   (0,),
    "blocked":   (0,),       # Nav2 STATUS_ABORTED ONLY -- this is what starts the press chain
    "timeout":   (6,),
    "rejected":  (5,),       # Nav2 would not accept the goal
    "refused":   (2, 3),     # we refused before sending: unknown or non-portable waypoint
    "no_server": (4,),       # navigate_to_pose never appeared
    "cancelled": (4, 130),   # someone cancelled the goal; 130 is our own Ctrl-C
    "error":     (4,),       # action server returned a non-terminal status: not a world statement
}


def emit_result(status: str, waypoint: str, elapsed_s: float, detail: str) -> None:
    """Print the machine-readable RESULT line. It MUST be the last thing written to stdout.

    `detail` is free text for a human reading a log; nothing parses it. It must not contain the
    words `arrived` or `blocked` unless that IS the status -- the legacy caller substring-matches
    the whole of stdout, so a stray word here would fake an outcome until it migrates.
    """
    assert status in STATUS_EXIT, f"'{status}' is not in the status enum"
    print("RESULT " + json.dumps({"status": status, "waypoint": waypoint,
                                  "elapsed_s": round(float(elapsed_s), 3), "detail": detail},
                                 sort_keys=True), flush=True)


def cancel_and_wait(node, handle, result_fut, rclpy, timeout_s=CANCEL_TIMEOUT_S) -> str:
    """Request cancellation, then wait for both its acknowledgement and terminal result.

    Returning from a navigation process immediately after ``cancel_goal_async`` leaves a window
    in which Nav2 can still publish motion while the caller starts perception or arm work.  Keep
    the helper ROS-type agnostic so its control-plane behaviour can be tested without a ROS install.
    """
    cancel_fut = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_fut, timeout_sec=timeout_s)
    if not cancel_fut.done():
        return "cancellation acknowledgement timed out"

    response = cancel_fut.result()
    goals_canceling = getattr(response, "goals_canceling", None)
    if goals_canceling is not None and not goals_canceling:
        return "Nav2 rejected the cancellation request"

    rclpy.spin_until_future_complete(node, result_fut, timeout_sec=timeout_s)
    if not result_fut.done():
        return "cancellation acknowledged but terminal result timed out"
    status = getattr(result_fut.result(), "status", None)
    return f"cancellation confirmed with terminal status {status}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="waypoint name (must have been recorded in the MAP frame)")
    ap.add_argument("--go", action="store_true", help="actually move; without it this is a dry run")
    ap.add_argument("--timeout", type=float, default=GOAL_TIMEOUT_S)
    ap.add_argument("--force", action="store_true",
                    help="drive even if the waypoint's map provenance cannot be confirmed")
    a = ap.parse_args()

    from waypoints import load as load_waypoints
    from safety.map_frame import FRAME_KEY, FRAME_MAP, MAP_NAME_KEY

    store = load_waypoints()
    if a.name not in store:
        print(f"unknown waypoint '{a.name}'. Known: {sorted(store) or 'none'}", file=sys.stderr)
        emit_result("refused", a.name, 0.0, "unknown waypoint")
        return 2
    wp = store[a.name]
    frame = wp.get(FRAME_KEY, "odom")
    if frame != FRAME_MAP and not a.force:
        print(f"waypoint '{a.name}' is in the '{frame}' frame, not '{FRAME_MAP}'. Nav2 needs a map "
              f"pose. Re-record it while localized in a NAMED map, or use waypoints.py goto.",
              file=sys.stderr)
        emit_result("refused", a.name, 0.0, f"waypoint frame is '{frame}', not '{FRAME_MAP}'")
        return 3
    if not wp.get(MAP_NAME_KEY) and not a.force:
        print(f"waypoint '{a.name}' carries no map name — it was recorded against a fresh SLAM "
              f"session whose origin is wherever the robot booted, so the coordinate is not "
              f"portable. Load a saved map, relocalize, and re-record. (--force overrides.)",
              file=sys.stderr)
        emit_result("refused", a.name, 0.0, "waypoint carries no map name; not portable")
        return 3

    # AND IT MUST BE THE MAP THAT IS ACTUALLY LOADED. Having *a* map name was never sufficient:
    # two maps of the same building have unrelated origins, so a coordinate valid in 'atrium' names
    # a different physical place in 'atrium2d'. Checking only for presence let a well-formed
    # waypoint be driven into the wrong map, which fails as a confident arrival at the wrong spot
    # -- the hardest kind of failure to notice, because nothing errors.
    #
    # This was live on 2026-09-01: every waypoint carried map_name 'atrium' while session.sh nav
    # defaulted MAP_NAME to 'atrium2d'. Nothing compared the two.
    # UTP_LOADED_MAP mirrors the existing UTP_WAYPOINTS override. Without it this check reads the
    # REAL provenance file from inside tests that mock everything else, so six behavioural tests in
    # tests/test_nav_backend.py failed with exit 3 the moment a real map was loaded on the robot --
    # the same shape as the maps_dir fixture problem earlier today. A check that consults global
    # machine state has to let a caller point it somewhere else, or it is not testable.
    loaded = Path(os.environ.get("UTP_LOADED_MAP") or (REPO / "maps" / ".loaded_map"))
    if loaded.exists() and not a.force:
        try:
            live_map = loaded.read_text().split()[0]
        except (IndexError, OSError):
            live_map = ""
        if live_map and wp.get(MAP_NAME_KEY) != live_map:
            print(f"waypoint '{a.name}' was recorded in map '{wp.get(MAP_NAME_KEY)}' but the map "
                  f"currently loaded is '{live_map}'. Their origins are unrelated, so this "
                  f"coordinate does not mean here. Load '{wp.get(MAP_NAME_KEY)}' "
                  f"(MAP_NAME={wp.get(MAP_NAME_KEY)} bash bringup/session.sh nav) or re-record "
                  f"against '{live_map}'. (--force overrides.)", file=sys.stderr)
            emit_result("refused", a.name, 0.0,
                        f"waypoint map '{wp.get(MAP_NAME_KEY)}' is not the loaded map "
                        f"'{live_map}'")
            return 3

    x, y, yaw = float(wp["x"]), float(wp["y"]), float(wp.get("yaw", 0.0))
    print(f"goal '{a.name}' in {frame}: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):+.1f} deg "
          f"(map={wp.get(MAP_NAME_KEY)})")

    if not a.go:
        # NO RESULT LINE HERE, on purpose: nothing was sent, so there is no outcome to report.
        print("DRY RUN. Add --go to send the goal.")
        return 0

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped

    rclpy.init()
    node = Node("utp_nav2_goto")
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=10.0):
        # A silent bt_navigator is the classic half-failed Nav2 bringup: the lifecycle nodes come
        # up unconfigured and nothing says so.
        print("no navigate_to_pose action server after 10 s — is Nav2 up AND activated? "
              "(ros2 lifecycle get /bt_navigator)", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        emit_result("no_server", a.name, 0.0, "no navigate_to_pose action server after 10 s")
        return 4

    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = node.get_clock().now().to_msg()
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    goal.pose = ps

    print(f"DRIVING to '{a.name}' via Nav2. Ctrl-C stops. E-stop is faster.")
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print("Nav2 REJECTED the goal — outside the map, or in an inflated cell?", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown()
        emit_result("rejected", a.name, 0.0, "Nav2 would not accept the goal")
        return 5

    result_fut = handle.get_result_async()
    t0 = time.time()
    # EXIT CODES ARE A CONTRACT WITH RosWorld.navigate_to_goal, separate from the RESULT line and
    # coarser than it:
    #   0      a real navigation outcome (arrived / blocked)
    #   6      a real TIMEOUT
    #   2..5   this backend cannot serve the request -> caller falls back to odom waypoints
    #   1      anything else, i.e. we crashed -> caller must also fall back, NOT record a timeout
    # 1 previously meant timeout, which collided with an uncaught exception: a nav2_goto that died
    # on an import would have been recorded as a legitimate navigation timeout.
    rc = 6
    verdict, detail, elapsed = "timeout", "", 0.0
    try:
        while rclpy.ok() and time.time() - t0 < a.timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
            if result_fut.done():
                break
        elapsed = time.time() - t0
        if not result_fut.done():
            cancel_detail = cancel_and_wait(node, handle, result_fut, rclpy)
            print(f"TIMEOUT after {a.timeout:.0f} s — {cancel_detail}", file=sys.stderr)
            rc = 6
            verdict = "timeout"
            detail = f"no result within {a.timeout:.0f} s; {cancel_detail}"
        else:
            status = result_fut.result().status
            # 4 == STATUS_SUCCEEDED in action_msgs/GoalStatus
            if status == 4:
                print(f"arrived at '{a.name}' in {elapsed:.1f} s")
                rc, verdict, detail = 0, "arrived", "Nav2 STATUS_SUCCEEDED"
            elif status == 6:
                # 6 == STATUS_ABORTED. Nav2 exhausting its recoveries in front of an obstruction is
                # the same event the odom backend reports as `blocked`, and the FSM treats it the
                # same way: stop and let reason -> ground -> act run from here. NOTHING ELSE MAY
                # MAP TO `blocked`.
                print(f"blocked: Nav2 ABORTED after {elapsed:.1f} s "
                      f"(recoveries exhausted)")
                rc, verdict, detail = 0, "blocked", "Nav2 STATUS_ABORTED, recoveries exhausted"
            else:
                # EVERY OTHER STATUS IS NOT A STATEMENT ABOUT THE WORLD, and must not start the
                # reasoning chain. 5 == CANCELED (something cancelled us -- an operator, a
                # supervisor, another goal preempting), 0/1/2 == UNKNOWN/ACCEPTED/EXECUTING coming
                # back as a *result* means the action server is confused.
                #
                # Calling those "blocked" tells the FSM there is an obstruction to reason about,
                # so the VLM gets asked what is in the way, the grounder hunts for a control, and
                # the arm may be driven at whatever it finds -- all triggered by a cancelled goal.
                # A perception-and-action chain must never be started by a control-plane event.
                # This is the same class of error as recording a crashed backend as a navigation
                # timeout: a claim about the world manufactured from a claim about the software.
                name = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 5: "CANCELED"}.get(
                    status, f"status {status}")
                print(f"Nav2 returned {name} after {elapsed:.1f} s — not a navigation "
                      f"outcome, and NOT reported as blocked", file=sys.stderr)
                # 5 == CANCELED is its own status; the non-terminal ones are a confused server.
                rc = 4
                verdict = "cancelled" if status == 5 else "error"
                detail = f"Nav2 returned {name}: a control-plane event, not a world outcome"
    except KeyboardInterrupt:
        cancel_detail = cancel_and_wait(node, handle, result_fut, rclpy)
        print(f"\ninterrupted — {cancel_detail}")
        rc, verdict = 130, "cancelled"
        detail = f"interrupted by the operator (Ctrl-C); {cancel_detail}"
        elapsed = time.time() - t0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    emit_result(verdict, a.name, elapsed, detail)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
