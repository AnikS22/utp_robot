#!/usr/bin/env bash
# THE PROVEN ROUTE: drive to the plate, press it, retract, drive out.
#
#     bash bringup/press_route.sh              # the whole thing
#     bash bringup/press_route.sh --dry-run    # every stage runs; nothing moves
#
# WHY THIS EXISTS AS ONE SCRIPT. Each of these commands was run BY HAND on 2026-09-01 and the
# sequence worked end to end: nav to button ARRIVED in 21.8 s (0.19 m off), grounding put the ADA
# plate at 0.581 with the fire alarm correctly rejected, the arm reached and pressed, and nav to
# outside ARRIVED in 28.0 s (0.24 m off). The same route under run_trial.py's FSM did not, because
# the FSM added a blockage pre-check and staged legs that cancel the Nav2 goal mid-drive. This is
# the sequence that is known to work, written down so it can be repeated rather than retyped.
#
# WHAT IT DELIBERATELY DOES NOT DO: reason about blockages. There is no VLM decision here beyond
# grounding the plate. This is the mechanical chain -- navigate, press, leave. The reasoning loop
# is run_trial.py's job and is a separate claim.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"
DRY=""; [ "${1:-}" = "--dry-run" ] && DRY="--dry-run"

# WAYPOINT NAMES. Were the bare 'button' and 'outside', recorded 2026-09-01 in the atrium map.
# That map was only ever a .pgm/.yaml pair -- a picture, never localizable -- so those two names
# were deleted with the rest of the old floor 1 and this script has been driving names that are
# not in maps/waypoints.yaml. Re-recorded 2026-09-06 in maps/floor1.* as f1_ada_button and
# f1_outside, in floor1's own localization session. Overridable so the next floor does not need
# this file edited again.
BUTTON_WP="${UTP_BUTTON_WP:-f1_ada_button}"
OUTSIDE_WP="${UTP_OUTSIDE_WP:-f1_outside}"

VENV="$HOME/unlocking-the-path/env/.venv/bin/python"
NAV_TIMEOUT="${UTP_NAV_TIMEOUT:-180}"

# NAVIGATE, AND BELIEVE THE RESULT LINE -- NOT THE EXIT CODE.
#
# This script used to do `nav2_goto.py ... --go || die`. nav2_goto returns 0 for ANY real
# navigation outcome, arrived AND blocked alike -- that is a deliberate, documented contract with
# RosWorld.navigate_to_goal, which reads the JSON RESULT line to tell them apart. Reading the exit
# code here could not distinguish them, so on 2026-09-06 a drive that ABORTED after 158 s was
# treated as an arrival: the route continued and commanded the arm at a plate 4.99 m away, from
# the start pose, with the robot never having moved. The arm refused on reach limits. Nothing else
# in the chain would have.
#
# A blockage is NOT a reason to continue. It is the event the reasoning layer exists for, so it
# stops here and asks the VLM what is in the way.
nav_to() {
    local wp="$1" out status
    out="$(mktemp)"
    python3 "$REPO/bringup/nav2_goto.py" "$wp" --go --timeout "$NAV_TIMEOUT" 2>&1 | tee "$out"
    status="$(sed -n 's/^RESULT //p' "$out" | tail -1 \
              | python3 -c 'import sys,json; d=sys.stdin.read().strip();
print(json.loads(d).get("status","") if d else "")' 2>/dev/null)"
    rm -f "$out"
    case "$status" in
        arrived) return 0 ;;
        blocked|timeout)
            echo
            echo "  nav to '$wp' came back '$status'. NOT continuing -- asking what is in the way."
            ask_blockage_now "$wp" "$status"
            return 1 ;;
        "")  echo "  nav to '$wp': no RESULT line at all -- nav2_goto did not run to completion." >&2
             return 1 ;;
        *)   echo "  nav to '$wp' came back '$status', which is not an arrival." >&2
             return 1 ;;
    esac
}

# Perception, not a decision. ask_blockage.py reports WHAT IS THERE and deliberately never says
# what to do about it -- see its header for why that line matters to the paper's claim.
ask_blockage_now() {
    local wp="$1" why="$2" cap
    cap="$REPO/captures/blocked_$(date +%H%M%S)"
    if [ ! -x "$VENV" ]; then
        echo "  (pipeline venv missing at $VENV -- cannot ask the VLM)" >&2; return 0
    fi
    python3 "$REPO/bringup/grab_frame.py" "$cap" >/dev/null 2>&1 || {
        echo "  (could not grab a frame for the VLM)" >&2; return 0; }
    echo "  VLM on $cap:"
    "$VENV" "$REPO/bringup/ask_blockage.py" "$cap" 2>&1 | sed 's/^/    /'
    echo "  blocked at '$wp' ($why). Capture: $cap"
}

say() { echo; echo "=============================================================="; echo " $*"; \
        echo "=============================================================="; }
die() { echo "STOP: $*" >&2; exit 1; }

# Gates first. A blocked mux discards every command while odom and the mux both look healthy --
# the failure mode this repo has met more than any other.
say "0  PREFLIGHT"
python3 - <<'PY' || die "safety gates are not open"
import rclpy, json, time, sys
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init(); n = Node("route_pre"); got = []
n.create_subscription(String, "/safety/status", lambda m: got.append(m.data), 10)
t0 = time.time()
while not got and time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.3)
if not got:
    print("  no /safety/status -- the mux is down"); sys.exit(1)
g = json.loads(got[-1])["gates"]
print("  gates:", json.dumps(g))
sys.exit(0 if (g["arm_stowed"] and not g["estop_latched"]) else 1)
PY

say "1  NAVIGATE to '$BUTTON_WP'"
if [ -z "$DRY" ]; then
    nav_to "$BUTTON_WP" || die "did not arrive at '$BUTTON_WP' -- the arm is not being commanded"
else
    python3 "$REPO/bringup/nav2_goto.py" "$BUTTON_WP" || true
fi

# press_run.sh grounds with the arm parked, THEN moves it to the press orientation, THEN reaches.
# That order is load-bearing: grounding after the arm moves photographs the arm (2026-09-01).
say "2  PRESS  (ground with the arm parked, then reach)"
# ERROR 31 IS CONTACT, NOT A FAILURE.
#
# There is no force sensor on this rig: get_ft_sensor_data answers zeros. The ONLY thing that
# reports the gripper meeting the plate is the controller's abnormal-joint-current trip, which the
# SDK raises as ControllerError 31 -- see approach_target.py --speed, whose help says raising the
# Cartesian speed "raises the current the joints draw against contact, which is what error 31
# reads". So the press touching the button and the press faulting are THE SAME EVENT observed from
# the only sensor that can see it.
#
# Treating it as a failure meant the route stopped, standing in front of a door it had just
# pressed, while the opener swung and then timed out. Measured 2026-09-06: three runs in a row.
#
# WHAT THIS DOES NOT DO: it does not treat every arm fault as success. Only 31. A kinematic
# failure (21), a self-collision (22), a joint limit (23) or a speed limit (24) are not contact
# and still stop the route -- those say the arm never got there, which is a different claim.
PRESS_LOG="$(mktemp)"
set +e
# UTP_NO_STOW=1: press_run.sh must NOT fold and wait. Stage 3 below folds in the background
# while the base drives out -- see there for why the wait was never load-bearing.
UTP_NO_STOW=1 bash "$REPO/bringup/press_run.sh" $DRY 2>&1 | tee "$PRESS_LOG"
PRESS_RC=${PIPESTATUS[0]}
set -e 2>/dev/null || true
CONTACT=0
grep -qE "code: 31|err=31" "$PRESS_LOG" && CONTACT=1
if [ "$PRESS_RC" -ne 0 ] && [ "$CONTACT" != "1" ]; then
    rm -f "$PRESS_LOG"; die "press chain failed with no contact detected"
fi
if [ "$CONTACT" = "1" ]; then
    echo
    echo "  CONTACT (controller error 31) -- the gripper met the plate. Clearing the fault so the"
    echo "  arm can fold, and going. The opener is already swinging."
    "$REPO/.venv-arm/bin/python" - <<'PYCLR' || true
from xarm.wrapper import XArmAPI
a = XArmAPI("192.168.1.221", is_radian=False)
a.clean_error(); a.clean_warn(); a.motion_enable(True); a.set_mode(0); a.set_state(0)
print(f"    arm cleared: state={a.state} error={a.error_code}")
a.disconnect()
PYCLR
fi
rm -f "$PRESS_LOG"

say "3  RETRACT  (overlapped with the drive out)"
# WHY THE BASE NO LONGER WAITS FOR THE FOLD. This used to be two blocking waits in a row -- a
# stow_arm.py with wait=True, then up to 10 s watching /safety/status for arm_stowed to go true --
# and only then the nav goal. Both sat on the critical path and neither is load-bearing any more:
# config/safety.yaml sets require_arm_stowed: false, so the mux does NOT veto /cmd_vel on the arm's
# pose. The base was waiting on a gate that had already been opened.
#
# THE OVERLAP IS CONDITIONAL ON THAT FLAG, read at run time rather than assumed. If anyone sets
# require_arm_stowed back to true -- which the config tells you to do for anything unattended --
# the old blocking behaviour returns automatically. Without that read this script would issue a
# goal the mux silently discards and then report a drive that never happened.
#
# The fold is still waited on, just at the END: `wait` on its PID after the drive, so a retract
# that fails still fails the route instead of disappearing.
REQUIRE_STOW="$(python3 -c "import yaml,sys; print(str(yaml.safe_load(open(sys.argv[1])).get('require_arm_stowed', True)).lower())" "$REPO/config/safety.yaml" 2>/dev/null || echo true)"
STOW_PID=""
if [ -z "$DRY" ]; then
    if [ "$REQUIRE_STOW" = "false" ]; then
        "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go &
        STOW_PID=$!
        echo "  folding in the background -- require_arm_stowed is false, so the base does not wait"
    else
        echo "  require_arm_stowed is TRUE: folding to completion before any base motion"
        "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go || die "arm would not retract"
        python3 - <<'PYSTOW' || die "arm_stowed never went true; the base will not be allowed to move"
import rclpy, json, time, sys
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init(); n = Node("route_stow"); got = []
n.create_subscription(String, "/safety/status", lambda m: got.append(m.data), 10)
t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.3)
    if got and json.loads(got[-1])["gates"]["arm_stowed"]:
        print("  arm_stowed confirmed by the mux"); sys.exit(0)
print("  arm_stowed still false"); sys.exit(1)
PYSTOW
    fi
else
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" || true
fi

say "3b GO        contact made -- driving out NOW, nothing is consulted first"
# NOTHING GATES THIS. Three versions of this step were wrong in the same way: each put a check
# between the press and the drive, on a task whose entire problem is a door that is already
# closing.
#
#   operator keypress   made a human the slowest component in a timed sequence.
#   doors_open.py (VLM) 26.5 s over five looks, and still said SHUT after a press that landed.
#   doors_open_lidar.py measured the FORWARD sector -- but at the button pose the robot faces the
#                       PLATE, not the doorway. It was reading the wall it had just pressed, 0.54 m
#                       away, and would have reported SHUT no matter what the door did. A check
#                       pointed at the wrong thing is worse than no check: it is confidently wrong.
#
# The press making contact is the signal, and the opener is already swinging by the time this line
# runs. If the door somehow has not opened, Nav2's costmap sees it as an obstacle and the leg comes
# back blocked -- which is the honest failure, arriving from the layer that can actually see it.

say "4  NAVIGATE to '$OUTSIDE_WP'"
if [ -z "$DRY" ]; then
    nav_to "$OUTSIDE_WP" || die "did not arrive at '$OUTSIDE_WP'"
else
    python3 "$REPO/bringup/nav2_goto.py" "$OUTSIDE_WP" || true
fi

# Collect the fold that has been running underneath the drive. Late, but never skipped.
if [ -n "${STOW_PID:-}" ]; then
    wait "$STOW_PID" || die "the arm did not retract (it was folding while the base drove out)"
    echo "  arm fold completed during the drive"
fi

say "ROUTE COMPLETE"
