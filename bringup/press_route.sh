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
bash "$REPO/bringup/press_run.sh" $DRY || die "press chain failed"

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

say "3b DOORS     did the press work? the doors answer that, and start a clock"
# THE DOORS ARE THE CONFIRMATION, AND THEY ARE ALSO THE DEADLINE.
#
# The previous version of this step waited for an operator to press RETURN. That was the wrong
# instrument twice over. Nothing in the arm chain can see whether the plate actuated -- the arm
# reports reaching a pose, which is a different claim -- but the DOORS OPENING is direct evidence
# that it did. And an ADA opener holds for a bounded time and then shuts, so every second spent
# confirming is a second off the drive out. A human keypress is slower than the thing it confirms.
#
# doors_open.py exits 0 the moment they open, 1 if still shut, 2 if it could not tell (which it
# treats as shut, because the glass fooled both the camera and the lidar on 2026-09-01).
DOORS_TIMEOUT="${UTP_DOORS_TIMEOUT:-20}"
if [ -z "$DRY" ]; then
    if python3 "$REPO/bringup/doors_open.py" --timeout "$DOORS_TIMEOUT"; then
        echo "  DOORS OPEN -- going now, before they shut"
    else
        rc=$?
        echo
        echo "  doors did NOT open within ${DOORS_TIMEOUT}s (doors_open.py exit $rc)."
        echo "  That is the press failing, not the drive. Not driving into a closed door."
        # UTP_DOORS_OVERRIDE=1 for an operator who can see they are open and disagrees with the
        # detector -- glass has fooled it before, in both directions.
        if [ "${UTP_DOORS_OVERRIDE:-0}" = "1" ]; then
            echo "  UTP_DOORS_OVERRIDE=1 -- continuing anyway on the operator's word"
        else
            die "doors never opened; set UTP_DOORS_OVERRIDE=1 to drive out regardless"
        fi
    fi
fi

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
