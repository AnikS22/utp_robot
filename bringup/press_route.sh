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
    python3 "$REPO/bringup/nav2_goto.py" "$BUTTON_WP" --go || die "could not reach '$BUTTON_WP'"
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

say "4  NAVIGATE to '$OUTSIDE_WP'"
if [ -z "$DRY" ]; then
    python3 "$REPO/bringup/nav2_goto.py" "$OUTSIDE_WP" --go || die "could not reach '$OUTSIDE_WP'"
else
    python3 "$REPO/bringup/nav2_goto.py" "$OUTSIDE_WP" || true
fi

# Collect the fold that has been running underneath the drive. Late, but never skipped.
if [ -n "${STOW_PID:-}" ]; then
    wait "$STOW_PID" || die "the arm did not retract (it was folding while the base drove out)"
    echo "  arm fold completed during the drive"
fi

say "ROUTE COMPLETE"
