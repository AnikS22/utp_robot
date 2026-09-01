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

say "1  NAVIGATE to 'button'"
if [ -z "$DRY" ]; then
    python3 "$REPO/bringup/nav2_goto.py" button --go || die "could not reach 'button'"
else
    python3 "$REPO/bringup/nav2_goto.py" button || true
fi

# press_run.sh grounds with the arm parked, THEN moves it to the press orientation, THEN reaches.
# That order is load-bearing: grounding after the arm moves photographs the arm (2026-09-01).
say "2  PRESS  (ground with the arm parked, then reach)"
bash "$REPO/bringup/press_run.sh" $DRY || die "press chain failed"

say "3  RETRACT to the packed pose"
if [ -z "$DRY" ]; then
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go || die "arm would not retract"
else
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" || true
fi

# The base cannot move until the mux SEES the arm stowed -- it reads measured joint angles, not
# the script's belief that it just stowed one.
if [ -z "$DRY" ]; then
    python3 - <<'PY' || die "arm_stowed never went true; the base will not be allowed to move"
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
PY
fi

say "4  NAVIGATE to 'outside'"
if [ -z "$DRY" ]; then
    python3 "$REPO/bringup/nav2_goto.py" outside --go || die "could not reach 'outside'"
else
    python3 "$REPO/bringup/nav2_goto.py" outside || true
fi

say "ROUTE COMPLETE"
