#!/usr/bin/env bash
# The elevator route. This is press_route.sh -- the chain that pressed the ADA plate -- with the
# navigate/press pair run twice instead of once, because that is all the elevator actually is:
#
#     drive to the call button -> press it -> drive into the car -> press the floor -> drive out
#
# Nothing here is new. Same nav2_goto.py, same press_run.sh, same stow-and-confirm. The only things
# that differ from the door are the waypoint names and the two query strings. There is deliberately
# NO state machine, no --from ladder, no step counter and no fork of the press chain: the door
# proved a linear script works, and the elevator is the same script with more rungs.
#
# The waypoints live on the 'elevator' map (maps/elevator.{pgm,yaml,posegraph,data}) and were
# recorded by the operator on 2026-09-01:
#     call_button        outside the lift, facing the call plate
#     lift_door_reverse  outside, BACK to the doors -- we reverse in, because rotating inside a
#                        2 m car is what confused Nav2 the first time
#     car_facing_out     inside the car, facing the doors
#     car_panel          inside, square to the button panel
#     lift_door          back outside the doors -- the exit
#
# Run it:   bash bringup/elevator_route.sh            (real)
#           bash bringup/elevator_route.sh --dry-run  (plans and grounds, moves nothing)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"

DRY=""; [ "${1:-}" = "--dry-run" ] && DRY="--dry-run"
say() { echo; echo "=== $*"; }
die() { echo "STOP: $*" >&2; exit 1; }

# ALWAYS FOLD THE ARM ON THE WAY OUT, INCLUDING A FAILED RUN. The base is gated on MEASURED joint
# angles, so an arm left extended makes the mux silently discard every Nav2 command and the next
# leg reads as a navigation failure a long way from its cause. press_run.sh is not modified for
# this -- it is the ADA chain and it works; the cleanup belongs to the route that called it.
_fold_on_exit() {
    rc=$?
    if [ "$rc" -ne 0 ] && [ -z "$DRY" ]; then
        echo >&2; echo "[elevator_route] rc=$rc -- folding the arm so the base is not left gated" >&2
        "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go >&2 \
            || echo "[elevator_route] ARM DID NOT FOLD. The base will refuse to move." >&2
    fi
    exit $rc
}
trap _fold_on_exit EXIT

# ARRIVAL IS NOT AN EXIT CODE. nav2_goto.py exits 0 for BOTH "arrived" and "blocked", so `|| die`
# cannot catch a blocked leg -- on 2026-09-01 the robot stopped 3 m short and press_route.sh
# pressed anyway. Parse the status it prints. (press_route.sh still has the `|| die` form; it is
# left alone on purpose, being the known-good door script.)
nav() {
    local wp="$1" out status
    say "NAVIGATE to '$wp'"
    if [ -n "$DRY" ]; then python3 "$REPO/bringup/nav2_goto.py" "$wp" || true; return 0; fi
    out="$(python3 "$REPO/bringup/nav2_goto.py" "$wp" --go 2>&1)"; echo "$out"
    status="$(printf '%s\n' "$out" | grep -o 'RESULT {.*}' | tail -1 \
              | python3 -c 'import sys,json; print(json.loads(sys.stdin.read()[7:]).get("status",""))' 2>/dev/null || true)"
    [ "$status" = "arrived" ] || die "leg to '$wp' ended as '${status:-unknown}', not arrived."
}

# press_run.sh UNMODIFIED, with --query. It already takes one; the default is the ADA plate, which
# is why an early elevator run grounded a fire-alarm cover 3.9 m away. Always pass the query here.
press() {
    local query="$1"
    say "PRESS  '$query'"
    bash "$REPO/bringup/press_run.sh" $DRY --query "$query" || die "press chain failed on '$query'"

    say "RETRACT and wait for the mux to SEE it"
    if [ -n "$DRY" ]; then "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" || true; return 0; fi
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go || die "arm would not retract"
    python3 - <<'PY' || die "arm_stowed never went true; the base will not be allowed to move"
import rclpy, json, time, sys
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init(); n = Node("elev_stow"); got = []
n.create_subscription(String, "/safety/status", lambda m: got.append(m.data), 10)
t0 = time.time()
while time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.3)
    if got and json.loads(got[-1])["gates"]["arm_stowed"]:
        print("  arm_stowed confirmed by the mux"); sys.exit(0)
print("  arm_stowed still false"); sys.exit(1)
PY
}

# ---------------------------------------------------------------------------------- preflight
say "0  PREFLIGHT"
python3 - <<'PY' || die "safety gates are not open"
import rclpy, json, time, sys
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init(); n = Node("elev_pre"); got = []
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

# ---------------------------------------------------------------------------------- the route
nav   call_button
press "the elevator call button on the wall"

# The doors. Deliberately a human hold, not a detector: bringup/doors_open.py exists and works, but
# the operator stands at the door holding it open for every run, so a poller here would only be a
# second thing that can be wrong. Wire doors_open.py in when the run is unattended.
say "DOORS -- hold them open, then press RETURN"
[ -n "$DRY" ] || read -r _

nav   lift_door_reverse   # back to the doors, so we reverse in rather than turn around inside
nav   car_facing_out      # in the car
nav   car_panel           # square to the panel

press "the blue elevator button"

nav   lift_door           # out

say "ROUTE COMPLETE"
