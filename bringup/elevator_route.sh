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
#           bash bringup/elevator_route.sh --dry-run  (plans and grounds; SEE THE CAVEAT)
#
# --dry-run CAVEAT, stated plainly because the previous wording here ("moves nothing") was false:
# it does not drive the base and does not reach for anything, but press_run.sh's final STOW stage
# runs `stow_arm.py --go` unconditionally -- it consults --hold, never MODE -- so if the arm is not
# already folded, a dry run folds it. That is press_run.sh's behaviour and press_run.sh is the
# ADA-proven chain, so it is documented here rather than edited there. Preflight verifies the arm
# is stowed before either mode starts, which makes the stow a no-op in practice.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"
# Marks moments for the paper figures. A no-op unless UTP_RUN_DIR is set, so the route
# behaves identically whether or not anyone is recording -- see bringup/run_event.sh.
source "$REPO/bringup/run_event.sh"

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
    event leg_start "$wp"
    if [ -n "$DRY" ]; then python3 "$REPO/bringup/nav2_goto.py" "$wp" || true; return 0; fi
    # `out="$(cmd)"` is itself a simple command, so under `set -e` a non-zero exit terminates the
    # script AT THIS LINE -- echo, the RESULT parse and the die message below never run. nav2_goto
    # exits non-zero for timeout(6), rejected(5), refused(2/3), no_server(4) and cancelled(4/130),
    # i.e. every failure except `blocked`. So the whole "arrival is not an exit code" apparatus only
    # ever ran on success, and an operator saw one bare line: "rc=6 -- folding the arm". Capture the
    # code instead of dying on it; we want the RESULT line precisely when the command failed.
    local rc=0
    out="$(python3 "$REPO/bringup/nav2_goto.py" "$wp" --go 2>&1)" || rc=$?
    echo "$out"
    status="$(printf '%s\n' "$out" | grep -o 'RESULT {.*}' | tail -1 \
              | python3 -c 'import sys,json; print(json.loads(sys.stdin.read()[7:]).get("status",""))' 2>/dev/null || true)"
    event leg_end "$wp:${status:-unknown}"

    # BRAKES, NOT A SPEED CAP. A fast turn outruns the scan matcher (coarse_angle_resolution is
    # 2.0 deg and /scan is ~2 Hz), the pose slides mid-rotation, and the controller then drives
    # against a stale estimate -- which on 2026-09-03 read as stuttering and a light wall contact
    # while reversing into the car, with the robot never actually in a wall. Capping wz_max would
    # slow the straight legs too, to fix something that only happens while turning. Standing still
    # afterwards costs nothing and lets the matcher converge before the next goal goes out.
    [ -n "$DRY" ] || python3 "$REPO/bringup/settle.py" "$wp" "${UTP_SETTLE_MAX_S:-6}" || true
    [ "$status" = "arrived" ] || die "leg to '$wp' ended as '${status:-unknown}' (nav2_goto exit $rc), not arrived."
}

# press_run.sh UNMODIFIED, with --query. It already takes one; the default is the ADA plate, which
# is why an early elevator run grounded a fire-alarm cover 3.9 m away. Always pass the query here.
press() {
    local query="$1"
    say "PRESS  '$query'"
    event press_start "$query"
    bash "$REPO/bringup/press_run.sh" $DRY --query "$query" || die "press chain failed on '$query'"

    event press_done "$query"
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

# THE WAYPOINTS THIS ROUTE WILL ACTUALLY DRIVE TO. A checker that nothing calls is not a check.
# check_waypoint.py samples the padded 0.72 x 0.50 m footprint against the map; car_panel was
# recorded with 18 lethal cells under the robot while its centre cell read free, and MPPI
# (consider_footprint: true, collision_cost 1e6) can never terminate a trajectory there. That is a
# thirty-second refusal here instead of an unexplained abort at the lift.
say "0b PREFLIGHT -- waypoint footprints"
python3 "$REPO/bringup/check_waypoint.py" call_button lift_door_reverse car_facing_out car_panel lift_door \
    || die "a waypoint on this route is not drivable as recorded. Re-record it and check again."

say "0c PREFLIGHT -- localization fit"
if [ -z "$DRY" ]; then
    # `|| true` again: grep exits 1 when it matches nothing, pipefail propagates that out of the
    # command substitution, and the assignment then dies under set -e -- so the "could not score the
    # localization fit" message below could never actually print. relocalise.py legitimately emits
    # no fit line when it has no map->base_link yet, which is exactly the case that message is for.
    fitline="$(timeout 60 python3 "$REPO/bringup/relocalise.py" --check 2>&1 | grep -oE 'fit [0-9.]+%' | tail -1 || true)"
    fitpct="${fitline#fit }"; fitpct="${fitpct%\%}"
    if [ -z "$fitpct" ]; then
        die "could not score the localization fit. Is slam_toolbox up in LOCALIZATION mode on 'elevator'?"
    fi
    awk -v f="$fitpct" 'BEGIN{exit !(f < 60)}' && die "localization fit is ${fitpct}% -- the robot does
        not know where it is, and every waypoint below is in a frame it is not actually in.
        Seed the pose in RViz (2D Pose Estimate; ignored in MAPPING mode) or run:
          python3 bringup/relocalise.py"
    awk -v f="$fitpct" 'BEGIN{exit !(f < 75)}' \
        && echo "  WARNING: fit ${fitpct}% is low. Fine if someone is standing near the robot; not fine otherwise."
    echo "  localization fit ${fitpct}%"
fi

# ---------------------------------------------------------------------------------- the route
nav   call_button
press "the elevator call button on the wall"

# The doors. Deliberately a human hold, not a detector: bringup/doors_open.py exists and works, but
# the operator stands at the door holding it open for every run, so a poller here would only be a
# second thing that can be wrong. Wire doors_open.py in when the run is unattended.
say "DOORS -- hold them open, then press RETURN"
# `read` returns 1 at EOF, and as the last command of an OR-list that trips `set -e` -- so running
# this route with stdin not a terminal (nohup, ssh -n, a wrapper) aborted it here, mid-route, and
# the trap folded the arm. Only wait when there is a terminal to wait on.
if [ -z "$DRY" ]; then
    if [ -t 0 ]; then
        read -r _ || true
    else
        echo "  stdin is not a terminal -- not waiting. Hold the doors NOW; continuing in 20 s." >&2
        sleep 20
    fi
fi

nav   lift_door_reverse   # back to the doors, so we reverse in rather than turn around inside
nav   car_facing_out      # in the car
nav   car_panel           # square to the panel

press "the blue elevator button"

nav   lift_door           # out

event route_complete ""
say "ROUTE COMPLETE"
