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
    local wp="$1" out status rc tries
    say "NAVIGATE to '$wp'"
    event leg_start "$wp"

    # CLEAR THE COSTMAPS BEFORE EVERY LEG, AND RETRY ONCE.
    #
    # Every leg that failed on 2026-09-04 failed for a TRANSIENT reason that a clear removes:
    #   * the operator standing behind the robot -- the goals into the car sit at bearing +179 deg,
    #     directly astern, and a person there marks them LETHAL. Measured: scan returns at 1.13 m
    #     against a 1.20 m goal distance. The leg fails, the person moves, and nothing un-marks it.
    #   * the lift doors shut while the robot was elsewhere, painting the doorway lethal. They
    #     reopen; the mark stays.
    # The obstacle layer only clears a cell by raytracing THROUGH it, which needs the robot to look
    # from a pose it may never take. So a stale mark from thirty seconds ago can block a leg
    # indefinitely, and the failure looks like a planner problem.
    #
    # Clearing is cheap and safe: the static map is untouched, and the layer re-marks anything the
    # lidar can still see within one update cycle. What it removes is only history.
    _clear_costmaps() {
        timeout 15 ros2 service call /global_costmap/clear_entirely_global_costmap \
            nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1 || true
        timeout 15 ros2 service call /local_costmap/clear_entirely_local_costmap \
            nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1 || true
        sleep 2
    }
    if [ -n "$DRY" ]; then python3 "$REPO/bringup/nav2_goto.py" "$wp" || true; return 0; fi
    # `out="$(cmd)"` is itself a simple command, so under `set -e` a non-zero exit terminates the
    # script AT THIS LINE -- echo, the RESULT parse and the die message below never run. nav2_goto
    # exits non-zero for timeout(6), rejected(5), refused(2/3), no_server(4) and cancelled(4/130),
    # i.e. every failure except `blocked`. So the whole "arrival is not an exit code" apparatus only
    # ever ran on success, and an operator saw one bare line: "rc=6 -- folding the arm". Capture the
    # code instead of dying on it; we want the RESULT line precisely when the command failed.
    tries=0
    while :; do
        tries=$((tries+1))
        [ -n "$DRY" ] || _clear_costmaps
        rc=0
        out="$(python3 "$REPO/bringup/nav2_goto.py" "$wp" --go 2>&1)" || rc=$?
        echo "$out"
        status="$(printf '%s\n' "$out" | grep -o 'RESULT {.*}' | tail -1 \
                  | python3 -c 'import sys,json; print(json.loads(sys.stdin.read()[7:]).get("status",""))' 2>/dev/null || true)"
        # One retry only, and only for the transient statuses. A REFUSED goal is a provenance
        # failure (wrong map) and retrying it just wastes a minute; a second identical failure
        # means the obstruction is real and someone should look at it.
        if [ "$status" = "arrived" ] || [ "$tries" -ge 2 ]; then break; fi
        case "$status" in
            blocked|timeout) say "leg '$wp' came back '$status' -- clearing and retrying once" ;;
            *) break ;;
        esac
    done
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

    # RETRACT WHILE DRIVING, instead of standing still until the arm is home.
    #
    # This used to fold the arm and then BLOCK until /safety/status reported arm_stowed, because
    # the mux refused base motion with the arm out. That interlock is off now
    # (config/safety.yaml require_arm_stowed: false, set deliberately for this task), so the wait
    # buys nothing and costs the whole fold -- ~2.4 s at the current 45 deg/s, on a task whose
    # entire problem is being inside the car before the doors close.
    #
    # So the fold is launched in the BACKGROUND and the caller drives on. wait_stow() below joins
    # it when we actually need the arm home. Nothing is skipped: the fold still happens, still
    # reports, and a failure is still surfaced -- just not in the critical path.
    #
    # WHAT THIS GIVES UP: with the arm swinging during a drive, nothing in software stops it
    # meeting a door frame. There is no force sensor on this arm (get_ft_sensor_data answers zeros,
    # collision_sensitivity is 0), so the e-stop is the protection. That is the operator's call and
    # it is the same trade the interlock removal already made.
    if [ -n "$DRY" ]; then "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" || true; return 0; fi
    say "RETRACT (in the background -- the next leg starts now)"
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go > /tmp/utp_stow_$$.log 2>&1 &
    STOW_PID=$!
}

# Join the background fold. Call before anything that genuinely needs the arm home -- another
# press, or the end of the route.
wait_stow() {
    [ -n "${STOW_PID:-}" ] || return 0
    if wait "$STOW_PID"; then
        echo "  arm folded (overlapped with the drive)"
    else
        echo "  WARNING: the background fold FAILED. Arm may still be extended." >&2
        tail -3 /tmp/utp_stow_$$.log 2>/dev/null | sed 's/^/    /' >&2
    fi
    STOW_PID=""
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
# ANCHOR ON THE TAPE, NOT ON THE WORD "BUTTON". Measured 2026-09-03: this query as
# "the elevator call button on the wall" grounded the SILVER PANEL at score 0.3859 -- inside the
# band where every known-bad grounding in this project sits (fire-alarm cover 0.397, arm-in-frame
# 0.381), and the operator saw the arm aiming at the panel. The blue tape is the only saturated
# colour in an all-grey lift lobby, and naming it is what worked on the car panel:
# "the blue elevator button" 0.443 vs "the elevator button marked with blue tape" 0.267 -- so name
# the colour, not the tape.
press "the blue elevator call button"

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
# STRAIGHT TO THE PANEL, NO STAGING POSE INSIDE THE CAR.
# car_facing_out faces the doors (+58 deg) while any panel pose faces the panel (~0 deg), so the
# leg between them is a PURE SIDESTEP -- measured lateral/distance 1.00 for the old car_panel and
# 0.97 for the re-recorded one. nav2_params motion_model is DiffDrive, so MPPI never emits
# linear.y: to move 0.26 m sideways it must spin ~90 deg, creep, and spin back, inside a 2 m box.
# That is what "trying to position itself in front of the button is rough" was, and no retry or
# tuning fixes it. car_facing_out was only ever a staging pose, and Nav2 has now been shown to
# drive into the car in a single leg -- so we go straight to the pose we actually want and the
# strafe leg stops existing.
nav   car_panel           # into the car and square to the panel, one leg

wait_stow
press "the blue elevator button"

nav   lift_door           # out

wait_stow
event route_complete ""
say "ROUTE COMPLETE"
