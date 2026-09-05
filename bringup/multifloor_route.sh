#!/usr/bin/env bash
# The multi-floor route. This is elevator_route.sh -- the chain that calls the lift, rides it and
# gets out -- with the RIDE made real: the car actually goes somewhere, and the map under the robot
# has to change with it.
#
#     call the lift -> get in -> select the destination -> face the doors -> RIDE
#     -> swap the map -> doors open -> VERIFY -> drive out
#
# Almost nothing here is new. Same nav2_goto.py, same press_run.sh, same stow-and-confirm, same
# linear shape with no state machine -- elevator_route.sh's argument holds one storey up. The three
# steps that ARE new are steps 9-12, and they exist because of one fact:
#
#     A LIFT CAR IS GEOMETRICALLY IDENTICAL ON EVERY FLOOR.
#
# With the doors shut the scan is four walls about a metre away, and that matches floor 1's map
# exactly as well as floor 2's. So the robot cannot find out which floor it is on until the doors
# open, every localization number taken before that is confident and meaningless, and the swap is
# a SEED made on trust which --verify then tests. safety/floor_plan.py is where this is argued and
# unit-tested; bringup/floor_swap.py is what carries it out.
#
# Run it:   bash bringup/multifloor_route.sh                    (real, floor 1 -> 2)
#           bash bringup/multifloor_route.sh --dry-run
#           bash bringup/multifloor_route.sh --from 2 --to 1    (come back down)
#
# --dry-run CAVEAT, inherited from elevator_route.sh and restated because it is not obvious: it
# does not drive the base and does not reach for anything, but press_run.sh's final STOW stage runs
# `stow_arm.py --go` unconditionally, so if the arm is not already folded a dry run folds it. It
# also does NOT swap the map -- floor_swap.py without --go only prints what it would restart.
#
# NOT DEMONSTRATED. No part of this has run on the robot, and the lift has never been ridden by
# anything in this repo. docs/MULTIFLOOR.md lists what has to be recorded before it can be.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"
source "$REPO/bringup/run_event.sh"

DRY=""; FROM=1; TO=2
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) DRY="--dry-run"; shift ;;
        --from)    FROM="${2:?--from needs a floor id}"; shift 2 ;;
        --to)      TO="${2:?--to needs a floor id}"; shift 2 ;;
        -h|--help) sed -n '2,30p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

say() { echo; echo "=== $*"; }
die() { echo "STOP: $*" >&2; exit 1; }

# ALWAYS FOLD THE ARM ON THE WAY OUT, INCLUDING A FAILED RUN -- elevator_route.sh's reasoning,
# unchanged: the base is gated on MEASURED joint angles, so an arm left extended makes the mux
# silently discard every Nav2 command and the next leg reads as a navigation failure a long way
# from its cause.
_fold_on_exit() {
    rc=$?
    if [ "$rc" -ne 0 ] && [ -z "$DRY" ]; then
        echo >&2; echo "[multifloor] rc=$rc -- folding the arm so the base is not left gated" >&2
        "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go >&2 \
            || echo "[multifloor] ARM DID NOT FOLD. The base will refuse to move." >&2
    fi
    exit $rc
}
trap _fold_on_exit EXIT

# THE WAYPOINT NAMES COME FROM config/floors.yaml, NOT FROM THIS FILE. Two floors' worth of names
# hardcoded in a bash script is two places to typo and no way to check either. The config is what
# safety/floor_plan.check_building() validates, so reading from it is what makes that validation
# cover this route.
eval "$(python3 - "$REPO" "$FROM" "$TO" <<'PY'
import sys, yaml, shlex
sys.path.insert(0, sys.argv[1])
from safety.floor_plan import floors_of
repo, a, b = sys.argv[1], sys.argv[2], sys.argv[3]
cfg = yaml.safe_load(open(repo + "/config/floors.yaml"))
fl = floors_of(cfg)
for tag, fid in (("A", a), ("B", b)):
    if fid not in fl:
        sys.stderr.write(f"unknown floor '{fid}'; known: {', '.join(sorted(fl))}\n"); raise SystemExit(1)
    f = fl[fid]
    for role, name in f.waypoints.items():
        print(f"{tag}_{role.upper()}={shlex.quote(name)}")
    print(f"{tag}_MAP={shlex.quote(f.map)}")
    print(f"{tag}_CALL_QUERY={shlex.quote(f.call_query)}")
    print(f"{tag}_SELECT_QUERY={shlex.quote(f.select_query)}")
PY
)" || die "could not read config/floors.yaml for floors $FROM and $TO"

# ARRIVAL IS NOT AN EXIT CODE -- elevator_route.sh's finding, carried over verbatim. nav2_goto.py
# exits 0 for BOTH "arrived" and "blocked", so `|| die` cannot catch a blocked leg; on 2026-09-01
# the robot stopped 3 m short and the route pressed anyway. Parse the RESULT line.
nav() {
    local wp="$1" out status rc=0
    say "NAVIGATE to '$wp'"
    event leg_start "$wp"
    if [ -n "$DRY" ]; then python3 "$REPO/bringup/nav2_goto.py" "$wp" || true; return 0; fi
    out="$(python3 "$REPO/bringup/nav2_goto.py" "$wp" --go 2>&1)" || rc=$?
    echo "$out"
    status="$(printf '%s\n' "$out" | grep -o 'RESULT {.*}' | tail -1 \
              | python3 -c 'import sys,json; print(json.loads(sys.stdin.read()[7:]).get("status",""))' 2>/dev/null || true)"
    event leg_end "$wp:${status:-unknown}"
    # BRAKES, NOT A SPEED CAP: a fast turn outruns the scan matcher and the controller then drives
    # against a stale estimate. Standing still afterwards costs nothing.
    python3 "$REPO/bringup/settle.py" "$wp" "${UTP_SETTLE_MAX_S:-6}" || true
    [ "$status" = "arrived" ] || die "leg to '$wp' ended as '${status:-unknown}' (nav2_goto exit $rc), not arrived."
}

press() {
    local query="$1"
    say "PRESS  '$query'"
    event press_start "$query"
    bash "$REPO/bringup/press_run.sh" $DRY --query "$query" || die "press chain failed on '$query'"
    event press_done "$query"
    # RETRACT WHILE DRIVING. The arm interlock is off for this task (config/safety.yaml
    # require_arm_stowed: false), so waiting for the fold buys nothing and costs ~2.4 s on a task
    # whose entire problem is being inside the car before the doors close.
    if [ -n "$DRY" ]; then "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" || true; return 0; fi
    say "RETRACT (in the background -- the next leg starts now)"
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go > /tmp/utp_stow_$$.log 2>&1 &
    STOW_PID=$!
}

wait_stow() {
    [ -n "${STOW_PID:-}" ] || return 0
    if wait "$STOW_PID"; then echo "  arm folded (overlapped with the drive)"
    else
        echo "  WARNING: the background fold FAILED. Arm may still be extended." >&2
        tail -3 /tmp/utp_stow_$$.log 2>/dev/null | sed 's/^/    /' >&2
    fi
    STOW_PID=""
}

# Wait for the doors. Deliberately a human hold, as in elevator_route.sh: the operator stands at
# the door for every run, so a poller here would only be a second thing that can be wrong. Wire
# doors_open.py in (floor_swap.py --verify --check-doors already can) when the run is unattended.
# UTP_NO_DOOR_PROMPT=1 skips the waits entirely. The operator holds the RC transmitter, which
# revokes CAN authority below anything software can do (layer 1, docs/AGENT_BRIEF.md) -- so a
# prompt asking a human who is already watching to confirm what they can already see, and who can
# stop the robot faster than they can answer, buys nothing. It costs the door hold, which is the
# one resource this task is actually short of.
doors() {
    if [ "${UTP_NO_DOOR_PROMPT:-0}" = "1" ]; then echo "  (doors: $1 -- not waiting, operator on RC)"; return 0; fi
    say "DOORS -- $1, then press RETURN"
    [ -z "$DRY" ] || return 0
    # `read` returns 1 at EOF and would trip `set -e` mid-route with stdin not a terminal.
    if [ -t 0 ]; then read -r _ || true
    else echo "  stdin is not a terminal -- not waiting. Continuing in 20 s." >&2; sleep 20; fi
}


# CLEAR THE COSTMAPS WHILE THE DOORS ARE OPEN. Measured 2026-09-04 and 2026-09-05, on BOTH floors.
#
# The global costmap's obstacle layer holds the lift doors as they were when Nav2 started -- shut,
# a solid wall of inscribed-inflated cells straight across the opening. Opening the doors changes
# the world; it does NOT change the costmap, because the robot sits ~1.5 m back and off-axis and
# never gets the line of sight to raytrace-clear the gap it is trying to drive through.
#
# The failure is silent and reads as a planner bug: `GridBased plugin failed to plan ... Failed to
# create plan with tolerance of: 0.300000`, bt_navigator aborts "recoveries exhausted", and the
# robot never moves. It cost an evening. It is the same event behind the `blocked` and `rejected`
# leg outcomes in runs/20260904T21*.
#
# Verified 2026-09-04: clearing turned the door band from 99/100 (inscribed/lethal, on the
# published 0-100 scale) to 0 (free) in one update cycle.
clear_costmaps() {
    [ -z "$DRY" ] || { echo "  (dry run: costmaps not cleared)"; return 0; }
    echo "  clearing costmaps (doors must be OPEN right now)"
    for s in /global_costmap/clear_entirely_global_costmap /local_costmap/clear_entirely_local_costmap; do
        timeout 20 ros2 service call "$s" nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null 2>&1 \
            && echo "    ok   $s" || echo "    FAIL $s" >&2
    done
    sleep 2
}

# ---------------------------------------------------------------------------------- preflight
say "0  PREFLIGHT -- safety gates"
python3 - <<'PY' || die "safety gates are not open"
import rclpy, json, time, sys
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init(); n = Node("mf_pre"); got = []
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

# EVERY FLOOR, BEFORE THE FIRST ONE MOVES. A missing floor-2 map or a mistyped floor-2 waypoint is
# invisible until the robot is standing in a lift with the doors closing -- which is the worst
# place in this building to discover a config error, and the only place this route can discover it
# if the check is not here. Thirty seconds now, or an unexplained abort in the car.
say "0b PREFLIGHT -- the whole building, both floors"
python3 "$REPO/bringup/floor_swap.py" --check \
    || die "config/floors.yaml is not drivable. Nothing has moved. See docs/MULTIFLOOR.md."
python3 "$REPO/bringup/floor_swap.py" --plan "$FROM" "$TO"

# check_waypoint.py samples the padded 0.72 x 0.50 m footprint against EACH waypoint's own map
# (it reads map_name per waypoint), so this covers floor 2 as well while standing on floor 1.
say "0c PREFLIGHT -- waypoint footprints, both floors"
python3 "$REPO/bringup/check_waypoint.py" \
    "$A_CALL_BUTTON" "$A_DOOR_REVERSE" "$A_CAR_FACING_OUT" "$A_CAR_PANEL" "$A_EXIT" \
    "$B_CALL_BUTTON" "$B_DOOR_REVERSE" "$B_CAR_FACING_OUT" "$B_CAR_PANEL" "$B_EXIT" \
    || die "a waypoint on this route is not drivable as recorded. Re-record it and check again."

say "0d PREFLIGHT -- localization fit on floor $FROM"
if [ -z "$DRY" ]; then
    # RETRY. Scoring the fit is a measurement and measurements are occasionally unavailable; an
    # intermittently empty reading is not evidence that the robot is lost, and aborting the run on
    # one is how a healthy stack gets reported as "it didn't localize" (2026-09-05, 1 attempt in 4).
    fitline=""
    for _try in 1 2 3; do
        fitline="$(timeout 90 python3 "$REPO/bringup/relocalise.py" --check 2>&1 | grep -oE 'fit [0-9.]+%' | tail -1 || true)"
        [ -n "$fitline" ] && break
        echo "  fit not scored on attempt $_try; retrying"
        sleep 2
    done
    fitpct="${fitline#fit }"; fitpct="${fitpct%\%}"
    [ -n "$fitpct" ] || die "could not score the localization fit. Is slam_toolbox up in
        LOCALIZATION mode on '$A_MAP'?  MAP_NAME=$A_MAP bash bringup/session.sh nav"
    awk -v f="$fitpct" 'BEGIN{exit !(f < 60)}' && die "localization fit is ${fitpct}% -- the robot
        does not know where it is, and every waypoint below is in a frame it is not actually in.
        Seed the pose in RViz (2D Pose Estimate) or run: python3 bringup/relocalise.py"
    awk -v f="$fitpct" 'BEGIN{exit !(f < 75)}' \
        && echo "  WARNING: fit ${fitpct}% is low. Fine if someone is standing near the robot."
    echo "  localization fit ${fitpct}%"
fi

# ---------------------------------------------------------------------------------- floor FROM
event floor_start "$FROM"
nav   "$A_CALL_BUTTON"
press "$A_CALL_QUERY"
doors "hold them open"
clear_costmaps

# FORWARD ENTRY when the floor defines door_facing + car_facing_in, else the original reverse.
# See config/floors.yaml for the two measurements behind this.
ENTRY_APPROACH="${A_DOOR_FACING:-$A_DOOR_REVERSE}"
ENTRY_POSE="${A_CAR_FACING_IN:-$A_CAR_FACING_OUT}"
if [ -n "${A_DOOR_FACING:-}" ]; then say "ENTRY: forward (nose-first), no turn at the doors"
else say "ENTRY: reverse (no door_facing/car_facing_in defined for floor $FROM)"; fi

nav   "$ENTRY_APPROACH"    # line up square with the doorway; near-zero turn from the call plate

# CLEAR AGAIN, HERE, IMMEDIATELY BEFORE DRIVING IN. Measured 2026-09-05: clearing once at the
# doors-open prompt is not enough. That clear is followed by a ~21 s approach leg, and an ADA
# opener holds for a bounded time -- so by the time the entry leg starts the doors have shut and
# the obstacle layer has re-marked them as a ~0.5 m band (a thin door plus 0.20 m inflation on
# each side) straight across the opening, with the goal cell itself at 99. The planner then cannot
# terminate at car_facing_out_f2 and bt_navigator aborts "recoveries exhausted".
# THE DOORS MUST BE OPEN AT THIS INSTANT, not when the prompt was answered.
if [ "${UTP_NO_DOOR_PROMPT:-0}" != "1" ]; then
    say "DOORS -- confirm they are OPEN and being held, then press RETURN"
    if [ -z "$DRY" ] && [ -t 0 ]; then read -r _ || true; fi
fi
clear_costmaps

nav   "$ENTRY_POSE"        # straight in through the doorway
nav   "$A_CAR_PANEL"       # square to the panel

wait_stow
press "$B_SELECT_QUERY"    # the in-car button for the DESTINATION floor

# FACE THE DOORS NOW, NOT AFTER THE RIDE. The robot is at car_panel, about 53 degrees off the
# doors. Turning is the one manoeuvre that hurts on this stack -- settle.py's whole reason -- and
# after the swap it would be turning on a map it has been SEEDED into and not yet verified
# against. Do the turn here, where the robot is genuinely localized, and leave the post-swap move
# as a straight drive out through open doors.
wait_stow
nav   "$A_CAR_FACING_OUT"

# ---------------------------------------------------------------------------------- the swap
# THE DOORS MUST BE SHUT FIRST. While they stand open the scan reaches out into floor $FROM's
# lobby, and the restarted matcher would be handed floor-$FROM geometry to reconcile against a
# floor-$TO seed. Sealed, the scan is only the car -- which is the one thing both maps agree
# about, and the whole reason the seed survives the ride.
doors "let them CLOSE"

say "SWAP  localization -> floor $TO, map '$B_MAP'"
event swap_start "$TO"
if [ -n "$DRY" ]; then
    python3 "$REPO/bringup/floor_swap.py" --to "$TO" || true
else
    python3 "$REPO/bringup/floor_swap.py" --to "$TO" --go \
        || die "the floor swap failed. maps/.loaded_map has been left absent on purpose, so every
        map-frame waypoint is refused and the base cannot be driven anywhere by accident.
        Recover by hand: MAP_NAME=$B_MAP bash bringup/session.sh nav"
fi
event swap_done "$TO"

# ---------------------------------------------------------------------------------- the ride
# The swap above runs WHILE the car moves, on purpose: deserializing a pose graph and waiting for
# map->odom is tens of seconds, the ride is tens of seconds in which the robot must not move
# anyway, and the alternative spends that time on the destination floor's door hold instead.
say "RIDE  floor $FROM -> $TO"
event ride_start "$FROM->$TO"
cat <<RIDE

  NOTHING IN SOFTWARE IS TRUE ABOUT WHICH FLOOR THIS IS, and that stays true after the swap.

  The robot is now seeded into floor $TO's map and will publish a confident map->odom the whole
  way. It is not lying about anything it was asked -- the car looks the same on every floor, so
  the scan really does match, and it would match just as well if the lift were stuck.

  Do not drive and do not read anything into the fit until the doors open.

RIDE
doors "the car has arrived and the doors are open"
clear_costmaps
event ride_end "$FROM->$TO"

# ---------------------------------------------------------------------------------- the gate
# THIS IS THE ONLY STEP THAT CHECKS ANYTHING ABOUT WHICH FLOOR THIS IS. Everything above it was
# either a drive on a map the robot was already localized in, or a seed taken on trust.
say "VERIFY floor $TO -- with the doors OPEN"
event verify_start "$TO"
if [ -n "$DRY" ]; then
    echo "  (dry run: no swap happened, so there is nothing to verify)"
else
    python3 "$REPO/bringup/floor_swap.py" --verify "$TO" --doors-open \
        || die "the handover gate REFUSED. The robot is not confirmed on floor $TO and the base
        must not move. Look at the floor indicator by eye. If the floor is right, seed by hand
        (RViz 2D Pose Estimate) or run python3 bringup/relocalise.py, then re-run:
            python3 bringup/floor_swap.py --verify $TO --doors-open"
fi
event verify_ok "$TO"

# ---------------------------------------------------------------------------------- floor TO
event floor_start "$TO"
nav   "$B_EXIT"            # out

wait_stow
event route_complete "$FROM->$TO"
say "ROUTE COMPLETE -- floor $FROM to floor $TO"
