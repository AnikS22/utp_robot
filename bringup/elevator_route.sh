#!/usr/bin/env bash
# THE ELEVATOR SEQUENCE. Separate from press_route.sh on purpose: different scene, different
# queries, different failure modes.
#
#     bash bringup/elevator_route.sh              # from the top
#     bash bringup/elevator_route.sh --from 5     # resume at a stage
#     bash bringup/elevator_route.sh --dry-run
#
#   1  navigate to 'call_button'
#   2  press the OUTSIDE call button
#   3  navigate to 'lift_door_reverse'
#   4  reverse into the car        <- ON THE RC. Not autonomous, and should not be.
#   5  navigate to 'car_panel'
#   6  press the FLOOR button
#   7  navigate to 'car_facing_out'
#
# TWO THINGS THIS GETS RIGHT THAT THE FIRST DRAFT DID NOT, both found on hardware 2026-09-01:
#
# ARRIVAL IS NOT AN EXIT CODE. nav2_goto returns 0 for BOTH `arrived` and `blocked` -- that is its
# documented contract, because both are real outcomes of the world. A `|| die` therefore treats a
# blocked leg as success: the robot stayed 3.0 m from the call button, the script walked on, and
# the press stage ground a target across the room. Every leg here parses the RESULT json and
# demands status == arrived.
#
# THE QUERY IS PER STAGE. press_run.sh defaults to "the accessible door push button" -- the ADA
# door. Run unqualified at the lift it grounded that plate 3.89 m away and correctly refused to
# reach. The call button and the floor panel each get their own query, and the floor panel's is
# the tape, because "elevator button" loses to four larger fire-service keyswitches above it.
set -uo pipefail

# ---- PREFLIGHT -------------------------------------------------------------------------------
# This route runs on its OWN Nav2 params. nav2_params_os0_map.yaml (the door trial) leaves the MPPI
# cost critic checking the full rectangle against an inflation radius smaller than the circumscribed
# radius; on 2026-09-01 that starved the control loop to 1.2 Hz and aborted the leg to 'call_button'
# after 3.0 s with "Failed to make progress" -- while the goal cell itself read cost 0.

# The base is gated on MEASURED joint angles. If the arm is out, the mux DISCARDS every Nav2
# command and the failure surfaces as a navigation abort several minutes later, nowhere near its
# cause. Check it before the first goal, not after the first mystery.
stowed="$(timeout 6 ros2 topic echo /safety/arm_stowed std_msgs/msg/Bool --once 2>/dev/null | grep -o 'data: .*' | head -1 | awk '{print $2}')"
if [ "$stowed" != "true" ]; then
    echo "STOP: /safety/arm_stowed is '${stowed:-<no publisher>}'. The base will not move." >&2
    echo "      Fold with:  .venv-arm/bin/python bringup/stow_arm.py --go" >&2
    exit 1
fi
echo "  preflight: arm stowed, nav2 params = elevator fork"
# ----------------------------------------------------------------------------------------------

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"

CALL_QUERY="${UTP_CALL_QUERY:-the elevator call button on the wall}"
FLOOR_QUERY="${UTP_FLOOR_QUERY:-the blue elevator button}"
DRY=""; FROM=1
while [ $# -gt 0 ]; do
  case "$1" in --dry-run) DRY="--dry-run";; --from) FROM="$2"; shift;; esac; shift
done

say(){ echo; echo "=============================================================="; echo " $*"; \
       echo "=============================================================="; }
die(){ echo; echo "STOP: $*" >&2; exit 1; }

# Demand ARRIVED. Anything else -- blocked, timeout, rejected, refused -- stops the sequence,
# because every later stage assumes the robot is where the waypoint says.
nav(){
  local wp="$1" out status
  if [ -n "$DRY" ]; then python3 "$REPO/bringup/nav2_goto.py" "$wp" || true; return 0; fi
  out="$(python3 "$REPO/bringup/nav2_goto.py" "$wp" --go 2>&1)"; echo "$out"
  status="$(printf '%s\n' "$out" | grep -o 'RESULT {.*}' | tail -1 \
            | python3 -c 'import json,sys
s=sys.stdin.read().strip()
print(json.loads(s[7:])["status"] if s.startswith("RESULT ") else "unparsed")' 2>/dev/null)"
  [ "$status" = "arrived" ] || die "leg to '$wp' ended as '${status:-unknown}', not arrived.
        Later stages assume the robot is AT that waypoint, so continuing would press from the
        wrong place -- which is exactly what happened on 2026-09-01."
  echo "  arrived at '$wp'"
}

press(){
  local what="$1" query="$2"
  say "PRESS  $what"
  echo "  query: '$query'"
  bash "$REPO/bringup/elevator_press.sh" $DRY --query "$query" || die "$what press failed"
}

[ "$FROM" -le 1 ] && { say "1  NAVIGATE to 'call_button'"; nav call_button; }
[ "$FROM" -le 2 ] && press "the OUTSIDE call button" "$CALL_QUERY"
[ "$FROM" -le 3 ] && { say "3  NAVIGATE to 'lift_door_reverse'"; nav lift_door_reverse; }
[ "$FROM" -le 4 ] && {
    say "4  REVERSE INTO THE CAR -- DRIVE THIS ON THE RC"
    echo "  The self-occlusion mask blanks |bearing| 74-180 deg within 0.90 m, so the robot cannot"
    echo "  see close behind it -- and that is where you stand holding the doors. Reversing beats"
    echo "  turning inside a ~1.5 m car (rotation is where the matcher loses lock at 4.6 Hz), but"
    echo "  it means a human who can see drives the entry."
    echo
    echo "  Back it in to roughly 'car_facing_out', then:  bash bringup/elevator_route.sh --from 5"
    exit 0; }
[ "$FROM" -le 5 ] && { say "5  NAVIGATE to 'car_panel'"; nav car_panel; }
[ "$FROM" -le 6 ] && press "the FLOOR button" "$FLOOR_QUERY"
[ "$FROM" -le 7 ] && { say "7  NAVIGATE to 'car_facing_out'"; nav car_facing_out; }

say "SEQUENCE COMPLETE -- confirm the doors, then drive straight out"
