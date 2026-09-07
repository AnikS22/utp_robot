#!/usr/bin/env bash
# THE WHOLE RUN, AS ONE COMMAND.
#
#     bash bringup/mission.sh                 # floor 2 -> floor 1, press the ADA plate, drive out
#     bash bringup/mission.sh --dry-run       # every stage runs, nothing moves
#     bash bringup/mission.sh --from 2 --to 1
#
# WHY THIS EXISTS. Every piece of this run already worked, and the run still needed a human typing
# the next command each time -- claim CAN, clear an arm fault, stow, load a map, seed a pose,
# relocalise, then the legs. That is not a robot doing a task, it is a person doing a task with a
# robot. Worse, the operator was the only thing holding the ordering constraints, and they are not
# obvious: localize before driving, fold before grounding, swap while the car moves, relocalise
# only once the doors are open.
#
# WHAT IT FIGURES OUT ITSELF, rather than being told:
#   * CAN authority. The chassis silently discards every command in CONTROL_MODE_RC or STANDBY
#     while odom, the mux and /cmd_vel all look perfectly healthy. Claimed automatically.
#   * Arm faults. ControllerError 31 latches state=4 and the arm then refuses to move. Cleared.
#   * WHERE IT IS. No SEED_POSE argument anywhere. relocalise.py searches every free cell of the
#     map x 72 headings -- 831,312 poses in about a second on floor1 -- so the mission never asks
#     a human for a coordinate it can measure.
#
# WHAT IT STILL WAITS FOR: the doors, twice. A lift door is not observable from software until it
# moves, and nothing here pretends otherwise.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh" >/dev/null 2>&1 || { echo "env.sh failed" >&2; exit 1; }

FROM=2; TO=1; DRY=""; ARRIVAL_ONLY=0; MANUAL_CALL=0
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY="--dry-run"; shift ;;
    # Resume at the arrival half. The ride is the one part a script cannot own -- the robot is
    # carried and nothing in software is true about which floor it is on -- so a run interrupted
    # anywhere in the car needs a way back in without re-driving floor 2.
    --arrival-only) ARRIVAL_ONLY=1; shift ;;
    # The OPERATOR presses the call plate; the robot does everything else. Not a convenience:
    # measured 2026-09-06, the call press grounds the right button (0.564) with clean depth
    # (810/810 valid, 2 mm std) and still lands wide, because the hand-eye solve is
    # flange-relative while a 172 mm tool makes set_position mean the tool tip -- roughly
    # 100-170 mm along the approach axis. A 120 mm ADA plate absorbs that; a 27 px call button
    # does not. arm_tool.py says which state is right "cannot be settled from software" and needs
    # the physical measurement in docs/CALIBRATION.md item 2. Until that is done, this flag keeps
    # the rest of the run autonomous instead of blocking it behind a calibration.
    --manual-call) MANUAL_CALL=1; shift ;;
    --from) FROM="$2"; shift 2 ;;
    --to)   TO="$2";   shift 2 ;;
    *) echo "usage: bash bringup/mission.sh [--dry-run] [--from N] [--to N]" >&2; exit 2 ;;
  esac
done

say()  { echo; echo "=============================================================="; \
         echo " $*"; echo "=============================================================="; }
die()  { echo; echo "STOP: $*" >&2; exit 1; }
note() { echo "  $*"; }

# ---------------------------------------------------------------------------- config
eval "$(python3 - "$REPO" "$FROM" "$TO" <<'PY'
import sys, shlex, yaml
sys.path.insert(0, sys.argv[1] + "/safety")
from floor_plan import floors_of
repo, a, b = sys.argv[1], sys.argv[2], sys.argv[3]
fl = floors_of(yaml.safe_load(open(repo + "/config/floors.yaml")))
for tag, fid in (("A", a), ("B", b)):
    if fid not in fl:
        sys.stderr.write(f"unknown floor {fid}\n"); raise SystemExit(1)
    f = fl[fid]
    for role, name in f.waypoints.items():
        print(f"{tag}_{role.upper()}={shlex.quote(name)}")
    print(f"{tag}_MAP={shlex.quote(f.map)}")
    print(f"{tag}_KIND={shlex.quote(f.kind)}")
    print(f"{tag}_CALL_QUERY={shlex.quote(f.call_query)}")
    print(f"{tag}_SELECT_QUERY={shlex.quote(f.select_query)}")
    print(f"{tag}_TASK_QUERY={shlex.quote(f.task_query)}")
    print(f"{tag}_SELECT_INDEX={shlex.quote(str(f.extra.get('select_index_from_bottom', '')))}")
PY
)" || die "could not read config/floors.yaml for floors $FROM and $TO"

# ---------------------------------------------------------------------------- helpers
# ARRIVAL IS NOT AN EXIT CODE. nav2_goto.py returns 0 for BOTH "arrived" and "blocked" -- a
# documented contract with RosWorld, which reads the JSON RESULT line. Reading the exit code here
# is how a drive that aborted after 158 s was treated as an arrival on 2026-09-06, and the arm was
# then commanded at a plate 5 m away.
nav() {
    local wp="$1" out status
    say "NAVIGATE  '$wp'"
    [ -n "$DRY" ] && { python3 "$REPO/bringup/nav2_goto.py" "$wp" || true; return 0; }
    out="$(mktemp)"
    python3 "$REPO/bringup/nav2_goto.py" "$wp" --go --timeout "${UTP_NAV_TIMEOUT:-180}" 2>&1 | tee "$out"
    status="$(sed -n 's/^RESULT //p' "$out" | tail -1 | python3 -c \
        'import sys,json; d=sys.stdin.read().strip(); print(json.loads(d).get("status","") if d else "")' 2>/dev/null)"
    rm -f "$out"
    [ "$status" = "arrived" ] || die "leg to '$wp' came back '${status:-no RESULT}', not an arrival"
}

# CONTACT IS THE PRESS SUCCEEDING. There is no force sensor -- get_ft_sensor_data answers zeros --
# so the controller's abnormal-current trip (ControllerError 31) is the only thing that can report
# the gripper meeting the plate. Only 31: 21/22/23/24 mean the arm never got there.
press() {
    local query="$1" profile="${2:-}" pick="${3:-}" log rc contact=0
    say "PRESS  '$query'${profile:+   [offset: $profile]}${pick:+   [button ${pick} from bottom]}"
    # FOLD BEFORE GROUNDING, ALWAYS. press_run.sh grounds with the arm parked and only then moves
    # it to the press orientation, because grounding after the arm moves PHOTOGRAPHS THE ARM --
    # its header records a run where the detector, handed a picture of the robot's own arm, chose
    # a corner of the fire alarm. A press that refused on reach leaves the arm at `ready`, so the
    # NEXT press in the same run starts with the arm in frame. Measured 2026-09-06 in the lift car:
    # the second attempt grounded at 1.29 m, off the panel entirely, for exactly this reason.
    [ -n "$DRY" ] || "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go >/dev/null 2>&1
    [ -n "$DRY" ] && { UTP_OFFSET_PROFILE="$profile" UTP_PICK_FROM_BOTTOM="$pick" \
        bash "$REPO/bringup/press_run.sh" --dry-run --query "$query" || true; return 0; }
    log="$(mktemp)"
    # ONE MOTION, AND FASTER. approach_target.py splits the reach into 60 mm segments so a fault
    # stops at a known pose and joint headroom is re-checked before each commit -- worth it while
    # the chain was being debugged, and pure cost now that it works: four commanded moves and four
    # settles on a task timed by a door closer. UTP_STEP_MM=1000 collapses it to a single move.
    # UTP_STANDOFF 30, not the 60 default. At 60 the floor-2 call press completed with no
    # contact at all -- correct target, clean move, retreat, no error 31. Contact is the only
    # evidence of a press this rig can produce, so stopping short of it is a silent no-op dressed
    # as a success. 30 mm leaves the gripper travelling into the plate rather than beside it.
    UTP_NO_STOW=1 UTP_OFFSET_PROFILE="$profile" UTP_PICK_FROM_BOTTOM="$pick" \
    # --hold: DO NOT RETREAT TO THE START POSE. approach_target.py's success path drives the arm
    # all the way back to wherever it began -- 407 mm of Cartesian motion at 60 mm/s, about seven
    # seconds -- and then the caller folds it to stow anyway. The retreat exists so a press run by
    # hand leaves the arm where it was found; inside a route it is pure cost, and it is spent
    # while a lift door closes. Folding straight from the pressed pose is one joint-space move at
    # SPEED_DEG_S, and stow_arm.py checks its own limit violations before committing.
    UTP_STANDOFF="${UTP_STANDOFF:-30}" \
    UTP_STEP_MM="${UTP_STEP_MM:-1000}" UTP_REACH_SPEED="${UTP_REACH_SPEED:-90}" \
        bash "$REPO/bringup/press_run.sh" --query "$query" --hold 2>&1 | tee "$log"
    rc=${PIPESTATUS[0]}
    grep -qE "code: 31|err=31" "$log" && contact=1
    rm -f "$log"
    if [ "$contact" = 1 ]; then
        note "CONTACT (controller error 31) -- the gripper met the plate"
        arm_clear
    elif [ "$rc" -ne 0 ]; then
        die "press chain failed on '$query' with no contact detected"
    fi
    # Fold in the BACKGROUND. config/safety.yaml sets require_arm_stowed: false, so the arbiter
    # does not gate base motion on the arm -- waiting for the fold buys nothing and costs it.
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go >/dev/null 2>&1 &
    STOW_PID=$!
}

arm_clear() {
    [ -n "$DRY" ] && return 0
    "$REPO/.venv-arm/bin/python" - <<'PY' 2>/dev/null | sed 's/^/  /'
from xarm.wrapper import XArmAPI
a = XArmAPI("192.168.1.221", is_radian=False)
a.clean_error(); a.clean_warn(); a.motion_enable(True); a.set_mode(0); a.set_state(0)
print(f"arm cleared: state={a.state} error={a.error_code}")
a.disconnect()
PY
}

wait_stow() { [ -n "${STOW_PID:-}" ] && { wait "$STOW_PID" 2>/dev/null; STOW_PID=""; }; return 0; }

clear_costmaps() {
    [ -n "$DRY" ] && return 0
    for s_ in /global_costmap/clear_entirely_global_costmap \
              /local_costmap/clear_entirely_local_costmap; do
        timeout 20 ros2 service call "$s_" nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null 2>&1 \
            && note "cleared $s_" || echo "    FAIL $s_" >&2
    done
    sleep 2
}

# doors <message> [open|close]
#
# THE SECOND ARGUMENT IS NOT DECORATION. Every call once waited for OPEN, including "let them
# CLOSE, then ride", which would have waited for the opposite of what it wanted and then killed the
# run one step before the ride.
#
# CLEAR THE COSTMAPS WHILE WAITING, NOT AFTER SEEING THEM OPEN. This used to detect open, then
# clear both costmaps and sleep 2 s before returning -- about five seconds of the door hold spent
# standing still. A lift door does not hold long: measured 2026-09-06, the lidar read 3.10 m clear,
# the clear ran, and the entry leg then aborted after 25 s with "recoveries exhausted" because by
# then the doors were closing and the obstacle layer had re-marked the opening. The stale marks all
# date from BEFORE the doors opened, so they can be cleared at any time during the wait -- and then
# the instant the lidar says open there is nothing to do but drive.
doors() {
    local want="${2:-open}" t0=$SECONDS last=$SECONDS
    say "DOORS -- $1"
    [ -n "$DRY" ] && return 0

    if [ "$want" = close ]; then
        while [ $(( SECONDS - t0 )) -lt "${UTP_DOOR_CLOSE_WAIT:-45}" ]; do
            if ! python3 "$REPO/bringup/doors_open_lidar.py" --once --quiet \
                    --clear-m "${UTP_DOOR_CLEAR:-1.6}" >/dev/null 2>&1; then
                note "doors are shut -- the scan is only the car now"
                clear_costmaps
                return 0
            fi
            sleep 2
        done
        note "doors still read OPEN after ${UTP_DOOR_CLOSE_WAIT:-45}s -- continuing anyway"
        note "the seed survives an open-door swap less well; watch the fit after the ride"
        clear_costmaps
        return 0
    fi

    clear_costmaps
    while [ $(( SECONDS - t0 )) -lt "${UTP_DOOR_WAIT:-60}" ]; do
        if python3 "$REPO/bringup/doors_open_lidar.py" --once --quiet \
                --clear-m "${UTP_DOOR_CLEAR:-1.6}" >/dev/null 2>&1; then
            note "doors are open -- going NOW (costmaps already clear)"
            return 0
        fi
        if [ $(( SECONDS - last )) -ge 8 ]; then clear_costmaps >/dev/null 2>&1; last=$SECONDS; fi
        sleep 1
    done

    if [ -t 0 ]; then
        echo "  lidar still sees them shut. RETURN to go anyway, Ctrl-C to stop."
        read -r _ || true
        clear_costmaps
        return 0
    fi
    note "lidar still sees them shut after ${UTP_DOOR_WAIT:-60}s"
    [ "${UTP_DOORS_OVERRIDE:-0}" = "1" ] || die "the doors did not open. Nothing below may drive:
        Nav2 has them marked as a lethal band across the opening and the leg would abort into
        them. Set UTP_DOORS_OVERRIDE=1 to drive regardless."
    note "UTP_DOORS_OVERRIDE=1 -- driving anyway"
    clear_costmaps
}


# load_map <name> -- THE SLOW HALF. Restart slam_toolbox on the map and verify the sensing chain.
#
# Measured 2026-09-06: 80 seconds. Deserializing a building-sized pose graph, waiting for map->odom
# and re-probing every stage is tens of seconds however it is arranged.
#
# WHERE IT MUST NOT RUN: after the doors open on the destination floor. That was mission.sh's
# original shape and it is why exiting the lift kept failing -- the robot sat in the car through
# the whole bring-up while the doors, which hold for a bounded time, closed again and the obstacle
# layer re-marked the opening. multifloor_route.sh already had this right and I did not carry it
# over: "It is free there. Restarting the node... is tens of seconds; the ride is tens of seconds
# during which the robot must not move anyway."
load_map() {
    local map="$1"
    say "LOAD MAP '$map'"
    [ -n "$DRY" ] && return 0
    local pat; pat=$(printf 'slam_%s' 'toolbox')
    local live=""
    [ -f "$REPO/maps/.loaded_map" ] && live="$(awk '{print $1}' "$REPO/maps/.loaded_map")"
    if [ "$live" != "$map" ]; then
        # KILL AND THEN VERIFY. A slam_toolbox that segfaulted mid-configure leaves a process that
        # no longer carries this repo's env markers; bringup_all.sh then refuses to touch it --
        # correctly, it cannot prove whose it is -- and refuses to start a second copy, so the run
        # dies with "1 process(es) matching 'slam_toolbox' are NOT ours".
        local tries
        for tries in 1 2 3; do
            local found=0
            for p in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
                [ "$(cat /proc/$p/comm 2>/dev/null)" = "bash" ] && continue
                local c; c=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null) || continue
                case "$c" in
                    *"$pat"*) found=1
                              if [ "$tries" = 1 ]; then kill -INT "$p" 2>/dev/null
                              else kill -TERM "$p" 2>/dev/null; fi ;;
                esac
            done
            [ "$found" = 0 ] && break
            sleep 4
        done
        rm -f "$REPO/maps/.loaded_map"
    else
        note "already on '$map' -- not restarting SLAM"
    fi
    # EITHER WAY. bringup_all is idempotent, and it is the only thing that checks the SENSING CHAIN
    # matches the map: floor2 was built from /ouster/points_clean and floor1 from raw points, and
    # localizing one against the other's chain cost 27.8% fit.
    SEED_POSE="${SEED_POSE:-0.0,0.0,0.0}" bash "$REPO/bringup/bringup_all.sh" \
        --mode nav --map "$map" || die "could not bring the stack up on '$map'"
    python3 "$REPO/bringup/startup_mark_map.py" "$map" >/dev/null 2>&1 || true
}

# find_self <name> -- THE FAST HALF, AND IT MUST AGREE WITH ITSELF BEFORE ANYTHING DRIVES.
#
# A single fit number is not evidence. Measured 2026-09-06, in the car on floor 1: the global
# search returned 81.1% -- the best score of the whole session, comfortably over the 80% the tools
# call localized -- on a pose that was simply WRONG. The operator saw it immediately; the number
# did not. A high score means "this scan matches the map well HERE", and inside a lift car a
# doorway-sized gap matches a lot of places well.
#
# What a wrong lock cannot fake is AGREEMENT. The search starts from the live scan each time, so
# repeating it and demanding the answers land in the same place is a far stronger test than any one
# score: a correct lock reconverges to within centimetres, a spurious one wanders between the
# candidates that happened to match. So this runs the search up to UTP_RELOC_TRIES times and
# requires UTP_RELOC_AGREE consecutive results inside UTP_RELOC_TOL_M / UTP_RELOC_TOL_DEG.
#
# It still cannot run before the doors open, for the reason seed_pose gives: sealed in, the scan is
# four blank walls and the search would agree with itself about the wrong answer. Agreement is
# necessary, not sufficient.
find_self() {
    local map="$1" tries="${UTP_RELOC_TRIES:-5}" need="${UTP_RELOC_AGREE:-3}"
    local tol="${UTP_RELOC_TOL_M:-0.30}" told="${UTP_RELOC_TOL_DEG:-15}"
    local minfit="${UTP_RELOC_MIN_FIT:-40}"
    say "FIND SELF on '$map'   (needs $need of $tries searches to agree)"
    [ -n "$DRY" ] && return 0
    local i agree=1 px="" py="" pw="" fit=""
    for i in $(seq 1 "$tries"); do
        python3 "$REPO/bringup/relocalise.py" >/dev/null 2>&1 || true
        sleep 2
        read -r x y w fit <<<"$(python3 - <<'PY'
import rclpy, math, time, tf2_ros, rclpy.time, subprocess, os, re
from rclpy.node import Node
rclpy.init(); n=Node("utp_find_self"); b=tf2_ros.Buffer(); tf2_ros.TransformListener(b,n)
e=time.time()+12
while time.time()<e:
    rclpy.spin_once(n,timeout_sec=0.1)
    if b.can_transform("map","base_link",rclpy.time.Time()): break
try:
    t=b.lookup_transform("map","base_link",rclpy.time.Time())
    x,y=t.transform.translation.x,t.transform.translation.y
    yaw=math.degrees(2*math.atan2(t.transform.rotation.z,t.transform.rotation.w))
    print(f"{x:.4f} {y:.4f} {yaw:.2f}", end=" ")
except Exception:
    print("nan nan nan", end=" ")
rclpy.shutdown()
PY
)"
        fit="$(python3 "$REPO/bringup/relocalise.py" --check 2>&1 | sed -n 's/.*fit \([0-9.]*\)%.*/\1/p' | tail -1)"
        note "search $i: (${x},${y}) yaw ${w}  fit ${fit:-?}%"
        if [ -n "$px" ]; then
            agree=$(python3 -c "
import sys,math
dx=$x-$px; dy=$y-$py
dw=abs(($w-$pw+180)%360-180)
print(($agree+1) if (math.hypot(dx,dy)<=$tol and dw<=$told) else 1)")
        fi
        px="$x"; py="$y"; pw="$w"
        if [ "$agree" -ge "$need" ]; then
            awk -v f="${fit:-0}" -v m="$minfit" 'BEGIN{exit !(f < m)}' && \
                die "the searches AGREE at (${x},${y}) but only score ${fit}% -- consistently
                confident and consistently wrong is the failure this check exists to catch."
            note "$need searches agree within ${tol} m / ${told} deg at (${x},${y}), fit ${fit}%"
            return 0
        fi
    done
    die "the global search did NOT converge: $tries attempts never produced $need consecutive
        results within ${tol} m / ${told} deg. The robot does not know where it is, and a fit
        score alone would not have told you -- 81.1% was measured on a demonstrably wrong pose.
        Open the doors fully so the scan reaches past the car, or set the pose in RViz."
}

localize_on() { load_map "$1"; find_self "$1"; }

# ---------------------------------------------------------------------------- 0  PREP
say "0  PREP   authority, arm, stack"
if [ -z "$DRY" ]; then
    python3 "$REPO/bringup/claim_can.py" 2>&1 | tail -2 | sed 's/^/  /'
    python3 "$REPO/bringup/chassis_mode.py" >/dev/null 2>&1 \
        || die "the chassis is not taking commands from the computer. Flip SWB UP on the
        transmitter, then re-run. Until then every command is discarded in firmware while odom,
        the mux and /cmd_vel all look healthy."
    arm_clear
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go 2>&1 | tail -1 | sed 's/^/  /'
fi

# THE GOAL CHECKER IS A TRADE, AND BOTH ENDS OF IT HAVE BITTEN.
#
# Nav2 ships xy_goal_tolerance 0.25 m, so "arrived" meant anywhere within a quarter metre, while
# every press pose was tuned by parking the robot ON its waypoint. Measured 2026-09-06 in the lift
# car: the robot stopped 24 cm short of f2_car_panel, which put the floor button 0.95 m from
# base_link against 0.88 m of reach, and the arm refused. Re-issuing the same goal at 0.08 landed
# it 4 cm out and the press worked.
#
# But 0.08 is tighter than this base settles. On the very next run the f2_call_button leg sat
# 0.10 m from its goal with /cmd_vel 101/102 nonzero and odom at 0.018 m/s -- inching, not
# converging -- until Nav2 gave up: "ABORTED after 155.4 s, recoveries exhausted". A tolerance the
# controller cannot reach does not make arrivals more accurate, it turns them into timeouts.
#
# 0.14 is the compromise: inside it the worst-case shortfall is 14 cm rather than 24, which keeps
# a press pose inside the arm's 0.88 m reach, and it is loose enough that the controller settles.
# If a press ever reports "NOT REACHING" by a few centimetres, re-issuing that one nav goal is the
# cheap fix -- a second attempt from close range lands far tighter than the first.
if [ -z "$DRY" ]; then
    for _pp in "general_goal_checker.xy_goal_tolerance:${UTP_XY_TOL:-0.14}" \
               "general_goal_checker.yaw_goal_tolerance:${UTP_YAW_TOL:-0.20}"; do
        timeout 25 ros2 param set /controller_server "${_pp%%:*}" "${_pp##*:}" >/dev/null 2>&1 \
            && note "goal checker ${_pp%%:*} = ${_pp##*:}" \
            || echo "    could not set ${_pp%%:*} -- arrivals may be too loose for a press" >&2
    done
fi

# ---------------------------------------------------------------------------- 1  floor FROM
if [ "$ARRIVAL_ONLY" = 1 ]; then
    say "SKIPPING floor $FROM -- resuming at the arrival on floor $TO"
else
localize_on "$A_MAP"

# DOORS FIRST, THEN THE PLATE -- the operator's stated order for floor 2. It also puts the robot
# where it can see the lift before it asks for it, so a failed call is visible immediately rather
# than after a drive.
[ -n "${A_DOOR_FACING:-}" ] && nav "$A_DOOR_FACING"
if [ "$MANUAL_CALL" = 1 ]; then
    say "CALL PLATE -- the operator presses it; the robot waits at the doors"
    note "skipping the drive to '$A_CALL_BUTTON' and its press (--manual-call)"
    note "press the call button now; the lidar below is watching the doorway"
else
    nav   "$A_CALL_BUTTON"
    press "$A_CALL_QUERY"
fi

# Forward entry where the floor defines it, else the original reverse.
ENTRY_APPROACH="${A_DOOR_FACING:-${A_DOOR_REVERSE:-}}"
ENTRY_POSE="${A_CAR_FACING_IN:-${A_CAR_FACING_OUT:-}}"
[ -n "$ENTRY_APPROACH" ] || die "floor $FROM defines neither door_facing nor door_reverse"
# GO TO THE DOOR-FACING POSE FIRST, THEN LOOK AT THE DOORS. The check used to run right after the
# press, where the robot is square to the CALL PLATE -- so the forward sector was the wall it had
# just pressed, 0.59 m away, and it reported the doors shut for 60 s while they were beside it.
# Same error as at the ADA plate on floor 1. A door check is only meaningful from a pose that
# faces the door, and ENTRY_APPROACH is that pose by definition.
wait_stow
nav   "$ENTRY_APPROACH"
doors "waiting for the car -- the lidar is watching the doorway now"
nav   "$ENTRY_POSE"
nav   "$A_CAR_PANEL"

# The in-car button belongs to the DESTINATION floor, and gets the offset measured on it.
wait_stow
press "$B_SELECT_QUERY" lift_car_select "${B_SELECT_INDEX:-}"

# Face the doors NOW, while still localized in a map the robot is genuinely in.
wait_stow
nav   "$A_CAR_FACING_OUT"

# ---------------------------------------------------------------------------- 2  the ride
doors "let them CLOSE, then ride" close

# SWAP THE MAP NOW, WHILE THE CAR MOVES. This is the whole reason exiting the lift kept failing:
# load_map takes ~80 s (measured), and it used to run AFTER the doors opened on the destination
# floor -- so the robot sat in the car through the entire bring-up while the doors, which hold for
# a bounded time, closed again and the obstacle layer re-marked the opening. The leg out then
# aborted with "recoveries exhausted" and it read as a navigation fault.
#
# Here it is free. The ride is tens of seconds in which the robot must not move anyway, and the
# doors are shut so the scan is only the car -- the one thing both maps agree about. Nothing is
# seeded: a task floor has no in-car pose and must not be given an invented one. The robot does
# not know where it is until find_self runs with the doors open, and nothing below drives until
# then.
load_map "$B_MAP"

say "RIDE  floor $FROM -> $TO"
cat <<'RIDE'
  Nothing in software is true about which floor this is until the doors open again.
  The scan inside the car matches every floor equally well, and would match just as
  well if the lift were stuck.
RIDE
doors "the car has arrived -- they must be OPEN before anything below runs"
fi

# ---------------------------------------------------------------------------- 3  floor TO
# RELOCALIZE WITH THE DOORS OPEN, AND ONLY THEN. seed_pose's docstring is right that a global
# search inside a CLOSED car is four blank walls and would anchor on any doorway-sized gap. With
# the doors open the scan reaches out into this floor's lobby, and a lift car with a lobby beyond
# it is not a shape that repeats -- which is exactly what the search needs and cannot get sealed in.
# ONLY THE FAST HALF HERE. The map was loaded during the ride; all that is left is the global
# search, about a second, which is the one thing that genuinely cannot happen until the doors open.
# THE RESUME PATH NEEDS BOTH HALVES, AND IN THIS ORDER. --arrival-only skips the floor-FROM block,
# so nothing has loaded the destination map and nothing has waited for the doors. Running find_self
# first would be the exact failure seed_pose warns about: a global search inside a CLOSED car is
# four blank walls, and it anchored 12 m away when we tried it on 2026-09-06. Load the map (slow,
# but the robot is parked anyway), THEN wait for the doors, THEN search.
if [ "$ARRIVAL_ONLY" = 1 ]; then
    load_map "$B_MAP"
    doors "the car is on floor $TO -- they must be OPEN before the search"
fi
find_self "$B_MAP"

# CLEAR BEFORE THE FIRST LEG ON THE NEW FLOOR, ALWAYS. The obstacle layer still holds the lift
# doors as a lethal band across the only way out -- they were shut for the whole ride, and the
# costmap has been marking them the entire time. Measured 2026-09-06: without this the first leg
# aborted twice with "recoveries exhausted" from inside the car; with it, Nav2 planned its own
# route out of the car and across the lobby and arrived in 25.1 s.
#
# This used to happen inside doors(), which --arrival-only skips -- so the resume path was missing
# the one thing that makes the first leg possible.
clear_costmaps

if [ "$B_KIND" = "task" ]; then
    nav   "$B_TASK_BUTTON"
    press "$B_TASK_QUERY"
    # NOTHING between the press and the drive. The opener is already swinging; a check here is a
    # check against a closing door, and all three that were tried were slower than the event.
    nav   "$B_TASK_EXIT"
else
    nav   "$B_EXIT"
fi

wait_stow
say "MISSION COMPLETE -- floor $FROM to floor $TO"
