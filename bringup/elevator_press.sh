#!/usr/bin/env bash
# THE ELEVATOR PRESS. A COPY of press_run.sh, deliberately.
#
# press_run.sh is the ADA-door press and it WORKS -- it made contact on the plate on 2026-09-01
# after a long evening of tuning. It is not to be edited to accommodate a new scene. The elevator
# is a different scene with different queries, a different panel height, 25 mm targets instead of
# a 120 mm plate, and its own failure modes. Forking is cheaper than a regression in the one
# press chain that is known good.
#
# WHAT DIFFERS FROM press_run.sh:
#   * the arm ALWAYS folds on exit, including a failed one (see the trap below)
#   * no default query -- the caller must say what to look for
#
# One command: look -> ground -> show you the photo -> reach -> retreat.
#
#     bash bringup/press_run.sh                      # full run, 60 mm press standoff
#     bash bringup/press_run.sh --standoff 100       # stop further out
#     bash bringup/press_run.sh --dry-run            # look and ground, plan the reach, move nothing
#     bash bringup/press_run.sh --hold               # stay extended (to measure the gap by hand)
#
# THE ARM MOVES TOWARD A WALL unless --dry-run. Hand on the E-stop.
#
# WHY THIS IS A SCRIPT AND NOT ONE PROGRAM. Perception and motion run under DIFFERENT PYTHONS:
# the grounder needs torch (the pipeline venv) and the arm needs rclpy plus the xArm SDK (ROS's
# python). They cannot share a process. So the handoff is a FILE -- captures/<name>/detection.json
# -- which is also why the exact frame the detector saw is kept and can go in the paper.
#
# THE RETREAT IS NOT A SEPARATE STEP. approach_target.py returns to its start pose on success
# unless --hold is passed. Do not add a retreat here; two things retreating is how an arm gets
# commanded somewhere nobody chose.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"
VENV="$HOME/unlocking-the-path/env/.venv/bin/python"

STANDOFF=60          # mm, measured to the MARKER on the flange, not the tool tip.
                     # 60 was confirmed by the operator on 2026-08-25 to reach the plate.
NAME="press_$(date +%H%M%S)"
MODE="--go"
EXTRA=""
QUERY=""
while [ $# -gt 0 ]; do
    case "$1" in
        --standoff) STANDOFF="$2"; shift 2 ;;
        --name)     NAME="$2"; shift 2 ;;
        --query)    QUERY="$2"; shift 2 ;;
        --dry-run)  MODE="--dry-run"; shift ;;
        --hold)     EXTRA="--hold"; shift ;;
        -h|--help)  sed -n '2,8p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done
CAP="$REPO/captures/$NAME"

# READY BEFORE LOOKING. The camera is on the mast, and with the arm STOWED the folded arm sits
# in the lower-centre of the frame -- exactly where a plate 0.7 m dead ahead appears. Measured
# 2026-08-29: from the press pose the grounder returned the FIRE alarm because the plate was behind
# the housing (veto refused it); with the arm in the READY pose the same camera saw the plate
# plainly (0.413, SAFE) and its direct 3D lift put it 10 cm right and 5 cm above where a target
# reprojected through odometry had sent the arm -- which is why that press missed a 12 cm plate.
# Raise the arm first, THEN look, and the target the arm aims at was measured from where the arm is.


echo
echo "=============================================================="
echo " 2/6  LOOK      capturing an aligned RGB-D frame"
echo "=============================================================="
python3 "$REPO/bringup/grab_frame.py" --name "$NAME" --timeout 45

echo
echo "=============================================================="
echo " 3/6  GROUND    running the shipped detector on that frame"
echo "=============================================================="
[ -x "$VENV" ] || { echo "pipeline venv not found at $VENV" >&2; exit 1; }
# PREFER A FRESH REPROJECTED TARGET. At the press standoff the plate is dead ahead at ~0.7 m,
# behind the robot's own stowed arm in the mast camera's view; grounding from here returned the
# FIRE alarm (2026-08-29) and the veto refused it. face_target/reproject_target write the plate's
# position -- grounded and vetoed from further back -- re-expressed at THIS pose. Use it if it is
# recent; otherwise ground as before. The veto still runs on the fresh frame either way.
# The reprojected target is NOT used to aim any more -- it missed by 10 cm. It is kept as a
# CROSS-CHECK: the direct grounding (arm up, camera clear) and the odometry-reprojected point
# should agree to within the plate's size; if they do not, something upstream is wrong and the
# arm does not move. Free consistency test on odom + calibration, every press.
PT="$REPO/captures/press_target.json"
if [ -n "$QUERY" ]; then
    "$VENV" "$REPO/bringup/detect_frame.py" "$CAP" --query "$QUERY"
    if [ -f "$CAP/detection.json" ] && [ -f "$PT" ] && python3 -c "import json,sys,time; d=json.load(open('$PT')); sys.exit(0 if time.time()-d.get('written_at',0) < 300 else 1)"; then
        python3 - "$CAP/detection.json" "$PT" <<'EOF' || { echo "[press] REFUSED -- direct grounding and reprojected target disagree" >&2; exit 1; }
import json, math, sys
a = json.load(open(sys.argv[1]))["point3d_cam_m"]; b = json.load(open(sys.argv[2]))["point3d_cam_m"]
d = math.dist(a, b)
print(f"  cross-check: direct grounding vs odometry-reprojected target differ by {d*100:.1f} cm")
sys.exit(0 if d < 0.20 else 1)
EOF
    fi

else
    "$VENV" "$REPO/bringup/detect_frame.py" "$CAP"
fi
# The last gate before the ARM moves. reach_control checks too, but this is the one that matters:
# it is the only check between a grounded box and a fingertip on it, and press_run is runnable on
# its own. Fails closed -- if it cannot answer, nothing is pressed. See safety/press_veto.py.
[ -f "$CAP/detection.json" ] && "$VENV" "$REPO/bringup/check_press_safe.py" "$CAP" || {
    echo "[press] REFUSED -- the arm will not be commanded at that target." >&2
    exit 1
}
# detect_frame writes detection.json ONLY when it has a 3D point. No point, no aiming.
[ -f "$CAP/detection.json" ] || {
    echo >&2
    echo "STOPPING: no detection.json -- the detector found nothing it could place in 3D." >&2
    echo "  That is a result, not a crash. The arm is not moving. Look at $CAP/detection.png" >&2
    exit 3
}

echo
echo "=============================================================="
echo " 4/6  SHOW      opening what the detector chose"
echo "=============================================================="
echo "  $CAP/detection.png   (green = chosen, orange = runners-up)"
if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    ( setsid nohup xdg-open "$CAP/detection.png" >/dev/null 2>&1 & )
    sleep 1
else
    echo "  (no display; open it yourself)"
fi

# GROUND FIRST, THEN MOVE. This used to run before the LOOK stage, on the claim that the ready
# pose clears the camera. On 2026-09-01 it did the opposite: captures/press_151940 shows the arm
# filling the frame with the ADA plate hidden behind it, and the detector -- handed a photo of the
# robot's own arm -- chose a corner of the FIRE ALARM at 0.38. The operator stopped it.
# It also made --dry-run a lie: without --go the arm does not move, so the dry run grounded from
# the stowed pose and returned the plate at 0.58, a clean preview of a scene the real run would
# never see. The manual sequence that DID work grounded with the arm parked, then moved.
# The target is a point in base_link; moving the arm afterwards cannot change where it is.
echo
echo "=============================================================="
echo " 4b/6 READY     wrist to the press orientation -- AFTER grounding"
echo "=============================================================="
# Stow and press are different orientations: stow folds the wrist to J5=90 so the tool points up
# out of the way; a press needs it pointing AT the wall (J5 ~ 2.5). approach_target.py holds
# whatever orientation the arm starts in, so approaching straight out of stow reaches at the stow
# angle and skids off a round button. The ready pose is the OPERATOR'S, captured with
# `stow_arm.py --save-ready`, not a number invented here.
if [ "$MODE" = "--go" ]; then
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --ready --go || {
        echo "could not reach the press-ready pose; not approaching" >&2; exit 5; }
else
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --ready || true
fi

echo
echo "=============================================================="
echo " 5/6  REACH     $STANDOFF mm marker standoff, then retreat"
echo "=============================================================="
python3 "$REPO/bringup/approach_target.py" --capture "$CAP" $MODE \
        --min-standoff "$STANDOFF" $EXTRA

if [ -z "$EXTRA" ]; then
    echo
    echo "=============================================================="
    echo " 6/6  STOW      folding the arm so the base may move again"
    echo "=============================================================="
    # NOT optional in a route. approach_target.py retreats to its START pose -- wherever the arm
    # happened to be -- not to stow. config/safety.yaml gates ALL base motion on measured joint
    # angles, so without this the very next leg is refused with blocked_by="arm_not_stowed", and
    # the failure looks like a navigation problem immediately after a SUCCESSFUL press.
    # Skipped under --hold, where staying extended is the entire point.
    "$REPO/.venv-arm/bin/python" "$REPO/bringup/stow_arm.py" --go || {
        echo "STOW FAILED -- the base will refuse to move. Fix before driving." >&2; exit 4; }
fi

echo
echo "done. frame + detection kept in $CAP"
