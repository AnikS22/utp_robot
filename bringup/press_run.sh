#!/usr/bin/env bash
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

echo "=============================================================="
echo " 1/6  LOOK      capturing an aligned RGB-D frame"
echo "=============================================================="
python3 "$REPO/bringup/grab_frame.py" --name "$NAME" --timeout 45

echo
echo "=============================================================="
echo " 2/6  GROUND    running the shipped detector on that frame"
echo "=============================================================="
[ -x "$VENV" ] || { echo "pipeline venv not found at $VENV" >&2; exit 1; }
# PREFER A FRESH REPROJECTED TARGET. At the press standoff the plate is dead ahead at ~0.7 m,
# behind the robot's own stowed arm in the mast camera's view; grounding from here returned the
# FIRE alarm (2026-08-29) and the veto refused it. face_target/reproject_target write the plate's
# position -- grounded and vetoed from further back -- re-expressed at THIS pose. Use it if it is
# recent; otherwise ground as before. The veto still runs on the fresh frame either way.
PT="$REPO/captures/press_target.json"
if [ -f "$PT" ] && python3 -c "import json,sys,time; d=json.load(open('$PT')); sys.exit(0 if time.time()-d.get('written_at',0) < 300 else 1)"; then
    echo "  using reprojected target from $PT ($(python3 -c "import json;print(json.load(open('$PT'))['source'])"))"
    cp "$PT" "$CAP/detection.json"
elif [ -n "$QUERY" ]; then
    "$VENV" "$REPO/bringup/detect_frame.py" "$CAP" --query "$QUERY"

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
echo " 3/6  SHOW      opening what the detector chose"
echo "=============================================================="
echo "  $CAP/detection.png   (green = chosen, orange = runners-up)"
if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    ( setsid nohup xdg-open "$CAP/detection.png" >/dev/null 2>&1 & )
    sleep 1
else
    echo "  (no display; open it yourself)"
fi

echo
echo "=============================================================="
echo " 4/6  READY     wrist to the press orientation"
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
