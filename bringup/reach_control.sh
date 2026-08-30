#!/usr/bin/env bash
# Bring a grounded control within arm reach: look, ground, reposition the base, and stop.
#
#     bash bringup/reach_control.sh                                   # default query
#     bash bringup/reach_control.sh --query "the elevator call button"
#     bash bringup/reach_control.sh --dry-run                         # look and plan, move nothing
#
# THE BASE DRIVES unless --dry-run. It does NOT move the arm and never presses anything.
#
# WHY THIS IS A SEPARATE STEP FROM THE PRESS. approach_blockage stops the base ~0.55 m from the
# OBSTRUCTION -- the door. The control that opens it is on the wall BESIDE the door, so from a
# door-facing pose it is routinely outside the arm's 0.88 m envelope. Measured 2026-08-29 at the
# real doors: the grounder found the ADA plate correctly (score 0.403, over two stair-rail decoys)
# at 5.51 m range and +25.1 deg bearing -- out of reach by 4.63 m. Perfect perception, and the arm
# could not have touched it.
#
# IT DELIBERATELY STOPS AFTER MOVING. press_run.sh then does its OWN look-and-ground from the new
# pose, so the 3D point the arm aims at was measured from where the arm actually is. That is not
# redundancy: isaac_world records a case where the base yawed 0.69 rad between observing and
# pressing, which swings a target 1 m out by half a metre -- the arm reached at blank wall and the
# trial was booked as a GROUNDING failure although the detector and the reasoner had both been
# right. Re-grounding costs one capture and removes that whole class of failure.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"

QUERY="the accessible door push button"
STANDOFF="0.55"
DRY=""
while [ $# -gt 0 ]; do
    case "$1" in
        --query)    QUERY="$2"; shift 2 ;;
        --standoff) STANDOFF="$2"; shift 2 ;;   # metres, base standoff (not the arm's mm standoff)
        --dry-run)  DRY="--dry-run"; shift ;;
        *) shift ;;
    esac
done

NAME="reach_$(date +%s)"
CAP="$REPO/captures/$NAME"
VENV="$HOME/unlocking-the-path/env/.venv/bin/python"

echo "[reach] looking..."
python3 "$REPO/bringup/grab_frame.py" --name "$NAME" --timeout 45

echo "[reach] grounding '$QUERY'..."
"$VENV" "$REPO/bringup/detect_frame.py" "$CAP" --query "$QUERY" || {
    echo "[reach] grounding failed -- refusing to move the base toward a target it did not find" >&2
    exit 1
}
[ -f "$CAP/detection.json" ] || {
    echo "[reach] no detection.json -- nothing was grounded" >&2
    exit 1
}

# NEVER PRESS A FIRE ALARM. Verified against this exact scene 2026-08-29: asked for "the
# accessible door push button" the grounder returned the FAU atrium fire alarm at the highest
# confidence of the session, and the same box came back for two rephrased ADA queries. Low
# confidence is not the signal -- WHAT THE THING IS is the signal.
echo "[reach] checking what we are about to approach..."
"$VENV" "$REPO/bringup/check_press_safe.py" "$CAP" || {
    echo "[reach] REFUSED -- not moving the base toward that target." >&2
    exit 1
}

echo "[reach] positioning the base (coarse, from the far grounding)..."
# Base standoff is passed in metres; press_run.sh's --standoff is the ARM's, in mm. Different
# quantities, deliberately not shared.
python3 "$REPO/bringup/face_target.py" "$CAP" --standoff "$STANDOFF" $DRY

# ---------------------------------------------------------------------------------------------
# SECOND PASS. The first one servos to a target grounded from ~1.7 m, and can be no better than
# that grounding: measured 2026-08-29, reach_control reported "positioned 0.68 m, +0.0 deg off
# the press axis" while the plate was actually 10.4 cm to the side and 4.7 cm high of where the
# arm was then sent. The base had positioned perfectly -- relative to a point that was 10 cm
# wrong. The press missed by exactly that.
#
# From 0.68 m the plate is ~100x90 px instead of ~50 and the same angular error is worth ~4 cm,
# so a second grounding is far better. Two constraints set the order:
#   * the STOWED arm blocks the camera's view of a plate 0.7 m dead ahead, so the arm must be UP
#     to take the accurate frame;
#   * config/safety.yaml gates ALL base motion on measured joint angles, so the arm must be
#     STOWED again before the base may correct.
# Hence: raise -> ground -> stow -> correct. press_run then raises, re-grounds and reaches.
if [ -z "$DRY" ]; then
    ARM_PY="$REPO/.venv-arm/bin/python"
    echo "[reach] raising the arm to clear the camera view..."
    "$ARM_PY" "$REPO/bringup/stow_arm.py" --ready --go || {
        echo "[reach] could not reach the press-ready pose; keeping the coarse position" >&2; exit 0; }

    NAME2="${NAME}_near"
    CAP2="$REPO/captures/$NAME2"
    echo "[reach] re-grounding from the press pose..."
    python3 "$REPO/bringup/grab_frame.py" --name "$NAME2" --timeout 45
    if "$VENV" "$REPO/bringup/detect_frame.py" "$CAP2" --query "$QUERY"         && [ -f "$CAP2/detection.json" ]         && "$VENV" "$REPO/bringup/check_press_safe.py" "$CAP2"; then
        echo "[reach] stowing so the base may move..."
        "$ARM_PY" "$REPO/bringup/stow_arm.py" --go || {
            echo "[reach] STOW FAILED -- base is gated off" >&2; exit 1; }
        echo "[reach] correcting on the near grounding..."
        python3 "$REPO/bringup/face_target.py" "$CAP2" --standoff "$STANDOFF"
        echo "[reach] corrected. Capture kept at $CAP2"
    else
        echo "[reach] near re-grounding failed or was vetoed; stowing and keeping the coarse"              "position -- press_run will ground again from here" >&2
        "$ARM_PY" "$REPO/bringup/stow_arm.py" --go || true
    fi
fi

echo "[reach] done. Capture kept at $CAP"
