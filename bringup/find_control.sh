#!/usr/bin/env bash
# Sweep the camera across the scene, ground every view, throw out the fire alarms, and turn to
# face the best real control.
#
#     bash bringup/find_control.sh
#     bash bringup/find_control.sh --offsets " -40,0,40" --query "the accessible door push button"
#     bash bringup/find_control.sh --dry-run     # sweep and report, turn nothing at the end
#
# THE ROBOT ROTATES IN PLACE. It never translates.
#
# WHY THIS SHAPE. The reasoner told us what it needed, in its own words at the real doors on
# 2026-08-29: "I can see the closed glass doors, but the specific mechanism to open them (a button
# or a card reader) is not visible in the current field of view. I need to move closer to identify
# the correct tool." The D435 is ~69 deg wide and an ADA plate sits on the wall BESIDE the door, so
# from a pose square to the doors it is simply not in the picture, and re-asking the same picture
# cannot help.
#
# The first fix I wrote asked the VLM at each of five bearings -- five API calls, ~30 s, and the
# model comparing pictures it could only see one at a time. This does the obvious thing instead:
# the GROUNDER is local and takes about a second, so ground EVERY view, veto anything that is a
# fire alarm, and keep the best survivor. One sweep, no API calls, and the decision is made across
# all the views at once rather than one at a time.
#
# The fire-alarm veto is not optional here and is why this is not just "pick the highest score":
# at these doors the detector returned the FAU atrium pull station as "the accessible door push
# button" at the highest confidence of the session. Score does not tell you what a thing IS.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/bringup/env.sh"

OFFSETS="-40,0,40"
QUERY="the accessible door push button"
DRY=""
while [ $# -gt 0 ]; do
    case "$1" in
        --offsets) OFFSETS="$2"; shift 2 ;;
        --query)   QUERY="$2";   shift 2 ;;
        --dry-run) DRY="1";      shift ;;
        *) shift ;;
    esac
done

VENV="$HOME/unlocking-the-path/env/.venv/bin/python"
STAMP="$(date +%s)"
RESULT="$REPO/captures/sweep_${STAMP}.json"
CUR=0
BEST_SCORE="0"; BEST_OFF=""; BEST_CAP=""

echo "[find] sweeping ${OFFSETS} deg, grounding '${QUERY}'"
IFS=',' read -ra OFFS <<< "$OFFSETS"
for OFF in "${OFFS[@]}"; do
    OFF="$(echo "$OFF" | tr -d ' ')"
    DELTA="$(python3 -c "print(f'{float($OFF)-float($CUR):.1f}')")"
    python3 "$REPO/bringup/turn_by.py" --deg "$DELTA" >/dev/null || {
        echo "[find] could not reach ${OFF} deg; stopping the sweep" >&2; break; }
    CUR="$OFF"

    NAME="sweep_${STAMP}_${OFF}"
    CAP="$REPO/captures/$NAME"
    python3 "$REPO/bringup/grab_frame.py" --name "$NAME" --timeout 45 >/dev/null

    if ! "$VENV" "$REPO/bringup/detect_frame.py" "$CAP" --query "$QUERY" >/dev/null 2>&1; then
        echo "  ${OFF}deg: grounder found nothing"; continue
    fi
    [ -f "$CAP/detection.json" ] || { echo "  ${OFF}deg: no detection"; continue; }

    SCORE="$(python3 -c "import json;print(json.load(open('$CAP/detection.json')).get('score',0))")"
    # WHAT IS IT, not how confident. A fire alarm scored highest of the whole session once.
    if "$VENV" "$REPO/bringup/check_press_safe.py" "$CAP" >/dev/null 2>&1; then
        echo "  ${OFF}deg: score ${SCORE}  OK"
        if python3 -c "import sys;sys.exit(0 if float('$SCORE')>float('$BEST_SCORE') else 1)"; then
            BEST_SCORE="$SCORE"; BEST_OFF="$OFF"; BEST_CAP="$CAP"
        fi
    else
        echo "  ${OFF}deg: score ${SCORE}  REJECTED (reads as a fire alarm / emergency control)"
    fi
done

if [ -z "$BEST_OFF" ]; then
    python3 "$REPO/bringup/turn_by.py" --deg "$(python3 -c "print(f'{-float($CUR):.1f}')")" >/dev/null || true
    echo "[find] no usable control found in any view. Returned to the starting heading." >&2
    exit 1
fi

echo "[find] best at ${BEST_OFF} deg (score ${BEST_SCORE}) -> $BEST_CAP"
python3 - "$RESULT" "$BEST_OFF" "$BEST_SCORE" "$BEST_CAP" <<'EOF'
import json, sys
json.dump({"offset_deg": float(sys.argv[2]), "score": float(sys.argv[3]),
           "capture": sys.argv[4]}, open(sys.argv[1], "w"), indent=2)
EOF

if [ -n "$DRY" ]; then
    python3 "$REPO/bringup/turn_by.py" --deg "$(python3 -c "print(f'{-float($CUR):.1f}')")" >/dev/null || true
    echo "[find] DRY RUN: returned to the starting heading."
else
    DELTA="$(python3 -c "print(f'{float($BEST_OFF)-float($CUR):.1f}')")"
    python3 "$REPO/bringup/turn_by.py" --deg "$DELTA" >/dev/null
    echo "[find] facing the control. Next: reach_control.sh then press_run.sh"
fi
echo "[find] result -> $RESULT"
