#!/usr/bin/env bash
# Start or audit all hardware inputs without mapping, localization, navigation or motion.
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-up}" in
  up|check) ACTION="${1:-up}" ;;
  *) echo "usage: bash bringup/inputs.sh [up|check]" >&2; exit 2 ;;
esac
source "$REPO/bringup/env.sh" >/dev/null 2>&1 || exit 1
OUT="$REPO/captures/input_checks/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$OUT"
status=0
if [ "$ACTION" = up ]; then
  bash "$REPO/bringup/bringup_all.sh" --mode inputs 2>&1 | tee "$OUT/startup.log"
  status=${PIPESTATUS[0]}
fi
python3 "$REPO/bringup/check_inputs.py" --output "$OUT/inputs.json" 2>&1 | tee "$OUT/audit.log"
audit_status=${PIPESTATUS[0]}
echo "Reports: $OUT"
[ "$status" -eq 0 ] && [ "$audit_status" -eq 0 ]
