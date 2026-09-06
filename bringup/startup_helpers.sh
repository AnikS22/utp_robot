# Readiness helpers used by bringup_all.sh. No ROS or hardware action when sourced.
wait_ready() {
  local budget="$1" topic="$2" minimum="$3" deadline remaining good spec kind a b rest
  shift 3
  deadline=$((SECONDS + budget))
  while [ "$SECONDS" -lt "$deadline" ]; do
    remaining=$((deadline - SECONDS))
    note "waiting for $topic (${remaining}s remaining)"
    PROBE_LIMIT="$remaining" probe "$@"
    good=1
    [ -z "${P[err]:-}" ] || good=0
    if [ "$minimum" = latched ]; then
      [ "$(seen "$topic")" -ge 1 ] || good=0
    else
      ge "$(hz "$topic")" "$minimum" || good=0
    fi
    for spec in "$@"; do
      IFS=: read -r kind a b rest <<< "$spec"
      if [ "$kind" = tf ]; then tfok "$a" "$b" || good=0; fi
    done
    [ "$good" = 1 ] && return 0
    [ "$SECONDS" -lt "$deadline" ] && sleep 1
  done
  return 1
}

ensure_active() {
  local node="$1" deadline=$((SECONDS + $2)) state remaining
  while [ "$SECONDS" -lt "$deadline" ]; do
    remaining=$((deadline - SECONDS))
    state=$(timeout "$remaining" ros2 lifecycle get "$node" 2>/dev/null | awk 'NR==1 {print $1}')
    case "$state" in
      active) return 0 ;;
      unconfigured|inactive)
        remaining=$((deadline - SECONDS))
        [ "$remaining" -gt 0 ] || return 1
        if [ "$state" = unconfigured ]; then
          timeout "$remaining" ros2 lifecycle set "$node" configure >>"$LOG" 2>&1 || return 1
        else
          timeout "$remaining" ros2 lifecycle set "$node" activate >>"$LOG" 2>&1 || return 1
        fi ;;
    esac
    sleep 1
  done
  return 1
}
