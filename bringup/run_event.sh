#!/usr/bin/env bash
# Mark a moment in a run, for the paper figures. Sourced by routes.
#
#   source bringup/run_event.sh
#   event leg_start   "call_button"
#   event ground      "the elevator call button on the wall"
#   event press_ok    "contact"
#
# Appends one JSON line to $UTP_RUN_DIR/events.jsonl. The numbered markers on the
# trajectory figure come from these, so a decision with no event is a decision that
# cannot be drawn.
#
# Deliberately a file append, not a ROS topic or a service: routes are bash, and this
# has to work whether or not the recorder is running. With UTP_RUN_DIR unset it is a
# silent no-op, so a route runs identically when nobody is recording -- an instrumented
# run and a plain run must not be two different code paths.
event() {
    [ -n "${UTP_RUN_DIR:-}" ] || return 0
    [ -d "$UTP_RUN_DIR" ] || return 0
    python3 - "$UTP_RUN_DIR" "$1" "${2:-}" <<'PY' 2>/dev/null || true
import json, sys, time, pathlib
d, kind, detail = sys.argv[1], sys.argv[2], sys.argv[3]
with (pathlib.Path(d) / "events.jsonl").open("a") as f:
    f.write(json.dumps({"stamp": time.time(), "kind": kind, "detail": detail}) + "\n")
PY
}
