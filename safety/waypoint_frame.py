"""Are these waypoints still expressed in the odom frame the robot is living in right now?

Pure logic, no ROS, so it is testable headlessly.

THE BUG THIS EXISTS TO STOP. Waypoints are stored in the ODOM frame, and `ranger_base` zeroes
odom every time it starts. maps/waypoints.yaml has said so in a header comment since it was
written -- "restarting it re-zeros odom and silently invalidates every entry here. The
'odom_epoch' field is how you tell" -- and that guard was never actually implemented:

  * odom_epoch was written as round(time.time()), the WALL CLOCK at record time. It changes on
    every single recording by construction, so it can never identify an odom session. The three
    distinct values in the shipped file are just the three moments someone pressed record.
  * Nothing ever READ it. Three write sites, zero read sites, in either waypoints.py or
    route_run.py.
  * cmd_rebase dropped the field from its output entirely.

So the file documented a safeguard that did not exist, and a three-day-old waypoint looked
exactly like a fresh one. Driving to a coordinate from a dead odom frame is not a planner
failure, but it presents as one: the robot turns confidently towards a point that is no longer
anywhere, and then either drives into a wall or reports "arrived" without moving.

WHAT IDENTIFIES A SESSION. The DDS GID of the /odom publisher. It is a new value for every new
publisher instance, so it changes exactly when the driver restarts -- which is exactly when odom
re-zeroes. It costs nothing to read and needs no cooperation from the driver.
"""
from __future__ import annotations

SESSION_KEY = "odom_session"


def sessions_in(waypoints: dict) -> set:
    """Every distinct session id present. Missing ids count as None -- pre-guard waypoints."""
    return {(w or {}).get(SESSION_KEY) for w in waypoints.values()}


# Recording spread beyond which drift between waypoints is worth warning about. Odometry error
# accumulates with DRIVING, not with wall-clock, but the time between two recordings is the only
# proxy stored, and on this robot minutes between recordings has always meant metres of driving.
DRIFT_WARN_S = 300.0


def drift_warning(waypoints: dict, names=None) -> str:
    """Warn when a route's waypoints were recorded far apart in time.

    THE GAP THIS COVERS, and check_session cannot. The session id proves the odom frame was never
    RE-ZEROED. It says nothing about the frame DRIFTING, and wheel odometry drifts continuously:
    every metre driven, every turn, and worst of all every wheel rotation that does not move the
    body. The 4WS re-steer thrash of 2026-08-29 spun wheels for 90 s while the chassis stayed
    put -- encoders counting motion that never happened, straight into the pose estimate.

    So two waypoints recorded 15 minutes and several runs apart are in the same SESSION and no
    longer in the same FRAME, and the robot drives confidently to a coordinate that has moved out
    from under it. There is no way to measure that here; the honest thing is to say the recordings
    are far apart and that re-recording them together costs a minute.
    """
    sel = {k: v for k, v in waypoints.items()
           if (names is None or k in names) and (v or {}).get("odom_epoch")}
    if len(sel) < 2:
        return ""
    stamps = {k: float(v["odom_epoch"]) for k, v in sel.items()}
    spread = max(stamps.values()) - min(stamps.values())
    if spread < DRIFT_WARN_S:
        return ""
    oldest = min(stamps, key=stamps.get)
    newest = max(stamps, key=stamps.get)
    return (f"waypoints in this route were recorded {spread/60:.0f} minutes apart "
            f"('{oldest}' oldest, '{newest}' newest). They share an odom session, so the frame "
            f"was never re-zeroed -- but odometry DRIFTS within a session, and everything driven "
            f"in between has accumulated into the pose estimate. The older coordinates may no "
            f"longer point where they did.\n"
            f"  If the robot drives somewhere unexpected, this is the first thing to suspect. "
            f"Re-record them together, immediately before the run.")


def check_session(waypoints: dict, current: str | None, names=None) -> tuple[bool, str]:
    """Can we trust these coordinates right now?

    ``names`` limits the check to the waypoints a route actually visits: a stale entry nobody
    drives to is not a reason to refuse the run.

    Fail-closed on the unknown case. A waypoint recorded before this guard existed carries no
    session id, and there is no way to tell a still-valid one from a dead one -- so it is
    refused, with the two ways out named.
    """
    sel = {k: v for k, v in waypoints.items() if names is None or k in names}
    if not sel:
        return True, ""

    found = sessions_in(sel)
    unknown = sorted(k for k, v in sel.items() if not (v or {}).get(SESSION_KEY))
    if unknown:
        return False, (
            "waypoints %s carry no odom session id, so there is no way to tell whether they were "
            "recorded in the odom frame the robot is in NOW. Odom re-zeroes on every ranger_base "
            "restart, which silently invalidates every stored coordinate.\n"
            "  Fix: re-record them (bringup/waypoints.py record <name>), or if you know the robot "
            "has not moved since the restart, bringup/waypoints.py rebase."
            % ", ".join(unknown))

    if current is None:
        return False, ("cannot read the /odom publisher's session id -- is ranger_bringup "
                       "running? Refusing to drive to coordinates that cannot be validated.")

    stale = sorted(k for k, v in sel.items() if v[SESSION_KEY] != current)
    if stale:
        others = sorted(s for s in found if s != current)
        return False, (
            "waypoints %s were recorded in a DIFFERENT odom session (%s) than the one running now "
            "(%s). ranger_base has restarted since they were recorded, so odom re-zeroed and "
            "every one of those coordinates now points somewhere else entirely.\n"
            "  This is not a planner fault. Driving anyway sends the robot to a point that no "
            "longer exists.\n"
            "  Fix: re-record them, or bringup/waypoints.py rebase if the robot has not physically "
            "moved since the restart."
            % (", ".join(stale), ", ".join(str(o)[:8] for o in others) or "?", str(current)[:8]))

    return True, ""
