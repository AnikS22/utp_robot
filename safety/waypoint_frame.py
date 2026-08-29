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
