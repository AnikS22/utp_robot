"""Are these MAP-frame waypoints still valid? Pure logic, no ROS, testable headlessly.

THE POINT OF MAP-FRAME WAYPOINTS. Odom-frame coordinates die whenever `ranger_base` restarts
(safety/waypoint_frame.py is the guard for that) and drift continuously in between. A waypoint
expressed in a SLAM map's frame has neither problem -- provided the robot is localized in the
same map it was recorded in. That proviso is what this module checks.

TWO KINDS OF `map` FRAME, AND THEY LOOK IDENTICAL FROM THE TF TREE. This is the trap.

  * MOLA started FRESH, no map loaded. It invents a `map` frame whose origin is wherever the
    robot happened to be at startup. The TF tree looks exactly like the localized case. But the
    coordinates mean nothing to the next session -- this is session-scoped, exactly like odom,
    just far more accurate within the session.
  * MOLA started with a SAVED MAP loaded and relocalized into it. Now `map` is the saved map's
    own frame, the origin is fixed for all time, and coordinates ARE portable between sessions.
    This is the case that removes the re-record-before-every-run tax.

So "the map frame exists" is not evidence that a stored coordinate is meaningful. What
distinguishes them is whether a NAMED map was loaded, which is why recording stores the map name
and this module refuses to treat a nameless (fresh-MOLA) recording as portable.
"""
from __future__ import annotations

FRAME_KEY = "frame"
MAP_NAME_KEY = "map_name"
MOLA_SESSION_KEY = "mola_session"

FRAME_ODOM = "odom"
FRAME_MAP = "map"


def frame_of(wp: dict) -> str:
    """Which frame is this waypoint in? Absent field means odom -- every waypoint recorded before
    map-frame support existed is an odom waypoint, and must keep validating as one."""
    return ((wp or {}).get(FRAME_KEY) or FRAME_ODOM)


def split_by_frame(waypoints: dict, names=None) -> tuple[dict, dict]:
    """(odom_waypoints, map_waypoints), limited to ``names`` if given."""
    sel = {k: v for k, v in (waypoints or {}).items() if names is None or k in names}
    odom = {k: v for k, v in sel.items() if frame_of(v) == FRAME_ODOM}
    mapf = {k: v for k, v in sel.items() if frame_of(v) == FRAME_MAP}
    return odom, mapf


def check_map_session(waypoints: dict, current_map: str | None, current_mola: str | None,
                      names=None) -> tuple[bool, str]:
    """Can we trust these map-frame coordinates right now?

    current_map   name of the map currently loaded and relocalized into, or None if MOLA is
                  running fresh (no saved map).
    current_mola  id of the running MOLA instance (the DDS GID of its pose publisher), or None
                  if MOLA is not running at all.

    Fail-closed throughout, matching safety/waypoint_frame.py: an unanswerable question is a
    refusal, not a shrug.
    """
    _, sel = split_by_frame(waypoints, names)
    if not sel:
        return True, ""

    if current_mola is None:
        return False, (
            "waypoints %s are in the MAP frame, but MOLA is not publishing a pose -- so there is "
            "no map frame to interpret them in.\n"
            "  Fix: start it (bash bringup/lidar3d.sh, then the MOLA launch), or re-record these "
            "waypoints in the odom frame."
            % ", ".join(sorted(sel)))

    # Portable case: the waypoint names a map, and we are relocalized into that same map.
    named = {k: v for k, v in sel.items() if (v or {}).get(MAP_NAME_KEY)}
    nameless = sorted(k for k in sel if k not in named)

    wrong_map = sorted(k for k, v in named.items()
                       if current_map is not None and v[MAP_NAME_KEY] != current_map)
    if wrong_map:
        want = sorted({named[k][MAP_NAME_KEY] for k in wrong_map})
        return False, (
            "waypoints %s were recorded in map '%s', but the robot is localized in '%s'. Those "
            "are different coordinate frames and the numbers do not transfer.\n"
            "  Fix: load the map they were recorded in, or re-record them in this one."
            % (", ".join(wrong_map), ", ".join(want), current_map))

    if named and current_map is None:
        return False, (
            "waypoints %s were recorded against saved map '%s', but MOLA is running FRESH with no "
            "map loaded -- its `map` frame origin is wherever the robot happened to start, not "
            "the saved map's origin. The TF tree looks the same either way, which is exactly why "
            "this is checked rather than assumed.\n"
            "  Fix: load the map and relocalize (bash bringup/map_load.sh %s) before running."
            % (", ".join(sorted(named)), sorted({v[MAP_NAME_KEY] for v in named.values()})[0],
               sorted({v[MAP_NAME_KEY] for v in named.values()})[0]))

    # Session-scoped case: no map was loaded when these were recorded, so they are only valid
    # while that same MOLA instance keeps running.
    if nameless:
        stale = sorted(k for k in nameless
                       if (sel[k] or {}).get(MOLA_SESSION_KEY) != current_mola)
        if stale:
            return False, (
                "waypoints %s were recorded in a MOLA session that is no longer running, with no "
                "saved map to anchor them. A fresh MOLA puts its map origin wherever the robot "
                "started, so those coordinates now point somewhere else entirely.\n"
                "  This is the same failure as a re-zeroed odom frame, and it is not a planner "
                "fault.\n"
                "  Fix: re-record them. To make waypoints survive between sessions, save a map "
                "(bash bringup/map_save.sh <name>) and record against it."
                % ", ".join(stale))

    return True, ""
