"""Identify the odom session, so a stale waypoint cannot be mistaken for a fresh one.

The DDS GID of the /odom publisher is a new value for every publisher instance, so it changes
exactly when ranger_base restarts -- which is exactly when odom re-zeroes and every stored
waypoint silently becomes wrong. See safety/waypoint_frame.py for what that cost.
"""
from __future__ import annotations

ODOM_TOPIC = "/odom"


def odom_session_id(node, topic: str = ODOM_TOPIC) -> str | None:
    """Short stable id for the current /odom publisher, or None if nothing is publishing.

    None on two or more publishers as well: with a duplicate driver there is no single session to
    validate against, and that is itself the bug health.py's check_duplicates hunts.
    """
    try:
        infos = node.get_publishers_info_by_topic(topic)
    except Exception:
        return None
    if len(infos) != 1:
        return None
    gid = bytes(infos[0].endpoint_gid)
    return gid.hex()[:16]
