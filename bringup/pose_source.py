"""Where the robot thinks it is: wheel odometry, or a SLAM map. One interface for both.

WHY THIS EXISTS. waypoints.py and route_run.py both subscribed to /odom directly, so every stored
coordinate was a wheel-odometry coordinate -- dead on every ranger_base restart and drifting in
between. With the Ouster OS0 and MOLA running there is a second, far better source: the TF
`map -> base_link`, which on 2026-08-30 held a parked robot to 0.3 cm and 0.02 deg over 25 s.

This module is the seam. It fills `node.pose` and `node.stamp` exactly as the old /odom callback
did, so the consumers did not have to change shape -- only where the numbers come from.

FRAME SELECTION is explicit-by-default and loud either way, because the two frames are visually
indistinguishable once you are looking at numbers:

    auto (default)  prefer map if MOLA is publishing a usable TF, else fall back to odom, and
                    SAY WHICH in one line. Never silently.
    map             require the map frame; refuse if it is not there.
    odom            the old behaviour, unchanged.

WHAT `auto` DELIBERATELY DOES NOT DO: it does not decide whether the map frame is MEANINGFUL.
A fresh MOLA has a `map` frame whose origin is wherever the robot booted -- portable across
sessions only if a saved map was loaded and relocalized into. safety/map_frame.py owns that
judgement; this module only reports which frame is available and what identifies it.
"""
from __future__ import annotations

import math
import os
import pathlib
import time

from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

MOLA_POSE_TOPIC = "/lidar_odometry/pose"
MAP_FRAME = "map"
BASE_FRAME = "base_link"
ODOM_TOPIC = "/odom"
STALE_S = 0.5


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def mola_session_id(node, topic: str = MOLA_POSE_TOPIC) -> str | None:
    """Short stable id for the running MOLA instance, or None if it is not publishing.

    Same trick and same caveats as odom_session.odom_session_id: the DDS GID of the publisher is
    new for every instance, so it changes exactly when MOLA restarts -- which is exactly when a
    fresh (unloaded) map frame gets a new origin. None on zero OR multiple publishers.
    """
    try:
        infos = node.get_publishers_info_by_topic(topic)
    except Exception:
        return None
    if len(infos) != 1:
        return None
    return bytes(infos[0].endpoint_gid).hex()[:16]


LOADED_MAP_FILE = pathlib.Path(__file__).resolve().parent.parent / "maps" / ".loaded_map"


def current_map_name(node=None) -> str | None:
    """Name of the saved map currently loaded, or None if MOLA is running fresh.

    MOLA does not advertise which map it was given, so this is recorded on the side by
    bringup/map_load.sh at the same moment it calls /map_load.

    THE STALENESS TRAP. A name alone is not enough: load 'atrium', restart MOLA with no map, and
    a name-only record would still claim 'atrium' while the frame origin had moved to wherever
    the robot booted. So map_load.sh stores the map name AND the MOLA session it was loaded into,
    and this returns None when that session is not the one running -- which correctly demotes
    every named waypoint to "recorded against a map that is not loaded".

    ``node`` is optional only so this stays callable without ROS; without it the session cannot
    be checked and the file is trusted, which is why every caller in this repo passes one.
    """
    v = (os.environ.get("UTP_MAP") or "").strip()
    if v:
        return v
    try:
        parts = LOADED_MAP_FILE.read_text().split()
    except OSError:
        return None
    if not parts:
        return None
    name = parts[0]
    loaded_session = parts[1] if len(parts) > 1 else None
    if node is not None and loaded_session is not None:
        if mola_session_id(node) != loaded_session:
            return None            # stale: MOLA restarted since the map was loaded
    return name


class PoseSource:
    """Fills node.pose = (x, y, yaw) and node.stamp, from odom or from the map frame."""

    def __init__(self, node, frame: str = "auto"):
        self.node = node
        self.requested = frame
        self.frame = None          # resolved: "odom" or "map"
        self._buf = None
        self._listener = None
        self._sub = None
        self._timer = None
        self.description = ""
        node.pose = None
        node.stamp = 0.0

    # ---- selection ---------------------------------------------------------------------------
    def _map_available(self, settle_s: float = 2.5) -> bool:
        """Is there a usable map -> base_link right now? Spins, because TF needs time to fill."""
        import rclpy
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self.node)
        end = time.monotonic() + settle_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self._buf.can_transform(MAP_FRAME, BASE_FRAME, Time(),
                                       timeout=Duration(seconds=0.0)):
                return True
        return False

    def resolve(self) -> tuple[bool, str]:
        """Pick a frame and wire it up. Returns (ok, one-line description or refusal).

        Idempotent, and deliberately NOT called from __init__: settling TF costs up to 2.5 s and
        `waypoints.py list` has no business paying it.
        """
        if self.frame is not None:
            return True, self.description
        want = (self.requested or "auto").lower()

        if want in ("auto", "map"):
            if self._map_available():
                self.frame = MAP_FRAME
                self._wire_map()
                sess = mola_session_id(self.node)
                mp = current_map_name(self.node)
                where = f"saved map '{mp}'" if mp else "a FRESH map frame (no saved map loaded)"
                self.description = (f"pose from TF {MAP_FRAME} -> {BASE_FRAME}, in {where}"
                                    f" [mola {(sess or '?')[:8]}]")
                return True, self.description
            if want == "map":
                return False, (
                    f"--frame map was requested but there is no {MAP_FRAME} -> {BASE_FRAME} "
                    f"transform.\n"
                    f"  MOLA publishes it only once it can resolve odom -> base_link, so the "
                    f"CHASSIS must be running too, not just the lidar.\n"
                    f"  Check: ros2 topic echo {MOLA_POSE_TOPIC} --once")
            # auto: fall through to odom, loudly
            self.frame = ODOM_TOPIC and "odom"
            self._wire_odom()
            self.description = ("pose from /odom (wheel odometry) -- no map frame available. "
                                "These coordinates die on the next ranger_base restart and "
                                "drift in between.")
            return True, self.description

        self.frame = "odom"
        self._wire_odom()
        self.description = "pose from /odom (wheel odometry), as requested"
        return True, self.description

    # ---- wiring ------------------------------------------------------------------------------
    def _wire_odom(self) -> None:
        self._sub = self.node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)

    def _on_odom(self, m) -> None:
        p = m.pose.pose
        self.node.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.node.stamp = time.monotonic()

    def _wire_map(self) -> None:
        # TF is polled, not pushed: there is no callback for "a transform changed".
        self._timer = self.node.create_timer(1.0 / 20.0, self._poll_tf)
        self._poll_tf()

    def _poll_tf(self) -> None:
        try:
            t = self._buf.lookup_transform(MAP_FRAME, BASE_FRAME, Time(),
                                           timeout=Duration(seconds=0.0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return          # stale-ness is reported by fresh(); a gap is not an error here
        tr, q = t.transform.translation, t.transform.rotation
        self.node.pose = (tr.x, tr.y, yaw_of(q))
        self.node.stamp = time.monotonic()

    # ---- readout -----------------------------------------------------------------------------
    def wait_for_pose(self, timeout: float = 5.0) -> bool:
        import rclpy
        if self.frame is None:
            ok, why = self.resolve()
            if not ok:
                self.description = why
                return False
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.node.pose is not None:
                return True
        return False

    def fresh(self) -> bool:
        return (self.node.pose is not None
                and (time.monotonic() - self.node.stamp) <= STALE_S)
