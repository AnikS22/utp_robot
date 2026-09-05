#!/usr/bin/env python3
"""Draw every recorded waypoint on the RViz map, with its HEADING.

    python3 bringup/waypoint_markers.py        # publishes /waypoint_markers, latched

WHY HEADING AND NOT JUST POSITION. On 2026-09-01 the operator clicked three POSITIONS on the map
and I derived the orientations geometrically -- each waypoint pointed at the next. That put `door`
facing along the line door->outside, i.e. straight THROUGH the doorway. The robot drove there,
looked, and the camera returned "an open walkway with pillars": it was aimed down the corridor
beyond the glass, not at the doors. The VLM was right about the picture; the pose was wrong.
Nothing displayed the heading, so the error was invisible for hours.

A position marker would have looked perfectly correct. The arrow is the point.

Re-reads the store every second, so editing maps/waypoints.yaml (or recording a new waypoint)
updates RViz without restarting anything.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

from _ros_env import require_ros  # noqa: E402

require_ros()

import rclpy                                                     # noqa: E402
import yaml                                                      # noqa: E402
from rclpy.node import Node                                      # noqa: E402
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,    # noqa: E402
                       QoSProfile, QoSReliabilityPolicy)
from visualization_msgs.msg import Marker, MarkerArray           # noqa: E402

from pose_source import current_map_name                         # noqa: E402

TOPIC = "/waypoint_markers"
MAP_FRAME = "map"

# Distinct colours so a mis-clicked waypoint is obvious at a glance rather than needing the label.
COLOURS = {
    "start":   (0.20, 0.80, 1.00),   # cyan
    "door":    (1.00, 0.55, 0.00),   # orange
    "button":  (1.00, 0.10, 0.10),   # red    -- the one that must be right for the arm
    "outside": (0.20, 1.00, 0.30),   # green
}
DEFAULT_COLOUR = (0.85, 0.85, 0.85)


class WaypointMarkers(Node):
    def __init__(self, store: Path):
        super().__init__("utp_waypoint_markers")
        self.store = store
        self._last = None
        # TRANSIENT_LOCAL: RViz is usually started AFTER this node, and a volatile publisher would
        # leave the display empty until the next tick with nothing to say why.
        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, TOPIC, qos)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"drawing {store} on {TOPIC} (frame {MAP_FRAME})")

    def _tick(self) -> None:
        try:
            raw = self.store.read_text()
        except OSError:
            return
        # THE LIVE MAP IS PART OF THE CACHE KEY, not just the file. A floor swap changes which
        # waypoints are drawable without touching a byte of the store, and caching on the text
        # alone would leave the previous floor's markers on screen until someone edited the file.
        live = current_map_name(self)
        key = (raw, live)
        if key == self._last:
            return
        self._last = key
        try:
            data = yaml.safe_load(raw) or {}
        except yaml.YAMLError as e:
            self.get_logger().error(f"{self.store} is not valid YAML: {e}")
            return
        self.pub.publish(self._build(data, live))
        drawn = sorted(k for k, v in data.items()
                       if v.get("frame") == "map" and v.get("map_name") == live)
        self.get_logger().info(
            f"map '{live}': drawing {len(drawn)} of {len(data)} waypoint(s): "
            f"{', '.join(drawn) or 'none'}")

    def _build(self, data: dict, live: str | None) -> MarkerArray:
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL          # so a removed waypoint disappears
        out.markers.append(clear)
        now = self.get_clock().now().to_msg()
        for i, (name, wp) in enumerate(sorted(data.items())):
            # ONLY MAP-FRAME WAYPOINTS ARE DRAWABLE HERE. An odom waypoint's numbers mean something
            # in a frame whose origin moves every time the driver restarts; painting it on the map
            # would assert a physical location it does not have.
            if wp.get("frame") != "map":
                self.get_logger().warn(
                    f"'{name}' is frame={wp.get('frame')!r}, not map — not drawn")
                continue
            # AND IT MUST BE *THIS* MAP. Exactly the argument above, one level up: two maps have
            # unrelated origins, so a waypoint from another map painted on this one asserts a
            # physical location it does not have -- and it looks completely convincing, because
            # the numbers are real and the arrow is drawn correctly. Added 2026-09-04, when the
            # operator opened RViz on a freshly-built second-floor map and found floor 1's five
            # elevator waypoints drawn across it. Nothing was wrong with the map or the SLAM; the
            # DISPLAY was ignoring the provenance the store had recorded correctly all along.
            wp_map = wp.get("map_name")
            if live is None:
                self.get_logger().warn(
                    f"'{name}' not drawn: no NAMED map is loaded, so the map frame's origin is "
                    f"wherever the robot booted and no stored coordinate means anything in it")
                continue
            if wp_map != live:
                self.get_logger().warn(
                    f"'{name}' belongs to map {wp_map!r}, loaded map is {live!r} — not drawn")
                continue
            try:
                x, y = float(wp["x"]), float(wp["y"])
                yaw = float(wp.get("yaw", 0.0))
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn(f"'{name}' has no usable x/y/yaw — not drawn")
                continue
            r, g, b = COLOURS.get(name, DEFAULT_COLOUR)

            # THE POINT ITSELF, drawn first and small. You clicked a POSITION; that position must
            # be unambiguous on screen. An arrow alone is not: its tail is at the coordinate but
            # the head draws the eye 0.7 m away, so the marker reads as being somewhere it is not.
            # The sphere is the answer to "where is it"; the arrow is the answer to "which way
            # will it face when it gets there". Both questions matter and they are not the same.
            dot = Marker()
            dot.header.frame_id = MAP_FRAME
            dot.header.stamp = now
            dot.ns = "waypoint_point"
            dot.id = i * 3
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = x
            dot.pose.position.y = y
            dot.pose.position.z = 0.05
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.22
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = r, g, b, 1.0
            out.markers.append(dot)

            arrow = Marker()
            arrow.header.frame_id = MAP_FRAME
            arrow.header.stamp = now
            arrow.ns = "waypoint"
            arrow.id = i * 3 + 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.05
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x = 0.60          # shaft length -- the heading, at map scale
            arrow.scale.y = 0.06          # thin: the SPHERE is the position, this is only a bearing
            arrow.scale.z = 0.06
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = r, g, b, 0.55
            out.markers.append(arrow)

            label = Marker()
            label.header = arrow.header
            label.ns = "waypoint_label"
            label.id = i * 3 + 2
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.55
            label.pose.orientation.w = 1.0
            label.scale.z = 0.34
            label.color.r, label.color.g, label.color.b, label.color.a = r, g, b, 1.0
            # The numbers are on the label deliberately: a waypoint that looks right on the map and
            # reads wrong in metres is the disagreement worth catching early.
            label.text = f"{name}  ({x:+.2f}, {y:+.2f})  {math.degrees(yaw):+.0f}°"
            out.markers.append(label)
        return out


def main() -> int:
    store = Path(os.environ.get("UTP_WAYPOINTS") or (REPO / "maps" / "waypoints.yaml"))
    if not store.exists():
        print(f"no waypoint store at {store}", file=sys.stderr)
        return 1
    rclpy.init()
    n = WaypointMarkers(store)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
