#!/usr/bin/env python3
"""Publish maps/waypoints.yaml as RViz markers: an arrow for the heading, a label for the name.

    python3 bringup/waypoint_markers.py                # only the loaded map's waypoints
    python3 bringup/waypoint_markers.py --all          # every waypoint, whatever map it names

WHY THIS EXISTS. A waypoint is three numbers in a YAML file, and until now the only way to see
where one actually was involved driving to it. That is the wrong order: a pose recorded at a bad
localization fit, or with a heading 180 degrees out, looks identical on disk to a good one, and
the first symptom is an arm swinging at a wall. An arrow on the map is the cheapest possible check
and it costs nothing to leave running.

ONLY THE LOADED MAP BY DEFAULT. maps/waypoints.yaml is one flat store shared by every floor, and
two maps' origins are unrelated -- drawing floor 2's poses on floor 1's grid would put confident
arrows in physically meaningless places, which is worse than drawing nothing. --all overrides,
and colours the foreign ones differently so they cannot be mistaken for drivable goals.

TRANSIENT_LOCAL, because RViz is usually started after this and a volatile publisher would leave
an empty display until the next tick.
"""
from __future__ import annotations

import argparse
import math
import pathlib
import time
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

REPO = pathlib.Path(__file__).resolve().parent.parent
WAYPOINTS = REPO / "maps" / "waypoints.yaml"
LOADED_MAP = REPO / "maps" / ".loaded_map"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--all", action="store_true",
                    help="draw waypoints from every map, not just the loaded one")
    ap.add_argument("--topic", default="/waypoint_markers")
    ap.add_argument("--frame", default="map")
    a = ap.parse_args()

    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy,
                           QoSDurabilityPolicy)
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point

    store = yaml.safe_load(WAYPOINTS.read_text()) or {}

    rclpy.init()
    node = Node("utp_waypoint_markers")

    # pose_source.current_map_name, NOT a private read of .loaded_map. It is the same function
    # waypoints.py and nav2_goto.py trust, and it does the part a plain file read misses: it also
    # compares the SLAM session the map was loaded into against the one running now, and returns
    # None when they differ. A restarted SLAM has a new map-frame origin, so a name alone would
    # keep asserting 'floor1' over a frame whose origin has moved -- and these arrows would be
    # drawn in the wrong physical place while looking entirely correct.
    from pose_source import current_map_name, slam_session_id
    # SPIN BEFORE ASKING. current_map_name identifies the running SLAM by the DDS GID of the /map
    # publisher, and a node created a moment ago has not finished discovery -- get_publishers_info
    # returns nothing, the session reads as "not the one the map was loaded into", and this
    # reports a restarted SLAM on a stack that has been up for an hour. Measured here on the first
    # run of this file. Poll for the publisher with a real budget; success exits immediately, so
    # the budget is only ever spent on the failing path.
    _end = time.monotonic() + 12.0
    while rclpy.ok() and time.monotonic() < _end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if slam_session_id(node) is not None:
            break
    live = current_map_name(node)
    if not a.all and live is None:
        print("no certified loaded map (missing maps/.loaded_map, or SLAM restarted since it was "
              "written). Use --all to draw everything anyway.", file=sys.stderr)
        rclpy.shutdown()
        return 2
    qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(MarkerArray, a.topic, qos)

    def build(store, live):
        """Build the whole MarkerArray for the map that is loaded RIGHT NOW."""
        arr = MarkerArray()
        mid = 0
        drawn, skipped = [], []
        for name in sorted(store):
            wp = store[name]
            if wp.get("frame") != "map":
                skipped.append(f"{name} (frame={wp.get('frame')})")
                continue
            # THE PROVENANCE FILTER. 2026-09-04: RViz opened on a fresh floor-2 map showed floor 1's
            # five elevator waypoints painted across it. Nothing was broken -- the store is flat and
            # shared, and this drew all of it. Two maps' origins are unrelated, so those arrows named
            # physical places they do not occupy, with real numbers and correct-looking headings.
            # This module already refused odom-frame waypoints on that exact argument; this is the
            # same argument one level up.
            wp_map = wp.get("map_name")
            mine = not (wp_map != live)
            if wp_map != live and not a.all:
                skipped.append(f"{name} (map={wp_map})")
                continue
            x, y = float(wp["x"]), float(wp["y"])
            yaw = float(wp.get("yaw", 0.0))

            # ARROW from the pose, 0.6 m long, so the HEADING is visible and not just the position.
            # Green = a goal you can drive to right now; grey = recorded in another map, shown only
            # under --all and deliberately drab so it never reads as drivable.
            m = Marker()
            m.header.frame_id = a.frame
            m.header.stamp = node.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = "waypoint_arrow", mid, Marker.ARROW, Marker.ADD
            m.points = [Point(x=x, y=y, z=0.05),
                        Point(x=x + 0.6 * math.cos(yaw), y=y + 0.6 * math.sin(yaw), z=0.05)]
            m.scale.x, m.scale.y, m.scale.z = 0.08, 0.16, 0.0
            m.color.a = 1.0
            if mine:
                m.color.r, m.color.g, m.color.b = 0.1, 0.9, 0.2
            else:
                m.color.r, m.color.g, m.color.b = 0.5, 0.5, 0.5
            arr.markers.append(m); mid += 1

            t = Marker()
            t.header.frame_id = a.frame
            t.header.stamp = m.header.stamp
            t.ns, t.id, t.type, t.action = "waypoint_label", mid, Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y, 0.45
            t.pose.orientation.w = 1.0
            t.scale.z = 0.28
            t.color.a = 1.0
            t.color.r, t.color.g, t.color.b = (1.0, 1.0, 1.0) if mine else (0.6, 0.6, 0.6)
            t.text = name if mine else f"{name} [{wp_map}]"
            arr.markers.append(t); mid += 1
            drawn.append(f"{name} ({x:+.2f},{y:+.2f}) {math.degrees(yaw):+.1f} deg")
        return arr, drawn, skipped

    arr, drawn, skipped = build(store, live)
    print(f"  map loaded: {live or '<none>'}")
    for d in drawn:
        print(f"    drawn   {d}")
    for s in skipped:
        print(f"    skipped {s}")
    if not drawn:
        print("  NOTHING TO DRAW. Every waypoint on file belongs to another map.", file=sys.stderr)

    # Republish on a timer: TRANSIENT_LOCAL covers late subscribers, but re-sending is what makes
    # this survive an RViz restart in every case, and the payload is a few hundred bytes.
    pub.publish(arr)

    # REBUILD ON EVERY TICK, DO NOT REPUBLISH A SNAPSHOT.
    #
    # This built the array once and republished that same object forever. It was started while
    # floor1 was loaded; after a swap to floor2 it went on drawing floor1's two waypoints across
    # floor2's map -- the exact failure the provenance filter above exists to prevent, arriving
    # through the back door because the filter only ran once. Observed 2026-09-06 by the operator:
    # "the floor 1 waypoints are on the floor 2 thing".
    #
    # A map swap changes .loaded_map and restarts SLAM, and recording a waypoint rewrites
    # waypoints.yaml. Both happen while this is running, so both have to be re-read.
    state = {"markers": arr, "key": None}

    def rebuild():
        try:
            live_now = current_map_name(node)
            store_now = yaml.safe_load(WAYPOINTS.read_text()) or {}
        except Exception:
            pub.publish(state["markers"])
            return
        key = (live_now, tuple(sorted(
            (k, v.get("map_name"), v.get("x"), v.get("y"), v.get("yaw"))
            for k, v in store_now.items())))
        if key != state["key"]:
            state["key"] = key
            state["markers"], _d, _s = build(store_now, live_now)
            drawn_now = sum(1 for m in state["markers"].markers if m.ns == "waypoint_arrow")
            node.get_logger().info(
                f"map '{live_now}': drawing {drawn_now} waypoint(s), "
                f"skipping {len(store_now) - drawn_now}")
        pub.publish(state["markers"])

    node.create_timer(2.0, rebuild)
    print(f"  publishing {len(arr.markers)} markers on {a.topic} (Ctrl-C to stop)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
