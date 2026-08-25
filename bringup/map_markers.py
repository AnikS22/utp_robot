#!/usr/bin/env python3
"""Persist RViz clicked points as visible, map-frame semantic landmarks."""

from pathlib import Path
import os

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import yaml


class MapMarkers(Node):
    def __init__(self, path: Path):
        super().__init__("utp_map_markers")
        self.path = path
        self.items = []
        self.next_label = None
        if path.exists():
            self.items = yaml.safe_load(path.read_text()) or []
        self.pub = self.create_publisher(MarkerArray, "/map_markers", 10)
        self.create_subscription(PointStamped, "/clicked_point", self.on_point, 10)
        self.create_subscription(String, "/map_marker/next_label", self.on_label, 10)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f"click-to-save markers -> {path}")

    def on_label(self, msg):
        self.next_label = msg.data.strip() or None

    def on_point(self, msg):
        if msg.header.frame_id != "map":
            self.get_logger().error(f"refusing marker in frame {msg.header.frame_id!r}; RViz must use map")
            return
        label = self.next_label or f"landmark_{len(self.items)+1:03d}"
        self.next_label = None
        self.items.append({"label": label, "x": float(msg.point.x), "y": float(msg.point.y),
                           "z": float(msg.point.z)})
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        tmp.write_text(yaml.safe_dump(self.items, sort_keys=False))
        os.replace(tmp, self.path)
        self.get_logger().info(f"saved {label} at ({msg.point.x:.3f}, {msg.point.y:.3f})")
        self.publish()

    def publish(self):
        out = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, item in enumerate(self.items):
            dot = Marker(); dot.header.frame_id = "map"; dot.header.stamp = now
            dot.ns = "map_landmarks"; dot.id = i*2; dot.type = Marker.SPHERE; dot.action = Marker.ADD
            dot.pose.position.x=item["x"]; dot.pose.position.y=item["y"]; dot.pose.position.z=.12
            dot.pose.orientation.w=1.0; dot.scale.x=dot.scale.y=dot.scale.z=.22
            dot.color.r=1.0; dot.color.g=.75; dot.color.b=0.0; dot.color.a=1.0
            text = Marker(); text.header=dot.header; text.ns="map_landmark_labels"; text.id=i*2+1
            text.type=Marker.TEXT_VIEW_FACING; text.action=Marker.ADD; text.pose.orientation.w=1.0
            text.pose.position.x=item["x"]; text.pose.position.y=item["y"]; text.pose.position.z=.38
            text.scale.z=.22; text.color.r=text.color.g=text.color.b=1.0; text.color.a=1.0
            text.text=item["label"]; out.markers.extend((dot,text))
        self.pub.publish(out)


def main():
    rclpy.init(); path=Path(__file__).resolve().parent.parent/"maps"/"site_markers.yaml"
    node=MapMarkers(path)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == "__main__": main()
