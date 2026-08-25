#!/usr/bin/env python3
"""Remove the robot-occluded rear sector from the front-mounted lidar.

Raw /scan is retained for diagnostics. /scan_filtered is consumed by SLAM and Nav2. Values outside
the allowed forward/side arc become NaN (no observation), not infinity (observed free space), so
the chassis cannot clear real obstacles behind it or paint itself as a moving wall.
"""
from __future__ import annotations
import sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "safety"))
from scan_filter import KEEP_HALF_ANGLE_DEG, filtered_ranges


class Filter(Node):
    def __init__(self):
        super().__init__("utp_rear_scan_filter")
        self.pub=self.create_publisher(LaserScan,"/scan_filtered",qos_profile_sensor_data)
        self.create_subscription(LaserScan,"/scan",self.cb,qos_profile_sensor_data)
        self.get_logger().info(
            f"/scan -> /scan_filtered; keeping +-{KEEP_HALF_ANGLE_DEG:.0f} deg "
            f"(rear sector is the arm riser, battery and mast -- measured 0.16-0.19 m)")
    def cb(self,m):
        o=LaserScan();o.header=m.header;o.angle_min=m.angle_min;o.angle_max=m.angle_max
        o.angle_increment=m.angle_increment;o.time_increment=m.time_increment;o.scan_time=m.scan_time
        o.range_min=m.range_min;o.range_max=m.range_max
        o.ranges=filtered_ranges(m.ranges,m.angle_min,m.angle_increment);o.intensities=m.intensities
        self.pub.publish(o)

def main():
    rclpy.init();n=Filter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
