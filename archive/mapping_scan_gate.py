#!/usr/bin/env python3
"""Forward filtered lidar scans only during SLAM-safe Ranger motion.

The Ranger Mini can spin in place and crab sideways. During the 2026-08-24
mapping run, spin mode made wheel odometry and lidar scan matching disagree and
the map jumped by metres. Mapping therefore uses /scan_mapping, which is silent
unless fresh chassis state reports DualAckermann mode. Raw and filtered scans
remain available for diagnostics and RViz.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ranger_msgs.msg import SystemState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mapping_gate_policy import is_moving, must_latch, may_map


class MappingScanGate(Node):
    def __init__(self) -> None:
        super().__init__("utp_mapping_scan_gate")
        self.mode = None
        self.state_time = 0.0
        self.last_allowed = None
        self.spin_recovery = False
        self.stationary_since = None
        # slam_toolbox's message-filter subscriber requests Reliable in this installation. A
        # BestEffort publisher is incompatible and silently yields a map with no scans. Reliable
        # output still accepts BestEffort diagnostic subscribers, so it is the compatible choice.
        mapping_qos = QoSProfile(depth=10)
        mapping_qos.reliability = ReliabilityPolicy.RELIABLE
        mapping_qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(LaserScan, "/scan_mapping", mapping_qos)
        self.create_subscription(SystemState, "/system_state", self.on_state, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan_filtered", self.on_scan, qos_profile_sensor_data)
        self.get_logger().warning(
            "SLAM scan gate armed: Ackermann + slow crab allowed; spin blocked until 1 s stopped"
        )

    def on_state(self, msg: SystemState) -> None:
        self.mode = int(msg.motion_mode)
        self.state_time = time.monotonic()
        if self.mode == 2:
            self.spin_recovery = True
            self.stationary_since = None

    def on_odom(self, msg: Odometry) -> None:
        t = msg.twist.twist
        now = time.monotonic()
        if must_latch(self.mode, t.linear.x, t.linear.y, t.angular.z):
            self.spin_recovery = True
            self.stationary_since = None
        elif self.spin_recovery and self.mode in (0, 1):
            if is_moving(t.linear.x, t.linear.y, t.angular.z):
                self.stationary_since = None
            elif self.stationary_since is None:
                self.stationary_since = now
            elif now - self.stationary_since >= 1.0:
                self.spin_recovery = False
                self.stationary_since = None
                self.get_logger().info("stationary recovery complete: SLAM may resume")

    def on_scan(self, msg: LaserScan) -> None:
        state_age = time.monotonic() - self.state_time
        allowed = may_map(self.mode, state_age, self.spin_recovery)
        if allowed != self.last_allowed:
            if allowed:
                self.get_logger().info(f"mode={self.mode}: SLAM scans ENABLED")
            else:
                reason = ("spin transition; stop 1 second in Ackermann/crab to resume"
                          if self.spin_recovery else
                          "chassis state is stale/missing" if state_age > 0.5 else
                          f"mode={self.mode}; use DualAckermann or slow parallel/crab")
                self.get_logger().warning(f"SLAM scans BLOCKED: {reason}")
            self.last_allowed = allowed
        if allowed:
            self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MappingScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
