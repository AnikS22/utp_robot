#!/usr/bin/env python3
"""Does map -> base_link resolve right now? Exit 0 if yes. Moves nothing, prints nothing.

WHY THIS IS NOT `timeout 20 ros2 run tf2_ros tf2_echo map base_link`. tf2_echo NEVER EXITS -- it
prints the transform forever -- so `timeout` always kills it and the caller always sees a non-zero
status. mission.sh's adopt_lock used exactly that and therefore reported "no map -> base_link
transform" every single time, on a robot whose transform was fine: --ready never once adopted a
lock, it just fell through to the 80 s reload it was written to avoid. Measured 2026-09-07 20:51.
"""
import sys, time
import rclpy, tf2_ros
from rclpy.node import Node
from rclpy.time import Time

rclpy.init(args=None)
n = Node("utp_tf_check")
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, n)
# A listener created a moment ago has not discovered the /tf publishers yet; that discovery is
# what costs the seconds here, not the transform's availability.
end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 15.0)
ok = False
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if buf.can_transform("map", "base_link", Time()):
        ok = True
        break
rclpy.shutdown()
sys.exit(0 if ok else 1)
