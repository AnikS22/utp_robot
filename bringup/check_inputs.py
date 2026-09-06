#!/usr/bin/env python3
"""Read-only hardware input audit. No map, localization, goals or motion commands."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import time


def can_health(link, rx_delta):
    data = link.get('linkinfo', {}).get('info_data', {})
    state = data.get('state', 'UNKNOWN')
    good = link.get('operstate') == 'UP' and state == 'ERROR-ACTIVE' and rx_delta > 50
    return {'ok': good, 'state': state, 'received_frames': rx_delta,
            'bitrate': data.get('bittiming', {}).get('bitrate')}


def image_health(message):
    shape_ok = message.width > 0 and message.height > 0 and message.step > 0
    return shape_ok and len(message.data) == message.step * message.height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    from _ros_env import require_ros
    require_ros()
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2, Imu
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    from geometry_msgs.msg import Twist

    rel = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    specs = {
        '/odom': (Odometry, 5, qos_profile_sensor_data),
        '/ouster/points': (PointCloud2, 1.5, qos_profile_sensor_data),
        '/ouster/imu': (Imu, 10, qos_profile_sensor_data),
        '/scan': (LaserScan, 1.5, rel),
        '/scan_nav': (LaserScan, 1.5, rel),
        '/mast_cam/color/image_raw': (Image, 10, qos_profile_sensor_data),
        '/mast_cam/aligned_depth_to_color/image_raw': (Image, 10, qos_profile_sensor_data),
        '/mast_cam/color/camera_info': (CameraInfo, 10, qos_profile_sensor_data),
        '/safety/status': (String, 5, qos_profile_sensor_data),
        '/safety/arm_stowed': (Bool, 5, qos_profile_sensor_data),
        '/cmd_vel': (Twist, 5, rel),
    }
    rclpy.init()
    node = Node('utp_inputs_audit')
    buf = Buffer()
    listener = TransformListener(buf, node)
    last, counts, stowed, estops, nonzero = {}, {}, [], [], []
    arrival_age = {}
    def callback(topic):
        def receive(message):
            counts[topic] = counts.get(topic, 0) + 1
            last[topic] = message
            if hasattr(message, 'header'):
                arrival_age[topic] = (node.get_clock().now().nanoseconds / 1e9
                                      - message.header.stamp.sec - message.header.stamp.nanosec / 1e9)
            if topic == '/safety/arm_stowed':
                stowed.append(bool(message.data))
            elif topic == '/safety/status':
                try:
                    status = json.loads(message.data)
                    estops.append(status.get('gates', {}).get('estop_latched'))
                except ValueError:
                    estops.append(None)
            elif topic == '/cmd_vel':
                nonzero.append(any(abs(v) > 1e-6 for v in
                                   (message.linear.x, message.linear.y, message.linear.z,
                                    message.angular.x, message.angular.y, message.angular.z)))
        return receive
    try:
        heavy = {topic for topic, (kind, _, _) in specs.items() if kind in (Image, PointCloud2)}
        for topic, (kind, _, qos) in specs.items():
            if topic not in heavy:
                node.create_subscription(kind, topic, callback(topic), qos)
        start = time.monotonic()
        while time.monotonic() - start < 3:
            rclpy.spin_once(node, timeout_sec=.02)
        counts.clear(); stowed.clear(); estops.clear(); nonzero.clear()
        rx_path = Path('/sys/class/net/can0/statistics/rx_packets')
        rx_before = int(rx_path.read_text()) if rx_path.exists() else 0
        start = time.monotonic()
        while time.monotonic() - start < 4:
            rclpy.spin_once(node, timeout_sec=.02)
        elapsed = time.monotonic() - start
        measured_counts = dict(counts)
        # Sample large frames one topic at a time; do not turn the health audit into
        # a simultaneous RGB+depth+cloud bandwidth stress test.
        for topic in sorted(heavy):
            kind, _, qos = specs[topic]
            subscription = node.create_subscription(kind, topic, callback(topic), qos)
            deadline = time.monotonic() + 3
            while topic not in last and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            node.destroy_subscription(subscription)
        checks = {}
        now = node.get_clock().now().nanoseconds / 1e9
        for topic, (kind, minimum, _) in specs.items():
            message = last.get(topic)
            rate = measured_counts.get(topic, 0) / elapsed
            row = {'ok': rate >= minimum, 'hz': round(rate, 2),
                   'publishers': node.count_publishers(topic)}
            if topic in heavy:
                row.pop('hz')
                row['check_type'] = 'fresh frame sample, not throughput measurement'
                row['ok'] = message is not None
            row['ok'] &= row['publishers'] == 1
            if message is not None and hasattr(message, 'header'):
                age = now - message.header.stamp.sec - message.header.stamp.nanosec / 1e9
                if topic in heavy:
                    age = arrival_age[topic]
                    row['age_at_receipt_s'] = round(age, 3)
                else:
                    row['age_s'] = round(age, 3)
                row['frame'] = message.header.frame_id
                row['ok'] &= -.5 <= age <= 2
            if isinstance(message, Image):
                row.update(width=message.width, height=message.height, encoding=message.encoding)
                row['ok'] &= image_health(message)
                if 'depth' in topic and message.encoding in ('16UC1', '32FC1'):
                    dtype = ('>' if message.is_bigendian else '<') + ('u2' if message.encoding == '16UC1' else 'f4')
                    values = np.frombuffer(message.data, dtype=dtype)
                    row['valid_depth_fraction'] = round(float(np.mean(np.isfinite(values) & (values > 0))), 3)
                    row['ok'] &= row['valid_depth_fraction'] > .05
            if isinstance(message, PointCloud2):
                row['points'] = message.width * message.height
                row['ok'] &= row['points'] > 0 and len(message.data) == message.row_step * message.height
            if isinstance(message, LaserScan):
                valid = sum(math.isfinite(v) and message.range_min < v < message.range_max for v in message.ranges)
                row['valid_beams'] = valid
                row['ok'] &= valid > 100
            if isinstance(message, CameraInfo):
                row['ok'] &= message.k[0] > 0 and message.k[4] > 0
            row['ok'] = bool(row['ok'])
            checks[topic] = row
        try:
            link = json.loads(subprocess.check_output(['ip', '-j', '-details', 'link', 'show', 'can0'], text=True))[0]
            checks['CAN feedback'] = can_health(link, int(rx_path.read_text()) - rx_before)
        except (OSError, subprocess.SubprocessError, ValueError, IndexError) as exc:
            checks['CAN feedback'] = {'ok': False, 'error': str(exc)}
        checks['arm stowed'] = {'ok': bool(stowed) and all(stowed), 'samples': len(stowed)}
        checks['estop clear'] = {'ok': bool(estops) and all(v is False for v in estops)}
        checks['zero motion commands'] = {'ok': bool(nonzero) and not any(nonzero)}
        edges = [('odom', 'base_link'), ('base_link', 'os_lidar')]
        for topic in ('/mast_cam/color/image_raw', '/mast_cam/aligned_depth_to_color/image_raw'):
            if topic in last:
                edges.append(('base_link', last[topic].header.frame_id))
        for parent, child in edges:
            checks[f'TF {parent}->{child}'] = {'ok': bool(buf.can_transform(parent, child, Time()))}
        forbidden = sorted(set(node.get_node_names()) & {'slam_toolbox', 'amcl', 'bt_navigator', 'controller_server'})
        checks['no localization/navigation'] = {'ok': not forbidden and node.count_publishers('/map') == 0,
                                                'nodes': forbidden}
        warnings = [f'{topic}: {checks[topic]["hz"]} Hz, below the 6 Hz scan target'
                    for topic in ('/scan', '/scan_nav') if checks[topic]['hz'] < 6]
        result = {'warnings': warnings, 'timestamp_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  'hardware_ok': all(bool(row['ok']) for row in checks.values()), 'checks': checks}
        payload = json.dumps(result, indent=2)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(payload + '\n')
        print(payload)
        return 0 if result['hardware_ok'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
