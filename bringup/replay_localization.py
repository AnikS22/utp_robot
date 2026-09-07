#!/usr/bin/env python3
"""Search recorded scans without starting ROS nodes or publishing robot commands."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from localization_search import ScanMatcher, acceptance


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--map', required=True, help='saved map YAML')
    ap.add_argument('--start-time', type=float, required=True, help='Unix seconds')
    ap.add_argument('--samples', type=int, default=3)
    ap.add_argument('--interval', type=float, default=4)
    ap.add_argument('--min-range', type=float, default=0)
    args = ap.parse_args()
    from _ros_env import require_ros
    require_ros()
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    path = Path(args.map)
    meta = yaml.safe_load(path.read_text())
    if meta.get('negate', 0) or meta['origin'][2] != 0 or meta.get('mode', 'trinary') != 'trinary':
        ap.error('only unrotated, non-negated trinary maps are supported')
    pixels = np.array(Image.open(path.parent / meta['image']).convert('L'))[::-1]
    grid = np.full(pixels.shape, -1, dtype=np.int8)
    probability = (255 - pixels.astype(float)) / 255
    grid[probability < meta['free_thresh']] = 0
    grid[probability > meta['occupied_thresh']] = 100
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
    for i in range(args.samples):
        target = args.start_time + i * args.interval
        reader.seek(int(target * 1e9))
        if not reader.has_next():
            raise RuntimeError(f'no scan at {target}')
        _, data, stamp = reader.read_next()
        scan = deserialize_message(data, LaserScan)
        if scan.header.frame_id != 'base_link' or abs(stamp / 1e9 - target) > 2:
            raise RuntimeError('scan frame or requested recording time does not match')
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        keep = np.isfinite(ranges) & (ranges > max(scan.range_min, args.min_range)) & (ranges < min(15, scan.range_max))
        ranges, angles = ranges[keep], angles[keep]
        step = max(1, len(ranges) // 120)
        ranges, angles = ranges[::step], angles[::step]
        matcher = ScanMatcher(grid, meta['resolution'], meta['origin'][:2], ranges, angles)
        hypotheses = matcher.search()
        ok, reason = acceptance(hypotheses, len(ranges))
        print(json.dumps(dict(stamp=stamp / 1e9, beams=len(ranges), accepted=ok, reason=reason,
                              hypotheses=[dict(fit=100*s/len(ranges), x=x, y=y, yaw_deg=math.degrees(yaw))
                                          for s, x, y, yaw in hypotheses[:3]])), flush=True)
    reader.close()


if __name__ == '__main__':
    main()
