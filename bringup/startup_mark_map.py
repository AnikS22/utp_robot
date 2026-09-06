#!/usr/bin/env python3
"""Associate the verified localization session with its saved map for waypoint checks."""
from __future__ import annotations

import os
from pathlib import Path
import sys
import time


def record_session(repo: Path, name: str, mode: str, filename: str, session: str):
    stem = repo / 'maps' / name
    if mode != 'localization' or Path(filename).resolve() != stem.resolve() or not session:
        raise ValueError('live SLAM mode, map filename or session is unconfirmed')
    if not all(Path(str(stem) + '.' + ext).is_file() and Path(str(stem) + '.' + ext).stat().st_size
               for ext in ('pgm', 'yaml', 'posegraph', 'data')):
        raise ValueError('saved map files are missing or empty')
    marker = repo / 'maps' / '.loaded_map'
    temp = marker.with_name(f'.loaded_map.{os.getpid()}.tmp')
    try:
        with temp.open('w') as out:
            out.write(f'{name} {session}\n')
            out.flush()
            os.fsync(out.fileno())
        temp.replace(marker)
    finally:
        temp.unlink(missing_ok=True)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter_client import AsyncParameterClient
    from pose_source import slam_session_id

    repo = Path(__file__).resolve().parent.parent
    rclpy.init()
    node = Node('utp_startup_map_provenance')
    try:
        client = AsyncParameterClient(node, '/slam_toolbox')
        if not client.wait_for_services(timeout_sec=3):
            raise ValueError('SLAM parameter services unavailable')
        future = client.get_parameters(['mode', 'map_file_name'])
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done() or future.result() is None:
            raise ValueError('SLAM parameters unavailable')
        mode, filename = [p.string_value for p in future.result().values]
        session = None
        deadline = time.monotonic() + 3
        while session is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            session = slam_session_id(node)
        record_session(repo, sys.argv[1], mode, filename, session)
        print(f'localized map provenance recorded: {sys.argv[1]}')
        return 0
    except (ValueError, OSError) as exc:
        print(f'Cannot mark loaded map: {exc}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
