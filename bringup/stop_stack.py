#!/usr/bin/env python3
"""Stop this checkout's ROS processes. Save the live map before invoking this."""
from __future__ import annotations

import os
from pathlib import Path
import signal
import time

from startup_processes import matching_processes


def owned_processes(repo: Path, domain: str, proc: Path = Path('/proc')):
    # matching_processes excludes this process and every ancestor.
    for pid in matching_processes(' ', proc):
        try:
            folder = proc / str(pid)
            env = dict(part.split(b'=', 1) for part in (folder / 'environ').read_bytes().split(b'\0') if b'=' in part)
            if env.get(b'UTP_ROBOT_STACK') != str(repo).encode() or env.get(b'ROS_DOMAIN_ID') != domain.encode():
                continue
            command = (folder / 'cmdline').read_bytes()
            if command:
                yield pid, command
        except OSError:
            continue


def same_process(pid: int, command: bytes) -> bool:
    try:
        return Path(f'/proc/{pid}/cmdline').read_bytes() == command
    except OSError:
        return False


def main() -> int:
    repo = Path(__file__).resolve().parent.parent
    victims = dict(owned_processes(repo, os.environ.get('UTP_ROBOT_DOMAIN', '9')))
    # Detached/nohup recorders may inherit ignored SIGINT. SIGTERM lets ROS close bags.
    for sig, budget in ((signal.SIGINT, 5), (signal.SIGTERM, 10)):
        for pid, command in victims.items():
            if same_process(pid, command):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + budget
        while time.monotonic() < deadline and any(same_process(p, c) for p, c in victims.items()):
            time.sleep(0.1)
    remaining = [pid for pid, cmd in victims.items() if same_process(pid, cmd)]
    print(f'Stopped {len(victims) - len(remaining)} robot processes; remaining: {remaining}')
    if remaining:
        return 1
    marker = repo / 'maps' / '.loaded_map'
    if marker.exists():
        marker.unlink()
    os.sync()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
