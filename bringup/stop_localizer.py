#!/usr/bin/env python3
"""Stop only this robot's localization executable, preserving sensors and mapping sessions."""
import os
from pathlib import Path
import signal
import time
from stop_stack import owned_processes, same_process


def is_localizer(command):
    args = command.rstrip(b'\0').split(b'\0')
    if not args:
        return False
    name = b'localization_slam_toolbox_node'
    # Match executable or the exact ros2-run argument vector, never shell source text.
    if args[0].rsplit(b'/', 1)[-1] == name:
        return True
    for i in (0, 1):  # ros2 executable, optionally preceded by its Python interpreter
        if len(args) >= i + 4:
            if (args[i].rsplit(b'/', 1)[-1] == b'ros2' and
                    args[i+1:i+4] == [b'run', b'slam_toolbox', name]):
                return True
    return False


def main():
    repo = Path(__file__).resolve().parents[1]
    victims = {p: c for p, c in owned_processes(repo, os.environ.get('UTP_ROBOT_DOMAIN', '9'))
               if is_localizer(c)}
    if victims:
        (repo / 'maps/.loaded_map').unlink(missing_ok=True)
    for sig, budget in ((signal.SIGINT, 5), (signal.SIGTERM, 5)):
        for pid, command in victims.items():
            if same_process(pid, command):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
        end = time.monotonic() + budget
        while time.monotonic() < end and any(same_process(p, c) for p, c in victims.items()):
            time.sleep(.1)
    remaining = [p for p, c in victims.items() if same_process(p, c)]
    print(f'Localization processes stopped: {len(victims)-len(remaining)}; remaining: {remaining}')
    return 1 if remaining else 0


if __name__ == '__main__':
    raise SystemExit(main())
