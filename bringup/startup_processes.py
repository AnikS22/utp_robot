#!/usr/bin/env python3
"""Find process command lines without spawning a shell utility for every PID."""
from __future__ import annotations

import os
from pathlib import Path
import sys


def matching_processes(pattern: str, proc: Path = Path('/proc'), pid: int | None = None):
    if not pattern:
        raise ValueError('empty process pattern refused')
    current = os.getpid() if pid is None else pid
    excluded = set()
    while current > 1 and current not in excluded:
        excluded.add(current)
        try:
            current = int((proc / str(current) / 'stat').read_text().rsplit(') ', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) in excluded:
            continue
        try:
            command = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if command and pattern in command:
            yield int(entry.name)


if __name__ == '__main__':
    try:
        for match in matching_processes(sys.argv[1]):
            print(match)
    except (ValueError, IndexError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)
