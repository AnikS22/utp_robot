#!/usr/bin/env python3
"""Refuse to start if the hardware stack would collide with anything else on this machine.

    python3 bringup/preflight.py            # check, exit 0 if safe
    python3 bringup/preflight.py --verbose  # also list what is on our domain

Checked, in order:
  1. Our ROS_DOMAIN_ID carries nothing foreign. The workstation runs the sim campaign at the same
     time, and two ROS graphs on one host are isolated ONLY by domain id.
  2. No stale process of OURS is still holding the lidar's serial port. An orphaned rplidar_node
     keeps the port open and the next start fails with SL_RESULT_OPERATION_TIMEOUT, which looks
     exactly like a hardware fault and is not one.

Exits non-zero and explains rather than starting something that would interfere. Written after a
cleanup from this repo killed 22 of the running sim campaign's TF publishers on 2026-08-18: the
lesson is that "probably fine" is not good enough when someone else's multi-hour run is at stake.
"""
from __future__ import annotations

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read(pid: str, what: str) -> str:
    try:
        with open(f"/proc/{pid}/{what}", "rb") as f:
            return f.read().decode(errors="replace")
    except OSError:
        return ""


def _procs():
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        cmd = _read(pid, "cmdline").replace("\0", " ").strip()
        if not cmd:
            continue
        dom = None
        for e in _read(pid, "environ").split("\0"):
            if e.startswith("ROS_DOMAIN_ID="):
                dom = e.split("=", 1)[1] or "0"
        yield pid, cmd, dom


def session_of(pid: str) -> int:
    """Session id from /proc/<pid>/stat field 6. Used to recognise our OWN shell tree."""
    try:
        stat = _read(pid, "stat")
        # comm may contain spaces/parens; fields after the last ')' are stable.
        tail = stat[stat.rfind(")") + 1:].split()
        return int(tail[3])
    except (ValueError, IndexError):
        return -1


def ours(cmd: str) -> bool:
    """Ours iff the executable actually lives in this repo. Deliberately NOT a frame-name or
    topic-name match -- matching on `--child-frame-id lidar_link` is what killed the sim's
    publishers, because the sim uses the same child frame."""
    return REPO in cmd


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--port", default=os.environ.get("RPLIDAR_PORT", ""))
    args = ap.parse_args()

    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain is None:
        print("PREFLIGHT FAIL: ROS_DOMAIN_ID is unset. Run `source bringup/env.sh` first —"
              " an unset domain defaults to 0 and is shared with anything else that forgot.")
        return 1

    # Our own shell tree (the script that invoked us, its `sleep`, etc.) inherits ROS_DOMAIN_ID
    # and would otherwise be reported as a foreign collision -- which made this guard refuse to
    # let lidar.sh start at all. Same for the ros2 CLI daemon, which is a per-domain helper, not a
    # competing stack.
    my_session = session_of(str(os.getpid()))

    mine, foreign = [], []
    for pid, cmd, dom in _procs():
        if dom != domain:
            continue
        if "preflight.py" in cmd or "/proc" in cmd:
            continue
        if session_of(pid) == my_session:
            continue                      # our own shell tree
        if "ros2cli.daemon" in cmd or "ros2-daemon" in cmd:
            continue                      # per-domain CLI helper, harmless
        (mine if ours(cmd) else foreign).append((pid, cmd))

    print(f"domain           : {domain} (reserved for hardware)")
    if args.verbose or mine:
        for pid, cmd in mine:
            print(f"  ours     pid {pid}  {cmd[:88]}")

    if foreign:
        print(f"\nPREFLIGHT FAIL: {len(foreign)} process(es) on domain {domain} are NOT ours:")
        for pid, cmd in foreign[:10]:
            print(f"  pid {pid}  {cmd[:100]}")
        print("\nStarting here could interfere with them. Pick another domain:")
        print("  UTP_ROBOT_DOMAIN=<n> source bringup/env.sh")
        return 1

    # stale holders of the serial port
    port = args.port
    if port and os.path.exists(port):
        real = os.path.realpath(port)
        holders = []
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            fddir = f"/proc/{pid}/fd"
            try:
                for fd in os.listdir(fddir):
                    if os.path.realpath(os.path.join(fddir, fd)) == real:
                        holders.append((pid, _read(pid, "cmdline").replace("\0", " ").strip()))
                        break
            except OSError:
                continue
        if holders:
            print(f"\nPREFLIGHT FAIL: {real} is already open:")
            for pid, cmd in holders:
                tag = "OURS (stale)" if ours(cmd) else "foreign"
                print(f"  pid {pid}  [{tag}]  {cmd[:90]}")
            print("\nA stale rplidar_node holding the port makes the next start fail with")
            print("SL_RESULT_OPERATION_TIMEOUT, which looks like a hardware fault. Kill it by PID.")
            return 1
        print(f"serial port      : {real} free")
    elif port:
        print(f"serial port      : {port} NOT PRESENT — is the lidar plugged in?")
        return 1

    print("preflight        : OK, no collisions")
    return 0


if __name__ == "__main__":
    sys.exit(main())
