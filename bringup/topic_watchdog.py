#!/usr/bin/env python3
"""Keep a driver alive by watching its TOPIC, not its process.

    python3 bringup/topic_watchdog.py --topic /scan --type sensor_msgs/msg/LaserScan \
        --min-hz 3 -- bash bringup/lidar.sh

    python3 bringup/topic_watchdog.py --topic /mast_cam/color/camera_info \
        --type sensor_msgs/msg/CameraInfo --min-hz 20 -- bash bringup/camera.sh

Runs the command, watches the topic, and restarts the command if the topic goes quiet.

WHY WATCH THE TOPIC AND NOT THE PROCESS
On 2026-08-21 the lidar's USB device re-enumerated three times and the camera's twice. Every time,
the driver process stayed alive holding a stale file descriptor, the topic stayed advertised, and
no data flowed. No error was logged by anything. `pgrep` said all was well; a supervisor watching
the process would have seen a healthy service and done nothing.

The consequence during mapping is worse than a crash: SLAM keeps running, keeps publishing
`map -> odom`, and quietly stacks every subsequent scan at one pose. You finish walking the
building and discover the map stopped when the cable was nudged. A crash would at least have been
visible.

So the liveness signal is DATA, which is the only thing that actually matters to consumers.

DELIBERATE CHOICES
  * A restart is a full stop-and-start of the child's PROCESS GROUP, because `ros2 run`/`ros2
    launch` exec the real node as a child -- killing the pid we hold leaves the node alive still
    holding the serial port, and then the restart fails with SL_RESULT_OPERATION_TIMEOUT, which
    looks like a hardware fault and is not.
  * GRACE_S after each start before judging: drivers take seconds to open a device and begin
    streaming, and restarting one mid-startup loops forever.
  * MAX_RESTARTS then give up loudly. Infinite restarting of a device that has been physically
    unplugged just buries the real message.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

GRACE_S = 25.0          # let a freshly started driver open its device before judging it
CHECK_S = 6.0           # sampling window for the rate measurement
MAX_RESTARTS = 5


def measure_hz(topic: str, msg_type: str, seconds: float) -> float:
    """Messages per second seen on `topic`. Uses a subprocess so a wedged rclpy context
    cannot take the watchdog down with it."""
    script = f'''
import rclpy, importlib
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
mod, cls = "{msg_type}".rsplit("/", 1)
T = getattr(importlib.import_module(mod.replace("/", ".")), cls)
rclpy.init(); n = Node("utp_watchdog_probe")
c = [0]
n.create_subscription(T, "{topic}", lambda m: c.__setitem__(0, c[0] + 1), qos_profile_sensor_data)
t0 = n.get_clock().now().nanoseconds
while n.get_clock().now().nanoseconds - t0 < {seconds} * 1e9:
    rclpy.spin_once(n, timeout_sec=0.2)
d = (n.get_clock().now().nanoseconds - t0) / 1e9
print(c[0] / d)
n.destroy_node(); rclpy.shutdown()
'''
    try:
        r = subprocess.run([sys.executable, "-c", script], capture_output=True,
                           text=True, timeout=seconds + 30)
        return float(r.stdout.strip().splitlines()[-1])
    except Exception:
        return 0.0


def start(cmd):
    print(f"[watchdog] starting: {' '.join(cmd)}", flush=True)
    return subprocess.Popen(cmd, start_new_session=True)


def stop(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
        time.sleep(2)
        if proc.poll() is None:
            os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", required=True)
    ap.add_argument("--type", required=True, dest="msg_type",
                    help="e.g. sensor_msgs/msg/LaserScan")
    ap.add_argument("--min-hz", type=float, required=True)
    ap.add_argument("--max-restarts", type=int, default=MAX_RESTARTS)
    ap.add_argument("cmd", nargs=argparse.REMAINDER,
                    help="-- then the command to run and keep alive")
    a = ap.parse_args()
    cmd = [c for c in a.cmd if c != "--"]
    if not cmd:
        print("nothing to run: put the command after --", file=sys.stderr)
        return 2

    proc = start(cmd)
    restarts = 0
    try:
        while True:
            time.sleep(GRACE_S)
            hz = measure_hz(a.topic, a.msg_type, CHECK_S)
            alive = proc.poll() is None
            if hz >= a.min_hz:
                print(f"[watchdog] {a.topic} {hz:.1f} Hz  ok", flush=True)
                restarts = 0          # a sustained good period clears the budget
                continue

            # The interesting case: process alive, topic silent. That is the stale-fd failure,
            # and it is the one a process supervisor cannot see.
            why = "process alive but topic SILENT" if alive else "process exited"
            print(f"[watchdog] {a.topic} {hz:.1f} Hz < {a.min_hz:g} -- {why}", flush=True)
            if restarts >= a.max_restarts:
                print(f"[watchdog] GIVING UP after {restarts} restarts. This is not a software\n"
                      f"[watchdog] problem any more -- check the cable and the USB port.",
                      flush=True)
                return 1
            restarts += 1
            print(f"[watchdog] restart {restarts}/{a.max_restarts}", flush=True)
            stop(proc)
            time.sleep(3)
            proc = start(cmd)
    except KeyboardInterrupt:
        print("\n[watchdog] interrupted", flush=True)
        return 0
    finally:
        stop(proc)


if __name__ == "__main__":
    sys.exit(main())
