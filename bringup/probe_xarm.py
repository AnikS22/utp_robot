#!/usr/bin/env python3
"""Probe the xArm6 control box — link, then socket, then SDK. No ROS, no motion.

    python3 bringup/probe_xarm.py                 # uses UTP_XARM_IP or config/safety.yaml
    python3 bringup/probe_xarm.py 192.168.1.185
    python3 bringup/probe_xarm.py --scan          # sweep the subnet for a control box

Gate H3 is RED for a PHYSICAL reason (EXPERIMENT_LOG.md 2026-08-18): every candidate NIC reported
`Link detected: no`. So this probe deliberately answers the layers in order and STOPS at the first
one that fails, because each failure has a different fix and the SDK reports all of them the same
way — as a connect timeout:

  1. carrier   -- is anything electrically on the other end of the cable?   (cable / control box power)
  2. address   -- do we hold an IP on the control box's subnet?             (static IP not set)
  3. tcp       -- does the control box answer on its command port?          (wrong IP)
  4. sdk       -- does it identify itself, and what is its error state?     (real arm problems)

`XArmAPI(ip)` with no route gives a ~20 s hang and a bare timeout, which reads as "the arm is
broken" when the real cause is that nobody ever ran `ip addr add`. Layers 1-3 use plain sockets and
answer in milliseconds.

READ-ONLY, and that is a safety property, not a convenience. It never calls motion_enable, never
sets mode or state, never commands a position. It is safe to run with the arm powered and people
nearby. The first thing that MOVES the arm should be a human at the control box with an E-stop in
reach, not a probe script.
"""
from __future__ import annotations

import argparse
import os
import re
import socket
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CMD_PORT = 30000                       # SDK command channel; the one that must answer
PORTS = [30000, 30001, 30002, 30003, 502]   # cmd, report(normal/rich/dev), modbus-tcp

# xArm error/warn codes worth naming inline; the rest get looked up in the manual.
ERR_HINTS = {
    1: "emergency stop button pressed / released improperly",
    2: "emergency IO triggered",
    9: "servo not enabled (motion_enable not called)",
    19: "collision detected",
    21: "kinematics error",
    22: "self-collision",
    23: "joint angle exceeds limit",
    24: "speed exceeds limit",
    31: "collision caused abnormal current",
}
STATE_NAMES = {0: "READY (in motion)", 1: "SLEEPING", 2: "PAUSED / standby",
               3: "PAUSED", 4: "STOPPED", 5: "SYS_RESETTING"}
MODE_NAMES = {0: "position control", 1: "servo motion", 2: "joint teaching (free-drive)",
              4: "joint velocity", 5: "cartesian velocity", 6: "joint online traj",
              7: "cartesian online traj"}


# --------------------------------------------------------------------------------------------
# config
# --------------------------------------------------------------------------------------------
def configured_ip() -> str:
    """UTP_XARM_IP, else config/safety.yaml, else the uFactory factory default.

    safety.yaml writes the address as ``${ENV:UTP_XARM_IP:192.168.1.185}``. Nothing in this repo
    expands that syntax yet, so parse it here rather than hand the literal string to a socket and
    get a confusing DNS error.
    """
    env = os.environ.get("UTP_XARM_IP")
    if env:
        return env
    try:
        import yaml
        cfg = yaml.safe_load((REPO / "config" / "safety.yaml").read_text())
        raw = str(cfg["arm_monitor"]["xarm"]["ip"])
    except Exception:
        return "192.168.1.185"
    m = re.fullmatch(r"\$\{ENV:([A-Za-z_][A-Za-z0-9_]*):([^}]*)\}", raw.strip())
    if m:
        return os.environ.get(m.group(1)) or m.group(2)
    return raw


def stow_pose() -> tuple[list[float], float]:
    try:
        import yaml
        xc = yaml.safe_load((REPO / "config" / "safety.yaml").read_text())["arm_monitor"]["xarm"]
        return [float(a) for a in xc["stow_pose_deg"]], float(xc["joint_tolerance_deg"])
    except Exception:
        return [0.0, -45.0, -45.0, 0.0, 90.0, 0.0], 5.0


# --------------------------------------------------------------------------------------------
# layer 1-2: link and address (read straight from sysfs / the routing table)
# --------------------------------------------------------------------------------------------
def check_link(ip: str) -> bool:
    import subprocess

    ok = False
    print("[1] link")
    net = Path("/sys/class/net")
    for iface in sorted(p.name for p in net.iterdir()):
        if iface == "lo" or iface.startswith(("wl", "docker", "veth", "br-", "virbr")):
            continue
        try:
            carrier = (net / iface / "carrier").read_text().strip()
        except OSError:
            carrier = "?"      # sysfs refuses to answer for an admin-down interface
        state = (net / iface / "operstate").read_text().strip()
        mark = "OK  " if carrier == "1" else "-- "
        note = "" if carrier == "1" else "   no carrier: nothing on the other end, or link is admin-down"
        print(f"    {mark}{iface:20s} operstate={state:12s} carrier={carrier}{note}")
        ok = ok or carrier == "1"

    print("[2] address")
    try:
        out = subprocess.run(["ip", "-o", "route", "get", ip],
                             capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        out = ""
    if not out:
        print(f"    --  no route to {ip}")
        return False
    print(f"    OK  {out}")
    if " via " in out:
        print("    !!  routed via a GATEWAY, not a direct link. The control box should be on a")
        print("        directly-attached subnet; going through a router usually means the static")
        print("        address is missing and the default route is swallowing the traffic.")
    return ok


# --------------------------------------------------------------------------------------------
# layer 3: tcp
# --------------------------------------------------------------------------------------------
def probe_port(ip: str, port: int, timeout: float = 1.0) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        try:
            return s.connect_ex((ip, port)) == 0
        except OSError:
            return False


def check_tcp(ip: str) -> bool:
    print(f"[3] tcp {ip}")
    open_ports = []
    for p in PORTS:
        up = probe_port(ip, p)
        open_ports.append(p) if up else None
        print(f"    {'OK  ' if up else '--  '}{ip}:{p}")
    if CMD_PORT not in open_ports:
        print(f"    !!  command port {CMD_PORT} closed — the SDK cannot connect.")
        return False
    return True


# --------------------------------------------------------------------------------------------
# layer 4: sdk
# --------------------------------------------------------------------------------------------
def check_sdk(ip: str) -> bool:
    print("[4] sdk")
    try:
        from xarm.wrapper import XArmAPI
    except ImportError:
        print("    !!  xArm-Python-SDK not importable for this interpreter.")
        print("        pip install xArm-Python-SDK   (or use ~/utp_robot/.venv-arm/bin/python)")
        return False

    # do_not_open=True then open explicitly: the constructor otherwise connects during __init__,
    # so a failure surfaces as a traceback out of an assignment rather than a value we can report.
    arm = XArmAPI(ip, is_radian=False, do_not_open=True)
    try:
        arm.connect()
    except Exception as e:
        print(f"    !!  connect failed: {type(e).__name__}: {e}")
        return False
    if not arm.connected:
        print("    !!  connect returned but the SDK reports not connected")
        return False

    try:
        print(f"    OK  connected  {ip}")
        print(f"        version   : {arm.version}")
        print(f"        sn        : {arm.sn}")
        print(f"        axis      : {arm.axis}   (xArm6 -> 6)")
        state = arm.state
        mode = arm.mode
        print(f"        state     : {state}  {STATE_NAMES.get(state, '')}")
        print(f"        mode      : {mode}  {MODE_NAMES.get(mode, '')}")

        err, warn = arm.error_code, arm.warn_code
        print(f"        error/warn: {err} / {warn}"
              + (f"   <-- {ERR_HINTS[err]}" if err in ERR_HINTS else ""))
        if err:
            print("        !!  a non-zero error code LATCHES: the arm refuses motion until it is")
            print("            cleared (arm.clean_error()). Read the cause off the control box")
            print("            screen before clearing — clearing hides it, it does not fix it.")

        code, angles = arm.get_servo_angle(is_radian=False)
        if code == 0 and angles:
            shown = ", ".join(f"{a:8.2f}" for a in angles[:6])
            print(f"        joints deg: [{shown}]")
            stow, tol = stow_pose()
            deltas = [abs(a - s) for a, s in zip(angles[:len(stow)], stow)]
            stowed = all(d <= tol for d in deltas)
            print(f"        stow pose : {stow}  tol +/-{tol} deg")
            print(f"        STOWED    : {stowed}   (worst joint delta {max(deltas):.2f} deg)")
            print("        this is the exact measurement safety/arm_monitor_node.py gates the base on.")
        else:
            print(f"        !!  get_servo_angle failed, code={code}")

        code, pos = arm.get_position(is_radian=False)
        if code == 0 and pos:
            # The SDK's Cartesian API is MILLIMETRES and the rest of this stack is metres.
            # Printing both is the cheapest possible defence against the units bug.
            x, y, z = pos[0], pos[1], pos[2]
            print(f"        tcp mm    : x={x:.1f} y={y:.1f} z={z:.1f}  rpy={pos[3]:.1f},{pos[4]:.1f},{pos[5]:.1f}")
            print(f"        tcp m     : x={x/1000:.4f} y={y/1000:.4f} z={z/1000:.4f}   <-- our units")
        return True
    finally:
        arm.disconnect()


# --------------------------------------------------------------------------------------------
def scan(subnet: str) -> None:
    """Sweep a /24 for anything answering on the xArm command port.

    Worth having because the control box's address is set at the factory and changed by whoever
    set the arm up; the screen on the box is authoritative, but it is not always in the room.
    """
    from concurrent.futures import ThreadPoolExecutor

    base = subnet.rsplit(".", 1)[0]
    print(f"[scan] {base}.1-254 port {CMD_PORT} ...")
    hosts = [f"{base}.{i}" for i in range(1, 255)]
    with ThreadPoolExecutor(max_workers=128) as ex:
        hits = [h for h, up in zip(hosts, ex.map(lambda h: probe_port(h, CMD_PORT, 0.4), hosts)) if up]
    if hits:
        for h in hits:
            print(f"    OK  {h}:{CMD_PORT}")
        print(f"\n    UTP_XARM_IP={hits[0]} python3 bringup/probe_xarm.py")
    else:
        print("    -- nothing answered. Check carrier and that you hold an address on this subnet.")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip", nargs="?", default=None, help="control box IP (default: UTP_XARM_IP or config)")
    ap.add_argument("--scan", action="store_true", help="sweep the /24 for a control box")
    args = ap.parse_args()

    ip = args.ip or configured_ip()
    print(f"xArm6 probe -> {ip}   (read-only: no motion is ever commanded)\n")

    if args.scan:
        scan(ip)
        return 0

    if not check_link(ip):
        print("\nSTOP at layer 1/2. Nothing above this can work.")
        print("  * is the control box powered on, and past its boot screen?")
        print("  * is the cable seated at BOTH ends, and are the link LEDs lit?")
        print("  * bring the interface up and give it an address on the control box's subnet:")
        print("      sudo ip link set <iface> up")
        print("      sudo ip addr add 192.168.1.100/24 dev <iface>")
        return 1
    print()
    if not check_tcp(ip):
        print("\nSTOP at layer 3. The link is up but nothing answers at this address.")
        print("  * read the actual IP off the control box screen (Settings -> Network)")
        print("  * or sweep for it:  python3 bringup/probe_xarm.py --scan")
        return 1
    print()
    return 0 if check_sdk(ip) else 1


if __name__ == "__main__":
    sys.exit(main())
