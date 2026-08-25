#!/usr/bin/env python3
"""Take CAN command authority without power-cycling the rover.

    python3 bringup/claim_can.py            # ask until the chassis agrees, or explain why not
    python3 bringup/claim_can.py --watch    # just report control_mode, ask for nothing

The problem this solves. `ranger_base` calls `EnableCommandedMode()` exactly once, at driver
startup (agilex_base.hpp:78). If the chassis is not willing to hand authority over at that precise
moment, nothing ever asks again -- the driver runs, looks healthy, publishes odometry, and every
motion command is silently ignored. Recovering meant power-cycling the rover and racing to restart
the driver during the `STANDBY` window. On 2026-08-21 that cost most of an hour.

There is no reason the request has to be one-shot. This sends the same frame the SDK sends, on a
loop, and watches `0x211` until the chassis reports `CONTROL_MODE_CAN`.

Frame, from ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:620-632:
    can_id 0x421, dlc 8, byte0 = mode, bytes 1..7 = 0

**This commands no motion.** It changes only which source the chassis listens to. The wheels do
not turn as a result of anything in this file.

What it CANNOT do, and will tell you instead of hanging:
  * `EXCEPTION` -- the chassis refuses mode changes while faulted. Two causes, both seen, and the
    error code does not distinguish them because it is 0x0000 for both: an E-stop is pressed, OR
    the RC transmitter is off while the chassis is in RC mode (a lost-link failsafe).
  * The RC actively holding authority. Touching the sticks takes it back at any time, including
    one second after this succeeds.
"""
from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stale_cmd_test import (  # noqa: E402
    CAN_FRAME_FMT,
    CAN_FRAME_SIZE,
    CAN_SYSTEM_STATE,
    decode_system_state,
    open_can,
)

CAN_CTRL_MODE_CONFIG = 0x421
CONTROL_MODE_CAN = 0x01


def send_claim(sock: socket.socket) -> None:
    """One 0x421 asking for CONTROL_MODE_CAN. Byte 0 is the mode; the rest are reserved zeros."""
    data = bytes([CONTROL_MODE_CAN]) + bytes(7)
    sock.send(struct.pack(CAN_FRAME_FMT, CAN_CTRL_MODE_CONFIG, 8, data))


def read_state(sock: socket.socket, timeout_s: float = 0.5):
    """Latest 0x211 within timeout_s, or None."""
    deadline = time.monotonic() + timeout_s
    latest = None
    while time.monotonic() < deadline:
        sock.settimeout(max(0.01, deadline - time.monotonic()))
        try:
            raw = sock.recv(CAN_FRAME_SIZE)
        except (TimeoutError, socket.timeout):
            break
        cid, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        if (cid & socket.CAN_EFF_MASK) == CAN_SYSTEM_STATE:
            latest = decode_system_state(data[:dlc])
    return latest


def explain(vehicle: str, mode: str) -> str:
    if vehicle == "EXCEPTION":
        return ("chassis is in EXCEPTION and will not accept a mode change.\n"
                "  Both of these read EXCEPTION with error=0x0000 and cannot be told apart here:\n"
                "    * an E-stop is pressed  -> release both (chassis and RC transmitter)\n"
                "    * the RC transmitter is OFF while the chassis is in RC mode (lost-link\n"
                "      failsafe) -> turn the transmitter ON and leave the sticks alone")
    if mode == "RC":
        return ("the RC transmitter is holding authority. Leave the sticks alone -- any stick\n"
                "  input reclaims RC, including after this succeeds.")
    return f"chassis reports vehicle_state={vehicle}, control_mode={mode}."


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default=os.environ.get("RANGER_CAN_IFACE", "can0"))
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to keep asking")
    ap.add_argument("--watch", action="store_true", help="report only; send nothing")
    a = ap.parse_args()

    sock = open_can(a.iface)
    try:
        state = read_state(sock, 2.0)
        if state is None:
            print(f"chassis is silent on {a.iface} -- not powered, or CAN is not really connected.")
            return 1
        vehicle, mode, batt, err = state
        print(f"chassis: vehicle_state={vehicle}  control_mode={mode}  "
              f"battery={batt:.1f}V  error=0x{err:04x}")

        if a.watch:
            return 0
        if mode == "CAN":
            print("\nalready CAN. Nothing to do -- but do not touch the RC sticks.")
            return 0
        if vehicle == "EXCEPTION":
            print(f"\nREFUSING: {explain(vehicle, mode)}")
            return 1

        print(f"\nasking for CONTROL_MODE_CAN (0x421) at 10 Hz for up to {a.timeout:g}s...")
        deadline = time.monotonic() + a.timeout
        last = mode
        while time.monotonic() < deadline:
            send_claim(sock)
            st = read_state(sock, 0.1)
            if st is None:
                continue
            vehicle, mode, batt, err = st
            if mode != last:
                print(f"  {time.monotonic() - (deadline - a.timeout):4.1f}s  "
                      f"vehicle={vehicle} mode={mode}")
                last = mode
            if mode == "CAN":
                print("\nGRANTED -- control_mode=CAN. The chassis now listens to /cmd_vel.")
                print("  Do not touch the RC sticks: that reclaims RC and you will be back here.")
                return 0

        print(f"\nNOT GRANTED after {a.timeout:g}s -- {explain(vehicle, mode)}")
        print("\n  Fallback that is known to work: transmitter ON with sticks untouched,")
        print("  power-cycle the rover (it boots to STANDBY), then run this again immediately.")
        return 1
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
