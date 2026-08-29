#!/usr/bin/env python3
"""Is the chassis actually listening to the computer?

    python3 bringup/chassis_mode.py          # report, exit 0 only if control_mode == CAN

THE FAILURE THIS CATCHES, and it is invisible from every ROS indicator. The Ranger's SWB switch
selects who holds command authority. With SWB DOWN the chassis is in CONTROL_MODE_RC and DISCARDS
every CAN motion command it receives -- while ranger_base keeps publishing /odom at 50 Hz, the
safety mux keeps reporting "permitted", /cmd_vel keeps its one publisher, and ranger_base keeps
transmitting 0x111 motion frames onto the bus. Every check we have goes green and the robot does
not move, because the discard happens in the chassis firmware, below anything ROS can see.

That is the same shape as the two other silent-discard bugs in this stack (the safety mux
throwing commands away with no subscriber listening, and waypoints from a dead odom frame): the
system is working exactly as designed and saying so somewhere nobody was reading.

Recording waypoints under RC is fine -- odometry comes from the wheel encoders regardless of who
commands. It is the AUTONOMOUS run that needs SWB up. And note that touching the transmitter
sticks reclaims RC at any moment, including mid-run, so this is worth checking before every run
rather than once a day.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stale_cmd_test import open_can  # noqa: E402
from claim_can import read_state     # noqa: E402

GOOD = "CAN"

ADVICE = {
    "RC": ("the RC transmitter holds authority. Flip SWB UP on the transmitter for command "
           "control mode. Until then every motion command the computer sends is discarded by "
           "the chassis, silently -- odom and the mux will both look perfectly healthy."),
    "STANDBY": ("the chassis is in STANDBY and accepts no motion. Run bringup/claim_can.py to "
                "ask for CAN authority."),
    "UART": "something else has taken the chassis over the serial link.",
}


def chassis_mode(iface: str = "can0", timeout_s: float = 2.0):
    """(vehicle_state, control_mode, battery_v, error_code), or None if the bus is silent."""
    try:
        sock = open_can(iface)
    except Exception:
        return None
    try:
        return read_state(sock, timeout_s)
    finally:
        try:
            sock.close()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="can0")
    a = ap.parse_args()

    st = chassis_mode(a.iface)
    if st is None:
        print(f"no 0x211 on {a.iface} -- the bus is silent. Is can0 up "
              f"(ip -br link show {a.iface}) and the rover powered?", file=sys.stderr)
        return 2
    vehicle, mode, batt, err = st
    print(f"chassis: vehicle_state={vehicle}  control_mode={mode}  battery={batt:.1f}V  "
          f"error=0x{err:04X}")
    if vehicle == "ESTOP":
        print("\nE-STOP is engaged. Nothing will move until it is released.", file=sys.stderr)
        return 1
    if vehicle == "EXCEPTION":
        # This outranks the control_mode advice below: in EXCEPTION the chassis REFUSES mode
        # changes, so "flip SWB up" is useless and reads as the whole story when it is not.
        # error is 0x0000 for both causes, so the code cannot tell them apart -- name both.
        # Observed 2026-08-29 the moment the RC transmitter was disconnected while the chassis
        # was in RC mode: disconnecting the RC does not hand authority to the computer, it trips
        # a lost-link failsafe and takes the robot further from drivable, not closer.
        print(f"\nCHASSIS IS IN EXCEPTION (mode {mode}, error 0x{err:04X}). It will refuse mode\n"
              f"changes while faulted, so nothing the computer sends can help. Two causes, and\n"
              f"the error code is 0x0000 for both:\n"
              f"  * an E-stop is pressed -- release it; or\n"
              f"  * the RC TRANSMITTER IS OFF or disconnected while the chassis is in RC mode\n"
              f"    (a lost-link failsafe). Turn the transmitter back ON, leave the sticks\n"
              f"    alone, and flip SWB UP for command control.\n"
              f"If it stays faulted: transmitter on with sticks untouched, power-cycle the rover\n"
              f"(it boots to STANDBY), then run bringup/claim_can.py immediately.", file=sys.stderr)
        return 1
    if mode != GOOD:
        print(f"\nNOT DRIVABLE BY THE COMPUTER: {ADVICE.get(mode, 'unexpected mode ' + mode)}",
              file=sys.stderr)
        return 1
    print("\nOK -- the chassis is taking commands from the computer.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
