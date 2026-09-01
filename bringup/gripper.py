#!/usr/bin/env python3
"""Open, close or read the xArm Gripper. The arm body does NOT move -- only the fingers.

    python3 bringup/gripper.py status         # read-only, always safe
    python3 bringup/gripper.py open --go
    python3 bringup/gripper.py close --go
    python3 bringup/gripper.py set 500 --go   # 0 = closed, 850 = fully open

WHICH TOOL IS ACTUALLY FITTED, measured 2026-09-01 on the arm:

    get_gripper_version()  -> '5.2.1'      the standard ELECTRIC xArm Gripper
    get_bio_gripper_*()    -> 0            not a Bio Gripper
    robotiq_get_status()   -> code 23      not a Robotiq

docs/CALIBRATION.md says the end effector is "a ~0.12 m stylus". It is not, and has not been
since at least 2026-08-21. Commanding a Bio Gripper API at this hardware does nothing and
reports success, which is why the type is pinned here rather than assumed.

DO NOT CALL robotiq_get_status() TO PROBE WHAT IS FITTED. It retunes the TOOL modbus bus to
Robotiq's 115200 baud and does not put it back, so the controller loses the xArm Gripper (which
talks at 2000000) and raises controller error 19, "End Effector Communication Error". It looks
like a broken gripper and it is a baud rate. Recovery, no power cycle needed:

    arm.clean_error(); arm.set_tgpio_modbus_baudrate(2000000)

Done here on 2026-09-01, by me, while probing which gripper was fitted. get_gripper_version()
alone answers that question without touching the bus configuration.

WITHOUT --go THIS PRINTS WHAT IT WOULD DO AND SENDS NOTHING. Same contract as waypoints.py and
nav2_goto.py -- a dry run must never be able to touch the hardware.

WHAT OPENING CAN DO: drop whatever the fingers are holding. There is no way to ask the gripper
whether it is gripping something (position alone cannot tell a held object from a closed hand),
so `status` prints the position and you look before you open.
"""
from __future__ import annotations

import argparse
import sys

OPEN_POS = 850          # xArm Gripper travel is 0 (closed) .. 850 (open)
CLOSED_POS = 0
SPEED = 2000            # 1..5000. Deliberately not the maximum: this gripper is on a 0.74 m
                        # riser above a high-CoM base, and a slam is a disturbance we do not need.
XARM_IP_DEFAULT = "192.168.1.221"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["status", "open", "close", "set"])
    ap.add_argument("value", nargs="?", type=int, help="target position for `set` (0-850)")
    ap.add_argument("--go", action="store_true", help="actually move; without it this is a dry run")
    ap.add_argument("--ip", default=None)
    ap.add_argument("--speed", type=int, default=SPEED)
    a = ap.parse_args()

    import os
    ip = a.ip or os.environ.get("UTP_XARM_IP", XARM_IP_DEFAULT)

    if a.action == "set":
        if a.value is None:
            print("`set` needs a position, 0-850", file=sys.stderr)
            return 2
        if not CLOSED_POS <= a.value <= OPEN_POS:
            print(f"position {a.value} is outside 0-{OPEN_POS}", file=sys.stderr)
            return 2
        target = a.value
    else:
        target = {"open": OPEN_POS, "close": CLOSED_POS}.get(a.action)

    from xarm.wrapper import XArmAPI
    arm = XArmAPI(ip, is_radian=True)

    code, ver = arm.get_gripper_version()
    ecode, gerr = arm.get_gripper_err_code()
    pcode, pos = arm.get_gripper_position()
    print(f"gripper  version={ver} err={gerr} position={pos}")
    print(f"arm      state={arm.get_state()[1]} err/warn={arm.get_err_warn_code()[1]}")
    if code != 0 or pcode != 0:
        print(f"gripper did not answer cleanly (codes {code}/{pcode}) -- refusing to command it",
              file=sys.stderr)
        arm.disconnect()
        return 3
    if gerr:
        # A gripper in an error state accepts commands and ignores them.
        print(f"gripper error code {gerr} is set; clear it before commanding", file=sys.stderr)
        arm.disconnect()
        return 4

    if a.action == "status":
        arm.disconnect()
        return 0

    if not a.go:
        print(f"DRY RUN: would move gripper {pos} -> {target} at speed {a.speed}. "
              f"Add --go to actually move it.")
        arm.disconnect()
        return 0

    print(f"moving gripper {pos} -> {target} ...")
    arm.set_gripper_mode(0)               # 0 = position mode
    arm.set_gripper_enable(True)
    arm.set_gripper_speed(a.speed)
    rc = arm.set_gripper_position(target, wait=True)
    _, now = arm.get_gripper_position()
    print(f"done: rc={rc} position={now}")
    arm.disconnect()
    return 0 if rc == 0 else 5


if __name__ == "__main__":
    raise SystemExit(main())
