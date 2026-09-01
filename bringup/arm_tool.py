#!/usr/bin/env python3
"""Set the xArm's tool geometry and payload. MUST RUN AT EVERY ARM BRING-UP.

    python3 bringup/arm_tool.py            # report what the arm currently believes
    python3 bringup/arm_tool.py --set      # write the tool offset and load, then verify

WHY THIS EXISTS. On 2026-09-01 the arm reported tcp_offset [0,0,172,0,0,0] and tcp_load 0.82 kg
in the morning and [0,0,0,0,0,0] / 0 in the afternoon, on the same robot, with nothing in this
repo having touched it. In between, the battery pack was recharged and the arm lost power.

TCP SETTINGS ARE VOLATILE AND NOTHING RESTORED THEM. A grep of the whole repository for
`set_tcp_offset` found no caller: the value had been set by hand, once, and every bring-up since
has silently inherited whatever survived. That is a configuration the robot forgets.

WHAT IT COSTS WHEN IT IS ZERO. Every Cartesian command then refers to the FLANGE, so the tool tip
lands short by the tool length along the approach axis -- 172 mm here. The press has missed the
plate by ~10 cm, and the arm itself is not the suspect: hand-eye RMS is 2.96 mm and measured
placement accuracy 4.3 mm. An arm accurate to 4 mm cannot produce a 100 mm error; a 172 mm
uncompensated tool can.

Collision detection is the second cost. With tcp_load at zero the thresholds are calibrated for a
bare flange, so the arm either nuisance-trips on its own gripper or fails to notice a real contact.

THE NUMBERS BELOW ARE NOMINAL, NOT MEASURED. 172 mm is the catalogue length of the xArm Gripper
(get_gripper_version reports 5.2.1 on this arm) and 0.82 kg its catalogue mass. They reproduce the
state the arm was in this morning, which is the state the working presses were made in. What
docs/CALIBRATION.md item 2 still wants is a MEASURED flange-face-to-fingertip figure for this
gripper as mounted, to +-3 mm, verified by touching one fixed point from two arm configurations.
Until that is done these are a defensible default and not a calibration.
"""
from __future__ import annotations

import argparse
import sys

ARM_IP = "192.168.1.221"
TCP_OFFSET_MM = [0.0, 0.0, 172.0, 0.0, 0.0, 0.0]   # x, y, z, roll, pitch, yaw -- SDK is millimetres
TCP_LOAD_KG = 0.82
TCP_LOAD_CENTRE_MM = [0.0, 0.0, 48.0]
TOL_MM = 0.5


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", action="store_true", help="write the values; without it, only report")
    ap.add_argument("--ip", default=ARM_IP)
    a = ap.parse_args()

    import os
    from xarm.wrapper import XArmAPI
    arm = XArmAPI(os.environ.get("UTP_XARM_IP", a.ip), is_radian=True)

    off, load = arm.tcp_offset, arm.tcp_load
    print(f"  tcp_offset : {off}")
    print(f"  tcp_load   : {load}")
    ok = (off and all(abs(float(off[i]) - TCP_OFFSET_MM[i]) <= TOL_MM for i in range(6)))
    if not a.set:
        print("  -> " + ("MATCHES the expected tool" if ok else
                         "DOES NOT match: the arm believes its tool is a bare flange. "
                         "Every press lands short. Re-run with --set."))
        arm.disconnect()
        return 0 if ok else 1

    # THE ARM MUST BE READY BEFORE IT WILL ACCEPT TOOL GEOMETRY. Measured 2026-09-01: with the
    # arm in state 4 (STOP), set_tcp_load returns code 9 = STATE_NOT_READY and set_tcp_offset
    # reports 0 while the readback stays zero -- one setter refuses loudly, the other lies. That
    # is why this verifies by reading back instead of trusting a return code.
    #
    # THIS ENERGISES THE SERVOS. It does NOT command any motion: the arm holds the pose it is in.
    # The state it was found in is restored afterwards, so this leaves the robot as it was.
    was_state = arm.get_state()[1]
    print(f"  arm state {was_state} -> enabling servos to accept tool geometry (NO motion)")
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(enable=True)
    arm.set_mode(0)
    arm.set_state(0)
    print(f"  setting tcp_offset -> {TCP_OFFSET_MM}")
    rc1 = arm.set_tcp_offset(TCP_OFFSET_MM, is_radian=True)
    print(f"  setting tcp_load   -> {TCP_LOAD_KG} kg at {TCP_LOAD_CENTRE_MM}")
    rc2 = arm.set_tcp_load(TCP_LOAD_KG, TCP_LOAD_CENTRE_MM)
    if was_state == 4:
        arm.set_state(4)                     # put it back exactly as found
        print("  arm returned to state 4 (stopped), as found")
    # VERIFY BY READING BACK, not by trusting a return code. A setter that reports success while
    # the arm keeps its old value is exactly the failure class this repo keeps meeting.
    arm.disconnect()
    arm2 = XArmAPI(os.environ.get("UTP_XARM_IP", a.ip), is_radian=True)
    off2, load2 = arm2.tcp_offset, arm2.tcp_load
    arm2.disconnect()
    print(f"  readback offset: {off2}")
    print(f"  readback load  : {load2}")
    good = off2 and all(abs(float(off2[i]) - TCP_OFFSET_MM[i]) <= TOL_MM for i in range(6))
    if not good:
        print(f"  FAILED: the arm did not take the offset (rc {rc1}/{rc2})", file=sys.stderr)
        return 1
    print("  ok -- tool geometry set and verified. Re-run after ANY arm power cycle.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
