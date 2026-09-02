#!/usr/bin/env python3
"""Report the xArm's tool geometry and payload. READ THE CONFLICT BELOW BEFORE USING --set.

*** 2026-09-02: DO NOT RUN --set WITHOUT RESOLVING THIS. ***
This file argues that tcp_offset should be [0,0,172,0,0,0] and treats zero as a fault. That
conflicts with the calibration the robot actually aims with, and the conflict is unresolved:

  calib/handeye.json was solved on 2026-08-21 with the arm at tcp_offset [0,0,0,0,0,0]
  (EXPERIMENT_LOG.md:875), and calib/pairs/*.json store arm_xyz_m straight from get_position().
  So marker_on_flange_mm is FLANGE-relative, and approach_target.py:254-291 reads get_position()
  and commands set_position() on that basis. Installing a 172 mm tool offset makes both calls
  refer to the TOOL TIP: the commanded flange retreats by the tool length and the marker lands
  172 mm SHORT. There is no force sensor to catch it -- get_ft_sensor_data answers zeros and
  collision_sensitivity is 0 -- so approach_target returns 0, press_run prints done, and the route
  reports success over a press that touched nothing.

So for the CURRENT calibration, zero is the correct state and this file's "DOES NOT match" verdict
is backwards. bringup/session.sh therefore runs this WITHOUT --set, for reporting only.

Which state is truly right cannot be settled from software: EXPERIMENT_LOG.md:2149 records that
SDK 1.18.4 has no live TCP getter, so the property read back below is a local cache and proves
nothing either way. It needs the physical measurement in docs/CALIBRATION.md item 2 -- touch one
fixed point from two arm configurations. Until that is done, either set the tool AND re-solve
hand-eye with it set, or leave both at zero. Do not mix them.

Original rationale follows, and is still correct about VOLATILITY -- the setting does not survive
a power cycle, whichever value it holds.


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
