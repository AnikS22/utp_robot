#!/usr/bin/env python3
"""Does the base keep driving after the thing commanding it dies?

    python3 bringup/stale_cmd_test.py driver     # E-STOP ENGAGED. Zero risk. Do this first.
    python3 bringup/stale_cmd_test.py firmware   # THE BASE MOVES. Read the warning below.
    python3 bringup/stale_cmd_test.py listen     # just decode CAN traffic, command nothing

Closes the UNVERIFIED item in HARDWARE_SPECS.md that gates all driving, including Nav2 and
mapping. Written after 2026-08-20, when the base ran away under keyboard teleop with the
operator's hands off the controls and was stopped by the hardware E-stop.

There are TWO independent failure modes and they need different tests. Conflating them is how
you "fix" a runaway and still have one:

  driver    (Run with the RANGER's E-stop pressed -- the chassis one, not the arm's. On this
            chassis that reads vehicle_state=EXCEPTION, not ESTOP; both are accepted.)
            Does ranger_base keep TRANSMITTING the last twist on CAN after its /cmd_vel
            publisher dies?  Reading the source says no -- ranger_messenger.cpp:391 calls
            SetMotionCommand straight from the subscription callback and agilex_base.hpp:92
            emits exactly one frame per call, with no repeat timer.  This phase confirms that
            on the real bus, because "I read the code" is not the same as "I measured it", and
            because a future driver version could add a timer.

  firmware   If 0x111 stops ARRIVING, does the chassis keep executing the last command?
            Nothing in ugv_sdk or the ranger_ros2 README states a required command rate or a
            chassis-side timeout.  It is a property of firmware we cannot read.  The only way
            to know is to command motion, cut the commands, and watch the chassis' own
            reported velocity in 0x221 decay -- or not.

A PASS on `driver` and a FAIL on `firmware` is a runaway, even though the driver is blameless.

The frame layout is from ugv_sdk/src/protocol_v2/agilex_protocol_v2.h.  Note that struct16_t
under USE_LITTLE_ENDIAN (the default, line 21) declares high_byte FIRST, and the encoder
memcpy's the struct straight into the frame -- so despite the macro name the wire order is
MSB-first.  Getting this backwards decodes 0.15 m/s as 13.8 m/s and every verdict inverts.
"""
from __future__ import annotations

import argparse
import os
import signal
import socket
import struct
import subprocess
import sys
import time
from dataclasses import dataclass

CAN_MOTION_COMMAND = 0x111   # host -> chassis, what we command
CAN_SYSTEM_STATE = 0x211     # chassis -> host, vehicle_state / control_mode
CAN_MOTION_STATE = 0x221     # chassis -> host, what the chassis says it is ACTUALLY doing

VEHICLE_STATE = {0x00: "NORMAL", 0x01: "ESTOP", 0x02: "EXCEPTION"}
CONTROL_MODE = {0x00: "STANDBY", 0x01: "CAN", 0x02: "UART", 0x03: "RC"}

CAN_FRAME_FMT = "=IB3x8s"     # struct can_frame: can_id, can_dlc, pad, data
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

# A command that survives longer than this after the publisher dies is a latch, not a straggler.
# Deliberately generous: at the driver's ~20 Hz one in-flight frame is 50 ms, so 0.5 s is ten
# frames' worth of slack before we call it a fault.
SETTLE_S = 0.5

TEST_LINEAR = 0.15           # m/s -- the mux's own clamp, so this is the fastest teleop can go


@dataclass(frozen=True)
class Motion:
    """A 0x111 command or a 0x221 report. Same layout, opposite direction."""
    linear: float            # m/s
    angular: float           # rad/s
    lateral: float           # m/s
    steer: float             # RAW field / 1000 -- NOT radians. See steer_deg.

    @property
    def steer_deg(self) -> float:
        """Steering angle in degrees.

        The raw 0x111/0x221 steering field is NOT an angle in our units. ranger_base.hpp:155
        encodes it as `-angle_rad / 10.0 / 3.14 * 180` and decodes state as
        `-raw * 10 / 180.0 * 3.14`, so raw carries (degrees / 10) with the sign flipped
        relative to ROS convention. Reading raw as radians is wrong by 5.7x AND backwards --
        -0.935 raw is +9.4 deg, not -0.9 rad.
        """
        return -self.steer * 10.0

    def is_zero(self, eps: float = 1e-6) -> bool:
        return all(abs(v) <= eps for v in (self.linear, self.angular, self.lateral, self.steer))

    def __str__(self) -> str:
        return (f"lin={self.linear:+.3f}m/s ang={self.angular:+.3f}rad/s "
                f"lat={self.lateral:+.3f}m/s steer={self.steer_deg:+.1f}deg")


def _i16(data: bytes, off: int) -> int:
    """One MSB-first signed 16-bit field. See the module docstring on why MSB-first."""
    return struct.unpack_from(">h", data, off)[0]


def decode_motion(data: bytes) -> Motion:
    """0x111 / 0x221 payload -> Motion. Scale is 1000 for every field (agilex_msg_parser_v2.c)."""
    if len(data) < 8:
        raise ValueError(f"motion frame needs 8 bytes, got {len(data)}")
    return Motion(_i16(data, 0) / 1000.0, _i16(data, 2) / 1000.0,
                  _i16(data, 4) / 1000.0, _i16(data, 6) / 1000.0)


def decode_system_state(data: bytes) -> tuple[str, str, float, int]:
    """0x211 -> (vehicle_state, control_mode, battery_volts, error_code)."""
    if len(data) < 6:
        raise ValueError(f"system state frame needs 6 bytes, got {len(data)}")
    return (VEHICLE_STATE.get(data[0], f"UNKNOWN(0x{data[0]:02x})"),
            CONTROL_MODE.get(data[1], f"UNKNOWN(0x{data[1]:02x})"),
            struct.unpack_from(">h", data, 2)[0] * 0.1,
            struct.unpack_from(">H", data, 4)[0])


def open_can(iface: str) -> socket.socket:
    state_path = f"/sys/class/net/{iface}/operstate"
    if not os.path.exists(state_path):
        sys.exit(f"no such interface: {iface}\n"
                 f"  The USB-CAN adapter must be on a DIRECT laptop port -- HARDWARE_SPECS.md\n"
                 f"  records it dropping off a shared hub after 2-3 s, twice.")
    with open(state_path) as f:
        state = f.read().strip()
    if state != "up":
        sys.exit(f"{iface} is '{state}', not 'up'.  Try: sudo ip link set {iface} up "
                 f"type can bitrate 500000")
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s


def collect(sock: socket.socket, seconds: float, want: set[int]) -> list[tuple[float, int, bytes]]:
    """Drain the bus for `seconds`, keeping frames whose id is in `want`. Timestamps monotonic."""
    out: list[tuple[float, int, bytes]] = []
    deadline = time.monotonic() + seconds
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return out
        sock.settimeout(remaining)
        try:
            raw = sock.recv(CAN_FRAME_SIZE)
        except (TimeoutError, socket.timeout):
            return out
        can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
        can_id &= socket.CAN_EFF_MASK
        if can_id in want:
            out.append((time.monotonic(), can_id, data[:dlc]))


def chassis_state(sock: socket.socket, seconds: float = 2.0):
    """Latest 0x211 seen in `seconds`, or None if the chassis is not talking."""
    frames = collect(sock, seconds, {CAN_SYSTEM_STATE})
    return decode_system_state(frames[-1][2]) if frames else None


def start_publisher(topic: str, linear: float) -> subprocess.Popen:
    """A stand-in for teleop/Nav2: publishes a constant twist until killed.

    Its own process group, so the kill takes the whole `ros2` wrapper and its python child --
    and only them.  Never pattern-matched: we kill this pid, nothing else (CLAUDE.md rule 5).
    """
    twist = (f"{{linear: {{x: {linear}, y: 0.0, z: 0.0}}, "
             f"angular: {{x: 0.0, y: 0.0, z: 0.0}}}}")
    return subprocess.Popen(
        ["ros2", "topic", "pub", "-r", "20", topic, "geometry_msgs/msg/Twist", twist],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def kill_publisher(proc: subprocess.Popen) -> float:
    """SIGKILL the group and return the kill instant.

    SIGKILL, not SIGTERM, on purpose: a crashed node gets no destructor, no zero twist on the
    way out, and leaves its DDS participant undisposed.  Testing the polite exit tests the case
    that was never dangerous.
    """
    t = time.monotonic()
    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    proc.wait(timeout=5)
    return t


def summarize(frames, t_kill, label):
    """Report every frame after t_kill and whether any non-zero one outlived SETTLE_S."""
    # 0x111 and 0x221 share a layout (MotionCommandFrame / MotionStateFrame), so one
    # decoder serves both phases.
    after = [(t - t_kill, decode_motion(d)) for t, _, d in frames if t >= t_kill]
    print(f"\n  {label}: {len(after)} frames in the window")
    nonzero = [(dt, m) for dt, m in after if not m.is_zero()]
    for dt, m in after[:4]:
        print(f"    +{dt:6.3f}s  {m}")
    # The SHAPE of the stop matters as much as its duration: holding speed then cutting to zero
    # travels twice as far as ramping down over the same interval. Printing only the head hides
    # which one happened, so show the frames bracketing the last non-zero one too.
    if nonzero:
        last_i = next(i for i, (dt, _) in enumerate(after) if dt == nonzero[-1][0])
        tail = after[max(4, last_i - 3):last_i + 2]
        if tail:
            skipped = max(0, (last_i - 3) - 4)
            if skipped:
                print(f"    ... {skipped} more ...")
            for dt, m in tail:
                print(f"    +{dt:6.3f}s  {m}")
        speeds = [abs(m.linear) for _, m in nonzero]
        held = speeds[-1] > 0.8 * max(speeds)
        dist = sum(abs(m.linear) for _, m in nonzero) / max(1, len(nonzero)) * nonzero[-1][0]
        print(f"    last non-zero at +{nonzero[-1][0]:.3f}s  "
              f"({'HELD speed then cut' if held else 'ramped down'}, "
              f"~{dist*100:.0f} cm travelled after the kill)")
    elif len(after) > 4:
        print(f"    ... {len(after) - 4} more")
    return after, nonzero


def phase_listen(sock, seconds):
    print(f"Listening on the bus for {seconds:.0f}s. Commanding nothing.\n")
    frames = collect(sock, seconds, {CAN_MOTION_COMMAND, CAN_SYSTEM_STATE, CAN_MOTION_STATE})
    counts: dict[int, int] = {}
    for _, cid, _ in frames:
        counts[cid] = counts.get(cid, 0) + 1
    for cid, n in sorted(counts.items()):
        name = {CAN_MOTION_COMMAND: "0x111 motion command (host->chassis)",
                CAN_SYSTEM_STATE: "0x211 system state",
                CAN_MOTION_STATE: "0x221 motion state  (chassis->host)"}[cid]
        print(f"  {name:42s} {n:5d} frames  {n / seconds:5.1f} Hz")
    last = {cid: d for _, cid, d in frames}
    if CAN_SYSTEM_STATE in last:
        v, c, batt, err = decode_system_state(last[CAN_SYSTEM_STATE])
        print(f"\n  vehicle_state={v}  control_mode={c}  battery={batt:.1f}V  error=0x{err:04x}")
    if CAN_MOTION_STATE in last:
        print(f"  chassis reports moving: {decode_motion(last[CAN_MOTION_STATE])}")
    if CAN_MOTION_COMMAND in last:
        print(f"  someone is commanding:  {decode_motion(last[CAN_MOTION_COMMAND])}")
    if not frames:
        print("  Nothing at all. Is the chassis powered and the driver running?")
    return 0


def phase_driver(sock, topic, linear):
    """E-stop engaged. Does the DRIVER keep transmitting after its publisher dies?"""
    state = chassis_state(sock)
    if state is None:
        sys.exit("chassis is silent on 0x211 -- not powered, or can0 is not really connected")
    vehicle, mode, batt, err = state
    print(f"chassis: vehicle_state={vehicle}  control_mode={mode}  "
          f"battery={batt:.1f}V  error=0x{err:04x}")
    # MEASURED on this chassis 2026-08-21: pressing the Ranger E-stop moves vehicle_state
    # NORMAL -> EXCEPTION (0x02), NOT the ESTOP (0x01) that agilex_types.h names. Requiring
    # ESTOP here would have refused on correctly-stopped hardware. Both values mean the chassis
    # will not act on a motion command, which is the property this phase actually needs.
    if vehicle not in ("ESTOP", "EXCEPTION"):
        sys.exit(f"\nREFUSING: vehicle_state is {vehicle}.\n"
                 f"  This phase commands motion. It is only safe because the E-stop makes the\n"
                 f"  command unactionable while still visible on the bus. Press the RANGER's\n"
                 f"  E-stop -- the chassis one, not the arm's -- and re-run. Expect the state to\n"
                 f"  become EXCEPTION on this chassis. Use `firmware` if you intend it to move.")
    if vehicle == "EXCEPTION" and err != 0:
        sys.exit(f"\nREFUSING: vehicle_state=EXCEPTION with error=0x{err:04x}.\n"
                 f"  A pressed E-stop reports EXCEPTION with error 0x0000 on this chassis. A\n"
                 f"  NON-ZERO error means a real fault is also present, and this phase cannot\n"
                 f"  tell the two apart. Clear the fault first.")
    print(f"  {vehicle} with error 0x{err:04x} -- the chassis will not act on what we send."
          + (f"\n  control_mode={mode} also means it ignores CAN motion entirely: second layer."
             if mode != "CAN" else ""))

    print(f"\n[1/3] baseline: 2s of bus traffic with nothing publishing to {topic}")
    base = collect(sock, 2.0, {CAN_MOTION_COMMAND})
    print(f"      {len(base)} x 0x111.  " +
          ("idle, as expected" if not base else
           f"NOT idle -- something else is commanding: {decode_motion(base[-1][2])}"))

    print(f"[2/3] publishing linear.x={linear} to {topic} at 20 Hz for 3s")
    proc = start_publisher(topic, linear)
    driving = collect(sock, 3.0, {CAN_MOTION_COMMAND})
    live = [decode_motion(d) for _, _, d in driving]
    moving = [m for m in live if not m.is_zero()]
    print(f"      {len(driving)} x 0x111, {len(moving)} non-zero, "
          f"{len(driving) / 3.0:.1f} Hz")
    if not moving:
        kill_publisher(proc)
        sys.exit("\nINCONCLUSIVE: the command never reached CAN, so killing the publisher\n"
                 "  proves nothing. Check the driver is running and subscribed to "
                 f"{topic}, and\n  that control_mode is CAN (it is {mode}) -- under RC the "
                 "driver is ignored.")
    print(f"      commanded on the wire: {moving[-1]}")

    print("[3/3] SIGKILL the publisher, then watch 0x111 for 5s")
    t_kill = kill_publisher(proc)
    tail = collect(sock, 5.0, {CAN_MOTION_COMMAND})
    after, nonzero = summarize(tail, t_kill, "0x111 after the kill")

    print("\n" + "=" * 72)
    late = [dt for dt, _ in nonzero if dt > SETTLE_S]
    if late:
        print(f"FAIL -- the driver LATCHES. Non-zero 0x111 still on the bus {late[-1]:.1f}s\n"
              f"  after the publisher died. Do not drive. A watchdog node that republishes\n"
              f"  zero must sit between every commander and the driver.")
        return 1
    if not after:
        print("PASS (driver) -- 0x111 stopped dead at the kill. The driver relays live\n"
              "  commands and holds nothing, matching ranger_messenger.cpp:391.")
    else:
        print(f"PASS (driver) -- 0x111 went quiet or zero within {SETTLE_S}s of the kill.")
    print("\n  This does NOT clear the base to drive. It rules out ONE of the two failure\n"
          "  modes. The chassis now receives no commands at all rather than zero commands,\n"
          "  and whether firmware stops on that is unmeasured. Run the `firmware` phase.")
    print("=" * 72)
    return 0


def phase_firmware(sock, topic, linear, confirmed):
    """E-stop RELEASED, base moves. Does the CHASSIS stop when commands stop arriving?"""
    if not confirmed:
        sys.exit(
            "REFUSING without --i-am-holding-the-estop.\n\n"
            "  This phase drives the base and then deliberately cuts the commands. If firmware\n"
            "  latches -- the thing being tested -- the base keeps going until you stop it.\n\n"
            "  Before passing the flag:\n"
            "    - at least 5 m of clear floor ahead, nothing fragile, no people in the path\n"
            "    - the E-stop fob physically in your hand, thumb on it, for the whole run\n"
            "    - the arm stowed (CLAUDE.md: the base must not move otherwise)\n"
            "    - `driver` phase already PASSED\n\n"
            f"  It commands {linear} m/s forward for 3 s, then kills the publisher and watches.")

    state = chassis_state(sock)
    if state is None:
        sys.exit("chassis is silent on 0x211")
    vehicle, mode, batt, err = state
    print(f"chassis: vehicle_state={vehicle}  control_mode={mode}  "
          f"battery={batt:.1f}V  error=0x{err:04x}")
    if vehicle == "ESTOP":
        sys.exit("E-stop is engaged, so the base cannot move and this measures nothing.\n"
                 "  Release it -- but keep the fob in your hand.")
    if vehicle == "EXCEPTION":
        sys.exit("vehicle_state=EXCEPTION.\n"
                 "  On this chassis a PRESSED E-stop also reads EXCEPTION -- so first check it is\n"
                 "  actually released (both of them: chassis and RC transmitter). If it still\n"
                 "  reads EXCEPTION with everything released, that is a real fault. Clear it\n"
                 "  before commanding motion.")
    if mode != "CAN":
        sys.exit(f"control_mode={mode}, not CAN. The chassis will ignore the driver.\n"
                 "  Flip SWB on the RC to hand authority over, THEN restart ranger_bringup --\n"
                 "  EnableCommandedMode() is one-shot at driver startup.")

    print(f"\n[1/2] publishing linear.x={linear} to {topic} at 20 Hz for 3s. THE BASE WILL MOVE.")
    proc = start_publisher(topic, linear)
    driving = collect(sock, 3.0, {CAN_MOTION_STATE})
    reported = [decode_motion(d) for _, _, d in driving]
    rolling = [m for m in reported if not m.is_zero()]
    if not rolling:
        kill_publisher(proc)
        sys.exit("\nINCONCLUSIVE: the chassis never reported moving (0x221 stayed zero), so\n"
                 "  there is no motion to stop. Brakes engaged? Check 0x131 / parking mode.")
    print(f"      chassis reports: {rolling[-1]}")

    print("[2/2] SIGKILL the publisher. Watching 0x221 for 5s. KEEP YOUR THUMB ON THE E-STOP.")
    t_kill = kill_publisher(proc)
    tail = collect(sock, 5.0, {CAN_MOTION_STATE})
    after, nonzero = summarize(tail, t_kill, "0x221 (measured) after the kill")

    print("\n" + "=" * 72)
    late = [dt for dt, _ in nonzero if dt > SETTLE_S]
    if late:
        print(f"FAIL -- the CHASSIS latches. It was still reporting motion {late[-1]:.1f}s after\n"
              f"  the last command. Losing the commander means a runaway. Every commander must\n"
              f"  be fronted by a node that keeps publishing zero, and it must never be the\n"
              f"  same process that can crash.")
        return 1
    if nonzero:
        print(f"PASS (firmware) -- motion decayed to zero {nonzero[-1][0]:.3f}s after the last\n"
              f"  command. The chassis has its own command timeout.")
    else:
        print("PASS (firmware) -- zero on the first frame after the kill.")
    print("\n  Record the measured stop time in HARDWARE_SPECS.md. It is the floor on how fast\n"
          "  ANY software watchdog can act, and it is the number that decides whether the\n"
          "  0.35 s teleop watchdog is fast enough at 0.15 m/s.")
    print("=" * 72)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("phase", choices=["listen", "driver", "firmware"])
    ap.add_argument("--iface", default=os.environ.get("RANGER_CAN_IFACE", "can0"))
    ap.add_argument("--topic", default="/cmd_vel",
                    help="what the driver subscribes to (ranger_messenger.cpp:169). Deliberately "
                         "NOT via the twist mux: this tests the driver, not the mux.")
    ap.add_argument("--linear", type=float, default=TEST_LINEAR)
    ap.add_argument("--seconds", type=float, default=5.0, help="listen phase only")
    ap.add_argument("--i-am-holding-the-estop", action="store_true", dest="confirmed")
    a = ap.parse_args()

    sock = open_can(a.iface)
    try:
        if a.phase == "listen":
            return phase_listen(sock, a.seconds)
        if a.phase == "driver":
            return phase_driver(sock, a.topic, a.linear)
        return phase_firmware(sock, a.topic, a.linear, a.confirmed)
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
