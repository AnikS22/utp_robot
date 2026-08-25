"""Tests for bringup/stale_cmd_test.py's CAN decoding.

The decode is the whole test. If it is wrong, `stale_cmd_test.py` still prints a confident
PASS or FAIL -- just the wrong one -- and that verdict is what clears the base to drive.

The failure this file exists to prevent: struct16_t under USE_LITTLE_ENDIAN
(agilex_protocol_v2.h:20-21, the default) declares `high_byte` FIRST, and EncodeCanFrameV2
memcpy's the struct straight into the frame. So the wire order is MSB-first despite the macro
name. Decode it LSB-first instead and 0.15 m/s reads as 13.8 m/s -- non-zero either way, so the
"is it still commanding?" verdict survives, while every number a human reads is nonsense.
"""
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "bringup"))

from stale_cmd_test import (  # noqa: E402
    Motion,
    decode_motion,
    decode_system_state,
)


def encode_motion(linear, angular, lateral, steer) -> bytes:
    """Byte-for-byte port of EncodeCanFrameV2's AgxMsgMotionCommand branch.

    Independent of the decoder under test -- transcribed from
    ugv_sdk/src/protocol_v2/agilex_msg_parser_v2.c:383-404, not derived from it.
    """
    out = b""
    for v in (linear, angular, lateral, steer):
        cmd = int(v * 1000)                      # C int16_t cast, truncates toward zero
        out += bytes([(cmd >> 8) & 0xFF, cmd & 0xFF])   # high_byte first
    return out


# --------------------------------------------------------------------------------------------
# byte order -- the failure that inverts numbers while leaving the verdict looking sane
# --------------------------------------------------------------------------------------------

def test_wire_order_is_msb_first():
    """0.15 m/s is 150 = 0x0096, so the frame must start 0x00 0x96, not 0x96 0x00."""
    assert encode_motion(0.15, 0, 0, 0)[:2] == b"\x00\x96"
    assert decode_motion(b"\x00\x96" + bytes(6)).linear == pytest.approx(0.15)


def test_lsb_first_decode_would_be_caught():
    """The specific wrong reading, and it is worse than a scale error.

    0.15 m/s is 0x0096 on the wire. Read LSB-first that is 0x9600 = 38400, which overflows
    int16 to -27136, i.e. -27.1 m/s. A gentle crawl forward decodes as a violent reverse: the
    sign flips, the magnitude is 180x out, and it is still non-zero -- so a stale-command
    verdict based on is_zero() alone would look perfectly healthy.
    """
    frame = encode_motion(0.15, 0, 0, 0)
    wrong = struct.unpack_from("<h", frame, 0)[0] / 1000.0
    assert wrong == pytest.approx(-27.136)
    assert decode_motion(frame).linear == pytest.approx(0.15)


def test_roundtrip_all_four_fields_independently():
    """Field offsets must not be transposed: lateral in the steer slot is a plausible-looking bug."""
    m = decode_motion(encode_motion(0.15, -0.4, 0.05, 1.2))
    assert m.linear == pytest.approx(0.15)
    assert m.angular == pytest.approx(-0.4)
    assert m.lateral == pytest.approx(0.05)
    assert m.steer == pytest.approx(1.2)


def test_negative_velocity_is_signed():
    """Reverse must decode as reverse. Unsigned would read -0.15 m/s as +65.4 m/s."""
    assert decode_motion(encode_motion(-0.15, 0, 0, 0)).linear == pytest.approx(-0.15)
    assert decode_motion(b"\xff\x6a" + bytes(6)).linear == pytest.approx(-0.15)


def test_full_scale_extremes_do_not_wrap():
    for v in (32.767, -32.768):
        assert decode_motion(encode_motion(v, 0, 0, 0)).linear == pytest.approx(v, abs=1e-3)


# --------------------------------------------------------------------------------------------
# is_zero -- decides PASS vs FAIL, so both directions must be exact
# --------------------------------------------------------------------------------------------

def test_all_zero_frame_is_zero():
    assert decode_motion(bytes(8)).is_zero()


def test_one_nonzero_field_is_not_zero():
    """A latched STEER with zero velocity is still a latched command. Any field counts."""
    for i in range(4):
        fields = [0.0, 0.0, 0.0, 0.0]
        fields[i] = 0.05
        assert not decode_motion(encode_motion(*fields)).is_zero(), f"field {i} ignored"


def test_smallest_representable_command_is_not_zero():
    """1 mm/s is one LSB. Rounding it away would turn a real latch into a PASS."""
    assert not decode_motion(b"\x00\x01" + bytes(6)).is_zero()


def test_negative_field_is_not_zero():
    assert not decode_motion(encode_motion(-0.001, 0, 0, 0)).is_zero()


# --------------------------------------------------------------------------------------------
# system state -- gates whether the driver phase is allowed to run at all
# --------------------------------------------------------------------------------------------

def _state_frame(vehicle, mode, millivolts_x10=480, err=0):
    return bytes([vehicle, mode]) + struct.pack(">hH", millivolts_x10, err) + bytes(2)


def test_estop_state_decodes():
    """ESTOP is what phase_driver requires before it commands anything."""
    vehicle, mode, batt, err = decode_system_state(_state_frame(0x01, 0x01))
    assert vehicle == "ESTOP"
    assert mode == "CAN"
    assert batt == pytest.approx(48.0)
    assert err == 0


def test_rc_mode_is_three_not_two():
    """agilex_types.h:41 -- RC is 0x03. ranger_msgs disagrees and ranger_msgs is wrong;
    reading control_mode from there is how RC got misidentified on 2026-08-19."""
    assert decode_system_state(_state_frame(0x00, 0x03))[1] == "RC"
    assert decode_system_state(_state_frame(0x00, 0x02))[1] == "UART"


def test_normal_and_exception_states():
    assert decode_system_state(_state_frame(0x00, 0x01))[0] == "NORMAL"
    assert decode_system_state(_state_frame(0x02, 0x01))[0] == "EXCEPTION"


def test_unknown_state_is_reported_not_guessed():
    """An unmapped value must be visible, not silently coerced to a known one."""
    assert "UNKNOWN" in decode_system_state(_state_frame(0x07, 0x09))[0]
    assert "UNKNOWN" in decode_system_state(_state_frame(0x07, 0x09))[1]


def test_error_code_is_unsigned():
    """0xFFFF is 65535 of error bits, not -1."""
    assert decode_system_state(_state_frame(0x01, 0x01, err=0xFFFF))[3] == 0xFFFF


# --------------------------------------------------------------------------------------------
# malformed input -- fail loudly, never decode a truncated frame into a confident number
# --------------------------------------------------------------------------------------------

def test_short_motion_frame_raises():
    for n in range(8):
        with pytest.raises(ValueError):
            decode_motion(bytes(n))


def test_short_state_frame_raises():
    with pytest.raises(ValueError):
        decode_system_state(bytes(5))


def test_motion_str_shows_all_four_fields_with_units():
    """The operator reads this line to decide whether to trust the verdict, so every number
    on it must carry its unit -- the steering field especially, since it is neither radians
    nor degrees in raw form."""
    s = str(Motion(0.15, -0.4, 0.0, 1.2))
    for token in ("lin=+0.150m/s", "ang=-0.400rad/s", "lat=+0.000m/s", "steer=-12.0deg"):
        assert token in s, f"{token!r} missing from {s!r}"


def test_steer_raw_is_not_radians():
    """Raw carries (degrees / 10), sign-flipped vs ROS convention -- ranger_base.hpp:155/175.

    Measured on hardware 2026-08-21: a resting raw of -0.935 is +9.4 deg of steer. Read as
    radians it would be -53.6 deg: wrong magnitude by 5.7x and pointing the other way. That
    error is invisible because both readings are plausible wheel angles.
    """
    assert Motion(0, 0, 0, -0.935).steer_deg == pytest.approx(9.35)
    assert Motion(0, 0, 0, 1.2).steer_deg == pytest.approx(-12.0)
    assert Motion(0, 0, 0, 0.0).steer_deg == 0.0


def test_steer_zero_stays_zero_through_conversion():
    """The unit conversion must not disturb the PASS/FAIL decision, which is made on raw."""
    assert Motion(0, 0, 0, 0.0).is_zero()
    assert not Motion(0, 0, 0, 0.001).is_zero()
