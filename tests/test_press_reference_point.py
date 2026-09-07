"""The press must aim the GRIPPER TIP at the control, not the hand-eye calibration marker.

2026-09-07, floor-2 call button. approach_target.py placed `marker_on_flange_mm` at the standoff.
That point sits 96 mm BEHIND the flange face; the fingertip sits 172 mm in FRONT of it. At the
wrist orientation a press holds, those are 268 mm apart along the approach axis, so "stop with the
marker 30 mm from the button" commanded the fingertip to a point 238 mm inside the wall.

What the robot did: rammed. ControllerError 31 fired 293 mm short of the commanded pose with the
tip 26 mm past the wall plane, 71 mm lateral and 165 mm ABOVE the button. mission.sh reads error 31
as contact, so the route scored it a successful press -- of the wall, a handspan above the control.

Neither the detector nor the depth was at fault, which is why this is a test about arm geometry and
not about perception: gdino picked the right 28x32 px button at score 0.645, and its 0.583 m range
agreed with the lidar's 0.626 m at the same bearing to within the thickness of the plate.

The numbers below are the real ones from that run.
"""
import numpy as np
import pytest

MARKER_ON_FLANGE_MM = np.array([26.3577528015999, 0.5615060984229889, -95.36021958242466])
TOOL_TIP_MM = np.array([0.0, 0.0, 172.0])
PRESS_RPY_DEG = np.array([73.3, -88.4, 106.5])      # measured wrist orientation at the press
BUTTON_LINK_BASE_MM = np.array([577.9, -118.4, 370.6])
APPROACH = np.array([0.995, 0.04, -0.094])


def _R():
    from bringup.handeye_solve_rw import rpy_deg_to_R
    return rpy_deg_to_R(PRESS_RPY_DEG)


def _tip_error_mm(reference_offset_mm, standoff_mm):
    """Where the FINGERTIP ends up, relative to the button, for a given aim and standoff."""
    R = _R()
    flange = BUTTON_LINK_BASE_MM - APPROACH * standoff_mm - R @ reference_offset_mm
    return (flange + R @ TOOL_TIP_MM) - BUTTON_LINK_BASE_MM


def test_marker_and_tip_are_268_mm_apart_along_the_approach():
    """The bug's magnitude. If this shrinks, the tool or the marker moved and the press changed."""
    sep = float(np.dot(_R() @ (TOOL_TIP_MM - MARKER_ON_FLANGE_MM), APPROACH))
    assert sep == pytest.approx(268.0, abs=5.0)


def test_aiming_the_marker_drives_the_tip_through_the_wall():
    """The old behaviour, kept as evidence: a 30 mm marker standoff is 238 mm INTO the wall."""
    err = _tip_error_mm(MARKER_ON_FLANGE_MM, 30.0)
    assert float(np.dot(err, APPROACH)) == pytest.approx(238.0, abs=5.0)


def test_aiming_the_tip_lands_on_the_button():
    """The fix: the tip stops just past the target plane, and square to it.

    Past, not short of: a button has to be pushed, and contact (error 31) is the only evidence of a
    press this rig can produce. 25 mm also absorbs a ~20 mm error in the unverified 172 mm tool
    length in either direction -- docs/CALIBRATION.md item 2 is still open, and this fix does not
    depend on closing it because nothing is written to the controller's tcp_offset.
    """
    err = _tip_error_mm(TOOL_TIP_MM, -25.0)
    along = float(np.dot(err, APPROACH))
    off_axis = float(np.linalg.norm(err - along * APPROACH))
    assert along == pytest.approx(25.0, abs=3.0), "the tip must end just PAST the button plane"
    assert off_axis < 5.0, "and square to it -- this is what missed by 165 mm before"


def test_the_fix_survives_a_wrong_tool_length():
    """172 mm is a catalogue figure nobody has measured on this build. Be wrong by 20 mm, both ways."""
    for actual_tip in (152.0, 192.0):
        R = _R()
        flange = BUTTON_LINK_BASE_MM - APPROACH * (-25.0) - R @ TOOL_TIP_MM
        real_tip = flange + R @ np.array([0.0, 0.0, actual_tip])
        along = float(np.dot(real_tip - BUTTON_LINK_BASE_MM, APPROACH))
        assert along > 0.0, f"a {actual_tip:.0f} mm tool would stop short and never touch"
        assert along < 60.0, f"a {actual_tip:.0f} mm tool would drive too far past the plane"
