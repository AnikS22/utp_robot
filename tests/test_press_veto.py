"""The arm must never press a fire alarm."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.press_veto import check, iou

# The real numbers from 2026-08-29: the chosen "door button" WAS the fire alarm.
FIRE_BOX = (84, 430, 137, 501)


def test_the_real_2026_08_29_frame_is_refused():
    ok, why = check(FIRE_BOX, [("a red fire alarm pull station", (83, 429, 137, 501), 0.44)])
    assert not ok
    assert "REFUSING TO PRESS" in why and "evacuation" in why


def test_a_fire_alarm_elsewhere_on_the_wall_is_not_a_veto():
    """Refusing every frame containing a fire alarm would refuse every corridor in the building."""
    ok, why = check((900, 400, 950, 460),
                    [("a red fire alarm pull station", FIRE_BOX, 0.44)])
    assert ok


def test_a_missing_check_fails_closed():
    assert not check(FIRE_BOX, None)[0]


def test_no_target_fails_closed():
    assert not check(None, [])[0]


def test_no_forbidden_hits_passes():
    ok, why = check(FIRE_BOX, [("a red fire alarm pull station", None, 0.0)])
    assert ok


def test_iou_basics():
    assert iou((0, 0, 10, 10), (0, 0, 10, 10)) == 1.0
    assert iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0
    assert 0.1 < iou((0, 0, 10, 10), (5, 5, 15, 15)) < 0.2


def test_partial_overlap_at_the_threshold_vetoes():
    """These boxes are small and two queries rarely frame identically, hence a low threshold."""
    ok, _ = check((84, 430, 137, 501), [("x", (90, 435, 140, 505), 0.4)])
    assert not ok
