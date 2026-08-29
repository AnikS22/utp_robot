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


def test_one_confused_query_does_not_veto_the_real_plate():
    """The 2026-08-29 false positive, real boxes: target = the ADA plate at (118,408)-(220,500).

    Three forbidden queries found the actual alarm 18 cm to the right at higher confidence; one
    matched 'lever' to the round plate at 0.429. The old rule refused on that one."""
    plate = (118, 408, 220, 500)
    alarm = (297, 423, 362, 504)
    hits = [("a red fire alarm pull station", alarm, 0.470),
            ("a fire alarm activation lever", plate, 0.429),
            ("an emergency stop button", alarm, 0.539),
            ("a red emergency call button", alarm, 0.604)]
    ok, why = check(plate, hits, target_score=0.526)
    assert ok, why


def test_a_red_door_button_is_not_an_alarm_at_low_confidence():
    """Sim, 2026-08-29: the red door release button scored 0.586 as the target; every alarm query
    also landed on it, all at ~0.38. Three agreeing low-confidence votes must not out-vote a
    confident target -- a forbidden hit counts only at >= 90% of the target's own score."""
    btn = (400, 300, 500, 400)
    hits = [("a red fire alarm pull station", btn, 0.379),
            ("a fire alarm activation lever", btn, 0.36),
            ("an emergency stop button", btn, 0.38),
            ("a red emergency call button", None, 0.0)]
    ok, why = check(btn, hits, target_score=0.586)
    assert ok, why


def test_the_real_alarm_pick_is_still_refused_under_the_new_rule():
    """Same day, earlier: the grounder RETURNED the alarm as the door button. Two forbidden
    queries sat on it at 96%; two others found a strobe elsewhere. Must still veto."""
    target = (84, 430, 137, 501)
    strobe = (84, 44, 143, 119)
    hits = [("a red fire alarm pull station", strobe, 0.575),
            ("a fire alarm activation lever", strobe, 0.571),
            ("an emergency stop button", (83, 429, 137, 501), 0.508),
            ("a red emergency call button", (84, 430, 137, 501), 0.549)]
    ok, why = check(target, hits, target_score=0.441)
    assert not ok and "2 of 4" in why


def test_the_single_most_confident_forbidden_hit_on_the_target_vetoes_alone():
    """One vote is enough when it is the strongest forbidden evidence in the frame."""
    target = (100, 100, 150, 150)
    hits = [("a red fire alarm pull station", target, 0.70),
            ("an emergency stop button", (500, 500, 540, 540), 0.30)]
    assert not check(target, hits)[0]


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
