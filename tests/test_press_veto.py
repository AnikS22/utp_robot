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


# ---------------------------------------------------------------------------------------------
# 2026-09-11, floor 1 of the new building, the ADA door plate. Both frames are REAL: same robot,
# same wall, same four forbidden queries, ~60 s apart. Between them the arm was moved to its ready
# pose, which put it in front of the mast camera and changed which object won the target query.
# Together they are the tightest discriminator this guard has: the same detector, on the same
# wall, labelling the alarm and the plate.

ADA_PLATE = (645.7, 461.0, 813.8, 629.3)   # square push plate, 168x168 px
ADA_ALARM = (446.3, 446.4, 488.0, 543.9)   # the real pull station, narrow, ~200 px to the left


def test_the_real_2026_09_11_ada_plate_is_pressable():
    """One query confused a square plate for an emergency stop, at LOWER confidence than the
    target. Three others correctly placed the alarm 200 px away. That must not veto.

    The old rule refused this frame twice; the press only happened after the operator confirmed
    the plate by eye and authorised a bypass."""
    hits = [("a red fire alarm pull station", (445.88, 444.62, 489.23, 572.82), 0.527),
            ("a fire alarm activation lever", (446.58, 445.88, 487.94, 572.58), 0.524),
            ("an emergency stop button",      (645.62, 459.94, 813.95, 630.18), 0.593),
            ("a red emergency call button",   (447.06, 444.90, 488.92, 572.52), 0.574)]
    ok, why = check(ADA_PLATE, hits, target_score=0.6435)
    assert ok, why


def test_the_real_2026_09_11_alarm_pick_is_still_refused():
    """Minutes earlier, with the arm occluding the plate, the grounder returned the ALARM itself
    as 'the accessible door push button' at 0.4034. Three forbidden queries sat on it above the
    confidence floor. This is the failure the file exists for and must still veto."""
    hits = [("a red fire alarm pull station", (446.04, 445.86, 488.57, 544.34), 0.552),
            ("a fire alarm activation lever", (447.17, 446.67, 487.28, 544.08), 0.386),
            ("an emergency stop button",      (447.01, 445.16, 488.50, 543.77), 0.238),
            ("a red emergency call button",   (447.03, 445.24, 488.17, 543.42), 0.593)]
    ok, why = check(ADA_ALARM, hits, target_score=0.4034)
    assert not ok and "3 of 4" in why, why


def test_a_lone_forbidden_hit_more_confident_than_the_target_still_vetoes():
    """The relaxation is bounded: one vote still vetoes when the detector is MORE sure the thing
    is an alarm than that it is the control that was asked for."""
    target = (100, 100, 150, 150)
    hits = [("a red fire alarm pull station", target, 0.70),
            ("an emergency stop button", (500, 500, 540, 540), 0.30)]
    ok, why = check(target, hits, target_score=0.50)
    assert not ok, why
