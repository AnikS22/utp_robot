"""The reasoner may say where to look; it may not move the robot anywhere it likes."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.look_policy import (ANCHOR, NEUTRAL, TURN_CAP_DEG, TURN_STEP_DEG, decide_look,
                                neutralise_survey, normalise_hint)


def test_left_and_right_are_one_bounded_step():
    assert decide_look("left", 0.0, 1.4).amount == TURN_STEP_DEG
    assert decide_look("right", 0.0, 1.4).amount == -TURN_STEP_DEG


def test_turns_accumulate_only_to_the_cap():
    """Two lefts reach the flank wall (120 deg); a third is clipped to the cap, a fourth refused."""
    m = decide_look("left", 120.0, 1.4)
    assert m is not None and abs(m.amount - (TURN_CAP_DEG - 120.0)) < 1e-9
    assert decide_look("left", TURN_CAP_DEG, 1.4) is None


def test_closer_is_refused_when_a_wall_is_already_near():
    """From 0.54 m the plate was half out of frame; closing further only crops it more."""
    assert decide_look("closer", 0.0, 0.54) is None
    assert decide_look("closer", 0.0, 1.38) is not None
    assert decide_look("closer", 0.0, None) is not None      # nothing resolvable ahead: allowed


def test_unknown_or_missing_hint_yields_nothing():
    """No hint -> the FSM falls through to the blind sweep, exactly as the sim does."""
    assert decide_look(None, 0.0, 1.4) is None
    assert decide_look("", 0.0, 1.4) is None
    assert decide_look("upwards", 0.0, 1.4) is None


def test_loose_phrasings_are_accepted():
    assert normalise_hint("Look LEFT") == "left"
    assert normalise_hint("closer to the wall") == "closer"
    assert normalise_hint("reverse") == "back"


def test_the_anchoring_sentence_is_neutralised():
    """The sentence that held the model to 'no control' with the plate in frame, 2026-08-29."""
    text = "SURVEY ALREADY PERFORMED AT THIS BLOCKAGE: ... viewpoint(s), " + ANCHOR
    out = neutralise_survey(text)
    assert ANCHOR not in out and NEUTRAL in out


def test_text_without_the_anchor_is_untouched():
    assert neutralise_survey("Navigation is blocked.") == "Navigation is blocked."
