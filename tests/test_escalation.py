"""Escalating to the pipeline: act only on an unambiguous, budgeted answer."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.escalation import ACT, STOP, decide

WHY = "STUCK: no way around"


def test_a_recognised_blockage_authorises_the_branch():
    d = decide({"blocked": True, "kind": "door", "description": "a closed door"}, 2, 2, WHY)
    assert d.action == ACT and "door" in d.message


def test_no_answer_at_all_stops():
    assert decide(None, 2, 2, WHY).action == STOP


def test_unreachable_vlm_stops():
    """Guessing in front of a glass door is the wrong way to be wrong."""
    d = decide({"blocked": True, "note": "no venv", "description": "..."}, 2, 2, WHY)
    assert d.action == STOP and "failed closed" in d.message


def test_sensors_disagreeing_stops_and_says_so():
    """Lidar blocked, camera clear. Acting on whichever suits us is how a robot enters a door."""
    d = decide({"blocked": False, "description": "clear corridor"}, 2, 2, WHY)
    assert d.action == STOP
    assert "disagree" in d.message and "CLEAR" in d.message


def test_budget_exhaustion_stops():
    """Without a cap this is press-retry-press-retry, performed by a machine that is moving."""
    d = decide({"blocked": True, "kind": "door", "description": "still shut"}, 0, 2, WHY)
    assert d.action == STOP and "used up" in d.message


def test_the_stuck_reason_travels_into_every_refusal():
    for check in (None, {"blocked": True, "note": "x"}, {"blocked": False}):
        assert WHY in decide(check, 2, 2, WHY).message
