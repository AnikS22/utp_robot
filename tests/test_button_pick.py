"""Pick the right button on a lift panel, in the dark.

Numbers are from the real frame captures/press_190628 (2026-09-11, floor-5 car). The panel reads
6/5/4/3/2/1 top to bottom; the robot pressed 5 and rode to the wrong floor.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "bringup"))
from detect_frame import dedupe, pick_from_column  # noqa: E402

HW = (720, 1280)

# Exactly what the detector returned. Three of these frame the SAME button ("6", y~511).
CAR_PANEL = [
    {"bbox": [534, 491, 566, 530], "score": 0.339},   # "6"
    {"bbox": [543, 497, 561, 522], "score": 0.261},   # "6" again, smaller
    {"bbox": [524, 484, 573, 537], "score": 0.198},   # "6" again, larger
    {"bbox": [540, 532, 559, 559], "score": 0.208},   # "5"
    {"bbox": [524, 495, 584, 709], "score": 0.266},   # the whole button strip
]
# Where the buttons actually are, measured off the frame.
ACTUAL_Y = {6: 511, 5: 546, 4: 580, 3: 618, 2: 655, 1: 690}


def test_duplicate_boxes_of_one_button_collapse():
    assert len(dedupe(CAR_PANEL[:3])) == 1


def test_the_real_frame_picks_floor_1_not_floor_5():
    """The bug: 'button 1 from the bottom' resolved to 5 because the column was 2 buttons long."""
    bbox, note = pick_from_column(CAR_PANEL, 1, image_hw=HW, expect_buttons=6)
    assert bbox is not None, note
    cy = (bbox[1] + bbox[3]) / 2
    assert abs(cy - ACTUAL_Y[1]) <= 6, f"aimed at y={cy:.0f}, button 1 is at {ACTUAL_Y[1]} ({note})"
    assert abs(cy - ACTUAL_Y[5]) > 100, "still aiming at floor 5"


def test_every_floor_lands_on_its_own_button():
    for floor, y in ACTUAL_Y.items():
        idx = floor          # this panel's lowest button is 1, so index from bottom == floor
        bbox, note = pick_from_column(CAR_PANEL, idx, image_hw=HW, expect_buttons=6)
        assert bbox is not None, note
        cy = (bbox[1] + bbox[3]) / 2
        assert abs(cy - y) <= 6, f"floor {floor}: aimed y={cy:.0f}, actual {y} ({note})"


def test_a_short_column_without_a_strip_refuses_rather_than_guessing():
    """No strip to divide and an incomplete column must REFUSE, not index into what it has."""
    bbox, note = pick_from_column(CAR_PANEL[:4], 1, image_hw=HW, expect_buttons=6)
    assert bbox is None
    assert "REFUSING" in note


def test_a_complete_column_still_uses_the_detections_directly():
    """When the detector does resolve every button, nothing is synthesised."""
    cands = [{"bbox": [540, y - 13, 559, y + 13], "score": 0.3} for y in ACTUAL_Y.values()]
    bbox, note = pick_from_column(cands, 1, image_hw=HW, expect_buttons=6)
    assert bbox is not None and "6-button column" in note
    assert abs((bbox[1] + bbox[3]) / 2 - ACTUAL_Y[1]) <= 2


def test_without_expect_buttons_the_old_behaviour_is_unchanged():
    """The call plate passes no count; it must still pick the lowest of what it found."""
    two = [{"bbox": [604, 620, 633, 650], "score": 0.446},    # UP
           {"bbox": [604, 672, 633, 702], "score": 0.296}]    # DOWN
    bbox, note = pick_from_column(two, 1, image_hw=HW)
    assert bbox is not None and abs((bbox[1] + bbox[3]) / 2 - 687) <= 2, note
