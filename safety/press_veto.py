"""Never press a fire alarm.

Pure logic, no torch and no ROS, so the decision is testable without a GPU.

WHY THIS EXISTS. 2026-08-29, robot parked at the FAU atrium doors. Asked for "the accessible
door push button", the shipped grounder returned the red FIRE ALARM PULL STATION on the left
wall, score 0.441, ranked first over four alternatives. Re-asked as "the square metal ADA
wheelchair door activation plate" (0.296) and "the blue and white handicap accessible entry
button" (0.332) it returned THE SAME BOX both times. The ADA plate was simply not in frame, and
the detector anchored on the most button-like object that was.

Nothing downstream would have caught it. The 3D lift was valid, the confidence was the highest of
the run, the box passed the small-box preference, and face_target would have driven the base to
it and press_run would have pressed it. A grounding error here is not a missed trial; it is a
building evacuation.

THE SHAPE OF THE GUARD. Low confidence is NOT the signal -- 0.441 was the most confident
detection of the session. The signal is WHAT THE THING IS. So the same detector is asked, on the
same frame, what a fire alarm looks like; if its answer lands on the box we are about to press,
we do not press it. That is a second opinion from the component that was wrong, which is weaker
than an independent sensor -- but it is asking a DIFFERENT question, and the failure mode here is
a target the detector confidently mislabels, not one it cannot see.

Fail closed: if the check cannot run, nothing is pressed.
"""
from __future__ import annotations

# Things that must never be operated by a robot arm, whatever the reasoner asked for. Phrased as
# detector queries, one per line, deliberately over-broad -- a false refusal costs a trial and a
# false pass costs a fire brigade.
FORBIDDEN = (
    "a red fire alarm pull station",
    "a fire alarm activation lever",
    "an emergency stop button",
    "a red emergency call button",
)

# Box overlap above which the forbidden thing and the intended target are treated as the SAME
# object. Deliberately low: these boxes are small and the two queries rarely frame identically.
IOU_VETO = 0.30


def iou(a, b) -> float:
    """Intersection over union of two (x0, y0, x1, y1) boxes."""
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    ix0, iy0 = max(ax0, bx0), max(ay0, by0)
    ix1, iy1 = min(ax1, bx1), min(ay1, by1)
    iw, ih = max(0.0, ix1 - ix0), max(0.0, iy1 - iy0)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0.0, ax1 - ax0) * max(0.0, ay1 - ay0)
    area_b = max(0.0, bx1 - bx0) * max(0.0, by1 - by0)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def check(target_bbox, forbidden_hits, *, iou_veto: float = IOU_VETO) -> tuple[bool, str]:
    """May we press ``target_bbox``?

    ``forbidden_hits`` is [(query, bbox, score), ...] -- what the detector found when asked about
    each forbidden thing on the SAME frame. A hit is only disqualifying if it lands on the box we
    intend to press; a fire alarm elsewhere on the wall is not a reason to refuse.
    """
    if target_bbox is None:
        return False, "no target box to check; refusing"
    if forbidden_hits is None:
        return False, "the forbidden-target check did not run; refusing to press"
    worst = None
    for q, bbox, score in forbidden_hits:
        if bbox is None:
            continue
        o = iou(target_bbox, bbox)
        if worst is None or o > worst[0]:
            worst = (o, q, score)
    if worst and worst[0] >= iou_veto:
        o, q, score = worst
        return False, (f"REFUSING TO PRESS: the target overlaps {o:.0%} with what the detector "
                       f"identifies as {q!r} (score {score:.3f}). Pressing a fire alarm is not a "
                       f"failed trial, it is an evacuation. Reposition so the real control is in "
                       f"frame, or press it by hand.")
    return True, (f"clear (worst forbidden overlap {worst[0]:.0%})" if worst else "clear")
