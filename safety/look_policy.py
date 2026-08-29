"""Turn a reasoner's "look" hint into one bounded base motion. Pure logic, no ROS.

WHY THE REASONER STEERS THE LOOK-AROUND ON HARDWARE.

The sim's look-around is deliberately BLIND: a fixed, symmetric arc of bearings, so that "the
sweep does not know, and must not know, the target's true bearing, or the look-around would be
answering the question the benchmark is asking" (isaac_world.scan_view). That is the right rule
for a simulator that knows ground truth and must not leak it.

A hint from the VLM is not ground truth. It is the model looking at a picture of a door and
inferring where a wall-mounted control would be -- "the door is on my left, so its ADA plate is on
the wall to my left" -- which is exactly the semantic reasoning the paper claims a VLM brings and a
heuristic cannot. So on hardware the reasoner may say where to look next, and the world carries
that out. Methods whose reasoner emits no hint (heuristic, passive) fall through to the same blind
sweep as before, so the comparison stays about the reasoner.

WHAT THE HINT MAY NOT DO. It never names a target, never supplies coordinates, and never chooses
an action. It picks one of four bounded motions, each capped, and the FSM's recovery budget still
counts every one. The reasoner asks for a different picture; it does not get to move the robot
anywhere it likes.

MEASURED, 2026-08-29, FAU atrium doors. Four FSM trials abstained. On the fourth, the blind sweep
reached +80 deg with the ADA plate dead centre of the frame -- and the reasoner said it could see
no button. Asked cold on that same image it answered press_button, "a silver ADA push-button
plate visible on the wall near the door". The difference was one prompt line injected after the
first look: "you reported no control you could operate in any of them", which at temperature 0
the model stayed consistent with. The steered look and the neutral survey wording below exist
together because of that trial.
"""
from __future__ import annotations

from dataclasses import dataclass

TURN_STEP_DEG = 60.0        # one "left"/"right" is this much; two in a row reach the flank wall
TURN_CAP_DEG = 130.0        # never accumulate past this from the approach heading
CLOSER_STOP_M = 0.85        # "closer" drives to this range from what is ahead ...
CLOSER_MIN_ROOM_M = 1.00    # ... and is refused if something is already nearer than this
BACK_M = 0.35               # "back" is the sim's widen_view delta

HINTS = ("left", "right", "closer", "back")


@dataclass(frozen=True)
class LookMove:
    kind: str                # "turn" | "closer" | "back"
    amount: float            # degrees for turn (+ = left), metres otherwise
    why: str


def normalise_hint(raw) -> str | None:
    """Accept the model's phrasing loosely, return one of HINTS or None."""
    if not raw:
        return None
    s = str(raw).strip().lower()
    # Earliest-mentioned hint wins, so "look left" and "closer to the wall" both resolve and a
    # sentence naming two directions is read in the order the model wrote it.
    found = [(s.find(h), h) for h in HINTS if h in s]
    if found:
        return min(found)[1]
    if s in ("l", "turn left", "to the left"):
        return "left"
    if s in ("r", "turn right", "to the right"):
        return "right"
    if s in ("forward", "nearer", "approach", "zoom"):
        return "closer"
    if s in ("backwards", "reverse", "away", "wider"):
        return "back"
    return None


def decide_look(hint, offset_deg: float, nearest_ahead_m: float | None) -> LookMove | None:
    """One bounded motion for this hint from this state, or None if it cannot be honoured.

    ``offset_deg`` is how far the robot has already turned from its approach heading (+ = left).
    ``nearest_ahead_m`` is the lidar's nearest return straight ahead, or None if nothing resolves.
    """
    h = normalise_hint(hint)
    if h is None:
        return None

    if h in ("left", "right"):
        sign = 1.0 if h == "left" else -1.0
        room = TURN_CAP_DEG - sign * offset_deg
        if room < 5.0:
            return None          # already at the cap on that side; let the blind sweep decide
        deg = sign * min(TURN_STEP_DEG, room)
        return LookMove("turn", deg, f"reasoner asked to look {h}")

    if h == "closer":
        # A control that is visible but too small to identify is the case this serves -- the
        # plate was ~50 px at the 1.4 m survey standoff and ~85 px at 1.0 m, and the difference
        # was found-at-0.489 versus not-in-the-top-five. Refused when there is no room: closing
        # on a wall already inside a metre only crops the target, it does not enlarge it.
        if nearest_ahead_m is not None and nearest_ahead_m < CLOSER_MIN_ROOM_M:
            return None
        return LookMove("closer", CLOSER_STOP_M, "reasoner asked to look closer")

    if h == "back":
        return LookMove("back", BACK_M, "reasoner asked for a wider view")
    return None


# The sentence in the pipeline's survey block that anchored the model. It is factual and was
# written to be neutral; at temperature 0 the model treats "you reported no control" as a position
# to hold, and keeps holding it with the control in frame.
ANCHOR = "and you reported no control you could operate in any of them."
NEUTRAL = ("No operable control was visible from those earlier viewpoints. The attached image is "
           "a NEW viewpoint: judge it on its own, and if a control you can operate is visible in "
           "THIS image, name it precisely and act on it.")

STEER_INSTRUCTION = (
    "If you answer action_type='none' because the control is not visible yet, ALSO set "
    "params={\"look\": <one of \"left\", \"right\", \"closer\", \"back\">} to say where the "
    "robot should look next, from what you can see: e.g. the door is on the left of the image, so "
    "a wall-mounted button beside it is probably to the LEFT; a control is visible but too small "
    "to identify -> \"closer\"; the blockage does not fit in the frame -> \"back\". "
    "Do not name a target you cannot see. Give one word only.")


def neutralise_survey(user_text: str) -> str:
    """Rewrite the pipeline's survey sentence so a second look is judged fresh."""
    if ANCHOR in user_text:
        user_text = user_text.replace(ANCHOR, NEUTRAL)
    return user_text
