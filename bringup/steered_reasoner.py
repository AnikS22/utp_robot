"""The pipeline's VLM reasoner, with two hardware-only changes and nothing else.

Runs under the PIPELINE VENV. Subclasses utp.pipeline.reasoning.vlm_gpt5.GPT5Reasoner; the
pipeline repo is not modified.

1. THE SECOND LOOK IS JUDGED FRESH. After the first look-around the pipeline's prompt says "you
   reported no control you could operate in any of them". At temperature 0 the model held that
   position with the ADA plate dead centre of the frame (trial 4, 2026-08-29: "I cannot see any
   button" -- and, asked cold on the same image, "a silver ADA push-button plate visible on the
   wall near the door"). The sentence is rewritten as a neutral fact plus "judge THIS image on its
   own". The pipeline's exhausted-survey wording, which is what makes negative controls refuse, is
   left exactly as it is.

2. THE REASONER MAY SAY WHERE TO LOOK NEXT. When it abstains because the control is not visible,
   it may add params={"look": left|right|closer|back}. That is read by RosWorld.strafe_view and
   turned into one bounded motion (safety/look_policy.py). No hint -> the blind sweep, as before.
   Heuristic and passive reasoners never emit hints, so they get the sweep the sim gives them.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PIPELINE = Path(os.environ.get("UTP_PIPELINE_REPO", Path.home() / "unlocking-the-path"))
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(PIPELINE))

from safety.look_policy import STEER_INSTRUCTION, neutralise_survey, normalise_hint  # noqa: E402
from utp.pipeline.reasoning.vlm_gpt5 import GPT5Reasoner  # noqa: E402


class LookHints:
    """One slot the reasoner writes and the world reads. Cleared on every plan() and reset()."""

    def __init__(self) -> None:
        self.hint: str | None = None
        self.log: list = []

    def set(self, raw) -> None:
        self.hint = normalise_hint(raw)
        self.log.append(self.hint)

    def take(self) -> str | None:
        h, self.hint = self.hint, None
        return h

    def clear(self) -> None:
        self.hint = None


class SteeredReasoner(GPT5Reasoner):
    def __init__(self, vlm_cfg: dict, hints: LookHints) -> None:
        super().__init__(vlm_cfg)
        self.hints = hints

    def _user_text(self, blockage, done, failed=()) -> str:
        text = super()._user_text(blockage, done, failed)
        text = neutralise_survey(text)
        # Insert before the closing "Look at the attached camera image" line so the model reads
        # the instruction with the rest of the task, not after it.
        tail = "Look at the attached camera image to identify the correct target object."
        if tail in text:
            return text.replace(tail, STEER_INSTRUCTION + "\n" + tail)
        return text + "\n" + STEER_INSTRUCTION

    def plan(self, obs, blockage, history):
        p = super().plan(obs, blockage, history)
        if p.abstain and p.action_type == "none":
            self.hints.set((p.params or {}).get("look"))
        else:
            self.hints.clear()
        return p

    def reset(self) -> None:
        super().reset()
        self.hints.clear()
        self.hints.log.clear()

    def forget_blockage_evidence(self) -> None:
        super().forget_blockage_evidence()
        self.hints.clear()
