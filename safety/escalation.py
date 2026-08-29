"""When geometry runs out, should we ask the pipeline -- and may we act on what it says?

Pure decision logic, no ROS and no subprocesses, so every fail-closed branch is testable. Same
split as arbiter.py / twist_mux_node.py: the policy lives here, the plumbing lives in route_run.

THE IDEA. Driving is a geometric problem right up to the moment it stops being one. A gap the
robot fits through is arithmetic. A closed door is not -- no amount of steering opens it, and a
2D lidar cannot tell a shut door from a wall, because geometrically they ARE the same thing. So
the moment local avoidance reports no way around is the moment to stop reasoning about shapes and
ask what the obstruction MEANS.

What the answer may authorise is one PRE-WRITTEN, PRE-VALIDATED route, after which the leg is
retried. The VLM chooses between reviewed plans -- act, or stop. It never composes motion. A plan
assembled at runtime from model output cannot be reviewed before the run, and this robot weighs
50 kg.
"""
from __future__ import annotations

from dataclasses import dataclass

ACT = "act"        # run the branch, then retry the leg
STOP = "stop"      # hold position; a human decides


@dataclass(frozen=True)
class Decision:
    action: str
    message: str


def decide(check: dict | None, budget_left: int, budget_max: int, stuck_why: str) -> Decision:
    """Fail closed in every ambiguous case. Each branch below is a way of being wrong, and
    stopping is the only one of them that is cheap."""
    if budget_left <= 0:
        return Decision(STOP, f"{stuck_why} -- and the {budget_max} escalation(s) allowed are "
                              f"used up. Repeating an action that did not work is not a plan.")
    if check is None:
        return Decision(STOP, f"{stuck_why}; no answer from the blockage check at all.")
    if check.get("note"):
        # Unreachable, unparseable, no frame. Guessing in front of a glass door is the wrong
        # way to be wrong.
        return Decision(STOP, f"{stuck_why}; the blockage check failed closed "
                              f"({check['note']}): {check.get('description', '')}")
    if not check.get("blocked"):
        # The lidar says blocked, the camera says clear. That disagreement is INFORMATION --
        # glass, an obstacle under the scan plane, a mis-set lidar height would all look like
        # this -- but it is not a licence to drive. Picking whichever sensor suits us is how a
        # robot ends up in a door.
        return Decision(STOP, f"{stuck_why}; but the VLM says the way is CLEAR "
                              f"({check.get('description', '')!r}). The lidar and the camera "
                              f"disagree, so neither is acted on. Go and look.")
    return Decision(ACT, f"BLOCKED: {check.get('description', '')!r} "
                         f"(kind: {check.get('kind') or 'unclassified'})")
