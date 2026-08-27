"""Pure mission sequencing: a route is waypoints and actions in order. No ROS, no hardware.

THE PRINCIPLE THIS ENCODES: waypoints are APPROXIMATE, actions are VISUAL. A leg only has to park
the robot so the target is in the camera frame -- roughly +-0.3 m and +-15 deg. Everything that
actually touches the world is closed by the grounder and the visual servo, which repeated to 3 mm
across four runs on 2026-08-25. That is why odometry-only driving is sound here and not a
shortcut: the drift is absorbed downstream, by design, at the only place accuracy matters.

WHY VALIDATION IS THE POINT OF THIS MODULE. A route is a list of names. A typo in a waypoint name
is invisible until the robot has driven six legs and refuses the seventh, in a corridor, with an
arm on top. So the whole route is checked against the known waypoints and actions BEFORE anything
moves, and the check is a pure function of data -- no robot required to run it, and it is tested.
"""
from __future__ import annotations

from dataclasses import dataclass, field

GOTO, ACTION, WAIT, CHECK = "goto", "action", "wait", "check"
KINDS = (GOTO, ACTION, WAIT, CHECK)
CHECKS = ("blockage",)
MAX_WAIT_S = 300.0


@dataclass(frozen=True)
class Step:
    kind: str
    name: str = ""
    params: dict = field(default_factory=dict)

    def describe(self) -> str:
        if self.kind == GOTO:
            return f"drive to '{self.name}'"
        if self.kind == ACTION:
            q = self.params.get("query")
            return f"action '{self.name}'" + (f" on \"{q}\"" if q else "")
        if self.kind == CHECK:
            return (f"check '{self.name}'; if blocked, run route "
                    f"'{self.params.get('if_blocked', '?')}' then continue")
        return f"wait {self.params.get('seconds', 0):.1f}s"


def parse_route(spec) -> list[Step]:
    """Turn the YAML form into Steps. Raises ValueError with the offending index."""
    if not isinstance(spec, list):
        raise ValueError("a route must be a list of steps")
    out: list[Step] = []
    for i, raw in enumerate(spec):
        if not isinstance(raw, dict) or not raw:
            raise ValueError(f"step {i}: expected a mapping like {{goto: name}}, got {raw!r}")
        kind = next((k for k in KINDS if k in raw), None)
        if kind is None:
            raise ValueError(f"step {i}: no known key in {sorted(raw)}; expected one of {KINDS}")
        value = raw[kind]
        params = {k: v for k, v in raw.items() if k != kind}
        if kind == WAIT:
            try:
                params["seconds"] = float(value)
            except (TypeError, ValueError):
                raise ValueError(f"step {i}: wait needs a number, got {value!r}")
            value = ""
        out.append(Step(kind, str(value), params))
    return out


def validate_route(steps, known_waypoints, known_actions, subroutes=None) -> list[str]:
    """Every reason this route cannot run, all at once.

    ALL of them, not the first: fixing a route one error per run means one drive per typo.

    `subroutes` maps route name -> parsed Steps, for `check` steps to branch into. A branch is
    validated here too, with the SAME waypoint/action sets: a typo inside `if_blocked` would
    otherwise surface only when the robot is already parked at a closed door.
    """
    errs: list[str] = []
    subroutes = subroutes or {}
    if not steps:
        errs.append("route is empty")
    for i, s in enumerate(steps):
        where = f"step {i} ({s.describe()})"
        if s.kind == GOTO:
            if s.name not in known_waypoints:
                near = [w for w in known_waypoints if w.startswith(s.name[:3])] if s.name else []
                hint = f"; did you mean {near}?" if near else f"; known: {sorted(known_waypoints)}"
                errs.append(f"{where}: unknown waypoint '{s.name}'{hint}")
        elif s.kind == ACTION:
            if s.name not in known_actions:
                errs.append(f"{where}: unknown action '{s.name}'; "
                            f"known: {sorted(known_actions)}")
        elif s.kind == CHECK:
            if s.name not in CHECKS:
                errs.append(f"{where}: unknown check '{s.name}'; known: {sorted(CHECKS)}")
            sub = s.params.get("if_blocked")
            if not sub:
                errs.append(f"{where}: a check needs 'if_blocked: <route>' -- the steps to run "
                            f"when the way is blocked")
            elif sub not in subroutes:
                errs.append(f"{where}: if_blocked route '{sub}' not found; "
                            f"known: {sorted(subroutes)}")
            else:
                if any(t.kind == CHECK for t in subroutes[sub]):
                    errs.append(f"{where}: route '{sub}' contains a check itself. One level of "
                                f"branching only -- a robot re-deciding inside a decision is "
                                f"unreviewable before the run.")
                errs.extend(f"{where}, inside '{sub}': {e}"
                            for e in validate_route(subroutes[sub], known_waypoints,
                                                    known_actions))
        elif s.kind == WAIT:
            w = s.params.get("seconds", 0.0)
            if not (w == w) or w < 0:
                errs.append(f"{where}: wait must be a non-negative number, got {w!r}")
            elif w > MAX_WAIT_S:
                errs.append(f"{where}: wait of {w:.0f}s exceeds {MAX_WAIT_S:.0f}s. A robot "
                            f"parked in a corridor for that long needs a person, not a timer.")
    return errs


@dataclass
class RouteState:
    """Where we are in the route. Advancing is explicit -- nothing auto-advances on a failure."""
    steps: list[Step]
    index: int = 0
    done: bool = False
    failed_reason: str = ""

    @property
    def current(self) -> Step | None:
        if self.done or self.index >= len(self.steps):
            return None
        return self.steps[self.index]

    def advance(self) -> None:
        self.index += 1
        if self.index >= len(self.steps):
            self.done = True

    def splice(self, sub_steps) -> None:
        """Insert a branch's steps right after the current one. advance() then walks into it.

        Used by a `check` step whose condition fired: the branch becomes part of THIS route, so
        progress(), failure handling and Ctrl-C all keep working with no special cases.
        """
        k = self.index + 1
        self.steps = self.steps[:k] + list(sub_steps) + self.steps[k:]

    def fail(self, reason: str) -> None:
        """Stop the route where it is. The robot holds position; a human decides what next.

        Deliberately NOT a retry or a skip. A leg that failed did so because the world did not
        match the recorded route -- something moved, a door was shut, odometry drifted past the
        corridor check. Continuing to the next waypoint on a stale pose estimate is how a robot
        ends up somewhere nobody chose.
        """
        self.failed_reason = reason
        self.done = True

    def progress(self) -> str:
        n = len(self.steps)
        if self.failed_reason:
            return f"FAILED at step {self.index}/{n}: {self.failed_reason}"
        if self.done:
            return f"complete ({n}/{n})"
        return f"step {self.index + 1}/{n}: {self.steps[self.index].describe()}"
