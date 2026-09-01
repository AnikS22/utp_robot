"""RosWorld.reset() must clear EVERY per-trial field, not just the obvious ones.

WHY THIS TEST EXISTS. run_trial.py builds a fresh RosWorld per invocation, so stale per-trial
state was invisible. run_campaign.py reuses ONE world across N trials. When reset() left the
look-ladder cursor alone, trial 2 onward began with `_scan_i` already past the end of
SCAN_BEARINGS_DEG: the survey was inert, the reasoner abstained with no new viewpoint to reason
about, and 49 of 50 trials would have failed identically for a reason that has nothing to do with
the robot or the method under test. A whole campaign of invalid data that looks like a result.

The sim has the same reset for the same reason (IsaacWorldClient.reset clears _scan_i/_strafe_i).

This test reads the source rather than constructing a RosWorld, because constructing one needs
rclpy, a camera and a chassis. The contract being guarded is "reset() assigns every field that
__init__ assigns", and that is checkable statically and cheaply.
"""
from __future__ import annotations

import ast
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "bringup" / "ros_world.py"

# Fields that are configuration, not per-trial state: set once at construction and deliberately
# NOT cleared between trials. Everything else __init__ touches must be reset.
CONFIG_FIELDS = {
    "goal", "dry_run", "capture_prefix", "hints", "_ros_py", "_repo",
}


def _assigned_fields(fn: ast.FunctionDef) -> set[str]:
    out = set()
    for node in ast.walk(fn):
        if isinstance(node, ast.Assign):
            for tgt in node.targets:
                if (isinstance(tgt, ast.Attribute) and isinstance(tgt.value, ast.Name)
                        and tgt.value.id == "self"):
                    out.add(tgt.attr)
        elif isinstance(node, ast.AugAssign):
            tgt = node.target
            if (isinstance(tgt, ast.Attribute) and isinstance(tgt.value, ast.Name)
                    and tgt.value.id == "self"):
                out.add(tgt.attr)
    return out


def _methods(cls_name: str) -> dict[str, ast.FunctionDef]:
    tree = ast.parse(SRC.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == cls_name:
            return {f.name: f for f in node.body
                    if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef))}
    raise AssertionError(f"class {cls_name} not found in {SRC}")


def test_reset_clears_every_per_trial_field():
    m = _methods("RosWorld")
    assert "reset" in m, "RosWorld must implement reset(); the FSM calls it at trial start"
    init_fields = _assigned_fields(m["__init__"]) - CONFIG_FIELDS
    reset_fields = _assigned_fields(m["reset"])
    missed = sorted(init_fields - reset_fields)
    assert not missed, (
        "RosWorld.reset() does not clear: " + ", ".join(missed) +
        ". Every field __init__ sets is per-trial state unless it is listed in CONFIG_FIELDS. "
        "Stale state here silently corrupts every trial after the first in a multi-trial campaign."
    )


def test_look_ladder_cursor_is_reset_explicitly():
    """The specific field whose absence would have invalidated a 50-trial campaign."""
    reset_fields = _assigned_fields(_methods("RosWorld")["reset"])
    for field in ("_scan_i", "_scan_offset", "_widened", "_looks"):
        assert field in reset_fields, f"reset() must clear {field} — see this module's docstring"
