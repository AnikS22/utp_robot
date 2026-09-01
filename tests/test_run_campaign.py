"""End-to-end offline exercise of the campaign loop in bringup/run_campaign.py.

WHY. run_campaign.py is the script that will drive tomorrow's 50-trial session, and until now
nothing had executed a single line of its loop. Its imports are function-local, so the whole loop
can be run here against fakes: no rclpy, no camera, no chassis. What is exercised is exactly the
logic that decides whether a campaign produces usable data — anchor-relative drift, the stop
conditions, resume, per-trial capture naming, and the return-to-start call — none of which can be
checked by reading the file.

Every assertion below corresponds to a way a campaign can look successful and be invalid.
"""
from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "bringup"))


# --------------------------------------------------------------------------- fakes
class FakePose:
    def __init__(self, x, y, yaw): self.x, self.y, self.yaw = x, y, yaw


class FakeWorld:
    """Stands in for RosWorld. Records what the campaign asked of it."""

    def __init__(self, *_, **kw):
        self.capture_prefix = kw.get("capture_prefix", "fsm")
        self.prefixes_seen: list[str] = []
        self.poses: list[FakePose] = []          # queued return poses, popped per trial
        self._default = FakePose(0.0, 0.0, 0.0)

    def _pose(self):
        return self.poses.pop(0) if self.poses else self._default


class FakeRec(dict):
    """TrialRecord stand-in: the campaign treats a dict as its __dict__."""


def _install_fakes(monkeypatch, *, world, trial_results, goto_rc=0, calls=None):
    calls = calls if calls is not None else {}
    calls.setdefault("goto", [])
    calls.setdefault("trials", 0)

    mod_wp = types.ModuleType("waypoints")
    mod_wp.load = lambda: {"start": {"x": 0, "y": 0, "yaw": 0}}
    monkeypatch.setitem(sys.modules, "waypoints", mod_wp)

    mod_rw = types.ModuleType("ros_world")
    mod_rw.RosWorld = lambda *a, **k: world
    monkeypatch.setitem(sys.modules, "ros_world", mod_rw)

    mod_sr = types.ModuleType("steered_reasoner")
    mod_sr.LookHints = lambda *a, **k: object()
    mod_sr.SteeredReasoner = lambda *a, **k: object()
    monkeypatch.setitem(sys.modules, "steered_reasoner", mod_sr)

    class FakeCfg:
        def __init__(self): self.data = {"runtime": {}, "methods": {"vlm": {}}}
        def method(self, _name): return {"reasoning": "none", "label": "fake"}
    mod_cfg = types.ModuleType("utp.common.config"); mod_cfg.Config = types.SimpleNamespace(
        load=lambda _d=None: FakeCfg())
    mod_fsm = types.ModuleType("utp.pipeline.fsm")

    def _run_trial(cfg, w, mods, scene, seed, method):
        w.prefixes_seen.append(w.capture_prefix)
        calls["trials"] += 1
        r = trial_results[min(calls["trials"] - 1, len(trial_results) - 1)]
        return FakeRec(r)
    mod_fsm.run_trial = _run_trial
    mod_reg = types.ModuleType("utp.pipeline.registry")
    mod_reg.build_modules = lambda *a, **k: types.SimpleNamespace(reasoner=None)
    for name, mod in (("utp", types.ModuleType("utp")),
                      ("utp.common", types.ModuleType("utp.common")),
                      ("utp.pipeline", types.ModuleType("utp.pipeline")),
                      ("utp.common.config", mod_cfg), ("utp.pipeline.fsm", mod_fsm),
                      ("utp.pipeline.registry", mod_reg)):
        monkeypatch.setitem(sys.modules, name, mod)

    import run_campaign as rc
    monkeypatch.setattr(rc, "topic_alive", lambda *a, **k: True)
    monkeypatch.setattr(rc, "llm_reachable", lambda: True)

    class FakeProc:
        def __init__(self, rc_): self.returncode = rc_
    def fake_run(cmd, **kw):
        calls["goto"].append(list(cmd))
        return FakeProc(goto_rc)
    monkeypatch.setattr(rc.subprocess, "run", fake_run)
    monkeypatch.setattr(rc.time, "sleep", lambda *_: None)
    return rc, calls


def _argv(tmp_path, **over):
    a = {"--trials": "3", "--method": "passive", "--start": "start",
         "--out": str(tmp_path / "campaign.jsonl"), "--settle": "0",
         "--config": str(REPO / "config" / "pipeline")}
    a.update(over)
    out = ["run_campaign.py"]
    for k, v in a.items():
        out += [k, v] if v is not None else [k]
    return out


def _records(p: Path):
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]


# --------------------------------------------------------------------------- tests
def test_happy_path_writes_one_record_per_trial(monkeypatch, tmp_path):
    w = FakeWorld()
    rc, calls = _install_fakes(monkeypatch, world=w,
                               trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path))
    assert rc.main() == 0
    recs = _records(tmp_path / "campaign.jsonl")
    assert len(recs) == 3, "one record per trial, appended and fsync'd"
    assert [r["campaign_index"] for r in recs] == [1, 2, 3]
    assert all(r["hardware"] and r["world"] == "ros_hardware" for r in recs), \
        "a hardware trial must not be indistinguishable from a simulated one"


def test_each_trial_gets_its_own_capture_prefix(monkeypatch, tmp_path):
    """Frames are named {prefix}_{n:03d} and n resets per trial; a shared prefix silently
    overwrites earlier trials' evidence while every record still points at a real path."""
    w = FakeWorld()
    rc, _ = _install_fakes(monkeypatch, world=w,
                           trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path))
    rc.main()
    assert len(set(w.prefixes_seen)) == 3, f"prefixes collided: {w.prefixes_seen}"


def test_return_to_start_is_not_a_dry_run(monkeypatch, tmp_path):
    """`waypoints.py goto` prints what it WOULD do unless --go is passed. Without it the robot
    never returns, and every drift residual is measured against a robot that did not move."""
    w = FakeWorld()
    rc, calls = _install_fakes(monkeypatch, world=w,
                              trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path))
    rc.main()
    assert calls["goto"], "the campaign never tried to return to start"
    for cmd in calls["goto"]:
        assert "goto" in cmd and "--go" in cmd, f"return-to-start was a dry run: {cmd}"


def test_drift_is_measured_against_the_anchor_not_the_previous_trial(monkeypatch, tmp_path):
    """Anchor-relative is the whole point: 5 cm of creep per trial is invisible trial-to-trial and
    fatal by trial 20."""
    w = FakeWorld()
    # returns after trials 1,2,3 creep 0.05 m each time
    w.poses = [FakePose(0, 0, 0), FakePose(0.05, 0, 0),
               FakePose(0, 0, 0), FakePose(0.10, 0, 0),
               FakePose(0, 0, 0), FakePose(0.15, 0, 0)]
    rc, _ = _install_fakes(monkeypatch, world=w,
                           trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path))
    rc.main()
    drifts = [r["return_drift_m"] for r in _records(tmp_path / "campaign.jsonl")]
    assert drifts == pytest.approx([0.05, 0.10, 0.15], abs=1e-6), \
        f"drift must accumulate against the anchor, got {drifts}"


def test_campaign_stops_when_drift_exceeds_budget(monkeypatch, tmp_path):
    w = FakeWorld()
    w.poses = [FakePose(0, 0, 0), FakePose(0.05, 0, 0),     # trial 1: fine
               FakePose(0, 0, 0), FakePose(0.90, 0, 0)]     # trial 2: way past budget
    rc, _ = _install_fakes(monkeypatch, world=w,
                           trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path, **{"--trials": "5", "--max-drift": "0.30"}))
    rc.main()
    recs = _records(tmp_path / "campaign.jsonl")
    assert len(recs) == 2, "must stop as soon as the waypoint frame has moved, not finish the run"


def test_campaign_stops_on_collision(monkeypatch, tmp_path):
    w = FakeWorld()
    rc, _ = _install_fakes(monkeypatch, world=w, trial_results=[
        {"success": True, "failure_category": None},
        {"success": False, "failure_category": "execution", "collided": True},
        {"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path, **{"--trials": "5"}))
    rc.main()
    assert len(_records(tmp_path / "campaign.jsonl")) == 2, "a collision must halt the campaign"


def test_a_failed_trial_is_data_and_does_not_stop_the_campaign(monkeypatch, tmp_path):
    """The experiment is allowed to fail. Only things that make LATER trials unmeasurable stop it."""
    w = FakeWorld()
    rc, _ = _install_fakes(monkeypatch, world=w, trial_results=[
        {"success": False, "failure_category": "reasoning"}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path, **{"--trials": "4"}))
    rc.main()
    recs = _records(tmp_path / "campaign.jsonl")
    assert len(recs) == 4 and all(r["success"] is False for r in recs)


def test_resume_continues_and_does_not_duplicate(monkeypatch, tmp_path):
    out = tmp_path / "campaign.jsonl"
    w = FakeWorld()
    rc, _ = _install_fakes(monkeypatch, world=w,
                           trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path, **{"--trials": "2"}))
    rc.main()
    assert len(_records(out)) == 2
    w2 = FakeWorld()
    rc2, _ = _install_fakes(monkeypatch, world=w2,
                            trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path, **{"--trials": "5"}) + ["--resume"])
    rc2.main()
    recs = _records(out)
    assert len(recs) == 5, "resume must top up to --trials, not restart"
    assert [r["campaign_index"] for r in recs] == [1, 2, 3, 4, 5]


def test_dry_run_neither_returns_nor_measures_drift(monkeypatch, tmp_path):
    """--dry-run must not claim a drift number it did not measure."""
    w = FakeWorld()
    rc, calls = _install_fakes(monkeypatch, world=w,
                               trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path, **{"--trials": "2"}) + ["--dry-run"])
    rc.main()
    assert not calls["goto"], "a dry run must not drive the robot home"
    assert all(r["return_drift_m"] is None for r in _records(tmp_path / "campaign.jsonl"))


def test_missing_start_waypoint_fails_before_moving(monkeypatch, tmp_path):
    w = FakeWorld()
    rc, calls = _install_fakes(monkeypatch, world=w,
                               trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", _argv(tmp_path, **{"--start": "nonexistent"}))
    assert rc.main() == 2
    assert calls["trials"] == 0, "must refuse before running a single trial"


def test_pose_err_wraps_yaw():
    import run_campaign as rc
    d, dy = rc.pose_err((0, 0, 3.10), (0, 0, -3.10))
    assert d == pytest.approx(0.0)
    assert dy == pytest.approx(0.0834, abs=1e-3), "yaw error must wrap, not read as 6.2 rad"
