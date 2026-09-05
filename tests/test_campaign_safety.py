"""The campaign interlock: bringup/run_campaign.py must refuse to start an unsafe campaign.

WHY. config/safety.yaml's mux is the only publisher of /cmd_vel, and `requires_enable: true` on
the autonomous sources (`nav`, `servo`) is what makes a lost commander a bounded event -- the
deadman stops publishing /safety/enable and the arbiter drops every autonomous command 0.2 s
later. On 2026-09-01 the operator stood that gate down for `nav`, deliberately and for a good
reason (a browser deadman costs the hand that would otherwise be on the chassis E-stop), and
wrote in the config that it MUST be restored before any campaign run. Nothing enforced that. A
comment is not an interlock, and run_campaign.py would have executed 50 autonomous trials with
the gate down and nobody holding anything.

HOW THIS IS TESTED. The same way tests/test_run_campaign.py does it: the real main() runs against
injected fakes -- no rclpy, no chassis, no camera -- so what is exercised is the actual refusal
path an operator would hit, not the text of the source file. Grepping the source for a call to a
check would pass just as happily against a check that is never reached, or that is skipped under
--dry-run, and those are precisely the bugs worth catching here.

Every safety.yaml below is synthetic and lives in tmp_path. The repo's live config/safety.yaml is
deliberately NOT consulted: its contents are the thing under dispute and they change day to day,
so a test that read it would flip between passing and failing for reasons that have nothing to do
with this code.
"""
from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "bringup"))


# --------------------------------------------------------------------------- fakes
# Deliberately a local copy of test_run_campaign.py's harness rather than an import of it: these
# two files must be able to fail independently, and a safety test that breaks because someone
# refactored another test module is a safety test people start ignoring.
class FakePose:
    def __init__(self, x, y, yaw): self.x, self.y, self.yaw = x, y, yaw


class FakeWorld:
    def __init__(self, *_, **kw):
        self.capture_prefix = kw.get("capture_prefix", "fsm")
        self.prefixes_seen: list[str] = []
        self.poses: list[FakePose] = []
        self._default = FakePose(0.0, 0.0, 0.0)

    def _pose(self):
        return self.poses.pop(0) if self.poses else self._default


class FakeRec(dict):
    pass


def _install_fakes(monkeypatch, *, world, trial_results, tmp_path, goto_rc=0, calls=None,
                    start_frame="odom"):
    calls = calls if calls is not None else {}
    calls.setdefault("goto", [])
    calls.setdefault("trials", 0)

    mod_wp = types.ModuleType("waypoints")
    mod_wp.load = lambda: {"start": {"x": 0, "y": 0, "yaw": 0, "frame": start_frame}}
    monkeypatch.setitem(sys.modules, "waypoints", mod_wp)

    # run_campaign.py's return-to-start leg re-reads the waypoints FILE directly (rather than
    # going through waypoints.load(), the seam faked above) to learn `start`'s frame -- see
    # tests/test_run_campaign.py's _install_fakes for the same fix and full explanation. Point
    # that direct read at an isolated file instead of the operator's live maps/waypoints.yaml.
    store = tmp_path / "test_waypoints.yaml"
    store.write_text(f"start:\n  frame: {start_frame}\n  x: 0\n  y: 0\n  yaw: 0\n")
    monkeypatch.setenv("UTP_WAYPOINTS", str(store))

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
    mod_cfg = types.ModuleType("utp.common.config")
    mod_cfg.Config = types.SimpleNamespace(load=lambda _d=None: FakeCfg())
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


# --------------------------------------------------------------------------- synthetic configs
def safe_sources() -> list[dict]:
    """The mux as it is supposed to look for a campaign: teleop is the human's, the two
    autonomous sources are gated on the deadman, and only teleop may drive with the arm out."""
    return [
        {"name": "teleop", "topic": "/cmd_vel_teleop", "priority": 100,
         "requires_enable": False, "allows_arm_override": True},
        {"name": "servo", "topic": "/cmd_vel_servo", "priority": 50,
         "requires_enable": True, "allows_arm_override": False},
        {"name": "nav", "topic": "/cmd_vel_nav", "priority": 10,
         "requires_enable": True, "allows_arm_override": False},
    ]


def write_safety(tmp_path: Path, sources: list[dict] | None = None, *,
                 name: str = "safety.yaml") -> Path:
    doc = {
        "enabled": True,
        "output_topic": "/cmd_vel",
        "rate_hz": 20.0,
        "sources": safe_sources() if sources is None else sources,
        "gates": {"enable": "/safety/enable", "arm_stowed": "/safety/arm_stowed"},
        "timeouts": {"input_s": 0.3, "gate_s": 0.2},
        "limits": {"max_vx": 0.6, "max_wz": 0.8},
    }
    p = tmp_path / name
    p.write_text(yaml.safe_dump(doc, sort_keys=False))
    return p


def with_source(**changes) -> list[dict]:
    """safe_sources() with one named source's fields overwritten: with_source(name='nav', ...)."""
    target = changes.pop("name")
    out = []
    for s in safe_sources():
        s = dict(s)
        if s["name"] == target:
            s.update(changes)
        out.append(s)
    return out


def argv(tmp_path, safety: Path, **over):
    a = {"--trials": "2", "--method": "passive", "--start": "start",
         "--out": str(tmp_path / "campaign.jsonl"), "--settle": "0",
         "--config": str(REPO / "config" / "pipeline"),
         "--safety-config": str(safety)}
    a.update(over)
    out = ["run_campaign.py"]
    for k, v in a.items():
        out += [k, v] if v is not None else [k]
    return out


def records(p: Path):
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]


def run(monkeypatch, tmp_path, safety: Path, *extra, **over):
    w = FakeWorld()
    rc, calls = _install_fakes(monkeypatch, world=w, tmp_path=tmp_path,
                              trial_results=[{"success": True, "failure_category": None}])
    monkeypatch.setattr(sys, "argv", argv(tmp_path, safety, **over) + list(extra))
    return rc.main(), calls


# --------------------------------------------------------------------------- refusals
def test_nav_without_the_deadman_gate_is_refused(monkeypatch, tmp_path, capsys):
    """`nav` is Nav2's output. With requires_enable:false it reaches the base whether or not a
    human is holding /safety/enable, and a campaign has nobody holding it."""
    cfg = write_safety(tmp_path, with_source(name="nav", requires_enable=False))
    code, calls = run(monkeypatch, tmp_path, cfg)
    err = capsys.readouterr().err
    assert code == 2, "an unsafe mux config must be a non-zero exit, not a warning"
    assert calls["trials"] == 0, "must refuse BEFORE the first trial, not after"
    assert not (tmp_path / "campaign.jsonl").exists(), "no records for a campaign that never ran"
    assert "requires_enable" in err and "nav" in err, \
        f"the refusal must name the setting and the source; got:\n{err}"
    assert "true" in err, "the refusal must state the edit that fixes it"


def test_servo_without_the_deadman_gate_is_refused(monkeypatch, tmp_path, capsys):
    """`servo` is the pipeline's own approach/retreat motion. Checking only `nav` would pass a
    config where the gate was stood down on the source that does the close-in driving."""
    cfg = write_safety(tmp_path, with_source(name="servo", requires_enable=False))
    code, calls = run(monkeypatch, tmp_path, cfg)
    err = capsys.readouterr().err
    assert code == 2 and calls["trials"] == 0
    assert "servo" in err and "requires_enable" in err, err


def test_autonomous_arm_override_is_refused(monkeypatch, tmp_path, capsys):
    """allows_arm_override lets a source drive the base with the arm extended, where the tool tip
    sweeps ~0.88 m through space the costmap believes is empty. Teleop has it as the human
    recovery path for a stuck arm; nothing autonomous may."""
    cfg = write_safety(tmp_path, with_source(name="nav", allows_arm_override=True))
    code, calls = run(monkeypatch, tmp_path, cfg)
    err = capsys.readouterr().err
    assert code == 2 and calls["trials"] == 0
    assert "allows_arm_override" in err and "nav" in err, err


def test_teleop_keeps_its_exemptions(monkeypatch, tmp_path):
    """teleop is requires_enable:false and allows_arm_override:true BY DESIGN (safety/arbiter.py:
    a human already has their hand on the control, and this is the recovery path when everything
    else has failed). A check that refused those would be one people delete."""
    cfg = write_safety(tmp_path)                     # teleop is exempt in safe_sources()
    code, calls = run(monkeypatch, tmp_path, cfg)
    assert code == 0 and calls["trials"] == 2


@pytest.mark.parametrize("case", ["missing", "malformed", "not_a_mapping", "no_sources",
                                  "no_nav_source", "non_boolean", "unreadable"])
def test_unreadable_config_fails_closed(monkeypatch, tmp_path, capsys, case):
    """A config that cannot be read or understood is a REFUSAL, never a pass.

    WHY THIS IS THE RIGHT BEHAVIOUR. The preflight's job is not "spot something obviously wrong",
    it is "prove the deadman still bounds autonomous motion". A missing file, a YAML syntax error,
    a file that turns out not to be the mux config, or `requires_enable: "false"` as a string all
    leave that unproven -- and unproven is exactly the state this exists to stop. Worse, a
    preflight that waves through what it could not read is worse than no preflight at all: after
    it "passes" once, nobody opens config/safety.yaml themselves again. This is the same rule
    every gate in safety/arbiter.py already follows -- never-seen and stale both mean no.
    """
    if case == "missing":
        cfg = tmp_path / "nope" / "safety.yaml"
    elif case == "malformed":
        cfg = tmp_path / "safety.yaml"
        cfg.write_text("sources: [ - name: nav\n  bad: : yaml\n")
    elif case == "not_a_mapping":
        cfg = tmp_path / "safety.yaml"
        cfg.write_text("- just\n- a\n- list\n")
    elif case == "no_sources":
        cfg = tmp_path / "safety.yaml"
        cfg.write_text(yaml.safe_dump({"enabled": True, "limits": {"max_vx": 0.6}}))
    elif case == "no_nav_source":
        cfg = write_safety(tmp_path, [s for s in safe_sources() if s["name"] != "nav"])
    elif case == "non_boolean":
        cfg = write_safety(tmp_path, with_source(name="nav", requires_enable="false"))
    else:
        cfg = write_safety(tmp_path)
        cfg.chmod(0o000)

    try:
        code, calls = run(monkeypatch, tmp_path, cfg)
    finally:
        if case == "unreadable":
            cfg.chmod(0o644)
    err = capsys.readouterr().err
    assert code == 2, f"[{case}] an unverifiable mux config must refuse, not pass"
    assert calls["trials"] == 0, f"[{case}] refused campaigns must run no trials"
    assert "REFUSING" in err, err


def test_dry_run_still_refuses(monkeypatch, tmp_path, capsys):
    """--dry-run moves nothing, so it is tempting to let it through. Don't: the dry run is what
    session.sh runs first, and it is exactly where the operator should be told that safety.yaml
    still needs restoring -- while the robot is parked. A check that only fires on live runs
    teaches everyone that the check is optional."""
    cfg = write_safety(tmp_path, with_source(name="nav", requires_enable=False))
    code, calls = run(monkeypatch, tmp_path, cfg, "--dry-run")
    err = capsys.readouterr().err
    assert code == 2 and calls["trials"] == 0
    assert "requires_enable" in err, err


def test_resume_does_not_slip_past_the_preflight(monkeypatch, tmp_path, capsys):
    """--resume is the flag typed at the worst moment: half a campaign on disk, everyone waiting.
    It must not be a way past the interlock."""
    out = tmp_path / "campaign.jsonl"
    out.write_text(json.dumps({"campaign_index": 1}) + "\n")
    cfg = write_safety(tmp_path, with_source(name="nav", requires_enable=False))
    code, calls = run(monkeypatch, tmp_path, cfg, "--resume", **{"--trials": "5"})
    assert code == 2 and calls["trials"] == 0
    assert "REFUSING" in capsys.readouterr().err


# --------------------------------------------------------------------------- the safe path
def test_safe_config_proceeds_and_records_the_verdict(monkeypatch, tmp_path):
    code, calls = run(monkeypatch, tmp_path, write_safety(tmp_path))
    assert code == 0, "a campaign-safe mux config must not be blocked"
    assert calls["trials"] == 2
    recs = records(tmp_path / "campaign.jsonl")
    assert len(recs) == 2
    for r in recs:
        assert r["deadman_interlock_verified"] is True
        assert r["unsafe_campaign_override"] is False
        assert r["unsafe_campaign_reasons"] is None
        assert r["mux_config"].endswith("safety.yaml"), \
            "the record must say WHICH config was verified"


# --------------------------------------------------------------------------- the override
def test_override_runs_but_stamps_every_record(monkeypatch, tmp_path, capsys):
    """A supervised campaign with the gate down is a legitimate thing to choose, so it must be
    possible without editing code. What must NOT be possible is that choice disappearing: the
    dataset has to carry it, or six months from now these trials are indistinguishable from ones
    run with the interlock up."""
    cfg = write_safety(tmp_path, with_source(name="nav", requires_enable=False))
    code, calls = run(monkeypatch, tmp_path, cfg, "--i-accept-an-unsafe-campaign")
    err = capsys.readouterr().err
    assert code == 0 and calls["trials"] == 2
    assert "INTERLOCK DOWN" in err.upper(), f"the override must be loud about it:\n{err}"
    assert "requires_enable" in err, "the override must still print the reason"

    recs = records(tmp_path / "campaign.jsonl")
    assert len(recs) == 2
    for r in recs:
        assert r["unsafe_campaign_override"] is True, \
            "EVERY trial record must carry the marker, not just the first"
        assert r["deadman_interlock_verified"] is False
        assert r["unsafe_campaign_reasons"], "the marker must carry the reason, not just a flag"
        assert any("requires_enable" in reason for reason in r["unsafe_campaign_reasons"])


def test_override_flag_is_not_easy_to_type_by_habit(monkeypatch, tmp_path, capsys):
    """The friction IS the feature. --force is muscle memory within a week; this has to be meant.
    Anything shorter must not work."""
    import run_campaign as rc
    assert rc.UNSAFE_OVERRIDE_FLAG == "--i-accept-an-unsafe-campaign"
    cfg = write_safety(tmp_path, with_source(name="nav", requires_enable=False))
    for shorthand in ("--force", "-f", "--yes", "--i-accept"):
        with pytest.raises(SystemExit) as e:          # argparse rejects unknown/ambiguous flags
            run(monkeypatch, tmp_path, cfg, shorthand)
        assert e.value.code != 0
        capsys.readouterr()


# --------------------------------------------------------------------------- the checker itself
def test_violations_are_specific_enough_to_act_on(tmp_path):
    """Each violation must name the source and the setting. "config unsafe" sends someone reading
    a 120-line YAML file at the robot with the campaign not started."""
    import run_campaign as rc
    cfg = write_safety(tmp_path, with_source(name="nav", requires_enable=False,
                                             allows_arm_override=True))
    v = rc.mux_safety_violations(cfg)
    assert len(v) == 2, v
    assert any("requires_enable" in x and "nav" in x for x in v)
    assert any("allows_arm_override" in x and "nav" in x for x in v)
    assert rc.mux_safety_violations(write_safety(tmp_path, name="ok.yaml")) == []
