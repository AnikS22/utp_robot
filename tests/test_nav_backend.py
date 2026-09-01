"""The nav backend selection in RosWorld, and nav2_goto's refusal rules.

WHY. navigate_to_goal is the one place the 50-trial session depends on for repeatability, and it
just changed from "always odom waypoints" to "Nav2 over the saved map, falling back to odom". The
parts that can be checked without a robot are: which backend gets invoked, that an invalid value
is rejected rather than silently defaulting, and that nav2_goto REFUSES a waypoint whose
coordinates are not portable instead of driving to a meaningless pose.

That last one is the whole point of the map work. A fresh-SLAM `map` frame is indistinguishable
from a localized one in the TF tree; only the recorded map name tells them apart. Driving to a
non-portable coordinate is exactly the failure that a 50-trial session cannot survive.
"""
from __future__ import annotations

import ast
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))


def _source(name: str) -> str:
    return (REPO / "bringup" / name).read_text()


# --------------------------------------------------------------- backend selection (static)
def test_default_backend_is_nav2():
    src = _source("ros_world.py")
    assert 'os.environ.get("UTP_NAV_BACKEND", "nav2")' in src, (
        "the leg must default to the map-based backend; odom waypoints drift continuously and die "
        "when ranger_base restarts, both fatal across 50 trials")


def test_invalid_backend_is_rejected_not_silently_defaulted():
    src = _source("ros_world.py")
    assert "UTP_NAV_BACKEND must be" in src, (
        "a typo'd backend name must raise, not quietly fall through to one of them")


def test_nav2_failure_falls_back_to_odom_rather_than_failing_the_trial():
    src = _source("ros_world.py")
    assert "returncode in (1, 2, 3, 4, 5)" in src, (
        "a Nav2 stack that is down, or an odom-frame waypoint, must degrade to the odom driver — "
        "losing a leg is not a reason to lose the trial")


def test_press_chain_is_not_moved_onto_the_map():
    """docs/NAV2.md: a map->odom correction mid-press moves the target under the arm. Only the LEG
    may use Nav2; approach_blockage and the look ladder must stay on odom."""
    src = _source("ros_world.py")
    approach = src[src.index("def approach_blockage"):src.index("def strafe_view")]
    assert "nav2_goto" not in approach, "approach_blockage must not route through Nav2"


# --------------------------------------------------------------- nav2_goto refusal rules
def _no_rclpy() -> bool:
    try:
        import rclpy  # noqa: F401
        return False
    except Exception:
        return True


def _run_nav2_goto(args, env=None):
    # ROS_DOMAIN_ID must be set, or bringup/_ros_env.require_domain() exits 1 before the script
    # reaches any of its own logic -- correctly, since domain 0 is an empty graph. Without this
    # the test passes when ROS is unsourced (it skips) and fails when it is sourced, measuring
    # the shell rather than nav2_goto.
    if env is None:
        env = {**os.environ, "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "9")}
    # POINT THE PROVENANCE CHECK AT NOTHING. nav2_goto refuses a waypoint whose map_name does not
    # match maps/.loaded_map -- correct on the robot, and fatal in a test that mocks the waypoint
    # store: the REAL .loaded_map leaks in and every mocked goal is refused with exit 3. Six tests
    # here failed that way the moment a real map was first loaded. Tests that mock a subsystem must
    # mock ALL of its inputs, including the ones that live outside the process.
    env.setdefault("UTP_LOADED_MAP", "/nonexistent/.loaded_map")
    return subprocess.run([sys.executable, str(REPO / "bringup" / "nav2_goto.py")] + args,
                          capture_output=True, text=True, timeout=60, cwd=str(REPO), env=env)


@pytest.mark.skipif(_no_rclpy(), reason="waypoints.py imports rclpy; runs on the robot laptop")
def test_nav2_goto_rejects_unknown_waypoint():
    r = _run_nav2_goto(["definitely_not_a_waypoint"])
    assert r.returncode == 2 and "unknown waypoint" in r.stderr


def test_nav2_goto_is_a_dry_run_without_go(monkeypatch, tmp_path):
    """Mirrors waypoints.py: printing the goal must never send it."""
    src = _source("nav2_goto.py")
    assert '"--go"' in src and "DRY RUN" in src
    # --go is checked BEFORE rclpy is imported, so a dry run cannot touch the robot
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "main")
    body = ast.dump(fn)
    assert body.index("DRY RUN") < body.index("'rclpy'"), \
        "the dry-run return must come before rclpy is initialised"


def test_nav2_goto_refuses_a_non_portable_waypoint():
    """A waypoint with no recorded map name came from a fresh SLAM session whose origin is wherever
    the robot booted. Its coordinates mean nothing in a later session, and driving to them is the
    failure mode a saved map exists to prevent."""
    src = _source("nav2_goto.py")
    assert "carries no map name" in src
    assert "MAP_NAME_KEY" in src and "FRAME_MAP" in src
    assert "--force" in src, "there must be an explicit override, not a silent one"


def test_nav2_goto_prints_the_words_rosworld_parses():
    """RosWorld.navigate_to_goal greps stdout for 'arrived' / 'blocked'. Both backends must speak
    the same vocabulary or the FSM cannot tell reached from blocked."""
    src = _source("nav2_goto.py")
    assert 'f"arrived at' in src, "success must print 'arrived'"
    assert 'f"blocked:' in src, "recoveries-exhausted must print 'blocked' so the FSM reasons"


def test_session_script_uses_localization_mode_not_mapping():
    """50 passes through one corridor must not keep rewriting the map underneath the waypoints."""
    src = (REPO / "bringup" / "session.sh").read_text()
    assert "localization_slam_toolbox_node" in src
    assert "mode:=localization" in src
    assert "localization:=slam" in src, \
        "Nav2 must not start map_server/AMCL: exactly one source may own /map and map->odom"


def test_timeout_and_crash_have_distinct_exit_codes():
    """1 meant TIMEOUT and is also what an uncaught exception exits with, so a nav2_goto that died
    on an import would have been recorded as a real navigation timeout — a claim about the world
    made by a crashed process. Timeout is now 6; 1 means crashed and triggers the fallback."""
    src = _source("nav2_goto.py")
    assert "rc = 6" in src and "TIMEOUT" in src
    assert not re.search(r"rc = 1\b", src), "1 must not be a real navigation outcome"


def test_behaviour_tree_paths_are_rewritten_at_launch():
    """nav2_params_os0_map.yaml hard-codes bt XML paths inside the SIM checkout. On the rover
    laptop that directory does not exist, bt_navigator fails to load its tree, and Nav2 comes up
    looking healthy while navigate_to_pose never works — the silent half-failure docs/NAV2.md
    warns about. session.sh must rewrite both paths to this repo's own copies before launching."""
    src = (REPO / "bringup" / "session.sh").read_text()
    assert "default_nav_to_pose_bt_xml" in src and "default_nav_through_poses_bt_xml" in src, \
        "both tree paths must be rewritten, not just the first"
    assert "behaviour-tree path rewrite failed" in src, \
        "the rewrite must be verified, not assumed — a failed sed would launch the original paths"
    for f in ("navigate_to_pose_no_spin.xml", "navigate_through_poses_no_spin.xml"):
        assert (REPO / "nav2_bringup" / "behavior_trees" / f).is_file(), \
            f"this repo must ship its own {f}"


def test_params_still_disable_spin_recovery():
    """The high-CoM base with the arm flips if it spins in place, so `spin` is deliberately absent
    from behavior_plugins and the trees are the _no_spin variants. Guard both."""
    params = (REPO / "nav2_bringup" / "nav2_params_os0_map.yaml").read_text()
    # Check the behavior_plugins LIST, not the whole file — a comment reading `NO "spin"` is
    # documentation, not configuration.
    plugins = next(l for l in params.splitlines() if "behavior_plugins:" in l)
    assert "spin" not in plugins, f"spin must not be a recovery plugin — the base flips: {plugins}"
    assert "no_spin" in params, "the no-spin behaviour trees must be selected"


# ============================================================================================
# BEHAVIOURAL tests: these RUN nav2_goto.main() against a fake rclpy/action layer, so the logic
# is executed rather than pattern-matched. Everything above this line is a static regression
# guard; everything below actually exercises the code path that will run tomorrow.
# ============================================================================================
import json as _json
import types as _types


class _FakeGoalHandle:
    def __init__(self, accepted=True, status=4):
        self.accepted = accepted
        self._status = status
        self.cancelled = False
        self.result_future = None
        self.cancel_future = None

    def get_result_async(self):
        f = _FakeFuture(); f._done = True
        f._value = _types.SimpleNamespace(status=self._status)
        self.result_future = f
        return f

    def cancel_goal_async(self):
        self.cancelled = True
        self.cancel_future = _FakeFuture(
            done=True, value=_types.SimpleNamespace(goals_canceling=[object()]))
        if self.result_future is not None:
            self.result_future._done = True
            self.result_future._value = _types.SimpleNamespace(status=5)
        return self.cancel_future


class _FakeFuture:
    def __init__(self, done=False, value=None):
        self._done, self._value = done, value

    def done(self): return self._done
    def result(self): return self._value


def _install_fake_ros(monkeypatch, *, server=True, accepted=True, status=4, never_finishes=False):
    """Minimal rclpy/nav2_msgs/geometry_msgs stand-ins. Records the goal that was sent."""
    sent = {}

    class _FakeClient:
        def __init__(self, *a, **k): pass
        def wait_for_server(self, timeout_sec=0): return server
        def send_goal_async(self, goal):
            sent["goal"] = goal
            h = _FakeGoalHandle(accepted=accepted, status=status)
            sent["handle"] = h
            return _FakeFuture(done=True, value=h)

    class _FakeNode:
        def __init__(self, *a, **k): pass
        def get_clock(self):
            return _types.SimpleNamespace(now=lambda: _types.SimpleNamespace(to_msg=lambda: 0))
        def destroy_node(self): sent["destroyed"] = True

    rclpy = _types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: sent.__setitem__("shutdown", True)
    rclpy.ok = lambda: True
    rclpy.spin_once = lambda *a, **k: None
    rclpy.spin_until_future_complete = lambda *a, **k: None
    action_mod = _types.ModuleType("rclpy.action"); action_mod.ActionClient = _FakeClient
    node_mod = _types.ModuleType("rclpy.node"); node_mod.Node = _FakeNode
    rclpy.action, rclpy.node = action_mod, node_mod

    nav_act = _types.ModuleType("nav2_msgs.action")
    class _NTP:
        class Goal:
            def __init__(self): self.pose = None
    nav_act.NavigateToPose = _NTP
    nav_msgs = _types.ModuleType("nav2_msgs"); nav_msgs.action = nav_act

    geo = _types.ModuleType("geometry_msgs.msg")
    class _PoseStamped:
        def __init__(self):
            self.header = _types.SimpleNamespace(frame_id="", stamp=None)
            self.pose = _types.SimpleNamespace(
                position=_types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=_types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    geo.PoseStamped = _PoseStamped
    geo_pkg = _types.ModuleType("geometry_msgs"); geo_pkg.msg = geo

    if never_finishes:
        h = _FakeGoalHandle()
        def _pending_result():
            h.result_future = _FakeFuture(done=False)
            return h.result_future
        h.get_result_async = _pending_result
        _FakeClient.send_goal_async = lambda self, goal: (
            sent.__setitem__("goal", goal), sent.__setitem__("handle", h),
            _FakeFuture(done=True, value=h))[-1]

    for name, mod in (("rclpy", rclpy), ("rclpy.action", action_mod), ("rclpy.node", node_mod),
                      ("nav2_msgs", nav_msgs), ("nav2_msgs.action", nav_act),
                      ("geometry_msgs", geo_pkg), ("geometry_msgs.msg", geo)):
        monkeypatch.setitem(sys.modules, name, mod)
    return sent


def _install_fake_waypoints(monkeypatch, wp):
    mod = _types.ModuleType("waypoints"); mod.load = lambda: wp
    monkeypatch.setitem(sys.modules, "waypoints", mod)
    mf = _types.ModuleType("safety.map_frame")
    mf.FRAME_KEY, mf.FRAME_MAP, mf.MAP_NAME_KEY = "frame", "map", "map_name"
    monkeypatch.setitem(sys.modules, "safety.map_frame", mf)


def _nav2_main(monkeypatch, argv):
    """Run nav2_goto.main() in-process against the installed fakes.

    UTP_LOADED_MAP is pointed at nothing on purpose. nav2_goto refuses a waypoint whose map_name
    does not match maps/.loaded_map, which is correct on the robot and fatal here: these tests mock
    the waypoint store but the provenance file is real machine state OUTSIDE the process, so the
    live map leaked in and every mocked goal was refused with exit 3. Six tests in this file broke
    that way the moment a real map was first loaded on hardware.

    The map-match refusal has its own test below, with the file mocked deliberately. A test that
    mocks a subsystem has to mock ALL of its inputs, including the ones on disk."""
    import importlib
    monkeypatch.setenv("UTP_LOADED_MAP", "/nonexistent/.loaded_map")
    monkeypatch.setattr(sys, "argv", ["nav2_goto.py"] + argv)
    mod = importlib.import_module("nav2_goto")
    importlib.reload(mod)
    return mod.main()


GOOD_WP = {"door": {"x": 12.5, "y": 3.25, "yaw": 1.5708, "frame": "map", "map_name": "atrium2d"}}


def test_behav_sends_a_map_frame_goal_with_correct_pose(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    sent = _install_fake_ros(monkeypatch)
    assert _nav2_main(monkeypatch, ["door", "--go"]) == 0
    goal = sent["goal"].pose
    assert goal.header.frame_id == "map", "Nav2 goals must be in the map frame"
    assert goal.pose.position.x == pytest.approx(12.5)
    assert goal.pose.position.y == pytest.approx(3.25)
    # yaw 1.5708 rad -> quaternion z=sin(yaw/2)=0.7071, w=cos(yaw/2)=0.7071
    assert goal.pose.orientation.z == pytest.approx(0.7071, abs=1e-3)
    assert goal.pose.orientation.w == pytest.approx(0.7071, abs=1e-3)
    assert "arrived" in capsys.readouterr().out


def test_behav_dry_run_sends_nothing(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    sent = _install_fake_ros(monkeypatch)
    assert _nav2_main(monkeypatch, ["door"]) == 0
    assert "goal" not in sent, "a dry run must not send a goal"
    assert "DRY RUN" in capsys.readouterr().out


def test_behav_no_action_server_returns_fallback_code(monkeypatch):
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    _install_fake_ros(monkeypatch, server=False)
    assert _nav2_main(monkeypatch, ["door", "--go"]) == 4, \
        "a down Nav2 must return a code RosWorld falls back on, not a nav outcome"


def test_behav_rejected_goal_returns_fallback_code(monkeypatch):
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    _install_fake_ros(monkeypatch, accepted=False)
    assert _nav2_main(monkeypatch, ["door", "--go"]) == 5


def test_behav_recoveries_exhausted_reports_blocked_not_arrived(monkeypatch, capsys):
    """Nav2 giving up in front of an obstruction is the same event the odom backend calls
    `blocked`, and the FSM must reason from there rather than record a nav failure."""
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    _install_fake_ros(monkeypatch, status=6)          # 6 == ABORTED
    assert _nav2_main(monkeypatch, ["door", "--go"]) == 0
    out = capsys.readouterr().out
    assert "blocked" in out and "arrived" not in out


def test_behav_timeout_returns_6_and_cancels_the_goal(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    sent = _install_fake_ros(monkeypatch, never_finishes=True)
    assert _nav2_main(monkeypatch, ["door", "--go", "--timeout", "0.2"]) == 6
    assert sent["handle"].cancelled, "a timed-out goal must be cancelled, not left running"
    assert sent["handle"].cancel_future.done(), "must wait for cancellation acknowledgement"
    assert sent["handle"].result_future.done(), "must wait for Nav2's terminal result"


def test_behav_odom_frame_waypoint_refused_before_touching_ros(monkeypatch):
    _install_fake_waypoints(monkeypatch,
                            {"door": {"x": 1, "y": 2, "yaw": 0, "frame": "odom"}})
    sent = _install_fake_ros(monkeypatch)
    assert _nav2_main(monkeypatch, ["door", "--go"]) == 3
    assert "goal" not in sent, "must refuse before sending anything"


def test_behav_nameless_map_waypoint_refused(monkeypatch):
    """The trap safety/map_frame.py exists for: a fresh-SLAM map frame looks identical in TF but
    its origin is wherever the robot booted."""
    _install_fake_waypoints(monkeypatch,
                            {"door": {"x": 1, "y": 2, "yaw": 0, "frame": "map"}})
    sent = _install_fake_ros(monkeypatch)
    assert _nav2_main(monkeypatch, ["door", "--go"]) == 3
    assert "goal" not in sent


def test_behav_force_overrides_the_refusal(monkeypatch):
    _install_fake_waypoints(monkeypatch,
                            {"door": {"x": 1, "y": 2, "yaw": 0, "frame": "odom"}})
    sent = _install_fake_ros(monkeypatch)
    assert _nav2_main(monkeypatch, ["door", "--go", "--force"]) == 0
    assert "goal" in sent, "--force must actually drive"


def test_behav_refuses_a_waypoint_recorded_in_a_different_map(monkeypatch, capsys, tmp_path):
    """A waypoint carrying SOME map name was never enough. Two maps of the same building have
    unrelated origins, so a coordinate valid in 'atrium' names a different physical place in
    'atrium2d'. Driving it anyway is a confident arrival at the wrong spot, with nothing to notice.

    This was live on 2026-09-01: every waypoint carried map_name 'atrium' while session.sh nav
    defaulted MAP_NAME to 'atrium2d', and nothing compared the two."""
    loaded = tmp_path / ".loaded_map"
    loaded.write_text("some_other_map 010fa575ab3b4f7a\n")
    monkeypatch.setenv("UTP_LOADED_MAP", str(loaded))
    _install_fake_waypoints(monkeypatch, GOOD_WP)      # GOOD_WP is map_name 'atrium2d'
    sent = _install_fake_ros(monkeypatch)
    import importlib, sys as _s
    monkeypatch.setattr(_s, "argv", ["nav2_goto.py", "door", "--go"])
    mod = importlib.import_module("nav2_goto"); importlib.reload(mod)
    assert mod.main() == 3, "must refuse a waypoint whose map is not the one loaded"
    assert not sent, "nothing may be sent when the map does not match"
    err = capsys.readouterr().err
    assert "some_other_map" in err and "atrium2d" in err, \
        "the refusal must name BOTH maps, or the operator cannot tell which to load"


def test_behav_force_overrides_the_map_mismatch(monkeypatch, tmp_path):
    """--force exists so a known-good coordinate is still drivable when provenance is unavailable.
    It must be explicit; it must never be the default path."""
    loaded = tmp_path / ".loaded_map"
    loaded.write_text("some_other_map 010fa575ab3b4f7a\n")
    monkeypatch.setenv("UTP_LOADED_MAP", str(loaded))
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    sent = _install_fake_ros(monkeypatch)
    import importlib, sys as _s
    monkeypatch.setattr(_s, "argv", ["nav2_goto.py", "door", "--go", "--force"])
    mod = importlib.import_module("nav2_goto"); importlib.reload(mod)
    assert mod.main() == 0
    assert sent, "--force must actually send the goal"


# ============================================================================================
# THE MACHINE-READABLE RESULT LINE
#
# RosWorld decides what happened by substring-matching this script's stdout, and it tests
# `arrived` FIRST -- so any stdout line carrying that substring reports success, a diagnostic
# reading "not arrived" included. A `blocked` verdict starts reason -> ground -> press, so a
# stray word can put the arm at a wall. nav2_goto now ends every attempt with a JSON RESULT line
# and that line is the contract; the prose lines stay only until RosWorld migrates.
#
# These tests run main() against the same fakes as the block above. None of them string-match
# source.
# ============================================================================================

# status -> the exit code(s) it may be reported with, pinned HERE so widening the enum or
# re-pointing a status at a different exit code has to be a deliberate edit in two files.
EXPECTED_STATUS_EXIT = {
    "arrived":   (0,),
    "blocked":   (0,),       # Nav2 STATUS_ABORTED only
    "timeout":   (6,),
    "rejected":  (5,),
    "refused":   (2, 3),
    "no_server": (4,),
    "cancelled": (4, 130),
    "error":     (4,),
}
LEGACY_WORDS = ("arrived", "blocked")


def _result_line(out: str) -> dict:
    """The RESULT line must be the LAST non-blank stdout line, and the only one."""
    lines = [l for l in out.splitlines() if l.strip()]
    assert lines, "nav2_goto produced no stdout at all"
    tagged = [l for l in lines if l.startswith("RESULT ")]
    assert len(tagged) == 1, f"expected exactly one RESULT line, got {len(tagged)}: {lines}"
    assert lines[-1] is tagged[0], (
        f"the RESULT line must be the LAST stdout line, or a caller reading the tail of a log "
        f"gets prose instead of a verdict. Last line was: {lines[-1]!r}")
    payload = _json.loads(lines[-1][len("RESULT "):])          # raises if it is not valid JSON
    assert set(payload) == {"status", "waypoint", "elapsed_s", "detail"}, payload
    assert payload["status"] in EXPECTED_STATUS_EXIT, f"status outside the enum: {payload}"
    assert isinstance(payload["elapsed_s"], (int, float)) and not isinstance(
        payload["elapsed_s"], bool), payload
    assert isinstance(payload["detail"], str), payload
    return payload


def _check(rc: int, out: str, waypoint: str, expect_status: str) -> dict:
    """Every assertion that must hold on every path, in one place."""
    res = _result_line(out)
    assert res["status"] == expect_status, f"expected {expect_status}, got {res}"
    assert res["waypoint"] == waypoint, res
    assert rc in EXPECTED_STATUS_EXIT[expect_status], (
        f"status {res['status']!r} came back with exit {rc}; the exit-code contract with RosWorld "
        f"says {EXPECTED_STATUS_EXIT[expect_status]}")
    # THE LEGACY CALLER IS STILL LIVE. It greps all of stdout, so no non-outcome path may leak
    # either word -- not in the detail, not anywhere.
    if expect_status not in ("arrived", "blocked"):
        for w in LEGACY_WORDS:
            assert w not in out, (
                f"a {expect_status} path put {w!r} on stdout; RosWorld still substring-matches "
                f"stdout and would report a navigation outcome that did not happen:\n{out}")
    return res


def test_result_enum_and_exit_map_are_the_closed_set():
    """The enum is closed. A new status is a change to the FSM's vocabulary, not a detail."""
    import importlib
    mod = importlib.import_module("nav2_goto")
    importlib.reload(mod)
    assert {k: tuple(v) for k, v in mod.STATUS_EXIT.items()} == EXPECTED_STATUS_EXIT


def test_result_arrived(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    _install_fake_ros(monkeypatch, status=4)              # 4 == SUCCEEDED
    rc = _nav2_main(monkeypatch, ["door", "--go"])
    out = capsys.readouterr().out
    _check(rc, out, "door", "arrived")
    # LEGACY, and it must survive verbatim: ros_world.py is mid-edit by someone else today.
    assert any(l.startswith("arrived at 'door'") for l in out.splitlines()), out


def test_result_blocked_is_reserved_for_nav2_aborted(monkeypatch, capsys):
    """STATUS_ABORTED (6) is the ONLY thing that may say `blocked` -- it is what starts the
    reason -> ground -> press chain and drives the arm at whatever the grounder finds."""
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    _install_fake_ros(monkeypatch, status=6)
    rc = _nav2_main(monkeypatch, ["door", "--go"])
    out = capsys.readouterr().out
    _check(rc, out, "door", "blocked")
    assert any(l.startswith("blocked: Nav2 ABORTED") for l in out.splitlines()), out


def test_result_canceled_goal_is_cancelled_not_blocked(monkeypatch, capsys):
    """5 == CANCELED is a control-plane event: an operator, a supervisor, a preempting goal.
    Calling it `blocked` manufactures a claim about the world out of a claim about software."""
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    _install_fake_ros(monkeypatch, status=5)
    rc = _nav2_main(monkeypatch, ["door", "--go"])
    _check(rc, capsys.readouterr().out, "door", "cancelled")


@pytest.mark.parametrize("nav_status", [0, 1, 2])          # UNKNOWN / ACCEPTED / EXECUTING
def test_result_non_terminal_status_is_error(monkeypatch, capsys, nav_status):
    """A non-terminal status coming back as a RESULT means the action server is confused. That is
    not a statement about the world and must not start the perception-and-action chain."""
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    _install_fake_ros(monkeypatch, status=nav_status)
    rc = _nav2_main(monkeypatch, ["door", "--go"])
    _check(rc, capsys.readouterr().out, "door", "error")


def test_result_timeout(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    sent = _install_fake_ros(monkeypatch, never_finishes=True)
    rc = _nav2_main(monkeypatch, ["door", "--go", "--timeout", "0.2"])
    res = _check(rc, capsys.readouterr().out, "door", "timeout")
    assert sent["handle"].cancelled, "a timed-out goal must still be cancelled"
    assert "terminal status 5" in res["detail"]
    assert res["elapsed_s"] >= 0.2, f"elapsed_s must report real waiting time: {res}"


def test_result_rejected_goal(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    _install_fake_ros(monkeypatch, accepted=False)
    rc = _nav2_main(monkeypatch, ["door", "--go"])
    _check(rc, capsys.readouterr().out, "door", "rejected")


def test_result_no_action_server(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    _install_fake_ros(monkeypatch, server=False)
    rc = _nav2_main(monkeypatch, ["door", "--go"])
    _check(rc, capsys.readouterr().out, "door", "no_server")


def test_result_unknown_waypoint_is_refused(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, {})
    sent = _install_fake_ros(monkeypatch)
    rc = _nav2_main(monkeypatch, ["door", "--go"])
    _check(rc, capsys.readouterr().out, "door", "refused")
    assert "goal" not in sent


def test_result_odom_frame_waypoint_is_refused(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, {"door": {"x": 1, "y": 2, "yaw": 0, "frame": "odom"}})
    sent = _install_fake_ros(monkeypatch)
    rc = _nav2_main(monkeypatch, ["door", "--go"])
    _check(rc, capsys.readouterr().out, "door", "refused")
    assert "goal" not in sent, "must refuse before sending anything"


def test_result_nameless_map_waypoint_is_refused(monkeypatch, capsys):
    _install_fake_waypoints(monkeypatch, {"door": {"x": 1, "y": 2, "yaw": 0, "frame": "map"}})
    _install_fake_ros(monkeypatch)
    rc = _nav2_main(monkeypatch, ["door", "--go"])
    _check(rc, capsys.readouterr().out, "door", "refused")


def test_result_map_mismatch_is_refused(monkeypatch, capsys, tmp_path):
    """The 2026-09-01 case: every waypoint carried map_name 'atrium' while the loaded map was
    'atrium2d'. A confident arrival at the wrong place is the hardest failure to notice."""
    loaded = tmp_path / ".loaded_map"
    loaded.write_text("some_other_map 010fa575ab3b4f7a\n")
    monkeypatch.setenv("UTP_LOADED_MAP", str(loaded))
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    sent = _install_fake_ros(monkeypatch)
    import importlib, sys as _s
    monkeypatch.setattr(_s, "argv", ["nav2_goto.py", "door", "--go"])
    mod = importlib.import_module("nav2_goto"); importlib.reload(mod)
    rc = mod.main()
    _check(rc, capsys.readouterr().out, "door", "refused")
    assert not sent, "nothing may be sent when the map does not match"


def test_dry_run_emits_no_result_line(monkeypatch, capsys):
    """A dry run attempted no navigation, so it has no outcome. Forcing one out of a closed
    outcome enum is the same error as calling a cancelled goal `blocked`. Absence of a RESULT
    line is therefore the signal, and it is never an arrival."""
    _install_fake_waypoints(monkeypatch, GOOD_WP)
    sent = _install_fake_ros(monkeypatch)
    assert _nav2_main(monkeypatch, ["door"]) == 0
    out = capsys.readouterr().out
    assert "goal" not in sent
    assert "RESULT " not in out, f"a dry run must not report a navigation outcome:\n{out}"
    assert "DRY RUN" in out
    for w in LEGACY_WORDS:
        assert w not in out, f"a dry run put {w!r} on stdout: {out}"
