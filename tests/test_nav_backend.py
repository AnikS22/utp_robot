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

    def get_result_async(self):
        f = _FakeFuture(); f._done = True
        f._value = _types.SimpleNamespace(status=self._status)
        return f

    def cancel_goal_async(self):
        self.cancelled = True
        return _FakeFuture(done=True)


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
        h.get_result_async = lambda: _FakeFuture(done=False)
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
    import importlib
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
