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
