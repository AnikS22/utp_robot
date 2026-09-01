"""The map has to survive between sessions, and every layer that claims it does is checked here.

WHY THIS FILE EXISTS. The map-persistence path was broken end to end and nothing noticed, because
each piece looked fine on its own:

  * session.sh brings up SLAM_TOOLBOX. pose_source identified the SLAM session by the DDS
    publisher on /lidar_odometry/pose, which is MOLA's topic. Nothing publishes it, so
    slam_session_id was always None, so `waypoints.py record --frame map` refused every recording
    and nav2_goto.py then refused every waypoint for carrying no map name. The map frame was
    healthy the whole time.
  * `session.sh nav` passed maps/<name> to slam_toolbox's localization mode, which deserializes
    <name>.posegraph + <name>.data. Only .pgm/.yaml existed. slam_toolbox comes up ACTIVE anyway
    and starts a NEW graph at the robot's current pose -- a fresh map frame wearing a saved map's
    name, which is the one thing safety/map_frame.py exists to prevent.

Both failures produce a stack that looks up. That is what makes them worth a test.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

pose_source = pytest.importorskip("pose_source", reason="needs a ROS 2 environment")


# --------------------------------------------------------------------------- fakes
class FakeEndpoint:
    def __init__(self, gid: bytes): self.endpoint_gid = list(gid)


class FakeNode:
    """Only the one method slam_session_id calls."""

    def __init__(self, pubs: dict[str, list[bytes]]): self._pubs = pubs

    def get_publishers_info_by_topic(self, topic):
        return [FakeEndpoint(g) for g in self._pubs.get(topic, [])]


GID_A = bytes(range(16))
GID_B = bytes(range(16, 32))


# --------------------------------------------------------------------------- session id
def test_slam_toolbox_publisher_yields_a_session_id():
    """THE BUG. With only slam_toolbox running this returned None and every map waypoint was
    refused downstream as 'nameless'."""
    sid = pose_source.slam_session_id(FakeNode({"/map": [GID_A]}))
    assert sid, "a live slam_toolbox must produce a session id"
    assert sid == GID_A.hex()[:16]


def test_mola_publisher_still_works():
    sid = pose_source.slam_session_id(FakeNode({"/lidar_odometry/pose": [GID_A]}))
    assert sid == GID_A.hex()[:16]


def test_slam_toolbox_wins_when_both_publish():
    """slam_toolbox owns map->odom in this stack; if both are somehow up, the id must track the
    one that actually defines the frame."""
    sid = pose_source.slam_session_id(
        FakeNode({"/map": [GID_A], "/lidar_odometry/pose": [GID_B]}))
    assert sid == GID_A.hex()[:16]


def test_no_slam_is_none_not_a_guess():
    assert pose_source.slam_session_id(FakeNode({})) is None


def test_two_publishers_on_map_is_none():
    """Two /map publishers means map_server AND slam_toolbox are both up -- the frame has two
    owners and no single session identifies it. Fail closed."""
    assert pose_source.slam_session_id(FakeNode({"/map": [GID_A, GID_B]})) is None


def test_a_restarted_slam_changes_the_id():
    """The whole point: the id must change exactly when the map frame's origin can move."""
    a = pose_source.slam_session_id(FakeNode({"/map": [GID_A]}))
    b = pose_source.slam_session_id(FakeNode({"/map": [GID_B]}))
    assert a != b


def test_mola_session_id_alias_still_resolves():
    """Stored waypoints carry the key `mola_session`; the old entry point must keep working."""
    assert pose_source.mola_session_id(FakeNode({"/map": [GID_A]})) == GID_A.hex()[:16]


# --------------------------------------------------------------------------- .loaded_map
def test_loaded_map_goes_stale_when_slam_restarts(monkeypatch, tmp_path):
    """A name alone is not enough: load 'atrium', restart SLAM, and a name-only file would still
    claim 'atrium' while the frame origin had moved to wherever the robot booted."""
    f = tmp_path / ".loaded_map"
    f.write_text(f"atrium {GID_A.hex()[:16]}\n")
    monkeypatch.setattr(pose_source, "LOADED_MAP_FILE", f)
    monkeypatch.delenv("UTP_MAP", raising=False)
    assert pose_source.current_map_name(FakeNode({"/map": [GID_A]})) == "atrium"
    assert pose_source.current_map_name(FakeNode({"/map": [GID_B]})) is None, \
        "a restarted SLAM must demote the map to nameless, not keep claiming the old name"
    assert pose_source.current_map_name(FakeNode({})) is None


def test_no_loaded_map_file_is_none(monkeypatch, tmp_path):
    monkeypatch.setattr(pose_source, "LOADED_MAP_FILE", tmp_path / "absent")
    monkeypatch.delenv("UTP_MAP", raising=False)
    assert pose_source.current_map_name(FakeNode({"/map": [GID_A]})) is None


# --------------------------------------------------------------------------- session.sh
def _session_src() -> str:
    return (REPO / "bringup" / "session.sh").read_text()


def test_nav_requires_the_posegraph_not_just_the_grid():
    src = _session_src()
    assert ".posegraph" in src or 'maps/$MAP_NAME.$ext' in src, \
        "session.sh nav must refuse a map that has no pose graph to relocalize into"
    for ext in ("posegraph", "data"):
        assert ext in src, f"session.sh must check for the .{ext} file"


def test_nav_writes_loaded_map():
    src = _session_src()
    assert ".loaded_map" in src, (
        "session.sh nav must record which named map is live; without it every map-frame waypoint "
        "is stored nameless and nav2_goto.py refuses to drive to it")


def test_every_existing_map_is_either_complete_or_obviously_grid_only():
    """Not a pass/fail on the maps themselves -- a report. A .pgm without a .posegraph cannot be
    used for a campaign, and finding that out in the lab costs the morning."""
    maps = sorted(p.stem for p in (REPO / "maps").glob("*.yaml"))
    usable = [m for m in maps if (REPO / "maps" / f"{m}.posegraph").exists()]
    if not usable:
        pytest.skip("no relocalizable map on disk yet — `session.sh map` + `map_persist.sh <name>` "
                    f"must be run in the lab. Grid-only maps present: {maps}")


# --------------------------------------------------------------------------- map_persist.sh
def test_map_persist_refuses_when_no_slam_is_running():
    """Behavioural: actually run it. It used to call MOLA's /map_save unconditionally, so on the
    slam_toolbox stack it failed with 'MOLA refused to save' and no map was written."""
    r = subprocess.run(["bash", str(REPO / "bringup" / "map_persist.sh"), "unittest_probe"],
                       capture_output=True, text=True, timeout=180)
    assert r.returncode != 0, "must not claim success with no SLAM running"
    out = r.stdout + r.stderr
    assert "no SLAM is running" in out or "serialize_map" in out, \
        f"unhelpful failure: {out[-400:]}"
    assert not (REPO / "maps" / "unittest_probe.posegraph").exists()


def test_map_persist_rejects_a_path_as_a_name():
    r = subprocess.run(["bash", str(REPO / "bringup" / "map_persist.sh"), "../etc/passwd"],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode != 0 and "must not contain" in (r.stdout + r.stderr)


def test_map_persist_saves_grid_and_posegraph_and_loaded_map():
    """The three artefacts are not independent: grid for the costmap, pose graph for
    relocalization, .loaded_map for waypoint provenance. Missing any one breaks the campaign in a
    different place."""
    src = (REPO / "bringup" / "map_persist.sh").read_text()
    assert "serialize_map" in src, "no pose graph -> `session.sh nav` cannot relocalize"
    assert "save_map" in src, "no grid -> Nav2 has no costmap to plan on"
    assert ".loaded_map" in src, "no provenance -> waypoints record as nameless"
    assert '[ -f "$OUT.posegraph" ]' in src, "must verify the FILE, not just the service reply"
