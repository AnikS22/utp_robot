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
    assert r.returncode != 0 and "not a path" in (r.stdout + r.stderr)


def test_map_persist_saves_grid_and_posegraph_and_loaded_map():
    """The three artefacts are not independent: grid for the costmap, pose graph for
    relocalization, .loaded_map for waypoint provenance. Missing any one breaks the campaign in a
    different place."""
    src = (REPO / "bringup" / "map_persist.sh").read_text()
    assert "serialize_map" in src, "no pose graph -> `session.sh nav` cannot relocalize"
    assert "save_map" in src, "no grid -> Nav2 has no costmap to plan on"
    assert ".loaded_map" in src, "no provenance -> waypoints record as nameless"
    assert '"$STEM.posegraph"' in src, "must verify the FILE, not just the service reply"
    assert "one-map-script" not in src


# --------------------------------------------------------------------------- consistency
# These are cheap, boring, and each one corresponds to a whole afternoon already lost: a stack
# that came up healthy while pointing at a topic, a service or a script that does not exist.
LIVE_DIRS = ("bringup", "safety", "config", "nav2_bringup", "docs")


def _live_files():
    for d in LIVE_DIRS:
        for p in (REPO / d).rglob("*"):
            if p.is_file() and p.suffix in (".py", ".sh", ".yaml", ".xml", ".md"):
                if "__pycache__" not in str(p):
                    yield p
    for name in ("README.md", "CLAUDE.md"):
        if (REPO / name).exists():
            yield REPO / name


def test_nothing_live_tells_you_to_run_an_archived_script():
    """Archiving a script that a live file still names is worse than leaving it: the instruction
    now points at nothing, and you find out standing in the lab."""
    archived = {p.name for p in (REPO / "archive").glob("*")
                if p.suffix in (".py", ".sh", ".yaml")}
    assert archived, "archive/ is empty — did the layout change?"
    bad = []
    for p in _live_files():
        txt = p.read_text(errors="ignore")
        for name in archived:
            # `archive/<name>` is a correct reference TO the archive; a bare `bringup/<name>` or
            # `bash <name>` is a dangling instruction.
            for form in (f"bringup/{name}", f"config/{name}", f"tests/{name}"):
                if form in txt:
                    bad.append(f"{p.relative_to(REPO)} -> {form}")
    assert not bad, "live files reference archived scripts:\n  " + "\n  ".join(sorted(bad))


def test_slam_is_launched_from_the_params_file_not_inline_flags():
    """Inline -p flags take slam_toolbox's DEFAULTS for everything not listed, including two that
    decide whether the map is usable: do_loop_closing (else the map comes out bent and every
    waypoint inherits the bend) and stack_size_to_use (else serializing a building-sized graph
    dies — the save fails on exactly the map worth keeping).

    Both were verified to take effect on 2026-09-01 by launching
    `localization_slam_toolbox_node --params-file config/slam_os0.yaml`, configuring it, and
    reading the parameters back off the running node."""
    src = (REPO / "bringup" / "session.sh").read_text()
    assert "slam_os0.yaml" in src, "session.sh must launch slam_toolbox with config/slam_os0.yaml"
    assert "-p scan_topic:=/scan -p resolution:=0.05" not in src, \
        "the inline-parameter launch is back"
    params = (REPO / "config" / "slam_os0.yaml").read_text()
    for key in ("do_loop_closing", "stack_size_to_use", "base_frame", "scan_topic"):
        assert key in params, f"config/slam_os0.yaml lost {key}"


def test_the_chassis_is_excluded_by_the_projection_not_by_min_laser_range():
    """MEASURED 2026-09-01: slam_toolbox on Jazzy never declares `min_laser_range`. Launched
    against config/slam_os0.yaml, configured, activated and fed scans, `ros2 param get
    /slam_toolbox min_laser_range` answers "Parameter not set" while max_laser_range answers 20.0.

    So the thing that keeps the chassis out of the map is pointcloud_to_laserscan's own
    `range_min` and its height band, in session.sh. If those are ever dropped, the config will
    still LOOK like it protects you and will not."""
    src = (REPO / "bringup" / "session.sh").read_text()
    p2l = [l for l in src.splitlines() if "range_min:=" in l]
    assert p2l, "pointcloud_to_laserscan has no range_min — nothing excludes the chassis"
    val = float(p2l[0].split("range_min:=")[1].split()[0])
    assert val >= 0.4, f"range_min {val} is inside the chassis; the robot gets mapped in"
    assert "min_height:=" in src and "max_height:=" in src, \
        "the height band is what drops the chassis geometrically"
    notes = (REPO / "config" / "slam_os0.yaml").read_text()
    assert "INERT ON JAZZY" in notes, \
        "slam_os0.yaml must say min_laser_range does nothing, or someone will rely on it"


def test_the_scan_slice_config_matches_what_session_sh_actually_passes():
    """config/ouster.yaml documents the 2D slice; session.sh passes it as flags. Nothing reads the
    config at runtime, so the two can drift apart while the config keeps reading as authoritative.

    They HAD drifted: range_min_m said 0.40 while session.sh used 0.50 (2026-09-01). Nobody was
    hurt because session.sh is the one that runs, but the next person to tune the slice would have
    edited the file that does nothing and measured no change."""
    import yaml
    slice_cfg = yaml.safe_load((REPO / "config" / "ouster.yaml").read_text())["scan_slice"]
    src = (REPO / "bringup" / "session.sh").read_text()

    def flag(name):
        line = next(l for l in src.splitlines() if f"{name}:=" in l)
        return float(line.split(f"{name}:=")[1].split()[0])

    assert flag("range_min") == slice_cfg["range_min_m"]
    assert flag("range_max") == slice_cfg["range_max_m"]
    assert flag("min_height") == slice_cfg["min_height_m"]
    assert flag("max_height") == slice_cfg["max_height_m"]


def test_the_two_ouster_configs_agree_on_lidar_mode():
    """ouster_driver.yaml is what the driver loads; ouster.yaml is what everyone READS. A stale
    512x10 in the driver file halves the sensor's horizontal resolution while the documented
    config still says 1024x10 -- and the projection in session.sh is configured for ~1024 rays,
    so half the scan's bins would be empty with nothing reporting it."""
    import yaml
    documented = yaml.safe_load((REPO / "config" / "ouster.yaml").read_text())["lidar_mode"]
    driver = yaml.safe_load((REPO / "config" / "ouster_driver.yaml").read_text())
    effective = driver["ouster/os_driver"]["ros__parameters"]["lidar_mode"]
    assert effective == documented, (
        f"ouster_driver.yaml runs {effective} but ouster.yaml documents {documented}")


def test_the_live_scan_chain_is_one_chain():
    """/scan_mapping belonged to the retired A1M8 gate. A params file still asking for it would
    leave slam_toolbox subscribed to a topic nobody publishes — /map never appears and the node
    looks perfectly healthy."""
    params = (REPO / "config" / "slam_os0.yaml").read_text()
    line = [l for l in params.splitlines() if l.strip().startswith("scan_topic:")]
    assert line and line[0].split(":", 1)[1].strip() == "/scan", \
        f"slam_os0.yaml must consume /scan (scan_relay's RELIABLE copy), got {line}"


def test_map_persist_is_the_only_map_script():
    """Four scripts disagreed about what a saved map consists of, so a save could report success
    with the campaign-critical half missing."""
    others = [p.name for p in (REPO / "bringup").glob("*map*")
              if p.name not in ("map_persist.sh", "map_watch.py")]
    assert not others, f"map scripts are splitting again: {others}"
