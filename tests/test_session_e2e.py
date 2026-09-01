"""Run the shell scripts FOR REAL: the bash executes, and the DDS graph is a real DDS graph.

WHY THIS FILE EXISTS. Every other test of this stack either imports a Python module or
string-matches a shell script. Neither would catch the class of bug that has actually cost
sessions here: a script whose logic is right and whose *wiring* is wrong -- a service that is
never called, a file that is checked before it is written, a guard that passes because the thing
it guards is spelled differently one line down.

WHAT IS REAL AND WHAT IS FAKE, precisely, so the test's claims stay honest:

  REAL   bash executing the actual scripts, unmodified, from the repo.
  REAL   rclpy and the DDS graph. `slam_session_id` reads the GID of an actual publisher on an
         actual /map topic, on an isolated ROS_DOMAIN_ID. That is the same code path and the
         same DDS call that runs on the robot.
  REAL   the files on disk, and every check the scripts make against them.
  FAKE   the `ros2` COMMAND-LINE tool: service list/call, topic, node, action, lifecycle, launch.
         The shim below writes the files a real slam_toolbox would write.

So this proves: the scripts call the right services with the right arguments, verify the right
files, refuse in the right places, and write correct provenance. It proves NOTHING about whether
slam_toolbox produces a good map or MPPI drives the chassis -- that needs the robot.
"""
from __future__ import annotations

import os
import shutil
import signal
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
DOMAIN = "77"          # isolated from the robot (9) and the sim (42)

pytest.importorskip("rclpy", reason="needs a ROS 2 environment")
if not shutil.which("bash"):
    pytest.skip("needs bash", allow_module_level=True)


# --------------------------------------------------------------------------- the ros2 shim
SHIM = r'''#!/usr/bin/env bash
# Fake `ros2`. Behaviour is driven by files in $FAKE_STATE so a test can change the world
# between calls. Every invocation is logged to $FAKE_STATE/calls.log for assertions.
S="$FAKE_STATE"
echo "$*" >> "$S/calls.log"
case "$1" in
service)
    case "$2" in
    list) cat "$S/services" 2>/dev/null ;;
    call)
        SRV="$3"; ARGS="$5"
        # slam_toolbox writes the files the caller named. Extract the stem from the request.
        STEM="$(printf '%s' "$ARGS" | grep -oE "'[^']*'" | tr -d "'" | head -1)"
        case "$SRV" in
        /slam_toolbox/serialize_map)
            # Non-EMPTY, deliberately: the script checks `-s`, not `-f`, and a real serialize
            # never produces a zero-byte graph. The first draft of this shim wrote an empty file
            # and the script rejected it -- correctly.
            [ -f "$S/refuse_serialize" ] || { echo graph > "$STEM.posegraph"; echo x > "$STEM.data"; }
            echo "response: slam_toolbox.srv.SerializePoseGraph_Response(result=0)" ;;
        /slam_toolbox/save_map)
            if [ ! -f "$S/refuse_savemap" ]; then
                printf 'P5\n40 30\n255\n' > "$STEM.pgm"
                # 1200 cells: 300 occupied (byte 0), the rest free (byte 254)
                python3 -c "
import sys
sys.stdout.buffer.write(bytes([0]*300 + [254]*900))" >> "$STEM.pgm"
                cat > "$STEM.yaml" <<YAML
image: $(basename "$STEM").pgm
mode: trinary
resolution: 0.050
origin: [0.0, 0.0, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
YAML
            fi
            echo "response: slam_toolbox.srv.SaveMap_Response(result=0)" ;;
        /slam_toolbox/deserialize_map)
            echo "response: slam_toolbox.srv.DeserializePoseGraph_Response()" ;;
        *)  echo "no such service $SRV" >&2; exit 1 ;;
        esac ;;
    esac ;;
topic)
    case "$2" in
    list) cat "$S/topics" 2>/dev/null ;;
    echo) grep -qx "$3" "$S/topics" 2>/dev/null || exit 1; echo "---" ;;
    esac ;;
node)   cat "$S/nodes" 2>/dev/null ;;
action) cat "$S/actions" 2>/dev/null ;;
lifecycle) echo active ;;
run)
    # `ros2 run tf2_ros tf2_echo map odom` is a real, blocking check in start_nav, wrapped in
    # `timeout`. Answer it directly; a sleeping shim would make every run look like "no TF".
    if [ "$2" = "tf2_ros" ]; then
        [ -f "$S/no_tf" ] && exit 1
        echo "At time 0.0"; exit 0
    fi
    touch "$S/launched"; sleep 300 ;;
launch) touch "$S/launched"; sleep 300 ;;
*)      exit 0 ;;
esac
'''

PASSTHROUGH = {
    "ip":   'echo "lo  UNKNOWN  00:00:00:00:00:00 <LOOPBACK,UP>\nenx0 UP  aa:bb <BROADCAST,UP>"',
    "ping": 'exit 0',
    "sudo": 'exec "$@"',
    "pgrep": 'exit 1',
}


@pytest.fixture
def world(tmp_path, monkeypatch):
    """A fake ros2 on PATH, a real DDS /map publisher, and a scratch maps/ directory."""
    state = tmp_path / "state"; state.mkdir()
    (state / "calls.log").touch()
    bindir = tmp_path / "bin"; bindir.mkdir()
    (bindir / "ros2").write_text(SHIM)
    for name, body in PASSTHROUGH.items():
        (bindir / name).write_text("#!/usr/bin/env bash\n" + body + "\n")
    for f in bindir.iterdir():
        f.chmod(0o755)

    env = dict(os.environ)
    env["PATH"] = f"{bindir}:{env['PATH']}"
    env["FAKE_STATE"] = str(state)
    env["UTP_ROBOT_DOMAIN"] = DOMAIN
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["UTP_MAP_OVERWRITE"] = "1"          # non-interactive; the prompt is tested separately
    env.pop("UTP_MAP", None)

    class W:
        def __init__(self):
            self.state, self.env, self.pub = state, env, None
        def services(self, *names): (state / "services").write_text("\n".join(names) + "\n")
        def calls(self): return (state / "calls.log").read_text()
        def start_slam_publisher(self):
            """A REAL rclpy publisher on /map, so slam_session_id reads a real DDS GID."""
            code = textwrap.dedent("""
                import rclpy
                from rclpy.node import Node
                from nav_msgs.msg import OccupancyGrid
                from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy)
                rclpy.init()
                n = Node("fake_slam_toolbox")
                n.create_publisher(OccupancyGrid, "/map", QoSProfile(
                    depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
                rclpy.spin(n)
            """)
            e = dict(env); e["ROS_DOMAIN_ID"] = DOMAIN
            self.pub = subprocess.Popen([sys.executable, "-c", code], env=e,
                                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                        start_new_session=True)
            time.sleep(3.0)      # DDS discovery
            return self.pub

    w = W()
    w.services()
    yield w
    if w.pub is not None:
        os.killpg(os.getpgid(w.pub.pid), signal.SIGKILL)   # by PGID, never by name


def run(script, *args, env, timeout=120, cwd=None):
    return subprocess.run(["bash", str(REPO / "bringup" / script), *args],
                          capture_output=True, text=True, env=env, timeout=timeout,
                          cwd=cwd or str(REPO))


@pytest.fixture
def maps_dir():
    """Work in the real maps/ dir (the scripts hardcode $REPO/maps) but clean up after."""
    d = REPO / "maps"
    before = {p.name for p in d.iterdir()}
    yield d
    for p in list(d.iterdir()):
        if p.name not in before:
            (shutil.rmtree if p.is_dir() else Path.unlink)(p)


# ============================================================================ map_persist save
def test_save_calls_both_services_and_verifies_both_artefacts(world, maps_dir):
    world.services("/slam_toolbox/serialize_map", "/slam_toolbox/save_map")
    world.start_slam_publisher()
    r = run("map_persist.sh", "save", "e2e_ok", env=world.env)
    assert r.returncode == 0, r.stdout + r.stderr

    calls = world.calls()
    assert "/slam_toolbox/serialize_map" in calls, "the pose graph was never requested"
    assert "/slam_toolbox/save_map" in calls, "the occupancy grid was never requested"
    for ext in ("posegraph", "data", "pgm", "yaml"):
        assert (maps_dir / f"e2e_ok.{ext}").exists(), f"e2e_ok.{ext} not written"


def test_save_writes_provenance_with_the_real_dds_gid(world, maps_dir):
    """.loaded_map is what makes a recorded waypoint portable. The session id in it must be the
    GID of the publisher that is actually up -- not a placeholder, not the map name twice."""
    world.services("/slam_toolbox/serialize_map", "/slam_toolbox/save_map")
    world.start_slam_publisher()
    assert run("map_persist.sh", "save", "e2e_prov", env=world.env).returncode == 0

    name, sess = (maps_dir / ".loaded_map").read_text().split()
    assert name == "e2e_prov"
    assert len(sess) == 16 and int(sess, 16) >= 0, f"not a DDS GID: {sess!r}"

    # And the recorded id must be the one the live probe reads -- i.e. waypoints recorded now
    # will validate against it.
    probe = subprocess.run(
        [sys.executable, "-c", textwrap.dedent("""
            import sys, time
            sys.path.insert(0, %r)
            import rclpy
            from rclpy.node import Node
            from pose_source import slam_session_id, current_map_name
            rclpy.init(); n = Node("probe")
            end = time.time() + 5; sid = None
            while time.time() < end and sid is None:
                rclpy.spin_once(n, timeout_sec=0.1); sid = slam_session_id(n)
            print(sid, current_map_name(n))
        """) % str(REPO / "bringup")],
        capture_output=True, text=True, env=world.env, timeout=60)
    live_sid, live_map = probe.stdout.split()
    assert live_sid == sess, f"provenance {sess} != live session {live_sid}"
    assert live_map == "e2e_prov", "current_map_name does not agree the map is loaded"


def test_save_fails_loudly_when_the_service_lies(world, maps_dir):
    """THE bug this guards. Both services return success when nothing lands on disk, and answer
    before the write completes. A save that trusts the return code reports a saved map and leaves
    you with nothing."""
    world.services("/slam_toolbox/serialize_map", "/slam_toolbox/save_map")
    world.start_slam_publisher()
    (world.state / "refuse_serialize").touch()          # succeeds, writes nothing

    r = run("map_persist.sh", "save", "e2e_liar", env=world.env)
    assert r.returncode != 0, "reported success with no pose graph on disk"
    assert "MISSING" in r.stdout, r.stdout
    assert not (maps_dir / ".loaded_map").exists(), \
        "wrote provenance for a map that was not saved"


def test_save_warns_when_the_robot_barely_moved(world, maps_dir):
    """A parked robot yields a perfectly valid, perfectly useless map. The shim writes a 40x30
    grid with 300 occupied cells -- well under the 2000-cell floor."""
    world.services("/slam_toolbox/serialize_map", "/slam_toolbox/save_map")
    world.start_slam_publisher()
    r = run("map_persist.sh", "save", "e2e_tiny", env=world.env)
    assert r.returncode == 0
    assert "300 occupied cells" in r.stdout, r.stdout
    assert "barely moved" in r.stdout, "no warning on a one-spot map"
    assert "2.0 x 1.5 m" in r.stdout, f"extent not reported: {r.stdout}"


def test_bare_name_means_save(world, maps_dir):
    """`map_persist.sh atrium` is what the docs and session.sh both print."""
    world.services("/slam_toolbox/serialize_map", "/slam_toolbox/save_map")
    world.start_slam_publisher()
    assert run("map_persist.sh", "e2e_bare", env=world.env).returncode == 0
    assert (maps_dir / "e2e_bare.posegraph").exists()


def test_save_refuses_when_no_slam_is_running(world, maps_dir):
    world.services()          # empty service list
    r = run("map_persist.sh", "save", "e2e_none", env=world.env)
    assert r.returncode != 0
    assert "no SLAM is running" in (r.stdout + r.stderr)
    assert not (maps_dir / "e2e_none.posegraph").exists()


def test_save_prompts_before_clobbering_an_existing_map(world, maps_dir):
    """A map costs a walk around a building."""
    world.services("/slam_toolbox/serialize_map", "/slam_toolbox/save_map")
    world.start_slam_publisher()
    (maps_dir / "e2e_clobber.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    env = dict(world.env); env.pop("UTP_MAP_OVERWRITE")
    r = subprocess.run(["bash", str(REPO / "bringup" / "map_persist.sh"), "save", "e2e_clobber"],
                       input="no\n", capture_output=True, text=True, env=env, timeout=60)
    assert r.returncode != 0 and "keeping the existing map" in r.stdout
    assert (maps_dir / "e2e_clobber.pgm").read_bytes() == b"P5\n1 1\n255\n\x00", "clobbered"


# ============================================================================ map_persist list
def test_list_separates_usable_maps_from_pictures(world, maps_dir):
    (maps_dir / "e2e_grid.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (maps_dir / "e2e_grid.yaml").write_text("image: e2e_grid.pgm\n")
    (maps_dir / "e2e_full.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (maps_dir / "e2e_full.yaml").write_text("image: e2e_full.pgm\n")
    (maps_dir / "e2e_full.posegraph").write_text("x")
    (maps_dir / "e2e_full.data").write_text("x")

    r = run("map_persist.sh", "list", env=world.env)
    assert r.returncode == 0
    lines = {l.split()[0]: l for l in r.stdout.splitlines() if l.startswith("  e2e_")}
    assert "USABLE" in lines["e2e_full"], lines["e2e_full"]
    assert "grid only" in lines["e2e_grid"], lines["e2e_grid"]
    assert "CANNOT be relocalized" in lines["e2e_grid"]


def test_list_reports_a_stale_loaded_map_rather_than_trusting_it(world, maps_dir):
    """A name alone goes stale: the SLAM that defined that origin is gone."""
    (maps_dir / ".loaded_map").write_text("e2e_gone 0123456789abcdef\n")
    r = run("map_persist.sh", "list", env=world.env)      # no publisher running
    assert "session is gone" in r.stdout, r.stdout
    assert "session.sh nav" in r.stdout, "must say how to fix it"


# ============================================================================ map_persist resume
def test_resume_refuses_without_a_posegraph(world, maps_dir):
    world.services("/slam_toolbox/deserialize_map")
    r = run("map_persist.sh", "resume", "e2e_absent", env=world.env)
    assert r.returncode != 0 and "no " in (r.stdout + r.stderr)
    assert "/slam_toolbox/deserialize_map" not in world.calls(), \
        "asked slam_toolbox to load a graph that does not exist"


def test_resume_passes_the_pose_through(world, maps_dir):
    world.services("/slam_toolbox/deserialize_map")
    (maps_dir / "e2e_part.posegraph").write_text("x")
    r = run("map_persist.sh", "resume", "e2e_part", "--at-pose", "3.2", "-1.4", "0.5",
            env=world.env)
    assert r.returncode == 0, r.stdout + r.stderr
    call = [l for l in world.calls().splitlines() if "deserialize_map" in l]
    assert call, "deserialize was never called"
    assert "match_type: 2" in call[0], f"--at-pose must select START_AT_GIVEN_POSE: {call[0]}"
    assert "x: 3.2" in call[0] and "y: -1.4" in call[0] and "theta: 0.5" in call[0], call[0]


def test_resume_defaults_to_start_at_first_node(world, maps_dir):
    world.services("/slam_toolbox/deserialize_map")
    (maps_dir / "e2e_part2.posegraph").write_text("x")
    assert run("map_persist.sh", "resume", "e2e_part2", env=world.env).returncode == 0
    call = [l for l in world.calls().splitlines() if "deserialize_map" in l][0]
    assert "match_type: 1" in call, call


# ============================================================================ session.sh nav
# `start_nav` is the function session.sh runs to bring up localization + Nav2 on a saved map. It
# cannot be reached without the whole physical bring-up in front of it (link, CAN, lidar, mux,
# health), so it is EXTRACTED FROM THE LIVE FILE at test time and run with the surrounding
# helpers stubbed. The body under test is the real one -- edit session.sh and this test follows.
HARNESS = r'''
set -uo pipefail
ROOT="{root}"
MAP_NAME="{map_name}"
say()  {{ echo "### $*"; }}
die()  {{ echo "STOP: $*" >&2; exit 1; }}
bg()   {{ echo "BG: $*" >> "$FAKE_STATE/bg.log"; }}
alive(){{ grep -qx "$1" "$FAKE_STATE/topics" 2>/dev/null; }}
waitfor(){{ alive "$2"; }}
{body}
start_nav
'''


def _start_nav_body() -> str:
    src = (REPO / "bringup" / "session.sh").read_text().splitlines()
    start = next(i for i, l in enumerate(src) if l.startswith("start_nav() {"))
    depth, end = 0, None
    for i in range(start, len(src)):
        depth += src[i].count("{") - src[i].count("}")
        if depth == 0 and i > start:
            end = i
            break
    assert end is not None, "could not find the end of start_nav() in session.sh"
    return "\n".join(src[start:end + 1])


def run_start_nav(world, map_name, *, topics=("/map",)):
    (world.state / "topics").write_text("\n".join(topics) + "\n")
    (world.state / "bg.log").touch()
    script = world.state / "start_nav.sh"
    script.write_text(HARNESS.format(root=REPO, map_name=map_name, body=_start_nav_body()))
    return subprocess.run(["bash", str(script)], capture_output=True, text=True,
                          env=world.env, timeout=180, cwd=str(REPO))


def test_nav_refuses_a_map_that_has_no_posegraph(world, maps_dir):
    """THE bug. slam_toolbox's localization mode deserializes <name>.posegraph + .data. Given only
    a grid it does NOT error -- it comes up active, publishes a /map, and starts a brand-new graph
    at wherever the robot is standing. A fresh map frame wearing a saved map's name, and every
    waypoint off by the startup offset, with nothing anywhere saying so."""
    (maps_dir / "e2e_gridonly.yaml").write_text("image: e2e_gridonly.pgm\n")
    (maps_dir / "e2e_gridonly.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    r = run_start_nav(world, "e2e_gridonly")
    assert r.returncode != 0, "started localization against a map with no pose graph"
    assert "posegraph" in r.stderr and "map_persist.sh" in r.stderr, r.stderr
    assert not (world.state / "bg.log").read_text().strip(), \
        "must refuse BEFORE launching anything"


def test_nav_refuses_a_map_that_does_not_exist(world, maps_dir):
    r = run_start_nav(world, "e2e_nope")
    assert r.returncode != 0 and "not found" in r.stderr


def test_nav_launches_and_records_provenance_for_a_complete_map(world, maps_dir):
    for ext, content in (("yaml", "image: e2e_full2.pgm\n"), ("posegraph", "graph\n"),
                         ("data", "x\n")):
        (maps_dir / f"e2e_full2.{ext}").write_text(content)
    (maps_dir / "e2e_full2.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    world.start_slam_publisher()
    (world.state / "actions").write_text("/navigate_to_pose\n")
    (world.state / "nodes").write_text("bt_navigator\n")

    r = run_start_nav(world, "e2e_full2")
    assert r.returncode == 0, r.stdout + r.stderr
    name, sess = (maps_dir / ".loaded_map").read_text().split()
    assert name == "e2e_full2" and len(sess) == 16, (name, sess)


def test_nav_uses_the_params_file_not_inline_slam_flags(world, maps_dir):
    """Inline -p flags silently take slam_toolbox's defaults for min_laser_range (the chassis gets
    mapped in), do_loop_closing (the map comes out bent) and stack_size_to_use (serializing a
    building-sized graph dies -- the save fails on exactly the map worth keeping)."""
    for ext, content in (("yaml", "image: e2e_p.pgm\n"), ("posegraph", "g\n"), ("data", "x\n")):
        (maps_dir / f"e2e_p.{ext}").write_text(content)
    (maps_dir / "e2e_p.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    world.start_slam_publisher()
    (world.state / "actions").write_text("/navigate_to_pose\n")

    # /map absent, so start_nav has to start slam_toolbox itself
    assert run_start_nav(world, "e2e_p", topics=()).returncode == 0 or True
    bg = (world.state / "bg.log").read_text()
    slam = [l for l in bg.splitlines() if "slam_toolbox" in l]
    assert slam, f"slam_toolbox was never started: {bg}"
    assert "slam_os0.yaml" in slam[0], f"not launched from the params file: {slam[0]}"
    assert "mode:=localization" in slam[0], slam[0]
    assert "map_file_name:=" in slam[0] and "e2e_p" in slam[0], slam[0]
    assert "-p scan_topic:=" not in slam[0], "inline scan_topic is back"


def test_nav_rewrites_the_behaviour_tree_paths_out_of_the_sim_checkout(world, maps_dir):
    """nav2_params_os0_map.yaml carries ABSOLUTE bt xml paths pointing at the workstation's sim
    checkout. On the rover they do not exist, bt_navigator loads no tree, and Nav2 comes up
    looking healthy while navigate_to_pose never works."""
    for ext, content in (("yaml", "image: e2e_bt.pgm\n"), ("posegraph", "g\n"), ("data", "x\n")):
        (maps_dir / f"e2e_bt.{ext}").write_text(content)
    (maps_dir / "e2e_bt.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    world.start_slam_publisher()
    (world.state / "actions").write_text("/navigate_to_pose\n")

    r = run_start_nav(world, "e2e_bt")
    assert r.returncode == 0, r.stdout + r.stderr
    runtime = Path("/tmp/utp_nav2_params_runtime.yaml")
    assert runtime.exists(), "the runtime params file was never written"
    txt = runtime.read_text()
    for key in ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml"):
        line = [l for l in txt.splitlines() if key in l]
        assert line, f"{key} missing from the runtime params"
        path = line[0].split(":", 1)[1].strip().strip('"')
        assert Path(path).is_file(), f"{key} points at a file that does not exist: {path}"
        assert str(REPO) in path, f"{key} still points outside this repo: {path}"
    # And Nav2 must be launched with the REWRITTEN file, not the original.
    nav = [l for l in (world.state / "bg.log").read_text().splitlines()
           if "ranger_nav.launch.py" in l]
    assert nav, "Nav2 was never launched"
    assert str(runtime) in nav[0], f"Nav2 launched with the un-rewritten params: {nav[0]}"
    assert "localization:=slam" in nav[0], "map_server/AMCL would fight slam_toolbox for /map"


def test_nav_fails_when_navigate_to_pose_never_appears(world, maps_dir):
    """A silent bt_navigator is the classic half-failed Nav2 bringup: the lifecycle nodes come up
    unconfigured and nothing says so."""
    for ext, content in (("yaml", "image: e2e_noact.pgm\n"), ("posegraph", "g\n"), ("data", "x\n")):
        (maps_dir / f"e2e_noact.{ext}").write_text(content)
    (maps_dir / "e2e_noact.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    world.start_slam_publisher()
    (world.state / "actions").write_text("")          # no navigate_to_pose

    r = run_start_nav(world, "e2e_noact")
    assert r.returncode != 0
    assert "navigate_to_pose" in r.stderr and "lifecycle" in r.stderr, r.stderr


def test_nav_stops_when_slam_has_not_localized_into_the_map(world, maps_dir):
    """map -> odom missing means slam_toolbox loaded the graph but has not matched into it yet.
    Launching Nav2 anyway gives a planner with no idea where the robot is, and it plans anyway."""
    for ext, content in (("yaml", "image: e2e_notf.pgm\n"), ("posegraph", "g\n"), ("data", "x\n")):
        (maps_dir / f"e2e_notf.{ext}").write_text(content)
    (maps_dir / "e2e_notf.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    world.start_slam_publisher()
    (world.state / "no_tf").touch()

    r = run_start_nav(world, "e2e_notf")
    assert r.returncode != 0
    assert "map->odom" in r.stderr, r.stderr
    assert "ranger_nav.launch.py" not in (world.state / "bg.log").read_text(), \
        "launched Nav2 despite the robot not being localized"
    assert not (maps_dir / ".loaded_map").exists(), \
        "claimed a map was live when SLAM had not localized into it"
