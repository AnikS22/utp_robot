"""THE SEAM TEST. One authoritative walk of the whole chain, in process, asserting every handoff.

    saved map -> localization -> waypoint provenance -> Nav2 goal -> /cmd_vel_nav -> safety mux
    -> /cmd_vel -> Nav2 result -> FSM outcome -> perception fusion -> press

WHY THIS FILE EXISTS, AND WHY IT IS NOT A UNIT TEST FILE
-------------------------------------------------------
On 2026-09-01 the repository's ~370-test suite was green while every one of these failed on
hardware. Each component was tested; nothing tested what one component HANDS to the next:

  * Nav2's costmaps consumed /scan_filtered (the RAW projection, containing the robot's own arm
    and mast) while the self-occlusion mask published /scan. Both BEST_EFFORT, so the WRONG data
    arrived perfectly, at full rate, for hours. Nav2 marked lethal cells around its own footprint
    and refused to plan on open floor.
  * ros_world.navigate_to_goal evaluated the camera BEFORE driving, so three consecutive live
    trials recorded path_length_m 0.0 with the goal 8 m away. It never navigated at all.
  * A distance gate subtracted a MAP-frame waypoint from an ODOM-frame pose: 5.09 m reported,
    2.59 m true. Two origins, one subtraction.
  * Every waypoint carried map_name 'atrium' while session.sh defaulted MAP_NAME to 'atrium2d'.
    Nothing compared them, so a valid coordinate named a different physical place.
  * Every non-SUCCEEDED Nav2 status was reported as `blocked`, so a CANCELLED goal would start
    reason -> ground -> press and put an arm at a wall.

Every one of those lives in a seam. So this file walks the seams and nothing else.

WHAT IS REAL AND WHAT IS FAKE, stated precisely so the claims stay honest
------------------------------------------------------------------------
  REAL  bringup/nav2_goto.py's own main(), executed, including its refusal rules, its quaternion
        arithmetic and its GoalStatus -> exit-code table.
  REAL  bringup/ros_world.py's RosWorld.navigate_to_goal / _drive_leg_staged / _nav2_unavailable,
        executed, including the distance gate and the stdout vocabulary parsing.
  REAL  bringup/map_persist.sh, executed by bash, against a synthetic maps/ directory.
  REAL  safety/blockage_fusion.py, safety/map_frame.py, safety/waypoint_drive.corridor_blocked.
  REAL  nav2_bringup/nav2_params_os0_map.yaml, nav2_bringup/ranger_nav.launch.py,
        config/safety.yaml, bringup/scan_relay.py -- parsed, not string-matched where it matters.
  REAL  captures/trial_ours_001/scan.json and trial_ours_002/scan.json -- genuine hardware scans.

  FAKE  rclpy / the NavigateToPose action server (the true hardware boundary).
  FAKE  bringup/ros_world._ros -- the single module-level function through which EVERY ROS-side
        action in that file leaves the process. Faking it fakes the hardware edge and NOTHING
        above it: the fake dispatches nav2_goto.py calls into nav2_goto.main() in this process,
        so the exit code and the stdout that RosWorld parses are the REAL ones the real script
        would have produced.
  FAKE  the VLM HTTP call (bringup/ask_blockage.ask_camera). Nothing below it: ask_blockage.ask
        itself runs, reads the real scan.json off disk and calls the real safety.blockage_fusion.
        A camera verdict is an INPUT to the seam, not part of it.

WHAT ONLY THE ROBOT CAN PROVE is at the bottom of this file as explicit skips, each naming what
must be verified on hardware. That skip list is a deliverable: it is the honest statement of what
this file does NOT prove.
"""
from __future__ import annotations

import contextlib
import importlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import types
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
for _p in (str(REPO), str(REPO / "bringup")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

PARAMS_FILE = REPO / "nav2_bringup" / "nav2_params_os0_map.yaml"
LAUNCH_FILE = REPO / "nav2_bringup" / "ranger_nav.launch.py"
SAFETY_YAML = REPO / "config" / "safety.yaml"
SCAN_RELAY = REPO / "bringup" / "scan_relay.py"
SESSION_SH = REPO / "bringup" / "session.sh"
MAP_PERSIST = REPO / "bringup" / "map_persist.sh"

# The two genuine hardware scans this chain was debugged against. trial_ours_001: 0.72 m from
# CLOSED GLASS DOORS, camera says "an open walkway with pillars". trial_ours_002: the same
# building, camera sees the doors, the lidar sees nothing at all in front of the robot.
CAP_LIDAR_ONLY = REPO / "captures" / "trial_ours_001"     # lidar is the only witness
CAP_CAMERA_ONLY = REPO / "captures" / "trial_ours_002"    # camera is the only witness


# =============================================================================================
# THE HARNESS. Everything below stands in for the hardware edge and nothing above it.
# =============================================================================================

def _read_capture_scan(cap: Path) -> dict:
    """A real saved LaserScan, or skip. grab_frame.py writes exactly these four fields."""
    f = cap / "scan.json"
    if not f.is_file():
        pytest.skip(f"{f} is absent; this assertion needs the real hardware capture")
    d = json.loads(f.read_text())
    return {"frame": d.get("frame", "base_link"),
            "angle_min": float(d["angle_min"]),
            "angle_increment": float(d["angle_increment"]),
            "ranges": [float(r) for r in d["ranges"]]}


def _require_glass_door_signature(scan: dict, cap: Path) -> None:
    """captures/ is gitignored, local scratch data. As of 2026-09-05, trial_ours_001/scan.json --
    the near-miss this handoff was written to prevent recurring -- has been overwritten by a
    later, unrelated capture reusing the same trial name (an open corridor, nearest forward
    return >3.9 m); the original is not recoverable from git. Rather than assert a near-miss
    against a scan that is no longer one (which would test nothing) or silently pass, skip
    loudly, by name, when the file no longer carries its documented signature -- see
    docs/TESTING.md for how to re-capture it."""
    ranges, a0, ai = scan["ranges"], scan["angle_min"], scan["angle_increment"]

    def bearing(i):
        return (math.degrees(a0 + i * ai) + 180.0) % 360.0 - 180.0

    fwd = [ranges[i] for i in range(len(ranges))
           if abs(bearing(i)) <= 20.0 and math.isfinite(ranges[i])]
    nearest = min(fwd, default=None)
    if nearest is None or nearest > 1.0:
        pytest.skip(
            f"{cap.relative_to(REPO)}/scan.json no longer holds the glass-door near-miss this "
            f"test needs (nearest forward return {nearest!r} m, expected ~0.70-0.72 m). The file "
            f"has been overwritten by a later, unrelated capture reusing this trial name -- "
            f"re-capture it (see docs/TESTING.md) to restore this regression check.")


def _clear_scan(n: int = 1031) -> dict:
    """A synthetic scan of an EMPTY room: 1031 bins, every return at 8 m. Same geometry as the
    real OS0 projection (angle_min -pi, increment 0.0061) so the corridor maths is identical."""
    return {"frame": "base_link", "angle_min": -math.pi, "angle_increment": 0.0061,
            "ranges": [8.0] * n}


class FakeRos:
    """Stands in for bringup/ros_world._ros -- the ONE function through which every ROS-side
    action in ros_world.py leaves the process.

    It is a dispatcher, not a simulator. `grab_frame.py` writes a capture directory with the scan
    it was configured with; `waypoints.py where` answers a pose; `nav2_goto.py` is executed FOR
    REAL, in this process, against a fake action server -- so the (returncode, stdout) pair that
    RosWorld then parses is produced by nav2_goto's own exit-code table rather than invented here.
    That is the seam under test: a mapping that is correct inside nav2_goto and lossy at the
    process boundary is exactly the bug class this file exists for.
    """

    def __init__(self, repo: Path, *, scan: dict, pose=(0.0, 0.0, 0.0),
                 goal_status: int = 4, server: bool = True, accepted: bool = True):
        self.repo = repo
        self.scan = scan
        self.pose = pose
        self.goal_status = goal_status
        self.server = server
        self.accepted = accepted
        self.camera_patch = ""                 # which VLM entry point was faked
        self.calls: list[list[str]] = []       # every argv, in order
        self.nav2_sent: list = []              # the PoseStamped goals actually sent
        self.nav2_runs = 0

    # -- what a caller asserts on -------------------------------------------------------------
    def scripts(self) -> list[str]:
        return [Path(a).name for c in self.calls for a in c if a.endswith(".py")]

    def scripts_after(self, name: str) -> list[str]:
        """Every script invoked AFTER the last call to `name`. This is how "a cancelled goal must
        not start perception" is asserted: perception BEFORE the leg is the distance gate and is
        correct; perception AFTER a control-plane event is the arm-at-a-wall bug."""
        seq = self.scripts()
        if name not in seq:
            return seq
        return seq[len(seq) - 1 - seq[::-1].index(name) + 1:]

    # -- the dispatcher -----------------------------------------------------------------------
    def __call__(self, args, timeout: int = 120):
        args = [str(a) for a in args]
        self.calls.append(args)
        script = next((Path(a).name for a in args if a.endswith(".py")), "")
        if script == "grab_frame.py":
            return self._grab_frame(args)
        if script == "waypoints.py":
            if "where" in args:
                return self._where()
            if "goto" in args:
                # THE ODOM DEAD-RECKONING DRIVER. Reaching this at all is a claim about the chain,
                # so it is recorded and answered with a deliberately inert result.
                return self._done(0, "")
            return self._done(0, "")
        if script == "nav2_goto.py":
            return self._nav2_goto(args)
        return self._done(0, "")

    # -- the individual edges -----------------------------------------------------------------
    @staticmethod
    def _done(rc, out, err=""):
        return subprocess.CompletedProcess(args=[], returncode=rc, stdout=out, stderr=err)

    def _where(self):
        x, y, yaw_deg = self.pose
        return self._done(0, f"now: x={x:.4f} y={y:.4f} yaw={yaw_deg:.2f}\n")

    def _grab_frame(self, args):
        name = args[args.index("--name") + 1]
        cap = self.repo / "captures" / name
        cap.mkdir(parents=True, exist_ok=True)
        from PIL import Image
        Image.new("RGB", (8, 8), (30, 30, 30)).save(cap / "rgb.png")
        (cap / "cam.json").write_text(json.dumps({"width": 8, "height": 8}))
        (cap / "scan.json").write_text(json.dumps(self.scan))
        return self._done(0, f"saved {cap}\n")

    def _nav2_goto(self, args):
        """Execute the REAL bringup/nav2_goto.py main() against a fake action server."""
        self.nav2_runs += 1
        argv = args[args.index(next(a for a in args if a.endswith("nav2_goto.py"))) + 1:]
        rc, out, err, sent = run_nav2_goto(
            argv, goal_status=self.goal_status, server=self.server, accepted=self.accepted)
        self.nav2_sent.extend(sent)
        return self._done(rc, out, err)


# ---------------------------------------------------------------------------- fake rclpy layer
class _Fut:
    def __init__(self, done=True, value=None):
        self._done, self._value = done, value

    def done(self):
        return self._done

    def result(self):
        return self._value


class _Handle:
    def __init__(self, accepted, status):
        self.accepted = accepted
        self._status = status
        self.cancelled = False

    def get_result_async(self):
        return _Fut(True, types.SimpleNamespace(status=self._status))

    def cancel_goal_async(self):
        self.cancelled = True
        return _Fut(True)


def _install_fake_rclpy(monkeypatch, *, server, accepted, status, sent):
    class _Client:
        def __init__(self, *a, **k):
            pass

        def wait_for_server(self, timeout_sec=0):
            return server

        def send_goal_async(self, goal):
            sent.append(goal)
            return _Fut(True, _Handle(accepted, status))

    class _Node:
        def __init__(self, *a, **k):
            pass

        def get_clock(self):
            return types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(to_msg=lambda: 0))

        def destroy_node(self):
            pass

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.ok = lambda: True
    rclpy.spin_once = lambda *a, **k: None
    rclpy.spin_until_future_complete = lambda *a, **k: None
    act = types.ModuleType("rclpy.action"); act.ActionClient = _Client
    nod = types.ModuleType("rclpy.node"); nod.Node = _Node
    rclpy.action, rclpy.node = act, nod

    nav_act = types.ModuleType("nav2_msgs.action")

    class _NTP:
        class Goal:
            def __init__(self):
                self.pose = None

    nav_act.NavigateToPose = _NTP
    nav_pkg = types.ModuleType("nav2_msgs"); nav_pkg.action = nav_act

    geo = types.ModuleType("geometry_msgs.msg")

    class _PoseStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id="", stamp=None)
            self.pose = types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))

    geo.PoseStamped = _PoseStamped
    geo_pkg = types.ModuleType("geometry_msgs"); geo_pkg.msg = geo

    for name, mod in (("rclpy", rclpy), ("rclpy.action", act), ("rclpy.node", nod),
                      ("nav2_msgs", nav_pkg), ("nav2_msgs.action", nav_act),
                      ("geometry_msgs", geo_pkg), ("geometry_msgs.msg", geo)):
        monkeypatch.setitem(sys.modules, name, mod)


def _install_fake_waypoints_module(monkeypatch):
    """bringup/waypoints.py imports rclpy at module scope, so it cannot be imported off the robot.

    The fake reproduces exactly ONE thing: `load()` reads the YAML store named by UTP_WAYPOINTS.
    That is the seam -- the file format and the override -- and test_the_waypoint_store_override_
    is_real() below pins the fake to the real module's own STORE line so this cannot drift into
    a lie.
    """
    mod = types.ModuleType("waypoints")

    def load():
        store = Path(os.environ["UTP_WAYPOINTS"])
        return yaml.safe_load(store.read_text()) or {}

    mod.load = load
    monkeypatch.setitem(sys.modules, "waypoints", mod)


@contextlib.contextmanager
def _fake_ros_layer(*, goal_status, server, accepted, sent):
    """monkeypatch is function-scoped; nav2_goto is run from inside FakeRos, so it needs its own."""
    mp = pytest.MonkeyPatch()
    try:
        _install_fake_rclpy(mp, server=server, accepted=accepted, status=goal_status, sent=sent)
        _install_fake_waypoints_module(mp)
        yield mp
    finally:
        mp.undo()


def run_nav2_goto(argv, *, goal_status=4, server=True, accepted=True):
    """Run bringup/nav2_goto.py's real main() in this process. Returns (rc, stdout, stderr, sent).

    safety.map_frame is NOT faked: it is pure Python and it is the module that defines what a
    portable waypoint is. Faking it would delete the assertion.
    """
    sent: list = []
    out, err = io.StringIO(), io.StringIO()
    with _fake_ros_layer(goal_status=goal_status, server=server, accepted=accepted, sent=sent) as mp:
        mp.setattr(sys, "argv", ["nav2_goto.py"] + list(argv))
        mod = importlib.import_module("nav2_goto")
        importlib.reload(mod)
        try:
            with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
                rc = mod.main()
        except SystemExit as e:            # argparse
            rc = int(e.code or 0)
    return rc, out.getvalue(), err.getvalue(), sent


# ------------------------------------------------------------------- the synthetic saved world
GOOD_MAP = "atrium_seam"


def _write_map(maps: Path, name: str, *, posegraph: bool) -> None:
    """A saved map on disk. `posegraph=False` is the .pgm-only map that CANNOT be relocalized."""
    (maps / f"{name}.pgm").write_bytes(b"P5\n8 8\n255\n" + bytes([254] * 64))
    (maps / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nmode: trinary\nresolution: 0.050\norigin: [0.0, 0.0, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    if posegraph:
        (maps / f"{name}.posegraph").write_text("posegraph\n")
        (maps / f"{name}.data").write_text("data\n")


@pytest.fixture
def world(tmp_path, monkeypatch):
    """A synthetic saved map, waypoint store, provenance file and capture directory.

    NOTHING here touches maps/ or captures/ in the real repo: ros_world.REPO is repointed, and
    the two file-backed globals ros_world and nav2_goto consult (UTP_WAYPOINTS, UTP_LOADED_MAP)
    are both overridden. A test that mocks a subsystem has to mock ALL of its inputs, including
    the ones that live on disk outside the process -- six tests in tests/test_nav_backend.py
    broke on exactly that the day a real map was first loaded on the robot.
    """
    repo = tmp_path / "repo"
    maps = repo / "maps"
    maps.mkdir(parents=True)
    (repo / "captures").mkdir()
    (repo / "bringup").mkdir()

    _write_map(maps, GOOD_MAP, posegraph=True)
    _write_map(maps, "gridonly", posegraph=False)
    (maps / ".loaded_map").write_text(f"{GOOD_MAP} 010fa5754e02d7ed\n")

    store = maps / "waypoints.yaml"
    store.write_text(yaml.safe_dump({
        "door": {"frame": "map", "map_name": GOOD_MAP,
                 "x": 5.777, "y": 6.068, "yaw": 0.9076},
        "far_door": {"frame": "map", "map_name": GOOD_MAP,
                     "x": 40.0, "y": 30.0, "yaw": 0.0},
        "odom_door": {"frame": "odom", "x": 1.5, "y": 2.5, "yaw": 0.0},
        "nameless": {"frame": "map", "x": 1.5, "y": 2.5, "yaw": 0.0},
        "wrong_map": {"frame": "map", "map_name": "atrium2d",
                      "x": 5.777, "y": 6.068, "yaw": 0.9076},
    }))

    monkeypatch.setenv("UTP_WAYPOINTS", str(store))
    monkeypatch.setenv("UTP_LOADED_MAP", str(maps / ".loaded_map"))
    monkeypatch.delenv("UTP_NAV_BACKEND", raising=False)

    class W:
        def __init__(self):
            self.repo, self.maps, self.store = repo, maps, store
            self.loaded_map = maps / ".loaded_map"

        def set_loaded_map(self, name):
            self.loaded_map.write_text(f"{name} 010fa5754e02d7ed\n")

    return W()


def _ros_world_module():
    """bringup/ros_world.py, imported. An unimportable ros_world IS a broken chain, so this
    FAILS rather than skips -- except for the pipeline repo itself, which is a machine fact."""
    try:
        return importlib.import_module("ros_world")
    except ImportError as e:
        if "utp" in str(e) or "torch" in str(e):
            pytest.skip(f"the pipeline repo (~/unlocking-the-path) is not importable here: {e}")
        raise


def _patch_camera(monkeypatch, rw, camera: dict):
    """Fake ONLY the VLM, at the lowest point it can be faked.

    bringup/ask_blockage.py owns the camera+scan fusion boundary: ask() calls ask_camera() (the
    OpenAI request -- the true network/hardware edge), then reads scan.json from the SAME capture
    directory and ORs the two through safety.blockage_fusion.fuse. Faking ask_camera therefore
    leaves the real fusion, the real scan file and the real ranges in the chain; faking ask()
    would delete the very handoff this file exists to test.

    If ask_camera ever disappears the fake drops back to ask(), and says so loudly, because a
    silently camera-only harness is exactly the 2026-09-01 configuration.
    """
    ab = importlib.import_module("ask_blockage")
    if hasattr(ab, "ask_camera"):
        monkeypatch.setattr(ab, "ask_camera", lambda cap: dict(camera))
        return "ask_camera"
    monkeypatch.setattr(rw, "ask_blockage", lambda cap: dict(camera))
    return "ask"


def make_world(world, monkeypatch, *, goal, scan=None, camera=None,
               goal_status=4, server=True, accepted=True, pose=(0.0, 0.0, 0.0)):
    """A RosWorld wired to the fake hardware edge. Returns (RosWorld instance, FakeRos)."""
    rw = _ros_world_module()
    fake = FakeRos(world.repo, scan=scan if scan is not None else _clear_scan(), pose=pose,
                   goal_status=goal_status, server=server, accepted=accepted)
    monkeypatch.setattr(rw, "REPO", world.repo)
    monkeypatch.setattr(rw, "_ros", fake)
    cam = camera if camera is not None else {"blocked": False, "kind": "",
                                             "description": "an empty corridor"}
    fake.camera_patch = _patch_camera(monkeypatch, rw, cam)
    w = rw.RosWorld(goal=goal, dry_run=False, capture_prefix="seam")
    return w, fake


# =============================================================================================
# HANDOFF 1.  SAVED MAP  ->  LOCALIZATION
# =============================================================================================

def test_handoff_1_map_to_localization_refuses_a_grid_without_a_pose_graph(tmp_path):
    """GUARDS: "a .pgm without a .posegraph cannot be relocalized into -- and slam_toolbox does
    not error on one, it silently starts a NEW graph at the robot's current pose."

    That is the fresh-SLAM frame safety/map_frame.py exists to refuse, wearing a saved map's
    name: the TF tree looks identical, /map is published, Nav2 comes up healthy, and every
    waypoint is off by the startup offset. maps/atrium2d is exactly such a map and it was the
    session.sh default on 2026-09-01.

    This runs the REAL bringup/map_persist.sh -- the script that encodes the rule -- against a
    synthetic maps/ directory, so the rule is executed, not paraphrased.
    """
    if not shutil.which("bash"):
        pytest.skip("needs bash")
    sandbox = tmp_path / "repo"
    (sandbox / "bringup").mkdir(parents=True)
    (sandbox / "maps").mkdir()
    shutil.copy2(MAP_PERSIST, sandbox / "bringup" / "map_persist.sh")
    # env.sh sources /opt/ros and is not what is under test here; the `list` verb needs nothing
    # from it. Stubbing it keeps this assertion about the map rule and not about the ROS install.
    (sandbox / "bringup" / "env.sh").write_text("# stub for the seam test\n")
    _write_map(sandbox / "maps", "relocalizable", posegraph=True)
    _write_map(sandbox / "maps", "picture_only", posegraph=False)

    r = subprocess.run(["bash", str(sandbox / "bringup" / "map_persist.sh"), "list"],
                       capture_output=True, text=True, timeout=120)
    out = r.stdout
    assert "relocalizable" in out and "picture_only" in out, \
        f"map_persist.sh list did not enumerate the synthetic maps:\n{out}\n{r.stderr}"

    line = next(l for l in out.splitlines() if l.strip().startswith("relocalizable"))
    assert "USABLE" in line, (
        f"a map with .pgm + .yaml + .posegraph + .data must be accepted as relocalizable: {line}")

    line = next(l for l in out.splitlines() if l.strip().startswith("picture_only"))
    assert "CANNOT be relocalized" in line, (
        f"a .pgm/.yaml pair with no pose graph must be REFUSED, not listed as a map: {line}. "
        f"slam_toolbox does not error on one -- it starts a new graph at the robot's current "
        f"pose, and every map-frame waypoint is then off by the startup offset.")


def test_handoff_1_session_nav_refuses_to_localize_into_a_grid_only_map():
    """The other end of the same handoff: `map_persist.sh list` REPORTS the rule, session.sh nav
    must ENFORCE it before slam_toolbox is launched. Listing a bad map as bad buys nothing if the
    launcher will still deserialize it."""
    src = SESSION_SH.read_text()
    nav = src[src.index("start_nav()"):]
    for ext in ("posegraph", "data"):
        direct = f'-f "maps/$MAP_NAME.{ext}"' in nav
        looped = (re.search(r"for ext in [^\n]*\b%s\b" % ext, nav) is not None
                  and '-f "maps/$MAP_NAME.$ext"' in nav)
        assert direct or looped, (
            f"session.sh nav does not test for maps/$MAP_NAME.{ext} before launching "
            f"localization. Without the pose graph, `mode: localization` comes up ACTIVE, "
            f"publishes /map, and silently starts a brand-new graph whose origin is wherever the "
            f"robot is standing -- the fresh-SLAM frame wearing the saved map's name.")
    # GENERALISED 2026-09-05: this used to pin the literal default 'atrium' by name. The project
    # has since grown a second, legitimate map (elevator/floor2) and the default has been renamed
    # more than once as a result -- the invariant that actually matters, and is still incident 1's
    # shape, is that WHATEVER session.sh defaults to is a map that exists AND is relocalizable
    # (has a pose graph), not a specific name.
    m = re.search(r'MAP_NAME=\$\{MAP_NAME:-([^}]+)\}', src)
    assert m, "session.sh no longer gives MAP_NAME a default -- an unset MAP_NAME must fail loudly"
    default = m.group(1)
    maps_dir = REPO / "maps"
    for ext in ("yaml", "posegraph", "data"):
        assert (maps_dir / f"{default}.{ext}").is_file(), (
            f"session.sh defaults MAP_NAME to {default!r} but maps/{default}.{ext} is missing. "
            f"'atrium2d' was the default on 2026-09-01: grid-only, and it disagreed with every "
            f"waypoint on disk. A default that cannot be relocalized into is the same bug.")


# =============================================================================================
# HANDOFF 2.  LOCALIZATION  ->  WAYPOINT PROVENANCE
# =============================================================================================

def test_handoff_2_waypoint_recorded_in_another_map_is_refused_before_any_goal_is_sent(world):
    """GUARDS 2026-09-01: every waypoint on disk carried map_name 'atrium' while session.sh
    defaulted MAP_NAME to 'atrium2d', and NOTHING compared the two.

    Two maps of the same building have unrelated origins, so a coordinate valid in one names a
    different physical place in the other. The failure mode is a CONFIDENT ARRIVAL AT THE WRONG
    SPOT -- the hardest kind to notice, because nothing errors and the robot looks purposeful.

    Three things are asserted, and all three are the handoff rather than the parts:
      * the refusal happens BEFORE anything reaches the action client (nothing sent);
      * it names BOTH maps, or the operator cannot tell which one to load;
      * the exit code is one RosWorld classifies as "cannot serve", never as a nav outcome.
    """
    world.set_loaded_map("atrium2d")
    rc, out, err, sent = run_nav2_goto(["door", "--go"])
    assert not sent, "a provenance mismatch must be caught before the goal is built and sent"
    assert rc == 3, f"expected the refusal code 3, got {rc}. stdout={out!r} stderr={err!r}"
    assert GOOD_MAP in err and "atrium2d" in err, (
        f"the refusal must name BOTH the waypoint's map and the loaded map. Got: {err!r}")
    assert "blocked" not in out and "arrived" not in out, (
        "a refusal must not speak the FSM's navigation vocabulary on stdout")


def test_handoff_2_a_nameless_map_waypoint_is_refused(world):
    """A `frame: map` waypoint with no map_name came from a fresh SLAM session whose origin is
    wherever the robot booted. In the TF tree it is indistinguishable from a localized pose."""
    rc, out, err, sent = run_nav2_goto(["nameless", "--go"])
    assert not sent and rc == 3, f"rc={rc} sent={len(sent)} err={err!r}"
    assert "not portable" in err or "carries no map name" in err


def test_handoff_2_matching_provenance_is_accepted(world):
    """The rule must be a gate, not a wall: the correct map must pass, or the refusal above is
    just an outage."""
    rc, out, err, sent = run_nav2_goto(["door", "--go"])
    assert rc == 0 and len(sent) == 1, f"rc={rc} err={err!r}"


def test_handoff_2_the_waypoint_store_override_is_real():
    """This file's fake `waypoints.load()` reads UTP_WAYPOINTS. Pin that to the real module so the
    fake cannot quietly become a lie about where waypoints live."""
    src = (REPO / "bringup" / "waypoints.py").read_text()
    assert 'os.environ.get("UTP_WAYPOINTS"' in src, \
        "bringup/waypoints.py no longer honours UTP_WAYPOINTS; the fake in this file is now wrong"
    src = (REPO / "bringup" / "nav2_goto.py").read_text()
    assert 'os.environ.get("UTP_LOADED_MAP"' in src, \
        "nav2_goto.py no longer honours UTP_LOADED_MAP; provenance became untestable"


# =============================================================================================
# HANDOFF 3.  WAYPOINT  ->  NAV2 GOAL
# =============================================================================================

def test_handoff_3_the_goal_is_the_waypoint_in_the_map_frame(world):
    """The store's x/y/yaw must arrive at the action server unchanged, in the `map` frame, with a
    correctly built quaternion. A yaw silently dropped or halved is a robot that arrives at the
    right place facing the wrong way, and the door-facing pose is what every downstream look
    rung and the 35.1 deg camera half-frame are calibrated against."""
    wp = yaml.safe_load(world.store.read_text())["door"]
    rc, out, err, sent = run_nav2_goto(["door", "--go"])
    assert rc == 0 and len(sent) == 1, f"rc={rc} err={err!r}"
    ps = sent[0].pose
    assert ps.header.frame_id == "map", (
        f"Nav2 goals must be sent in the map frame, got {ps.header.frame_id!r}")
    assert ps.pose.position.x == pytest.approx(wp["x"], abs=1e-6)
    assert ps.pose.position.y == pytest.approx(wp["y"], abs=1e-6)
    assert ps.pose.orientation.z == pytest.approx(math.sin(wp["yaw"] / 2.0), abs=1e-9)
    assert ps.pose.orientation.w == pytest.approx(math.cos(wp["yaw"] / 2.0), abs=1e-9)
    # A quaternion must be a unit quaternion; z=sin(yaw), w=cos(yaw) is the classic off-by-a-half
    # and it still normalises, so check the ANGLE it decodes to, not just the norm.
    assert 2.0 * math.atan2(ps.pose.orientation.z, ps.pose.orientation.w) == \
        pytest.approx(wp["yaw"], abs=1e-9), "the quaternion does not decode back to the stored yaw"


def test_handoff_3_an_odom_waypoint_is_refused_not_silently_converted(world):
    """GUARDS the two-origins-one-subtraction family: 5.09 m reported against 2.59 m true, and an
    odom pose of (4.96, 2.93) driven as if it were the map pose (5.35, 5.59).

    An odom coordinate has no map-frame meaning. There is no transform to apply, because the two
    origins are unrelated -- so the only correct behaviours are REFUSE or an explicit --force, and
    the one behaviour that must never appear is a silent pass-through into a map-frame goal.
    """
    rc, out, err, sent = run_nav2_goto(["odom_door", "--go"])
    assert not sent, "an odom-frame waypoint must never reach the action client"
    assert rc == 3, f"expected 3, got {rc}: {err!r}"
    assert "odom" in err and "map" in err

    # --force must exist and must be the only way through, or operators will invent one.
    rc, out, err, sent = run_nav2_goto(["odom_door", "--go", "--force"])
    assert rc == 0 and len(sent) == 1, "--force must be an explicit, working override"
    assert sent[0].pose.header.frame_id == "map", (
        "even under --force the goal is stamped `map`; the operator is asserting the coordinate "
        "is right, not asking for a frame conversion that does not exist")


def test_handoff_3_a_dry_run_sends_nothing(world):
    """The default is a dry run, and the dry run is checked before rclpy exists. This is the
    difference between printing a goal and moving a 60 kg base."""
    rc, out, err, sent = run_nav2_goto(["door"])
    assert rc == 0 and not sent
    assert "DRY RUN" in out


# =============================================================================================
# HANDOFF 4.  NAV2 RESULT  ->  FSM OUTCOME
# =============================================================================================
#
# THE ONE THAT COULD DRIVE AN ARM AT A WALL. This walks the FULL seam: a GoalStatus enters the
# real nav2_goto result handler, becomes a (exit code, stdout) pair, crosses the process boundary,
# and is parsed by the real RosWorld._drive_leg_staged. Both halves are executed.

# action_msgs/GoalStatus
STATUS_UNKNOWN, STATUS_ACCEPTED, STATUS_EXECUTING = 0, 1, 2
STATUS_CANCELING, STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED = 3, 4, 5, 6


def _run_leg(world, monkeypatch, *, goal="door", goal_status=STATUS_SUCCEEDED, **kw):
    w, fake = make_world(world, monkeypatch, goal=goal, goal_status=goal_status, **kw)
    outcome = w.navigate_to_goal()
    return w, fake, outcome


@pytest.mark.xfail(
    strict=True,
    reason=(
        "REAL BUG in bringup/ros_world.py's DEFAULT nav path. _drive_leg_staged (used only when "
        "UTP_NAV_STAGED=1) correctly requires _leg_should_stop(perceived_blockage) to confirm a "
        "physical obstruction before reporting NavOutcome(status='blocked') on an ABORTED Nav2 "
        "goal (ros_world.py:676-686). _drive_leg_single -- the DEFAULT since UTP_NAV_STAGED "
        "defaults to '0' (ros_world.py:595-597) -- has no such gate: its 'blocked' branch "
        "(ros_world.py:612-614) sets status='blocked' the instant nav2_goto's stdout contains "
        "the word 'blocked', for ANY Nav2 ABORT including TF, planner, controller and costmap "
        "faults that are not physical obstructions at all, and only calls _perceive_blockage() "
        "afterward to attach a blockage object -- it never checks whether that perception "
        "confirms anything. Reproduced here: with a perceptually clear corridor (current_blockage "
        "prints blocked=False all three times it is asked) and an ABORTED goal, the leg still "
        "returns status='blocked', which downstream starts the reason->ground->press chain (i.e. "
        "commands the arm) on what the leg's own perception says is empty space. This is the "
        "exact failure the module docstring warns about ('Manufacturing a physical blockage from "
        "that control-plane status can start reason -> ground -> press on a software bug'), "
        "currently unguarded on the code path the live elevator pipeline actually runs by "
        "default. Cannot fix: bringup/ is off-limits to this suite. Fix: give "
        "_drive_leg_single's 'blocked' branch the same _leg_should_stop confirmation "
        "_drive_leg_staged already has."
    ),
)
def test_handoff_4_succeeded_reaches_and_unconfirmed_abort_does_not_block(world, monkeypatch):
    """SUCCEEDED reaches; ABORTED needs perceptual evidence before it can start manipulation.

    Nav2 also aborts for TF, planner, controller, and costmap faults. Manufacturing a physical
    blockage from that control-plane status can start reason -> ground -> press on a software bug.
    """
    _, fake, out = _run_leg(world, monkeypatch, goal_status=STATUS_SUCCEEDED)
    assert out.status == "reached", f"SUCCEEDED must reach, got {out.status!r}"
    assert fake.nav2_sent, "a `reached` outcome must have actually sent a goal"

    w, fake, out = _run_leg(world, monkeypatch, goal_status=STATUS_ABORTED)
    assert out.status == "unreachable", \
        f"an ABORTED goal in a perceptually clear corridor must not be blocked, got {out.status!r}"
    assert w.at_goal() is False, "a blocked leg must not leave at_goal() answering True"
    assert "grab_frame.py" in fake.scripts_after("nav2_goto.py"), (
        "ABORTED is a real obstruction: the FSM must perceive from the stopped pose so the "
        "reasoner has something to reason about")


@pytest.mark.parametrize("status,name", [
    (STATUS_CANCELED, "CANCELED"),
    (STATUS_UNKNOWN, "UNKNOWN"),
    (STATUS_EXECUTING, "EXECUTING"),
    (STATUS_ACCEPTED, "ACCEPTED"),
])
def test_handoff_4_control_plane_statuses_are_not_blocked_and_start_no_perception(
        world, monkeypatch, status, name):
    """GUARDS: "every non-SUCCEEDED Nav2 status was reported as `blocked`, so a CANCELLED goal
    would start the reason -> ground -> press chain and put an arm at a wall."

    CANCELED means SOMETHING CANCELLED US -- an operator, a supervisor, a preempting goal.
    UNKNOWN/ACCEPTED/EXECUTING coming back as a *result* means the action server is confused.
    None of them is evidence that anything is in the way. Calling any of them `blocked` asks the
    VLM what the obstruction is, sends the grounder hunting for a control, and lets act() drive
    the arm at whatever it finds -- a perception-and-action chain started by a control-plane
    event. The same class of error as recording a crashed backend as a navigation timeout.
    """
    w, fake, out = _run_leg(world, monkeypatch, goal_status=status)
    assert out.status != "blocked", (
        f"Nav2 returned {name}, which is not a statement about the world, and the FSM was told "
        f"{out.status!r}. `blocked` starts reason -> ground -> press.")
    assert out.blockage is None, f"{name} must carry no BlockageEvent"
    assert w.at_goal() is False, f"{name} must not be recorded as an arrival"
    after = fake.scripts_after("nav2_goto.py")
    assert "grab_frame.py" not in after, (
        f"Nav2 returned {name} and the FSM went on to perceive ({after}). Perception here is the "
        f"first step of reason -> ground -> press; a cancelled goal must end the leg, not start "
        f"a chain that reaches for a control.")
    assert "waypoints.py" not in after, (
        f"Nav2 returned {name} and the odom dead-reckoning driver was invoked ({after}). A human "
        f"or a supervisor cancelling a goal must not be answered by driving to the same place on "
        f"odom -- that is motion resumed by the very event that stopped it.")


def test_handoff_4_the_vocabulary_crossing_the_process_boundary_is_unambiguous(world):
    """RosWorld greps nav2_goto's STDOUT for the words `arrived` and `blocked`. Anything else
    printed on stdout that contains either word is indistinguishable from an outcome.

    This is the seam itself: nav2_goto's status table can be perfectly correct and still be
    destroyed by one stray word on the wrong stream.
    """
    for status, name in ((STATUS_CANCELED, "CANCELED"), (STATUS_UNKNOWN, "UNKNOWN"),
                         (STATUS_EXECUTING, "EXECUTING")):
        rc, out, err, _ = run_nav2_goto(["door", "--go"], goal_status=status)
        assert "blocked" not in out, (
            f"{name} printed the word 'blocked' on STDOUT: {out!r}. RosWorld parses stdout, so "
            f"this would be read as an obstruction.")
        assert "arrived" not in out, f"{name} printed 'arrived' on stdout: {out!r}"
    rc, out, _, _ = run_nav2_goto(["door", "--go"], goal_status=STATUS_SUCCEEDED)
    assert rc == 0 and "arrived" in out and "blocked" not in out
    rc, out, _, _ = run_nav2_goto(["door", "--go"], goal_status=STATUS_ABORTED)
    assert rc == 0 and "blocked" in out and "arrived" not in out


def test_handoff_4_a_cancelled_goal_is_distinguishable_from_a_dead_nav2(world):
    """CANCELED and no-server may share an exit code, but their structured statuses must differ.

    Those are opposite situations. "Nav2 is not there" is an infrastructure fault and falling back
    to another driver is the right answer. "Something cancelled my goal" is a control-plane event,
    frequently a human, and the right answer is to STOP. RosWorld._nav2_unavailable receives one
    number for both and treats them identically, so for any waypoint whose frame is not `map` --
    including every waypoint when maps/waypoints.yaml cannot be read, since _goal_waypoint()
    returns None and the frame defaults to "odom" -- a cancelled goal is answered by starting the
    odom dead-reckoning driver toward the same place.

    The status table inside nav2_goto is not lossy. The PROCESS BOUNDARY is where it becomes
    lossy, and this is the seam test, so this is where it is asserted.
    """
    rc_cancel, out_cancel, _, _ = run_nav2_goto(["door", "--go"], goal_status=STATUS_CANCELED)
    rc_no_server, out_no_server, _, _ = run_nav2_goto(["door", "--go"], server=False)
    assert rc_cancel == rc_no_server == 4
    result_cancel = json.loads(next(l[7:] for l in out_cancel.splitlines() if l.startswith("RESULT ")))
    result_no_server = json.loads(next(l[7:] for l in out_no_server.splitlines() if l.startswith("RESULT ")))
    assert result_cancel["status"] == "cancelled"
    assert result_no_server["status"] == "no_server"


def test_handoff_4_an_unresolvable_waypoint_fails_closed(world, monkeypatch):
    """WHY THE LOSSY EDGE ABOVE IS NOT ACADEMIC. It is currently masked by an ACCIDENT, not by a
    design: every waypoint on this robot happens to be map-frame, and _nav2_unavailable happens
    to refuse to fall back on map-frame goals.

    The masking fails here. _nav2_unavailable decides whether to start the odom DEAD-RECKONING
    DRIVER from

        frame = (self._goal_waypoint() or {}).get("frame", "odom")

    and _goal_waypoint() answers None for a waypoint it cannot resolve -- the store missing,
    unreadable, or being rewritten by `waypoints.py record`/`save()` in another process between
    nav2_goto's read of it and this one. None then defaults to "odom", which is the value that
    AUTHORISES MOTION.

    CLAUDE.md, non-negotiable: "All safety gates fail closed: never-seen and stale both mean
    'not permitted'." A waypoint that cannot be read must stop the leg, not be dead-reckoned to.
    The default must be the frame that refuses, or the record must be required.
    """
    w, fake = make_world(world, monkeypatch, goal="door", goal_status=STATUS_CANCELED)
    # The store became unreadable after nav2_goto read it. _goal_waypoint() swallows every
    # exception and answers None, which is the state under test.
    monkeypatch.setattr(w, "_goal_waypoint", lambda: None)
    out = w.navigate_to_goal()
    drove = [c for c in fake.calls if any(a.endswith("waypoints.py") for a in c) and "goto" in c]
    assert not drove, (
        f"Nav2 reported CANCELED, the goal waypoint could not be resolved, and RosWorld started "
        f"the odom dead-reckoning driver anyway ({drove}). Two failures compounding: a "
        f"control-plane event treated as 'this backend cannot serve the request', and an "
        f"unresolvable waypoint defaulting to the frame that permits motion. `waypoints.py goto` "
        f"then drives on odom -- the backend that cannot see glass -- and its own `blocked` "
        f"output starts perception, reasoning and the press chain.")
    assert out.status != "blocked", "and it must still not be reported as an obstruction"


# =============================================================================================
# HANDOFF 5.  SCAN  ->  COSTMAP SOURCE
# =============================================================================================

def _observation_sources(doc) -> list[tuple[str, str]]:
    """(where, topic) for EVERY costmap observation source, at ANY nesting depth.

    Nav2 nests costmaps twice -- global_costmap.global_costmap.ros__parameters -- so a fixed-depth
    walk finds nothing, and an assertion built on nothing PASSES VACUOUSLY. That exact mistake was
    made on 2026-09-01 while the costmaps were subscribed to the wrong topic. Recursion means a
    layout change breaks the test loudly instead of quietly, and the emptiness check below turns
    "found nothing" into a failure rather than a pass.
    """
    found: list[tuple[str, str]] = []

    def walk(node, path):
        if not isinstance(node, dict):
            return
        for layer_key, layer in node.items():
            if not isinstance(layer, dict):
                continue
            srcs = layer.get("observation_sources")
            if isinstance(srcs, str):
                for src in srcs.split():
                    s = layer.get(src)
                    if isinstance(s, dict) and "topic" in s:
                        found.append((".".join(path + [layer_key, src]), s["topic"]))
            walk(layer, path + [layer_key])

    walk(doc, [])
    return found


def test_handoff_5_every_costmap_source_is_the_topic_the_mask_publishes():
    """GUARDS 2026-09-01: both costmaps consumed /scan_filtered while the self-occlusion mask
    published /scan. Both ends BEST_EFFORT, so the WRONG data arrived perfectly, at full rate,
    for hours, with no error anywhere.

    /scan_filtered is the RAW pointcloud_to_laserscan projection and it contains the robot: the
    stowed arm, the mast and the chassis rear return between 0.39 m and 0.85 m across
    |bearing| >= 74 deg (measured, 10 stationary scans). Handed to the obstacle layer those became
    LETHAL cells wrapped around the footprint -- Nav2 believed it was standing inside an obstacle,
    accepted goals, produced no plan, and never moved with metres of clear floor ahead.

    tests/test_nav2_scan_source.py asserts the params file's contents. This asserts the HANDOFF:
    the costmap's topic is read from the params file, the mask's topic is read from the module
    that does the masking, and the two are compared to each other -- so renaming either side
    breaks this even if both files stay internally consistent.

    GENERALISED 2026-09-05: a third stage, safety/scan_temporal_filter.py, now sits between the
    relay and Nav2 (/scan -> /scan_nav, added to suppress a flickering near-field artifact that
    is real data and must not be masked out of SLAM's /scan). Nav2's costmaps consume THAT
    stage's output, not the relay's directly -- see tests/test_stack_wiring.py's
    test_slam_and_both_nav2_costmaps_consume_the_relay_output_not_the_raw_projection for the
    SLAM-side half of the same joint. IN_TOPIC/OUT_TOPIC are also now env-overridable, so read
    the module's actual defaults rather than pattern-matching a literal assignment.
    """
    relay = importlib.import_module("scan_relay")
    published, consumed = relay.OUT_TOPIC, relay.IN_TOPIC
    assert published != consumed, "the relay must not publish what it consumes"

    temporal_src = (REPO / "safety" / "scan_temporal_filter.py").read_text()
    m_in = re.search(r'UTP_SCAN_TEMPORAL_IN",\s*"([^"]+)"', temporal_src)
    m_out = re.search(r'UTP_SCAN_TEMPORAL_OUT",\s*"([^"]+)"', temporal_src)
    assert m_in and m_out, "safety/scan_temporal_filter.py no longer declares its default topics"
    assert m_in.group(1) == published, (
        f"safety/scan_temporal_filter.py defaults to consuming {m_in.group(1)!r} but "
        f"bringup/scan_relay.py publishes the masked scan on {published!r}.")
    nav_topic = m_out.group(1)

    doc = yaml.safe_load(PARAMS_FILE.read_text())
    sources = _observation_sources(doc)
    assert sources, (
        f"{PARAMS_FILE.name}: the recursive walk found ZERO costmap observation sources. That is "
        f"a FAILURE, not a pass -- a walk that finds nothing asserts nothing, and this file was "
        f"written because a fixed-depth walk passed vacuously over exactly this schema.")
    assert len(sources) >= 2, (
        f"expected an observation source in BOTH the local and the global costmap, found "
        f"{sources}. Nav2 nests costmaps twice; a walk that reaches only one of them is half a "
        f"test.")

    for where, topic in sources:
        assert topic == nav_topic, (
            f"{where} consumes {topic!r}, but safety/scan_temporal_filter.py publishes the "
            f"temporal-filtered, masked scan on {nav_topic!r} (which consumes "
            f"bringup/scan_relay.py's masked {published!r}, itself consuming raw {consumed!r}). "
            f"{consumed!r} contains the robot's own arm and mast and makes Nav2 refuse to plan "
            f"around its own footprint.")


def test_handoff_5_the_fusion_and_the_costmap_read_different_lidar_topics_harmlessly():
    """A SEAM THIS BRIEF DID NOT LIST, found while walking it.

    Nav2's costmaps consume the MASKED /scan (asserted above). bringup/grab_frame.py, which
    writes the scan.json that safety/blockage_fusion.py fuses, subscribes to
    os.environ.get("UTP_SCAN_TOPIC", "/scan_filtered") -- the RAW, UNMASKED topic, containing the
    robot's own arm, mast and chassis at 0.39-0.85 m across |bearing| >= 74 deg.

    So the two consumers of the same lidar read DIFFERENT topics. That is the identical shape as
    the bug that cost 2026-09-01, and it is currently harmless only because of geometry: the
    masked arc is astern and the fusion corridor is a forward rectangle. "Currently harmless
    because of geometry" is a claim, so it is asserted, on the REAL captures, against the REAL
    mask: masking the scan must not change the fused verdict, the evidence, or the measured
    range. The day it does, the fusion is reading the robot.
    """
    fuse = _fusion().fuse
    relay = importlib.import_module("scan_relay")

    src = (REPO / "bringup" / "grab_frame.py").read_text()
    assert "UTP_SCAN_TOPIC" in src, (
        "grab_frame.py no longer names the scan topic it records; the divergence between what "
        "Nav2 sees and what the fusion sees can no longer be reasoned about here.")

    for cap, camera in ((CAP_LIDAR_ONLY, {"blocked": False, "kind": "",
                                          "description": "an open walkway with pillars"}),
                        (CAP_CAMERA_ONLY, {"blocked": False, "kind": "", "description": ""})):
        scan = _read_capture_scan(cap)
        raw = fuse(camera, scan["ranges"], scan["angle_min"], scan["angle_increment"])
        m = relay.mask_self_returns(scan["ranges"], scan["angle_min"], scan["angle_increment"])
        masked = fuse(camera, m.ranges, scan["angle_min"], scan["angle_increment"])
        assert (raw["blocked"], raw["evidence"]) == (masked["blocked"], masked["evidence"]), (
            f"{cap.name}: masking the robot out of the scan CHANGES the fused verdict "
            f"({raw['blocked']}/{raw['evidence']} -> {masked['blocked']}/{masked['evidence']}). "
            f"grab_frame.py records the UNMASKED topic, so the fusion is reading the robot's own "
            f"arm as an obstruction while Nav2, on the masked /scan, is not. Point grab_frame at "
            f"the masked topic (UTP_SCAN_TOPIC=/scan) or mask what it saves.")
        assert raw["nearest_ahead_m"] == masked["nearest_ahead_m"], (
            f"{cap.name}: the reported forward range changes with the mask "
            f"({raw['nearest_ahead_m']} -> {masked['nearest_ahead_m']}). That number sets the "
            f"bounded reverse to the 1.40 m survey standoff, so the robot would back away from "
            f"its own mast.")


# =============================================================================================
# HANDOFF 6.  PERCEPTION FUSION  ->  BLOCKED
# =============================================================================================

def _fusion():
    """safety/blockage_fusion.py, imported defensively -- five other agents are editing source."""
    try:
        return importlib.import_module("safety.blockage_fusion")
    except Exception as e:                                  # noqa: BLE001
        pytest.fail(f"safety/blockage_fusion.py did not import: {type(e).__name__}: {e}. The "
                    f"fused verdict is the only thing standing between the camera's "
                    f"'open walkway with pillars' and a glass door 0.72 m away.")


def test_handoff_6_lidar_alone_blocks_when_the_camera_looks_through_the_glass():
    """REAL SCAN, captures/trial_ours_001, hardware, 2026-09-01.

    The camera frame genuinely shows an open covered walkway with pillars -- because the picture
    is of the corridor THROUGH the glass. The VLM answered blocked=False and was not wrong about
    the image; a correct perception of the picture is a wrong perception of the world. The lidar
    at the same instant had 85 returns within +-20 deg of forward, the nearest at 0.72 m.

    An AND, or a two-of-two vote, would have cleared this. It is a closed door 0.72 m away.
    """
    fuse = _fusion().fuse
    scan = _read_capture_scan(CAP_LIDAR_ONLY)
    _require_glass_door_signature(scan, CAP_LIDAR_ONLY)
    camera = {"blocked": False, "kind": "",
              "description": "an open walkway with pillars"}
    r = fuse(camera, scan["ranges"], scan["angle_min"], scan["angle_increment"])
    assert r["blocked"] is True, (
        f"the camera said clear and the lidar saw a surface at "
        f"{r.get('nearest_ahead_m')} m; the fused verdict must be BLOCKED. Got {r}")
    assert r["evidence"] == "lidar", f"the lidar was the only witness; evidence={r['evidence']!r}"
    assert r["nearest_ahead_m"] is not None and r["nearest_ahead_m"] < 1.0, (
        f"the measured range must survive the fusion for the back-off to be computable: {r}")
    assert "open walkway" not in r["description"] or "did not report" in r["description"], (
        f"a blocked=True verdict must not be described in the camera's words alone -- downstream "
        f"a language model reasons over this string: {r['description']!r}")


def test_handoff_6_camera_alone_blocks_when_the_lidar_sees_nothing():
    """REAL SCAN, captures/trial_ours_002. Same building, different pose and lighting: the camera
    correctly reports closed glass doors, and the lidar's forward corridor holds ZERO returns --
    the pane returns nothing to it at those angles.

    Each sensor is the only witness in exactly the case the other one gets wrong, so requiring
    agreement means requiring BOTH to succeed on the case each is worst at. The union is the only
    defensible combination.
    """
    fuse = _fusion().fuse
    scan = _read_capture_scan(CAP_CAMERA_ONLY)
    camera = {"blocked": True, "kind": "door", "description": "closed glass doors"}
    r = fuse(camera, scan["ranges"], scan["angle_min"], scan["angle_increment"])
    assert r["blocked"] is True, f"the camera saw the doors; the fused verdict must block: {r}"
    assert r["evidence"] == "camera", (
        f"evidence must name the camera as the sole witness, so an operator reading the trial "
        f"record knows the lidar contributed nothing here: {r}")
    assert r["kind"] == "door", "the camera's classification must survive the fusion"


def test_handoff_6_an_AND_would_have_cleared_both_real_captures():
    """The counterfactual, stated as an assertion so it cannot rot into a comment.

    Run both real captures through the rule that was NOT chosen. If an AND ever starts blocking
    them, the two captures have stopped being the pair of opposite failures this design rests on,
    and the OR needs re-arguing rather than re-asserting.
    """
    fuse = _fusion().fuse
    cases = [
        (CAP_LIDAR_ONLY, {"blocked": False, "kind": "",
                          "description": "an open walkway with pillars"}),
        (CAP_CAMERA_ONLY, {"blocked": True, "kind": "door",
                           "description": "closed glass doors"}),
    ]
    for cap, camera in cases:
        scan = _read_capture_scan(cap)
        if cap is CAP_LIDAR_ONLY:
            _require_glass_door_signature(scan, cap)
        r = fuse(camera, scan["ranges"], scan["angle_min"], scan["angle_increment"])
        cam_fired = camera["blocked"] is True
        lidar_fired = r["evidence"] in ("lidar", "both")
        assert not (cam_fired and lidar_fired), (
            f"{cap.name}: BOTH sensors now fire, so this capture no longer demonstrates the "
            f"single-witness case the OR was argued from. Re-check the pair.")
        assert r["blocked"] is True, f"{cap.name}: the OR must still block it: {r}"


def test_handoff_6_the_fused_verdict_is_what_reaches_the_fsm(world, monkeypatch):
    """The fusion being right is worth nothing if RosWorld hands the FSM the camera's answer.

    This walks the real seam and fakes only the VLM request: grab_frame writes a real scan.json
    into a real capture directory, ask_blockage.ask reads it back, the real fuse() ORs it with
    the camera verdict, and RosWorld.current_blockage translates the result into the
    BlockageEvent the FSM acts on. Every step between the VLM and the FSM is the shipping code.
    """
    scan = _read_capture_scan(CAP_LIDAR_ONLY)
    _require_glass_door_signature(scan, CAP_LIDAR_ONLY)
    w, fake = make_world(world, monkeypatch, goal="door", scan=scan,
                         camera={"blocked": False, "kind": "",
                                 "description": "an open walkway with pillars"})
    b = w.current_blockage()
    assert b is not None and b.blocked is True, (
        "RosWorld handed the FSM the CAMERA-ONLY verdict on the trial_ours_001 scan. That is the "
        "exact configuration that called closed glass doors 0.72 m ahead 'an open walkway with "
        "pillars' and drove at them.")
    assert w._evidence == "lidar"
    assert w._nearest_ahead_m is not None and w._nearest_ahead_m < 1.0, (
        "the measured forward range must reach RosWorld, or the bounded back-off to the 1.40 m "
        "survey standoff cannot be computed and the look ladder runs from 0.72 m")


# =============================================================================================
# HANDOFF 7.  /cmd_vel_nav  ->  SAFETY MUX  ->  /cmd_vel
# =============================================================================================

def test_handoff_7_nav2_publishes_to_the_mux_and_the_mux_alone_publishes_cmd_vel():
    """The chokepoint. config/safety.yaml makes safety/twist_mux_node.py the ONLY publisher of
    /cmd_vel; anything publishing /cmd_vel directly silently bypasses the E-stop, the arm
    interlock, the speed ceilings and the slew limiter -- every protection this repo has for base
    motion, with no error and no second publisher warning from DDS.

    Four links, asserted against each other rather than each on its own:
      1. every Nav2 node that DRIVES THE BASE remaps /cmd_vel -> /cmd_vel_nav;
      2. /cmd_vel_nav is a declared mux source;
      3. the mux's output_topic is /cmd_vel;
      4. enable_stamped_cmd_vel is false everywhere it is declared -- Jazzy defaults it to TRUE
         (TwistStamped) and the mux subscribes to Twist, so the mux would receive NOTHING AT ALL,
         at full rate, with no error. The same silent-QoS-shape failure as the scan relay.
    """
    cfg = yaml.safe_load(SAFETY_YAML.read_text())
    assert cfg.get("output_topic") == "/cmd_vel", (
        f"the mux must own /cmd_vel; output_topic is {cfg.get('output_topic')!r}")
    src_topics = {s["name"]: s["topic"] for s in cfg["sources"]}
    assert "/cmd_vel_nav" in src_topics.values(), (
        f"/cmd_vel_nav is not a declared mux source, so Nav2's remapped output goes nowhere. "
        f"Sources: {src_topics}")
    assert "/cmd_vel" not in src_topics.values(), (
        f"a mux source named /cmd_vel would make the mux its own input: {src_topics}")

    launch = LAUNCH_FILE.read_text()
    # BOTH base-driving servers. controller_server is the obvious one; behavior_server emits its
    # own Twist for BackUp and DriveOnHeading, and remapping only the controller left three
    # publishers on /cmd_vel bypassing the mux entirely (measured 2026-08-20, `ros2 topic info
    # /cmd_vel -v` reported publisher count 3, all behavior_server).
    for node in ("controller_server", "behavior_server"):
        i = launch.index(f'name="{node}"')
        block = launch[i:launch.index(")", launch.index("arguments=arguments", i))]
        assert '("/cmd_vel", "/cmd_vel_nav")' in block, (
            f"{node} does not remap /cmd_vel -> /cmd_vel_nav, so it publishes /cmd_vel directly "
            f"and bypasses the E-stop, the arm interlock and the speed ceilings.")

    doc = yaml.safe_load(PARAMS_FILE.read_text())
    decls = []

    def walk(node, path):
        if not isinstance(node, dict):
            return
        for k, v in node.items():
            if k == "enable_stamped_cmd_vel":
                decls.append((".".join(path), v))
            elif isinstance(v, dict):
                walk(v, path + [k])

    walk(doc, [])
    assert decls, (
        f"{PARAMS_FILE.name} declares enable_stamped_cmd_vel nowhere. Jazzy DEFAULTS it to true, "
        f"i.e. TwistStamped, and the mux subscribes to geometry_msgs/Twist -- the mux would "
        f"receive nothing at all, silently. An empty inventory is a FAILURE, not a pass.")
    for where, val in decls:
        assert val is False, (
            f"{where}.enable_stamped_cmd_vel is {val!r}. The mux consumes unstamped Twist; "
            f"TwistStamped delivers nothing and reports nothing.")


def test_handoff_7_nav_is_the_lowest_priority_source_and_the_gates_fail_closed():
    """The mux is a priority arbiter, and Nav2 must be arbitrable: teleop and the servo loop both
    outrank it, or a Nav2 goal cannot be overridden by the person standing next to the robot.

    Also pins the fail-closed rule: gates are declared with finite timeouts, because a stale gate
    is indistinguishable from a crashed publisher and the crash is what the interlock defends
    against.
    """
    cfg = yaml.safe_load(SAFETY_YAML.read_text())
    by_name = {s["name"]: s for s in cfg["sources"]}
    assert by_name["nav"]["priority"] < by_name["teleop"]["priority"], \
        "teleop must outrank nav, or the operator cannot take the robot off a Nav2 goal"
    assert by_name["nav"]["priority"] < by_name["servo"]["priority"], \
        "the servo loop (approach / retreat / look) must outrank nav"
    assert by_name["nav"].get("allows_arm_override") is False, (
        "nav must never drive the base with the arm extended; that path is teleop-only and "
        "requires a human-asserted /safety/override")
    for gate in ("arm_stowed", "enable", "override", "estop"):
        assert gate in cfg["gates"], f"the {gate} gate is not declared"
    assert float(cfg["timeouts"]["gate_s"]) > 0, "a gate with no staleness timeout cannot fail closed"
    assert float(cfg["timeouts"]["input_s"]) > 0, "a command source with no timeout never goes absent"


# =============================================================================================
# THE PRE-LEG GATE.  PERCEPTION  ->  WHETHER THE LEG RUNS AT ALL
# =============================================================================================

def test_the_leg_actually_drives_when_the_goal_is_far_and_nothing_is_close(world, monkeypatch):
    """GUARDS 2026-09-01: "three consecutive live trials recorded path_length_m 0.0 with the goal
    8 m away -- it never navigated at all."

    The camera can see a glass door from 8 m, so a camera-first check with no distance gate
    returned `blocked` before the wheels ever turned, navigate_to_goal returned, and the Nav2 leg
    -- the whole reason the saved map exists -- was never reached. From outside it looked like
    navigation.

    The rule is the operator's: the VLM is triggered when NAV2, ON ITS PATH, discovers it is
    blocked. You cannot be "blocked at a door" you are 8 m from and have not driven toward.
    """
    far = _clear_scan()                       # nothing within LEG_ABORT_RANGE_M
    w, fake = make_world(world, monkeypatch, goal="far_door", scan=far,
                         camera={"blocked": True, "kind": "door",
                                 "description": "glass doors, far away"},
                         goal_status=STATUS_SUCCEEDED, pose=(0.0, 0.0, 0.0))
    out = w.navigate_to_goal()
    assert fake.nav2_runs >= 1, (
        "the camera reported a door 50 m away and the leg never started. This is the "
        "path_length_m 0.0 failure: the robot did not navigate, and the record says it did.")
    assert out.status == "reached"


def test_the_pre_leg_gate_stops_the_leg_when_something_solid_is_right_there(world, monkeypatch):
    """The mirror of the test above, and the reason the gate is two rules and not one.

    A blockage inside LEG_ABORT_RANGE_M is a statement about the robot's NEXT METRE, not about
    its goal, so the NEAR_GOAL_M gate has no business vetoing it. Without this second rule a
    glass door standing 20 m short of the goal is driven at with nothing empowered to stop it --
    the hole the distance gate opened when it was added.
    """
    scan = _read_capture_scan(CAP_LIDAR_ONLY)          # a real door at 0.70 m
    _require_glass_door_signature(scan, CAP_LIDAR_ONLY)
    w, fake = make_world(world, monkeypatch, goal="far_door", scan=scan,
                         camera={"blocked": False, "kind": "",
                                 "description": "an open walkway with pillars"})
    out = w.navigate_to_goal()
    assert out.status == "blocked", (
        f"a real surface 0.70 m ahead did not stop the leg (status {out.status!r}). The goal is "
        f"far, so only the abort-range rule can catch this, and that is the whole point of it.")
    assert fake.nav2_runs == 0, "nothing may be driven at a door 0.70 m away"
    assert w.at_goal() is False, (
        "a leg that never started must not leave _last_nav at its 'reached' default -- fsm.py "
        "credits trace.reached_goal from exactly this call")


def test_the_distance_gate_compares_two_map_frame_poses(world, monkeypatch):
    """GUARDS 2026-09-01: 5.09 m reported against 2.59 m true, because `waypoints.py where` with
    no --frame resolves `auto`, which on this stack returns the ODOM pose, and it was subtracted
    from a MAP-frame waypoint. Odom read (4.96, 2.93) while the map pose was (5.35, 5.59). Two
    origins, one subtraction, and the gate then let the leg run when it should have stopped.
    """
    wp = yaml.safe_load(world.store.read_text())["door"]
    w, fake = make_world(world, monkeypatch, goal="door",
                         camera={"blocked": True, "kind": "door", "description": "doors"},
                         pose=(5.0, 6.0, 0.0))
    fake.calls.clear()
    d = w._distance_to_goal()
    where = [c for c in fake.calls if any(a.endswith("waypoints.py") for a in c) and "where" in c]
    assert where, "the distance gate never asked for a pose"
    for c in where:
        assert "--frame" in c and c[c.index("--frame") + 1] == "map", (
            f"the pose behind the distance gate was requested as {c}. Without --frame map this "
            f"resolves `auto` -> the ODOM pose, and subtracting it from a map-frame waypoint "
            f"produces a number with no meaning at all: 5.09 m reported against 2.59 m true.")
    assert d == pytest.approx(math.hypot(wp["x"] - 5.0, wp["y"] - 6.0), abs=1e-6), (
        "the distance must be the map-frame separation of the two map-frame poses and nothing "
        "else")

    # And an ODOM-frame waypoint has NO map-frame distance to give. Returning a plausible-looking
    # number here is the original bug; None is the only honest answer, and the caller treats None
    # as near, which fails toward stopping.
    w2, _ = make_world(world, monkeypatch, goal="odom_door", pose=(5.0, 6.0, 0.0))
    assert w2._distance_to_goal() is None, (
        "an odom-frame waypoint was given a map-frame distance. Two origins, one subtraction.")


def test_an_unreadable_pose_fails_toward_stopping(world, monkeypatch):
    """None means "do not gate on distance", and the caller must treat None as NEAR. An
    unreadable pose must not silently disable the glass check."""
    w, fake = make_world(world, monkeypatch, goal="door",
                         camera={"blocked": True, "kind": "door",
                                 "description": "closed glass doors"})
    monkeypatch.setattr(w, "_distance_to_goal", lambda: None)
    out = w.navigate_to_goal()
    assert out.status == "blocked", (
        f"the pose was unreadable and the leg ran anyway ({out.status!r}). A gate that cannot "
        f"measure must fail toward stopping.")
    assert fake.nav2_runs == 0


# =============================================================================================
# WHAT ONLY THE ROBOT CAN PROVE.
#
# These are not omissions. Each names a link this file walks a FAKE of, and states what has to be
# observed on hardware before the link may be called green. A gate is GREEN only when a human
# watched it pass.
# =============================================================================================

def test_hw_localization_actually_relocalizes_into_the_saved_graph():
    pytest.skip(
        "HARDWARE: this file proves map_persist.sh REFUSES a grid-only map and session.sh nav "
        "checks for .posegraph/.data. It cannot prove slam_toolbox deserialized the graph AT THE "
        "RIGHT POSE. VERIFY ON THE ROBOT: after `MAP_NAME=<name> bash bringup/session.sh nav`, "
        "the live /scan must lie ON the walls already in the map in RViz, not beside them. "
        "Beside them means the graph loaded at the wrong pose and every scan from there "
        "compounds the error, which looks exactly like ordinary drift.")


def test_hw_nav2_plans_and_the_costmap_is_not_full_of_the_robot():
    pytest.skip(
        "HARDWARE: this file proves the costmap observation sources name the masked topic. It "
        "cannot prove the costmap is CLEAN. VERIFY ON THE ROBOT: with the arm stowed on open "
        "floor, `ros2 topic info /scan -v` must show the costmaps subscribed to it and "
        "scan_relay.py as the publisher; the local costmap in RViz must show NO lethal cells "
        "wrapped around the footprint; and a goal 4 m ahead must produce a plan and motion. On "
        "2026-09-01 Nav2 accepted goals, produced no plan, and never moved on clear floor.")


def test_hw_cmd_vel_has_exactly_one_publisher():
    pytest.skip(
        "HARDWARE: this file proves the launch file remaps both base-driving Nav2 nodes and that "
        "config/safety.yaml declares the mux as the sole /cmd_vel publisher. It cannot count "
        "publishers on a live graph. VERIFY ON THE ROBOT, with Nav2 up and a goal running: "
        "`ros2 topic info /cmd_vel -v` must report exactly ONE publisher and it must be "
        "twist_mux_node; `ros2 topic hz /cmd_vel_nav` must be non-zero while Nav2 drives (zero "
        "means Jazzy handed us TwistStamped and the mux is receiving nothing); and releasing the "
        "E-stop gate must stop the base within the measured ~1.26 s / ~18 cm coast.")


def test_hw_a_cancelled_goal_leaves_the_arm_stowed():
    pytest.skip(
        "HARDWARE: this file proves a CANCELED GoalStatus does not become `blocked` and starts "
        "no perception in-process. It cannot prove the arm stayed put. VERIFY ON THE ROBOT: "
        "start a leg, cancel the goal from another terminal, and confirm the base stops, no "
        "capture is written to captures/, ask_blockage.py is never invoked, and the arm remains "
        "at stow_pose_deg by MEASURED joint angles -- never by the FSM's belief about itself.")


def test_hw_the_fused_verdict_stops_the_robot_in_front_of_the_real_doors():
    skip_msg = (
        "HARDWARE: this file proves the fusion blocks on the two REAL captures and that the "
        "verdict reaches the FSM. It cannot prove the robot stops. VERIFY ON THE ROBOT: drive "
        "the leg toward the closed glass doors and confirm the base halts with the fused verdict "
        "logged as blocked, the bounded reverse to the 1.40 m survey standoff runs exactly once, "
        "and the ADA plate is IN FRAME from at least two rungs of the look ladder. The 2026-09-01 "
        "run ended 0.72 m from the glass with the operator stopping it by hand.")
    pytest.skip(skip_msg)


def test_hw_grab_frame_records_a_scan_at_the_same_instant_as_the_frame():
    pytest.skip(
        "HARDWARE: the whole fusion argument rests on the camera frame and the scan being ONE "
        "moment. This file reads scans that were already on disk. VERIFY ON THE ROBOT: "
        "captures/<name>/scan.json must exist for every frame grab_frame.py writes (a missing "
        "one silently makes the verdict CAMERA-ONLY, which is the 2026-09-01 configuration), and "
        "its timestamp must be within one lidar period of the image. Note also that grab_frame "
        "defaults to /scan_filtered, the UNMASKED topic Nav2 does not use.")
