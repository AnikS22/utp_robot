"""The blocked path in RosWorld: fuse -> BACK UP -> look -> press -> resume, run for real.

WHAT THIS GUARDS, and every one of these is a measured hardware failure from 2026-09-01, not a
hypothetical:

  1. The robot stood 0.72 m from CLOSED GLASS DOORS and drove on, because current_blockage() asked
     the VLM about a camera frame and the frame shows an open walkway THROUGH the glass
     (captures/trial_ours_001: blocked=False, "an open walkway with pillars"). The lidar in the
     same instant had 85 returns inside +-20 deg with the nearest at 0.72 m. The operator stopped
     the robot by hand.
  2. The blocked path never backed up, so the survey ran from 0.72 m, where the ADA plate beside
     the door is outside the (measured) 70.2 deg colour frame at every sweep bearing but one.
  3. _distance_to_goal compared an ODOM pose against a MAP waypoint and answered 5.09 m when the
     truth was 2.59 m -- two origins, one subtraction.
  4. The early "blocked" return left _last_nav at "reached", so at_goal() would credit an arrival
     the robot never made.

HOW THIS IS TESTED. ros_world.py reaches ROS through exactly one module-level function, `_ros`,
because rclpy is not importable in the pipeline venv (see its module docstring). Monkeypatching
that ONE function makes the whole escalation runnable headlessly: the real navigate_to_goal, the
real current_blockage, the real back-off and the real look ladder execute, against a fake robot
that records every command it is given and answers with the strings the real tools print.

That is the standard set by tests/test_nav_backend.py's behavioural half, and it is the only kind
of test worth having here: a string-matching test on this file would have passed happily on the
version that drove into the doors, because the words were all present -- it was the ORDER and the
DATA FLOW that were wrong.

The scan used throughout is the REAL captures/trial_ours_001/scan.json off the robot.
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))
sys.path.insert(0, str(Path.home() / "unlocking-the-path"))

import ros_world                                            # noqa: E402
from utp.pipeline.types import Detection, Plan              # noqa: E402

# The measured scan from the failure this whole file is about. Loaded from disk rather than
# synthesised: a hand-written array of ranges is a description of the bug, not evidence of it.
TRIAL_SCAN = REPO / "captures" / "trial_ours_001" / "scan.json"


def _load_trial_scan() -> dict:
    """TRIAL_SCAN, loaded fresh and checked for the near-miss signature this file's escalation
    tests depend on.

    2026-09-05: captures/ is gitignored, local scratch data, and this exact path has since been
    overwritten by an unrelated later capture (an open corridor, nearest finite return ~1.0 m) --
    the same fixture loss hit tests/test_scan_mask.py and tests/test_blockage_fusion.py. The
    original near-miss is not recoverable from git. Every escalation test below needs the door to
    actually be close enough to trip current_blockage(); rather than let that silently degenerate
    into asserting a reverse/look-ladder sequence against an empty corridor (which would pass or
    fail for reasons that have nothing to do with the escalation logic itself), this checks the
    signature first and skips loudly, by name, if it no longer holds -- see docs/TESTING.md for
    how to re-capture it."""
    d = json.loads(TRIAL_SCAN.read_text())
    finite = [x for x in d["ranges"] if isinstance(x, (int, float)) and math.isfinite(x)]
    nearest = min(finite) if finite else None
    if nearest is None or nearest > 0.85:
        pytest.skip(
            f"{TRIAL_SCAN.relative_to(REPO)} no longer holds the glass-door near-miss this "
            f"escalation test needs: expected a lidar return under ~0.85 m, nearest finite "
            f"return is {nearest!r} m. The file has been overwritten by a later, unrelated "
            f"capture reusing this trial name -- re-capture a scan facing closed glass doors at "
            f"this path (see docs/TESTING.md) to restore this regression check.")
    return d

# What ask_blockage actually returned for that frame, verbatim from the brief / the reproduction
# `bringup/ask_blockage.py captures/trial_ours_001`.
CAMERA_SAYS_CLEAR = {"blocked": False, "kind": "",
                     "description": "an open walkway with pillars", "note": ""}
CAMERA_SAYS_DOOR = {"blocked": True, "kind": "door",
                    "description": "closed glass double doors", "note": ""}

# Scripts that MOVE THE ROBOT when invoked without --dry-run. bringup's convention (stated in
# approach_blockage.py's main) is that motion is the default and --dry-run is what stops it, so a
# call carrying --dry-run is a plan, not a motion.
MOVERS = {"nav2_goto.py", "approach_blockage.py", "turn_by.py", "face_target.py",
          "approach_target.py", "stow_arm.py", "sim_press.py"}


def _script(args) -> str:
    for a in args:
        s = str(a)
        if s.endswith(".py"):
            return Path(s).name
    return ""


def _is_motion(args) -> bool:
    if _script(args) == "waypoints.py":
        return "goto" in [str(a) for a in args] and "--go" in [str(a) for a in args]
    return _script(args) in MOVERS and "--dry-run" not in [str(a) for a in args]


def _far_scan(n: int = 720) -> dict:
    """A scan with nothing anywhere near: the corridor is open."""
    import math
    return {"frame": "base_link", "angle_min": -math.pi,
            "angle_increment": 2 * math.pi / n, "ranges": [8.0] * n}



@pytest.fixture(autouse=True)
def _staged_legs(monkeypatch):
    """These tests exercise the STAGED leg, which is now opt-in.

    The runtime default changed to one uninterrupted Nav2 goal on 2026-09-01: staging cancels the
    goal at every stage boundary, so the controller re-plans from wherever it stopped, and on
    hardware that stuttered and wandered where a single goal arrived cleanly (button 21.8 s /
    0.19 m, outside 28.0 s / 0.24 m). The staged path is still supported and still worth testing --
    it is what you want when closing on glass -- so these tests ask for it explicitly rather than
    relying on a default that no longer holds."""
    monkeypatch.setenv("UTP_NAV_STAGED", "1")

class FakeRobot:
    """Stands in for ros_world._ros. Records every ROS-side command and answers it.

    `nav2` is a list of (returncode, stdout) pairs consumed one per leg stage, so a test states
    the shape of a leg -- "two stage timeouts then arrived" -- as data. Running past the end is an
    AssertionError rather than a hang: an escalation that loops forever is a bug this suite must
    fail on, not wait out.
    """

    def __init__(self, *, scan: dict | None, nav2=None, reverse_rc: int = 0,
                 turn_rc: int = 0, capture_root: Path | None = None,
                 map_pose=(0.0, 0.0), odom_pose=(1.0, 2.0)):
        self.calls: list[list[str]] = []
        self.scan = scan
        self.nav2 = list(nav2 or [])
        self.reverse_rc = reverse_rc
        self.turn_rc = turn_rc
        self.capture_root = capture_root
        self.map_pose = map_pose
        self.odom_pose = odom_pose

    # -- the recorded log, in the forms the tests ask questions of ---------------------------
    def scripts(self) -> list[str]:
        return [_script(c) for c in self.calls]

    def index_of(self, script: str, *must_contain: str) -> int:
        for i, c in enumerate(self.calls):
            if _script(c) == script and all(m in [str(a) for a in c] for m in must_contain):
                return i
        return -1

    def motions(self) -> list[list[str]]:
        return [c for c in self.calls if _is_motion(c)]

    # -- the fake robot itself ----------------------------------------------------------------
    def __call__(self, args, timeout=120):
        args = [str(a) for a in args]
        self.calls.append(args)
        name = _script(args)
        if name == "grab_frame.py":
            return self._grab(args)
        if name == "waypoints.py" and "where" in args:
            x, y = (self.map_pose if "--frame" in args else self.odom_pose)
            return self._done(0, f"now: x={x:.3f} y={y:.3f} yaw=0.0\n")
        if name == "waypoints.py" and "goto" in args:
            return self._done(0, "arrived\n")
        if name == "nav2_goto.py":
            assert self.nav2, "the leg asked for another stage than the test scripted"
            rc, out = self.nav2.pop(0)
            # nav2_goto's runtime contract is structured; keep the compact legacy test vectors
            # above, but make the fake emit what the real process emits.
            status = ("refused" if rc in (2, 3) else
                      "no_server" if rc == 4 else
                      "timeout" if rc == 6 else
                      "blocked" if "blocked" in out else
                      "arrived" if "arrived" in out else "error")
            out += "RESULT " + json.dumps({"status": status, "waypoint": "outside",
                                            "elapsed_s": 0.0, "detail": "fake"}) + "\n"
            return self._done(rc, out, "" if rc == 0 else "nav2_goto said no\n")
        if name == "approach_blockage.py":
            if "--back" in args:
                d = args[args.index("--back") + 1]
                return self._done(self.reverse_rc, f"approach_blockage: reversed {d} m\n")
            return self._done(0, "approach_blockage: stopped 1.40 m from what is ahead "
                                 "after 0.30 m\n")
        if name == "turn_by.py":
            return self._done(self.turn_rc, "turned\n")
        return self._done(0, "ok\n")

    def _grab(self, args):
        """Write the capture grab_frame.py would have written: rgb.png, cam.json, scan.json."""
        name = args[args.index("--name") + 1]
        cap = Path(self.capture_root) / "captures" / name
        cap.mkdir(parents=True, exist_ok=True)
        from PIL import Image
        Image.new("RGB", (8, 8), (30, 30, 30)).save(cap / "rgb.png")
        (cap / "cam.json").write_text(json.dumps({"K": [[900, 0, 640], [0, 900, 360], [0, 0, 1]]}))
        if self.scan is not None:
            (cap / "scan.json").write_text(json.dumps(self.scan))
        return self._done(0, f"captured -> {cap}\n")

    @staticmethod
    def _done(rc, out="", err=""):
        return subprocess.CompletedProcess(args=[], returncode=rc, stdout=out, stderr=err)


def reference_fuse(camera, ranges, angle_min, angle_increment):
    """A stand-in for safety.blockage_fusion.fuse, honouring the agreed contract.

    Deliberately minimal -- the fusion POLICY is another module's to own and to test. What this
    file cares about is that ros_world hands the fuser the camera dict and the scan that belong to
    the SAME capture, and then reports what it gets back instead of the camera's answer. The
    recorded arguments are checked directly in test_the_fuser_is_given_the_scan_from_this_capture.
    """
    import math
    reference_fuse.seen = {"camera": camera, "ranges": ranges, "angle_min": angle_min,
                           "angle_increment": angle_increment}
    hits, nearest = 0, None
    for i, r in enumerate(ranges):
        if r != r or abs(r) == float("inf") or r <= 0:
            continue
        a = math.atan2(math.sin(angle_min + i * angle_increment),
                       math.cos(angle_min + i * angle_increment))
        if abs(math.degrees(a)) <= 20.0:
            hits += 1
            nearest = r if nearest is None else min(nearest, r)
    lidar_blocked = hits >= 3 and nearest is not None and nearest <= 1.5
    cam_blocked = bool(camera.get("blocked"))
    if cam_blocked and lidar_blocked:
        ev = "both"
    elif lidar_blocked:
        ev = "lidar"
    elif cam_blocked:
        ev = "camera"
    else:
        ev = "neither"
    desc = camera.get("description", "")
    if lidar_blocked and not cam_blocked:
        desc = (f"the camera reports {desc!r} but the lidar has {hits} returns ahead, "
                f"nearest {nearest:.2f} m")
    return {"blocked": cam_blocked or lidar_blocked, "kind": camera.get("kind", ""),
            "description": desc, "evidence": ev, "nearest_ahead_m": nearest}


# ------------------------------------------------------------------------------- fixtures
@pytest.fixture
def waypoints(tmp_path, monkeypatch):
    """A waypoint store on disk, so _goal_waypoint/_distance_to_goal run their real code."""
    def _write(entries: dict) -> Path:
        import yaml
        f = tmp_path / "waypoints.yaml"
        f.write_text(yaml.safe_dump(entries))
        monkeypatch.setenv("UTP_WAYPOINTS", str(f))
        return f
    return _write


@pytest.fixture
def world(tmp_path, monkeypatch):
    """Build a RosWorld wired to a FakeRobot. Returns (world, robot)."""
    def _make(*, scan, camera, dry_run=False, goal="outside", **kw):
        monkeypatch.setattr(ros_world, "REPO", tmp_path)
        robot = FakeRobot(scan=scan, capture_root=tmp_path, **kw)
        monkeypatch.setattr(ros_world, "_ros", robot)
        # ask_blockage.py is the one fusion boundary in production.  This fixture replaces its
        # network-facing VLM call, so its stand-in must return the same already-fused contract
        # rather than a raw camera verdict for RosWorld to fuse a second time.
        def fused_blockage(cap):
            try:
                captured_scan = json.loads((Path(cap) / "scan.json").read_text())
            except (OSError, ValueError, TypeError):
                return dict(camera)
            return reference_fuse(dict(camera), captured_scan.get("ranges"),
                                  captured_scan.get("angle_min"),
                                  captured_scan.get("angle_increment"))
        monkeypatch.setattr(ros_world, "ask_blockage", fused_blockage)
        w = ros_world.RosWorld(goal=goal, dry_run=dry_run, capture_prefix="t")
        w.reset()
        return w, robot
    return _make


@pytest.fixture
def fuse_installed(monkeypatch):
    mod = types.ModuleType("safety.blockage_fusion")
    mod.fuse = reference_fuse
    monkeypatch.setitem(sys.modules, "safety.blockage_fusion", mod)
    return mod


@pytest.fixture
def fuse_absent(monkeypatch):
    """None in sys.modules is how you make an importable module unimportable."""
    monkeypatch.setitem(sys.modules, "safety.blockage_fusion", None)


# ================================================================= 1. the glass-door failure
def test_camera_says_clear_but_lidar_sees_the_door_and_the_verdict_is_blocked(
        world, waypoints, fuse_installed, capsys):
    """THE hardware failure, reproduced from the real capture and required to come out blocked.

    2026-09-01: 0.72 m from closed glass doors, the VLM said "an open walkway with pillars" and
    blocked=False. With the scan fused in, the same camera answer must not be able to clear the
    way.
    """
    waypoints({"outside": {"frame": "map", "x": 0.0, "y": 0.0}})
    scan = _load_trial_scan()
    w, robot = world(scan=scan, camera=CAMERA_SAYS_CLEAR)

    b = w.current_blockage()

    assert b is not None and b.blocked, (
        "camera-only said an open walkway; the lidar had returns at 0.72 m. This exact "
        "combination drove the robot at the doors on 2026-09-01.")
    assert w._evidence == "lidar", "the camera did not see it; the evidence is the lidar alone"
    assert w._nearest_ahead_m == pytest.approx(0.722, abs=0.01), (
        "the standoff the back-up is computed from must be the measured one")
    assert "0.72" in b.description, "the reasoner is told WHY, not just that it is blocked"


def test_the_fuser_is_given_the_scan_from_this_capture_not_a_fresh_subscription(
        world, fuse_installed):
    """The camera dict and the scan must describe ONE instant.

    Two sensors disagreeing about one moment is the whole problem; two sensors sampled at
    different moments cannot be fused at all. grab_frame.py writes scan.json beside rgb.png, so
    the scan for this observation is already on disk -- reading it is the only way to keep the
    instants aligned from a process that cannot import rclpy.
    """
    scan = json.loads(TRIAL_SCAN.read_text())
    w, robot = world(scan=scan, camera=CAMERA_SAYS_CLEAR)
    w.current_blockage()

    seen = reference_fuse.seen
    assert seen["camera"]["description"] == "an open walkway with pillars"
    assert seen["ranges"] == scan["ranges"]
    assert seen["angle_min"] == pytest.approx(scan["angle_min"])
    assert seen["angle_increment"] == pytest.approx(scan["angle_increment"])
    assert "grab_frame.py" in robot.scripts(), "the scan must come from the capture just taken"


def test_ros_world_does_not_import_or_repeat_fusion(world, fuse_absent, capsys):
    """RosWorld consumes ask_blockage's fused contract even if the fuser later disappears."""
    scan = _load_trial_scan()
    w, robot = world(scan=scan, camera=CAMERA_SAYS_CLEAR)

    b = w.current_blockage()
    out = capsys.readouterr().out

    assert b.blocked is True
    assert "CAMERA-ONLY" not in out
    assert w._nearest_ahead_m == pytest.approx(0.722, abs=0.01), (
        "the back-up still needs a measured standoff even with no fuser -- that is geometry, "
        "not a blockage verdict")


def test_ros_world_does_not_call_a_fuser_after_ask_returns(world, monkeypatch, capsys):
    def _boom(*a, **k):
        raise RuntimeError("half-written")
    mod = types.ModuleType("safety.blockage_fusion")
    mod.fuse = _boom
    monkeypatch.setitem(sys.modules, "safety.blockage_fusion", mod)
    w, robot = world(scan=json.loads(TRIAL_SCAN.read_text()), camera=CAMERA_SAYS_DOOR)

    b = w.current_blockage()

    assert b.blocked is True
    assert "RuntimeError" not in capsys.readouterr().out


def test_no_frame_at_all_fails_closed(world, fuse_installed, monkeypatch):
    """A grab that fails must not leave the PREVIOUS pose's frame standing in for this one."""
    w, robot = world(scan=json.loads(TRIAL_SCAN.read_text()), camera=CAMERA_SAYS_CLEAR)
    w.get_observation()
    assert w._last_capture is not None
    monkeypatch.setattr(ros_world, "_ros",
                        lambda args, timeout=120: FakeRobot._done(1, "", "camera down"))
    w.get_observation()
    assert w._last_capture is None, "a stale capture describes a pose the robot has left"
    b = w.current_blockage()
    assert b.blocked and "no camera frame" in b.description


# ================================================================= 2. back up, then look around
def test_the_blocked_path_backs_up_before_it_looks_around(world, waypoints, fuse_installed):
    """The operator's sequence: blocked -> BACK UP -> look around. In that order.

    From 0.72 m the ADA plate beside the door is at atan(2.34/0.72) = 72.9 deg, and only the
    +-80 deg rung of the sweep can frame it in a 70.2 deg camera. Sweeping first and reversing
    afterwards spends the FSM's recovery budget on viewpoints taken from the wrong place.
    """
    waypoints({"outside": {"frame": "map", "x": 0.0, "y": 0.0}})
    w, robot = world(scan=_load_trial_scan(), camera=CAMERA_SAYS_CLEAR)

    nav = w.navigate_to_goal()
    assert nav.status == "blocked"
    w.approach_blockage()          # fsm.py's blocked-path hook
    w.scan_view()                  # the first look-around rung that moves the base

    back = robot.index_of("approach_blockage.py", "--back")
    turn = robot.index_of("turn_by.py")
    assert back >= 0, "a confirmed blockage at 0.72 m must produce a reverse"
    assert turn >= 0, "and the ladder must then actually sweep"
    assert back < turn, "the robot must back up BEFORE it looks around, not after"


def test_the_reverse_is_bounded_and_aimed_at_the_survey_standoff(
        world, waypoints, fuse_installed):
    """0.72 m measured + 1.40 m survey standoff = 0.68 m of reverse, and never more than 1.00 m."""
    waypoints({"outside": {"frame": "map", "x": 0.0, "y": 0.0}})
    w, robot = world(scan=_load_trial_scan(), camera=CAMERA_SAYS_CLEAR)
    w.current_blockage()
    assert w._back_off_from_blockage() is True

    call = robot.calls[robot.index_of("approach_blockage.py", "--back")]
    dist = float(call[call.index("--back") + 1])
    assert dist == pytest.approx(ros_world.BACKOFF_STANDOFF_M - 0.722, abs=0.02)
    assert 0.05 <= dist <= ros_world.MAX_BACKOFF_M, (
        "approach_blockage.reverse() refuses anything outside 0.05-1.00 m, and it refuses it "
        "because there is no lidar behind this robot")


def test_it_does_not_reverse_when_it_is_not_blocked(world, waypoints, fuse_installed):
    waypoints({"outside": {"frame": "map", "x": 0.0, "y": 0.0}})
    w, robot = world(scan=_far_scan(), camera=dict(CAMERA_SAYS_CLEAR),
                     nav2=[(0, "arrived at 'outside' in 12.0 s\n")])

    nav = w.navigate_to_goal()

    assert nav.status == "reached"
    assert robot.index_of("approach_blockage.py", "--back") == -1, (
        "nothing was in the way; reversing would be the robot retreating from an open corridor")


def test_it_does_not_reverse_when_it_is_already_far_enough_back(world, fuse_installed, capsys):
    """Blocked, but at 2.4 m. Backing up from there costs pixels and buys no bearing."""
    import math
    n = 720
    scan = {"frame": "base_link", "angle_min": -math.pi, "angle_increment": 2 * math.pi / n,
            "ranges": [2.4 if abs(math.degrees(math.atan2(
                math.sin(-math.pi + i * 2 * math.pi / n),
                math.cos(-math.pi + i * 2 * math.pi / n)))) <= 20 else 8.0 for i in range(n)]}
    w, robot = world(scan=scan, camera=CAMERA_SAYS_DOOR)
    w.current_blockage()

    assert w._back_off_from_blockage() is False
    assert robot.index_of("approach_blockage.py", "--back") == -1
    assert "no back-up needed" in capsys.readouterr().out


def test_it_refuses_to_reverse_blind(world, fuse_absent, capsys):
    """No scan on disk means no measured standoff. There is no obstacle check astern."""
    w, robot = world(scan=None, camera=CAMERA_SAYS_DOOR)
    w.current_blockage()

    assert w._nearest_ahead_m is None
    assert w._back_off_from_blockage() is False
    assert "NOT reversing" in capsys.readouterr().out
    assert robot.index_of("approach_blockage.py", "--back") == -1


def test_a_reverse_that_moves_rearms_the_look_ladder(world, fuse_installed):
    """After the base moves, every bearing is unvisited again -- from a pose never surveyed.

    reset() clears the cursor once per TRIAL. A trial can meet the same door more than once (the
    press fails, the leg is re-driven, it blocks again), and on the second meeting _scan_i was
    already past the end of SCAN_BEARINGS_DEG: scan_view() recentred and returned False without
    turning, widen_view() returned False without moving, and the survey was inert at a brand new
    viewpoint. An inert ladder reports "looked, saw nothing", which is a lie about a look that
    never happened.
    """
    w, robot = world(scan=_load_trial_scan(), camera=CAMERA_SAYS_DOOR)
    w.current_blockage()
    w._scan_i = len(w.SCAN_BEARINGS_DEG)         # ladder exhausted by an earlier blockage
    w._widened = True
    w._scan_offset = -80.0

    assert w._back_off_from_blockage() is True

    assert w._scan_i == 0 and w._widened is False, "the ladder must re-sweep from the new pose"
    assert w._scan_offset == 0.0
    assert w.scan_view() is True, "and the very next rung must actually turn the base"
    assert w.last_look_info()["backed_off"] is True, (
        "a sweep from 1.40 m and a sweep from 0.72 m are different experiments; the record has "
        "to say which one produced the answer")


def test_the_reverse_happens_along_the_approach_heading(world, fuse_installed):
    """recentre_view() first: reversing down a +80 deg sweep bearing walks the robot sideways
    away from the door it is trying to get a wider view of."""
    w, robot = world(scan=_load_trial_scan(), camera=CAMERA_SAYS_DOOR)
    w.current_blockage()
    w._scan_offset = 80.0
    w._back_off_from_blockage()

    turn = robot.index_of("turn_by.py")
    back = robot.index_of("approach_blockage.py", "--back")
    assert 0 <= turn < back, "recentre before reversing"
    assert float(robot.calls[turn][robot.calls[turn].index("--deg") + 1]) == pytest.approx(-80.0)


def test_backing_off_does_not_then_creep_forward_again(world, fuse_installed):
    """approach_blockage() must not reverse to 1.40 m and immediately re-approach 1.40 m.
    That oscillation is what fsm.py describes as the erratic stop/back/forward cycle."""
    w, robot = world(scan=_load_trial_scan(), camera=CAMERA_SAYS_DOOR)
    w.current_blockage()
    w.approach_blockage()

    approaches = [c for c in robot.calls
                  if _script(c) == "approach_blockage.py" and "--stop-at" in c]
    assert approaches == [], "there was no gap left to close; the robot had just backed out of it"


# ================================================================= 3. frames are never mixed
def test_distance_to_goal_asks_for_the_map_frame_explicitly(world, waypoints, fuse_installed):
    """The subtraction that produced 5.09 m for a 2.59 m gap. `waypoints.py where` with no
    --frame resolves `auto`, which on this stack is ODOM."""
    waypoints({"outside": {"frame": "map", "x": 3.0, "y": 4.0}})
    w, robot = world(scan=_far_scan(), camera=CAMERA_SAYS_CLEAR, map_pose=(0.0, 0.0),
                     odom_pose=(90.0, 90.0))

    d = w._distance_to_goal()

    assert d == pytest.approx(5.0), "map waypoint minus map pose, and nothing else"
    call = robot.calls[robot.index_of("waypoints.py", "where")]
    assert "--frame" in call and call[call.index("--frame") + 1] == "map", (
        "the frame must be stated on the command line, not assumed from `auto`")


def test_distance_to_goal_is_none_for_an_odom_waypoint_and_asks_the_robot_nothing(
        world, waypoints, fuse_installed):
    """An odom waypoint has no map-frame distance to give. Returning a plausible-looking wrong
    number is worse than returning nothing, and the pose is not even fetched."""
    waypoints({"outside": {"frame": "odom", "x": 3.0, "y": 4.0}})
    w, robot = world(scan=_far_scan(), camera=CAMERA_SAYS_CLEAR)

    assert w._distance_to_goal() is None
    assert robot.index_of("waypoints.py", "where") == -1, (
        "no pose was asked for, so no frame could have been mixed")


def test_an_unknown_waypoint_gates_toward_stopping(world, waypoints, fuse_installed):
    """None means 'do not gate on distance', and the caller must treat that as NEAR."""
    waypoints({"somewhere_else": {"frame": "map", "x": 3.0, "y": 4.0}})
    w, robot = world(scan=json.loads(TRIAL_SCAN.read_text()), camera=CAMERA_SAYS_DOOR)
    w.current_blockage()

    assert w._distance_to_goal() is None
    stop, why = w._leg_should_stop(w._blockage)
    assert stop is True and "unreadable" in why


# ================================================================= 4. the leg is not blind
def test_the_leg_is_driven_in_stages_with_a_look_between_them(world, waypoints, fuse_installed):
    """The leg used to be ONE blocking nav2_goto call: nothing looked between its start and its
    end, and on 2026-09-01 the operator had to stop the robot by hand. Each stage must end in a
    fresh capture and a fresh fused verdict."""
    waypoints({"outside": {"frame": "map", "x": 100.0, "y": 0.0}})
    w, robot = world(scan=_far_scan(), camera=CAMERA_SAYS_CLEAR,
                     nav2=[(6, ""), (6, ""), (0, "arrived at 'outside' in 20.0 s\n")])

    nav = w.navigate_to_goal()

    assert nav.status == "reached"
    assert robot.scripts().count("nav2_goto.py") == 3, "one call per stage"
    # A capture before the leg, and one at every stage boundary.
    assert robot.scripts().count("grab_frame.py") == 3
    for c in robot.calls:
        if _script(c) == "nav2_goto.py":
            assert "--timeout" in c, "an unbounded stage is the blind leg all over again"
            assert float(c[c.index("--timeout") + 1]) <= ros_world.LEG_STAGE_MAX_S


def test_a_door_found_mid_leg_stops_the_leg(world, waypoints, fuse_installed):
    """The glass door 20 m short of the goal: too far away for the NEAR_GOAL_M gate, and right in
    front of the bumper. The goal gate has no business vetoing a surface 0.72 m ahead."""
    waypoints({"outside": {"frame": "map", "x": 100.0, "y": 0.0}})
    w, robot = world(scan=_far_scan(), camera=CAMERA_SAYS_CLEAR,
                     nav2=[(6, ""), (6, "")])
    # After the first stage the world changes: the doors are now within lidar range.
    real = _load_trial_scan()
    original_grab = robot._grab

    def grab_then_change(args):
        r = original_grab(args)
        robot.scan = real
        return r
    robot._grab = grab_then_change

    nav = w.navigate_to_goal()

    assert nav.status == "blocked", "the leg must not run to completion at a door it can see"
    assert nav.blockage is not None and nav.blockage.blocked
    assert robot.scripts().count("nav2_goto.py") == 1, "the leg stopped at the first boundary"


def test_a_distant_blockage_does_not_stop_the_leg_before_it_starts(
        world, waypoints, fuse_installed, capsys):
    """The 8 m glass door that made three trials record path_length_m 0.0. The camera can see it;
    the robot has not driven toward it, so it is not 'blocked at the goal' yet."""
    waypoints({"outside": {"frame": "map", "x": 8.0, "y": 0.0}})
    w, robot = world(scan=_far_scan(), camera=CAMERA_SAYS_DOOR, map_pose=(0.0, 0.0),
                     nav2=[(0, "arrived at 'outside' in 30.0 s\n")])

    nav = w.navigate_to_goal()

    assert nav.status == "reached"
    assert "driving on" in capsys.readouterr().out
    assert robot.index_of("nav2_goto.py") >= 0, "the leg must actually be driven"


def test_a_map_frame_goal_never_falls_back_to_the_odom_driver(
        world, waypoints, fuse_installed, capsys):
    """Every waypoint on this robot is map-frame. `waypoints.py goto` drives on ODOM, so handing
    it a map coordinate is the two-origins-one-subtraction bug with the wheels turning -- and it
    turns a configuration error (Nav2 down, wrong map loaded) into a `timeout`, which is a claim
    about the world."""
    waypoints({"outside": {"frame": "map", "x": 8.0, "y": 0.0, "map_name": "atrium"}})
    w, robot = world(scan=_far_scan(), camera=CAMERA_SAYS_CLEAR,
                     nav2=[(3, "")])          # 3 == cannot serve: wrong/absent map name

    nav = w.navigate_to_goal()

    assert nav.status == "unreachable", "name the real problem instead of degrading silently"
    assert w._last_nav == "unreachable"
    assert robot.index_of("waypoints.py", "goto") == -1, "the odom driver must not be handed it"
    assert "NOT falling back" in capsys.readouterr().out


def test_an_odom_frame_goal_does_fall_back(world, waypoints, fuse_installed):
    """That IS what the odom driver is for, and it supervises itself at 20 Hz."""
    waypoints({"outside": {"frame": "odom", "x": 8.0, "y": 0.0}})
    w, robot = world(scan=_far_scan(), camera=CAMERA_SAYS_CLEAR, nav2=[(4, "")])

    nav = w.navigate_to_goal()

    assert nav.status == "reached"
    assert robot.index_of("waypoints.py", "goto") >= 0


# ================================================================= 5. resume to the original goal
def test_the_goal_survives_blocked_reverse_look_press(world, waypoints, fuse_installed,
                                                      monkeypatch, tmp_path):
    """press, THEN navigate back outside. The whole point of the mission is the leg AFTER the
    press, and it can only happen if self.goal is still there and at_goal() is honest."""
    waypoints({"outside": {"frame": "map", "x": 0.0, "y": 0.0}})
    w, robot = world(scan=_load_trial_scan(), camera=CAMERA_SAYS_CLEAR)

    # detect_frame / check_press_safe go out through subprocess.run, not _ros.
    runs: list[list[str]] = []

    def fake_run(args, **kw):
        args = [str(a) for a in args]
        runs.append(args)
        if _script(args) == "detect_frame.py":
            cap = Path(args[2])
            (cap / "detection.json").write_text(json.dumps(
                {"bbox_px": [100, 100, 190, 190], "score": 0.91,
                 "point3d_cam_m": [0.05, 0.0, 0.62]}))
        return subprocess.CompletedProcess(args, 0, "ok\n", "")
    monkeypatch.setattr(ros_world.subprocess, "run", fake_run)

    nav = w.navigate_to_goal()
    assert nav.status == "blocked"
    assert w.at_goal() is False, (
        "the early blocked return used to leave _last_nav at 'reached', so a trial that never "
        "navigated could be credited with an arrival the moment the blockage cleared")

    w.approach_blockage()
    w.scan_view()
    res = w.act(Plan(action_type="press_button", target_description="the ADA push button"),
                Detection(bbox=(100, 100, 190, 190), point3d=(0.05, 0.0, 0.62), score=0.91))

    assert res.success, res.detail
    assert w.goal == "outside", "nothing in the blocked path may destroy the goal"

    # ... and the resume leg then runs to that same goal.
    robot.nav2 = [(0, "arrived at 'outside' in 9.0 s\n")]
    robot.scan = _far_scan()
    monkeypatch.setattr(ros_world, "ask_blockage", lambda cap: dict(CAMERA_SAYS_CLEAR))
    again = w.navigate_to_goal()
    assert again.status == "reached" and w.at_goal() is True
    assert robot.calls[robot.index_of("nav2_goto.py")][2] == "outside"


def test_reset_clears_the_new_per_trial_state(world, fuse_installed):
    """run_campaign.py reuses ONE world across N trials. A 0.72 m standoff left over from trial 1
    would send trial 2 reversing before it had looked at anything."""
    w, robot = world(scan=_load_trial_scan(), camera=CAMERA_SAYS_DOOR)
    w.current_blockage()
    w._back_off_from_blockage()
    assert w._nearest_ahead_m is not None and w._backed_off is True

    w.reset()

    assert w._nearest_ahead_m is None and w._backed_off is False and w._evidence == ""
    assert w._scan_i == 0 and w._looks == []


# ================================================================= 6. a dry run moves nothing
def test_a_dry_run_performs_no_motion_at_all(world, waypoints, fuse_installed, capsys):
    """Every branch of the escalation, with dry_run=True, and not one command that moves the base
    or the arm. bringup's convention is that motion is the DEFAULT and --dry-run is what stops it,
    so a call that omits --dry-run is a call that drives."""
    waypoints({"outside": {"frame": "map", "x": 0.0, "y": 0.0}})
    w, robot = world(scan=_load_trial_scan(), camera=CAMERA_SAYS_CLEAR,
                     dry_run=True)

    nav = w.navigate_to_goal()
    assert nav.status == "blocked", "a dry run still reasons; it just does not drive"
    w.approach_blockage()
    w._back_off_from_blockage()
    w.scan_view()
    w.widen_view()
    w.strafe_view()
    w.recentre_view()
    w.act(Plan(action_type="press_button", target_description="the ADA push button"),
          Detection(bbox=(100, 100, 190, 190), point3d=(0.05, 0.0, 0.62), score=0.91))

    assert robot.motions() == [], (
        "a dry run issued these motion commands: "
        + "; ".join(" ".join(c) for c in robot.motions()))
    assert "DRY RUN: would back up" in capsys.readouterr().out, (
        "and it must still SAY what it would have done, or a dry run proves nothing about the "
        "sequence")
