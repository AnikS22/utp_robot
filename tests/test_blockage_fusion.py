"""Each test names the real failure it prevents.

The failure this whole module exists to stop, measured on hardware 2026-09-01: the robot stood
0.72 m from CLOSED GLASS DOORS, the camera reported "an open walkway with pillars" and
blocked=False -- correctly describing the picture, because glass is transparent to a camera --
and the system believed it. The lidar in the same instant had 85 returns within +-20 deg of
forward. Fusing with an AND, or a vote, would have cleared it. Only a fail-closed OR stops it.
"""
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from safety.blockage_fusion import fuse

# The two hardware captures this module was written from, kept as names so a test that cannot
# find them says WHICH real measurement it skipped rather than silently passing.
CAP_GLASS_SEEN_BY_LIDAR_ONLY = ROOT / "captures" / "trial_ours_001" / "scan.json"
CAP_GLASS_SEEN_BY_CAMERA_ONLY = ROOT / "captures" / "trial_ours_002" / "scan.json"


def load_scan(path: Path) -> dict:
    if not path.exists():
        pytest.skip("saved scan %s is not on this machine" % path)
    return json.loads(path.read_text())


# --- synthetic scans, for the cases no capture on disk covers -------------------------------
N = 360
INC = 2.0 * math.pi / N
A_MIN = -math.pi


def bin_at(deg: float) -> int:
    """Index of the bin pointing `deg` degrees off straight ahead."""
    return int(round((math.radians(deg) - A_MIN) / INC)) % N


def scan(hits=()) -> list:
    """All beams reporting no return (inf, which is what a LaserScan says for out-of-range),
    except the (degrees, metres) pairs given."""
    r = [float("inf")] * N
    for deg, m in hits:
        r[bin_at(deg)] = m
    return r


def wall_ahead(m: float = 0.72, n: int = 12) -> list:
    """A flat obstruction across the front, like the door in trial_ours_001."""
    return scan([(d, m) for d in range(-n // 2, n // 2 + 1)])


CLEAR = scan([(0.0, 6.0), (30.0, 5.0), (180.0, 2.0)])   # far wall ahead, something behind us

CAM_CLEAR = {"blocked": False, "kind": "", "description": "an open walkway with pillars"}
CAM_DOOR = {"blocked": True, "kind": "door", "description": "closed glass doors"}


# --- THE REGRESSION ------------------------------------------------------------------------

def test_trial_ours_001_the_frame_that_nearly_put_the_robot_through_a_glass_door():
    """REAL SAVED DATA, the exact instant of the 2026-09-01 near-miss.

    captures/trial_ours_001/scan.json is the scan captured with the rgb.png the VLM called "an
    open walkway with pillars", blocked=False. It is a correct reading of the image and a wrong
    reading of the world: the walkway is behind CLOSED GLASS. The scan holds 947 valid returns of
    1031 bins and 39 of them fall inside the drive corridor, the nearest at 0.70 m (the incident
    log's 0.72 m is the nearest within +-20 deg of forward, a narrower window; same door).

    With the camera's real verdict of blocked=False, fuse() must STILL say blocked, on the
    lidar's word alone. If this test ever goes red the robot drives into the door again.
    """
    d = load_scan(CAP_GLASS_SEEN_BY_LIDAR_ONLY)
    out = fuse(CAM_CLEAR, d["ranges"], d["angle_min"], d["angle_increment"])

    assert out["blocked"] is True, "the camera saw through the glass; the lidar did not"
    assert out["evidence"] == "lidar"
    assert out["nearest_ahead_m"] is not None
    assert 0.65 <= out["nearest_ahead_m"] <= 0.80, out["nearest_ahead_m"]
    # The description is what a language model downstream reasons over. "an open walkway with
    # pillars" attached to blocked=True would be worse than useless, so the lidar's finding leads
    # and the camera's contradicting words are kept, not dropped -- the disagreement is the hint
    # that the obstruction is transparent.
    assert "0.70 m ahead" in out["description"], out["description"]
    assert "camera did not report" in out["description"]
    assert "open walkway" in out["description"]
    # kind is NOT fabricated. The lidar cannot tell a door from a bollard, and a wrong "door"
    # sends the reasoner hunting a control that is not there.
    assert out["kind"] == ""


def test_trial_ours_002_the_same_doors_where_the_lidar_returns_nothing():
    """REAL SAVED DATA, the mirror-image failure, and the reason the rule cannot be an AND.

    Different pose and lighting on the same building. Here the camera is right -- blocked=True,
    kind=door, "closed glass doors" -- and the lidar corridor is EMPTY: zero returns in the box,
    nearest forward return 1.40 m. Each sensor is the only witness in exactly the case the other
    one gets wrong, so requiring agreement would clear both captures, one of which is a door at
    0.72 m.
    """
    d = load_scan(CAP_GLASS_SEEN_BY_CAMERA_ONLY)
    out = fuse(CAM_DOOR, d["ranges"], d["angle_min"], d["angle_increment"])

    assert out["blocked"] is True
    assert out["evidence"] == "camera"
    assert out["nearest_ahead_m"] is None, "nothing is inside the corridor in this scan"
    assert out["kind"] == "door" and out["description"] == "closed glass doors"


# --- the four combinations -----------------------------------------------------------------

def test_camera_blocked_and_lidar_clear_is_blocked_on_the_camera():
    """The glass the lidar misses. An AND would drive through it."""
    out = fuse(CAM_DOOR, CLEAR, A_MIN, INC)
    assert out["blocked"] is True and out["evidence"] == "camera"
    assert out["description"] == "closed glass doors" and out["kind"] == "door"


def test_lidar_blocked_and_camera_clear_is_blocked_on_the_lidar():
    """The glass the camera looks through -- trial_ours_001 in synthetic form."""
    out = fuse(CAM_CLEAR, wall_ahead(0.72), A_MIN, INC)
    assert out["blocked"] is True and out["evidence"] == "lidar"
    assert out["nearest_ahead_m"] == pytest.approx(0.72, abs=1e-6)


def test_both_sensors_agreeing_reports_both():
    out = fuse(CAM_DOOR, wall_ahead(0.72), A_MIN, INC)
    assert out["blocked"] is True and out["evidence"] == "both"
    # With the camera's own account of the obstruction available, it is not rewritten; the range
    # is carried in nearest_ahead_m instead.
    assert out["description"] == "closed glass doors"
    assert out["nearest_ahead_m"] == pytest.approx(0.72, abs=1e-6)


def test_neither_sensor_sees_anything_is_clear():
    """The OR must still be able to say no. A guard that never clears gets switched off."""
    out = fuse(CAM_CLEAR, CLEAR, A_MIN, INC)
    assert out["blocked"] is False and out["evidence"] == "neither"
    assert out["nearest_ahead_m"] is None
    assert out["description"] == "an open walkway with pillars"


# --- degenerate inputs ---------------------------------------------------------------------

def test_a_missing_camera_verdict_is_not_a_clear_verdict_but_is_not_a_stop_either():
    """WHY THIS IS THE SAFEST CHOICE, and it is a choice.

    camera=None, or a dict with no `blocked` key, means the VLM was down, timed out or replied
    with something unparseable. That is UNKNOWN, never False -- so it can never clear a blockage
    on its own, and the lidar keeps its full veto (asserted below).

    But UNKNOWN is not made to mean blocked while a WORKING sensor is reporting. When the scan is
    usable and the corridor is empty, a real instrument has returned a real negative, and this
    returns blocked=False with evidence "neither". The alternative -- freeze whenever the VLM
    hiccups -- fails in two ways at once: it is a policy about whether to run at all without a
    VLM, which belongs in preflight and not in a per-tick fusion; and it poisons the OR, because
    every dropped call becomes an "obstruction" and the reasoner goes looking for a control for a
    door that was never there. That is exactly the perception failure recorded as a reasoning
    failure that bringup/ask_blockage.py's docstring already warns about.

    The genuinely blind case -- no camera verdict AND no usable scan -- is the one that stops;
    see the test below.
    """
    for cam in (None, {}, {"kind": "door"}, {"blocked": None}, {"blocked": "true"},
                {"blocked": 1}, {"description": "x"}, "not a dict", 7):
        clear = fuse(cam, CLEAR, A_MIN, INC)
        assert clear["blocked"] is False, cam
        assert clear["evidence"] == "neither"

        stopped = fuse(cam, wall_ahead(0.72), A_MIN, INC)
        assert stopped["blocked"] is True, cam
        assert stopped["evidence"] == "lidar"
        # kind is passed through, never invented and never dropped: the lidar cannot classify
        # anything, so whatever the camera did or did not say about kind is what comes back.
        assert stopped["kind"] == (cam.get("kind", "") if isinstance(cam, dict) else ""), cam


def test_blind_on_both_sensors_fails_closed():
    """No camera verdict and nothing usable in the scan: there is no evidence in EITHER
    direction, so the robot stops. `evidence` stays "neither" because nothing was actually seen
    -- blocked=True with evidence "neither" means "stopped because nothing could see", which is a
    different fault to chase than a real door."""
    for rs in ([], [float("nan")] * N, [float("inf")] * N, None, [None] * N):
        out = fuse(None, rs, A_MIN, INC)
        assert out["blocked"] is True, rs
        assert out["evidence"] == "neither"
        assert out["nearest_ahead_m"] is None
        assert "nothing can confirm" in out["description"]
    # Broken scan GEOMETRY is just as blind as a broken scan: a NaN angle_increment makes every
    # computed coordinate NaN, every comparison False, and a dead lidar would otherwise read
    # exactly like an open corridor. A zero increment is the same -- every bin lands on one
    # bearing -- while a zero angle_min is perfectly legal and is NOT treated as broken.
    for bad in (float("nan"), float("inf"), 0.0):
        assert fuse(None, wall_ahead(0.72), A_MIN, bad)["blocked"] is True, bad
    for bad in (float("nan"), float("inf")):
        assert fuse(None, wall_ahead(0.72), bad, INC)["blocked"] is True, bad


def test_nan_and_inf_in_ranges_never_raise():
    """NaN and inf are LEGAL in a LaserScan (REP 117) and this lidar returns on only ~30-40% of
    its beams, so most bins are one or the other on every single scan."""
    rs = wall_ahead(0.72)
    for i in range(0, N, 3):
        rs[i] = float("nan")
    for i in range(1, N, 3):
        rs[i] = float("inf")
    rs[bin_at(150.0)] = None            # a malformed bin must not crash the guard
    rs[bin_at(160.0)] = "junk"
    for cam in (CAM_CLEAR, CAM_DOOR, None):
        out = fuse(cam, rs, A_MIN, INC)
        assert isinstance(out["blocked"], bool)
    # and the surviving wall is still seen
    assert fuse(CAM_CLEAR, rs, A_MIN, INC)["evidence"] == "lidar"


def test_one_stray_return_does_not_latch_the_robot():
    """min_hits exists because this lidar produces isolated spurious points; a single speckle
    must not pin the robot in place for the rest of the run."""
    for extra in ([], [(0.0, 0.55)], [(0.0, 0.55), (2.0, 0.56)]):
        out = fuse(CAM_CLEAR, scan([(0.0, 6.0)] + extra), A_MIN, INC)
        assert out["blocked"] is False, extra
        assert out["evidence"] == "neither"
    # Three is a thing, not a speckle.
    out = fuse(CAM_CLEAR, scan([(0.0, 0.55), (2.0, 0.56), (4.0, 0.57)]), A_MIN, INC)
    assert out["blocked"] is True and out["evidence"] == "lidar"
    # nearest_ahead_m is a MEASUREMENT, not a verdict: it is reported for the sub-threshold
    # speckle too, and reporting it must not have latched anything above.
    assert fuse(CAM_CLEAR, scan([(0.0, 0.55)]), A_MIN, INC)["nearest_ahead_m"] is not None


def test_returns_beside_and_behind_the_robot_are_not_an_obstruction():
    """A rectangle, not a cone -- corridor_blocked's reason. A wall 0.6 m off to the side, or
    anything behind, is not in the way, and vetoing on it would strand the robot in a corridor."""
    beside = scan([(80.0, 0.6), (85.0, 0.6), (90.0, 0.6), (-85.0, 0.6), (-90.0, 0.6)])
    behind = scan([(178.0, 0.4), (180.0, 0.4), (182.0, 0.4)])
    for rs in (beside, behind):
        out = fuse(CAM_CLEAR, rs, A_MIN, INC)
        assert out["blocked"] is False and out["nearest_ahead_m"] is None


def test_the_look_ahead_window_reaches_past_the_072_m_door():
    """Why the default is 1.30 m here and 0.90 m in corridor_blocked: 0.90 m catches a door at
    0.72 m with 18 cm to spare, under a second of travel at v_max, on a scan that may already be
    a tick old. Anything inside 1.30 m is reported; a wall across the room is not."""
    assert fuse(CAM_CLEAR, wall_ahead(1.20), A_MIN, INC)["blocked"] is True
    assert fuse(CAM_CLEAR, wall_ahead(1.60), A_MIN, INC)["blocked"] is False
    # and the window is still an argument, not a constant
    assert fuse(CAM_CLEAR, wall_ahead(1.60), A_MIN, INC, look_ahead_m=2.0)["blocked"] is True


def test_the_result_always_carries_the_full_contract():
    """Other agents code against these five keys; a missing one is an AttributeError deep in a
    control loop."""
    for cam in (None, {}, CAM_CLEAR, CAM_DOOR):
        for rs in ([], CLEAR, wall_ahead(0.72)):
            out = fuse(cam, rs, A_MIN, INC)
            assert set(out) == {"blocked", "kind", "description", "evidence", "nearest_ahead_m"}
            assert isinstance(out["blocked"], bool)
            assert isinstance(out["kind"], str) and isinstance(out["description"], str)
            assert out["evidence"] in ("camera", "lidar", "both", "neither")
            assert out["nearest_ahead_m"] is None or isinstance(out["nearest_ahead_m"], float)
