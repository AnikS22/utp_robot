"""A waypoint from a dead odom session must never be driven to."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.waypoint_frame import SESSION_KEY, check_session

LIVE = "aabbccdd11223344"
DEAD = "99887766ffeeddcc"


def wp(session, x=1.0):
    d = {"x": x, "y": 0.0, "yaw": 0.0}
    if session is not None:
        d[SESSION_KEY] = session
    return d


def test_same_session_passes():
    ok, why = check_session({"door": wp(LIVE), "button": wp(LIVE)}, LIVE)
    assert ok and why == ""


def test_waypoint_from_a_restarted_driver_is_refused():
    """The bug that produced zero successful trials: coordinates from a frame that is gone."""
    ok, why = check_session({"door": wp(DEAD)}, LIVE)
    assert not ok
    assert "door" in why and "DIFFERENT odom session" in why
    assert "rebase" in why          # the refusal names the way out


def test_legacy_waypoints_without_a_session_are_refused_not_trusted():
    """Pre-guard entries carry no id. A still-valid one is indistinguishable from a dead one."""
    ok, why = check_session({"door": wp(None)}, LIVE)
    assert not ok and "no odom session id" in why


def test_only_the_waypoints_the_route_visits_are_checked():
    """A stale waypoint nobody drives to is not a reason to refuse the run."""
    wps = {"door": wp(LIVE), "old_thing": wp(DEAD)}
    assert check_session(wps, LIVE, names={"door"})[0]
    assert not check_session(wps, LIVE, names={"door", "old_thing"})[0]


def test_unreadable_current_session_fails_closed():
    """No /odom publisher, or two of them: nothing to validate against, so do not drive."""
    ok, why = check_session({"door": wp(LIVE)}, None)
    assert not ok and "cannot read" in why


def test_no_relevant_waypoints_is_not_a_failure():
    assert check_session({"door": wp(DEAD)}, LIVE, names=set())[0]


def test_the_real_shipped_file_is_caught():
    """maps/waypoints.yaml as recorded 2026-08-26 -- three days and many restarts ago."""
    import yaml
    f = Path(__file__).resolve().parent.parent / "maps" / "waypoints.yaml"
    if not f.exists():
        return
    d = yaml.safe_load(f.read_text()) or {}
    if d and not any(SESSION_KEY in (v or {}) for v in d.values()):
        ok, why = check_session(d, LIVE)
        assert not ok and "no odom session id" in why
