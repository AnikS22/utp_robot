"""Each test names the real failure it prevents.

The failure this whole module exists to stop: a fresh MOLA and a MOLA localized in a saved map
produce an IDENTICAL-looking TF tree, and only one of them makes stored coordinates portable.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.map_frame import (FRAME_KEY, MAP_NAME_KEY, MOLA_SESSION_KEY,
                              check_map_session, frame_of, split_by_frame)

MOLA = "aabbccdd11223344"


def mapwp(name=None, sess=MOLA):
    w = {"x": 1.0, "y": 2.0, "yaw": 0.0, FRAME_KEY: "map", MOLA_SESSION_KEY: sess}
    if name:
        w[MAP_NAME_KEY] = name
    return w


def odomwp():
    return {"x": 1.0, "y": 2.0, "yaw": 0.0, "odom_session": "ffff0000ffff0000"}


def test_a_waypoint_with_no_frame_field_is_an_odom_waypoint():
    """Every waypoint recorded before map support existed must keep validating as odom, not
    silently become a map waypoint and get checked against a map that never existed."""
    assert frame_of(odomwp()) == "odom"
    assert frame_of({}) == "odom"
    odom, mapf = split_by_frame({"a": odomwp(), "b": mapwp()})
    assert list(odom) == ["a"] and list(mapf) == ["b"]


def test_map_waypoints_are_refused_when_mola_is_not_running():
    """No pose publisher means no map frame; the coordinates cannot be interpreted at all."""
    ok, why = check_map_session({"b": mapwp()}, current_map=None, current_mola=None)
    assert not ok and "not publishing a pose" in why


def test_named_map_waypoint_validates_against_the_same_map():
    """The portable case: recorded in 'atrium', localized in 'atrium'."""
    ok, why = check_map_session({"b": mapwp("atrium")}, "atrium", MOLA)
    assert ok, why


def test_named_map_waypoint_is_refused_in_a_DIFFERENT_map():
    ok, why = check_map_session({"b": mapwp("atrium")}, "garage", MOLA)
    assert not ok and "atrium" in why and "garage" in why


def test_named_waypoint_is_refused_when_mola_started_fresh():
    """THE TRAP. A fresh MOLA has a `map` frame whose origin is wherever the robot booted. The TF
    tree is indistinguishable from the localized case, so driving would go somewhere arbitrary."""
    ok, why = check_map_session({"b": mapwp("atrium")}, None, MOLA)
    assert not ok and "FRESH" in why and "map_load" in why.replace(" ", "_") or "load the map" in why


def test_nameless_waypoint_survives_within_its_own_mola_session():
    """No saved map, but the same MOLA instance is still running: valid, session-scoped."""
    ok, why = check_map_session({"b": mapwp(None)}, None, MOLA)
    assert ok, why


def test_nameless_waypoint_is_refused_after_mola_restarts():
    """Same failure as a re-zeroed odom frame: new origin, old numbers."""
    ok, why = check_map_session({"b": mapwp(None)}, None, "9999888877776666")
    assert not ok and "no longer running" in why


def test_only_the_named_route_is_checked():
    """A stale waypoint nobody drives to is not a reason to refuse the run."""
    wps = {"good": mapwp(None, MOLA), "stale": mapwp(None, "0000")}
    assert check_map_session(wps, None, MOLA, names=["good"])[0]
    assert not check_map_session(wps, None, MOLA, names=["good", "stale"])[0]


def test_odom_waypoints_are_never_touched_by_this_guard():
    """This module must have no opinion about odom waypoints -- waypoint_frame.py owns those."""
    ok, why = check_map_session({"a": odomwp()}, None, None)
    assert ok and why == ""
