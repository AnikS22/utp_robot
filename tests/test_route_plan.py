"""Each test names the failure it prevents. A route is a list of names; a typo in one of them
is invisible until the robot has driven six legs and refuses the seventh."""
import pytest
from safety.route_plan import (ACTION, GOTO, MAX_WAIT_S, RouteState, Step, WAIT,
                               parse_route, validate_route)

WPS = {"start", "door_approach", "through_door"}
ACTS = {"press_button", "call_elevator"}


def test_parses_the_three_step_kinds():
    r = parse_route([{"goto": "start"},
                     {"action": "press_button", "query": "the ADA plate"},
                     {"wait": 5}])
    assert [s.kind for s in r] == [GOTO, ACTION, WAIT]
    assert r[1].params["query"] == "the ADA plate"
    assert r[2].params["seconds"] == 5.0


def test_unknown_waypoint_is_caught_before_anything_moves():
    """The whole reason this module is pure: catching this needs no robot."""
    errs = validate_route(parse_route([{"goto": "door_aproach"}]), WPS, ACTS)
    assert errs and "unknown waypoint" in errs[0]


def test_typo_suggests_the_near_miss():
    errs = validate_route(parse_route([{"goto": "doo"}]), WPS, ACTS)
    assert "door_approach" in errs[0]


def test_reports_every_error_not_just_the_first():
    """One error per run means one drive per typo."""
    errs = validate_route(parse_route(
        [{"goto": "nope"}, {"action": "nope2"}, {"goto": "nope3"}]), WPS, ACTS)
    assert len(errs) == 3


def test_unknown_action_is_caught():
    errs = validate_route(parse_route([{"action": "open_door"}]), WPS, ACTS)
    assert errs and "unknown action" in errs[0]


def test_valid_route_has_no_errors():
    r = parse_route([{"goto": "start"}, {"action": "press_button"}, {"wait": 3},
                     {"goto": "through_door"}])
    assert validate_route(r, WPS, ACTS) == []


def test_empty_route_is_an_error():
    assert validate_route([], WPS, ACTS)


def test_absurd_wait_is_refused():
    """A robot parked in a corridor for ten minutes needs a person, not a timer."""
    errs = validate_route(parse_route([{"wait": MAX_WAIT_S + 1}]), WPS, ACTS)
    assert errs and "needs a person" in errs[0]


def test_negative_wait_is_refused():
    assert validate_route(parse_route([{"wait": -1}]), WPS, ACTS)


def test_malformed_steps_raise_with_the_index():
    with pytest.raises(ValueError, match="step 1"):
        parse_route([{"goto": "start"}, {"fly": "moon"}])
    with pytest.raises(ValueError, match="step 0"):
        parse_route(["start"])
    with pytest.raises(ValueError, match="wait needs a number"):
        parse_route([{"wait": "soon"}])


def test_state_advances_and_completes():
    st = RouteState(parse_route([{"goto": "start"}, {"wait": 1}]))
    assert st.current.kind == GOTO
    st.advance(); assert st.current.kind == WAIT
    st.advance(); assert st.done and st.current is None


def test_failure_stops_the_route_and_does_not_skip():
    """A failed leg means the world did not match the route. Continuing to the next waypoint on
    a stale pose is how a robot ends up somewhere nobody chose."""
    st = RouteState(parse_route([{"goto": "start"}, {"goto": "through_door"}]))
    st.fail("blocked")
    assert st.done and st.current is None
    assert "blocked" in st.progress() and "FAILED" in st.progress()


def test_progress_is_human_readable_at_each_stage():
    st = RouteState(parse_route([{"goto": "start"}, {"action": "press_button"}]))
    assert "step 1/2" in st.progress() and "start" in st.progress()
    st.advance()
    assert "step 2/2" in st.progress() and "press_button" in st.progress()
    st.advance()
    assert st.progress() == "complete (2/2)"
