"""Multi-floor ride planning and the handover gate. Headless: no ROS, no map, no lift.

The test that matters most in this file is the doors-closed one. Everything else here is ordinary
config validation; that one encodes the fact the whole design turns on, which is that a scan-match
fit taken inside a closed lift car is a confident number about the wrong question.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

from safety.floor_plan import (
    DOORS, MAX_FIT_AGE_S, MIN_FIT_PCT, NAV, PRESS, RIDE, SWAP, VERIFY, WARN_FIT_PCT,
    check_building, check_itinerary, floors_of, handover_gate, plan_ride, seed_pose,
)

REPO = Path(__file__).resolve().parents[1]
ROUTE_SH = REPO / "bringup" / "multifloor_route.sh"
FLOORS_YAML = REPO / "config" / "floors.yaml"

ALL_EXTS = {"pgm", "yaml", "posegraph", "data"}


def _cfg(**over):
    base = {
        "building": "test",
        "floors": {
            "1": {"map": "m1", "call_query": "the blue call button",
                  "select_query": "the blue button 1",
                  "waypoints": {"call_button": "cb1", "door_reverse": "dr1",
                                "car_facing_out": "co1", "car_panel": "cp1", "exit": "ex1"}},
            "2": {"map": "m2", "call_query": "the blue call button",
                  "select_query": "the blue button 2",
                  "waypoints": {"call_button": "cb2", "door_reverse": "dr2",
                                "car_facing_out": "co2", "car_panel": "cp2", "exit": "ex2"}},
        },
    }
    base.update(over)
    return base


def _wps(**over):
    out = {}
    for n, mp in (("cb1", "m1"), ("dr1", "m1"), ("co1", "m1"), ("cp1", "m1"), ("ex1", "m1"),
                  ("cb2", "m2"), ("dr2", "m2"), ("co2", "m2"), ("cp2", "m2"), ("ex2", "m2")):
        out[n] = {"frame": "map", "map_name": mp, "x": 1.0, "y": 2.0, "yaw": 0.5}
    out.update(over)
    return out


def _maps(*names):
    return {n: set(ALL_EXTS) for n in (names or ("m1", "m2"))}


# ---------------------------------------------------------------------------------- parsing
def test_floor_ids_are_strings_so_a_yaml_int_and_a_yaml_str_cannot_split_the_dict():
    """An unquoted `1:` in YAML is an int key and a quoted `"1":` is a str. A floors dict holding
    a mix of both looks perfectly correct in the file and misses every lookup at runtime, because
    nothing that reads a floor id off a command line will ever produce an int."""
    cfg = _cfg()
    cfg["floors"][3] = dict(cfg["floors"]["2"], map="m3")
    assert set(floors_of(cfg)) == {"1", "2", "3"}


@pytest.mark.parametrize("mutate,expect", [
    (lambda c: c["floors"]["1"].pop("map"), "missing: map"),
    (lambda c: c["floors"]["1"].pop("select_query"), "missing: select_query"),
    (lambda c: c["floors"]["1"]["waypoints"].pop("car_facing_out"), "car_facing_out"),
])
def test_a_malformed_floor_is_a_hard_parse_error_not_a_default(mutate, expect):
    cfg = _cfg()
    mutate(cfg)
    with pytest.raises(ValueError, match=re.escape(expect)):
        floors_of(cfg)


# ---------------------------------------------------------------------------------- building
def test_a_fully_recorded_building_validates():
    ok, why = check_building(_cfg(), _wps(), _maps())
    assert ok, why


def test_a_grid_only_map_is_refused_because_it_cannot_be_relocalized_into():
    """maps/<n>.pgm + .yaml is a picture. slam_toolbox localization deserializes the .posegraph,
    and given only a grid it comes up ACTIVE on a brand-new graph at the robot's feet -- the exact
    silent failure bringup/map_persist.sh and session.sh already refuse on the way in."""
    ok, why = check_building(_cfg(), _wps(), {"m1": ALL_EXTS, "m2": {"pgm", "yaml"}})
    assert not ok
    assert "posegraph" in why and "data" in why


def test_two_floors_naming_one_map_is_refused():
    """The most dangerous typo available here: every identity check in this stack compares MAP
    names, so one map on two floors makes the swap a no-op that reports success while the robot
    drives floor 1's route standing on floor 2."""
    cfg = _cfg()
    cfg["floors"]["2"]["map"] = "m1"
    ok, why = check_building(cfg, _wps(), _maps("m1"))
    assert not ok
    assert "would succeed while changing nothing" in why


def test_a_waypoint_name_reused_across_floors_is_refused():
    """maps/waypoints.yaml is one flat store shared by every map, so recording floor 2's
    'car_panel' overwrites floor 1's and both floors then drive to whichever survived."""
    cfg = _cfg()
    cfg["floors"]["2"]["waypoints"]["car_panel"] = "cp1"
    ok, why = check_building(cfg, _wps(), _maps())
    assert not ok
    assert "one flat store" in why


def test_an_odom_frame_waypoint_is_refused():
    ok, why = check_building(_cfg(), _wps(co2={"frame": "odom", "x": 0.0, "y": 0.0}), _maps())
    assert not ok
    assert "'odom' frame" in why


def test_a_waypoint_recorded_in_another_floors_map_is_refused():
    ok, why = check_building(
        _cfg(), _wps(co2={"frame": "map", "map_name": "m1", "x": 0.0, "y": 0.0}), _maps())
    assert not ok
    assert "different physical place" in why


def test_a_nameless_waypoint_is_refused_as_not_portable():
    ok, why = check_building(_cfg(), _wps(ex2={"frame": "map", "x": 0.0, "y": 0.0}), _maps())
    assert not ok
    assert "no map name" in why


def test_every_problem_is_reported_not_just_the_first():
    """An operator who has to re-record waypoints wants the whole list before walking to the lift,
    not one problem per trip."""
    ok, why = check_building(
        _cfg(),
        _wps(co2={"frame": "odom", "x": 0.0, "y": 0.0},
             ex2={"frame": "map", "x": 0.0, "y": 0.0}),
        {"m1": ALL_EXTS, "m2": {"pgm", "yaml"}})
    assert not ok
    assert len([ln for ln in why.splitlines() if ln.strip().startswith("-")]) >= 3


# ---------------------------------------------------------------------------------- itinerary
@pytest.mark.parametrize("itin,expect", [
    ([], "at least two floors"),
    (["1"], "at least two floors"),
    (["1", "9"], "unknown floor"),
    (["1", "1"], "rides from floor 1 to itself"),
])
def test_a_bad_itinerary_is_refused_before_anything_moves(itin, expect):
    ok, why = check_itinerary(_cfg(), itin)
    assert not ok
    assert expect in why


def test_a_round_trip_is_two_rides():
    steps = plan_ride(_cfg(), ["1", "2", "1"])
    assert [s.arg for s in steps if s.kind == SWAP] == ["2", "1"]
    assert [s.arg for s in steps if s.kind == VERIFY] == ["2", "1"]


# ---------------------------------------------------------------------------------- the plan
def test_the_plan_is_the_ride_in_order():
    steps = plan_ride(_cfg(), ["1", "2"])
    assert [(s.kind, s.arg) for s in steps] == [
        (NAV, "cb1"),
        (PRESS, "the blue call button"),
        (DOORS, "open"),
        (NAV, "dr1"),
        (NAV, "co1"),
        (NAV, "cp1"),
        (PRESS, "the blue button 2"),        # the DESTINATION floor's in-car button
        (NAV, "co1"),                        # face the doors while still genuinely localized
        (DOORS, "closed"),
        (SWAP, "2"),
        (RIDE, "2"),
        (DOORS, "open"),
        (VERIFY, "2"),
        (NAV, "ex2"),
    ]


def test_the_in_car_button_query_is_the_destinations_not_the_current_floors():
    """Pressing floor 1's own select_query while standing on floor 1 sends the lift nowhere. The
    query belongs to where you are GOING, which is easy to get backwards in a config."""
    steps = plan_ride(_cfg(), ["1", "2"])
    presses = [s.arg for s in steps if s.kind == PRESS]
    assert presses == ["the blue call button", "the blue button 2"]


def test_the_swap_is_before_the_ride_and_after_the_doors_close():
    """Before the ride so its tens of seconds overlap the ride instead of spending the destination
    floor's door hold. After the doors close because an open door lets the scan out into the
    ORIGIN floor's lobby, which is floor-1 geometry handed to a matcher holding a floor-2 seed."""
    kinds = [(s.kind, s.arg) for s in plan_ride(_cfg(), ["1", "2"])]
    close = kinds.index((DOORS, "closed"))
    swap = kinds.index((SWAP, "2"))
    ride = kinds.index((RIDE, "2"))
    assert close < swap < ride


def test_the_verify_step_is_after_the_doors_open_and_before_any_driving():
    """The gate is worthless before the doors open and useless after the robot has already left."""
    steps = plan_ride(_cfg(), ["1", "2"])
    kinds = [(s.kind, s.arg) for s in steps]
    verify = kinds.index((VERIFY, "2"))
    doors_after_ride = max(i for i, (k, a) in enumerate(kinds)
                           if k == DOORS and a == "open" and i < verify)
    assert doors_after_ride < verify
    driving_after = [i for i, (k, _a) in enumerate(kinds) if k == NAV and i > kinds.index((SWAP, "2"))]
    assert driving_after and min(driving_after) > verify


def test_nothing_drives_between_the_swap_and_the_gate():
    """The window in which the robot is seeded but unverified must contain no motion at all."""
    kinds = [s.kind for s in plan_ride(_cfg(), ["1", "2"])]
    between = kinds[kinds.index(SWAP) + 1: kinds.index(VERIFY)]
    assert NAV not in between and PRESS not in between


# ---------------------------------------------------------------------------------- seed
def test_the_seed_is_the_destination_floors_car_pose():
    wps = _wps(co2={"frame": "map", "map_name": "m2", "x": -1.86, "y": 0.44, "yaw": 1.02})
    assert seed_pose(_cfg(), "2", wps) == (-1.86, 0.44, 1.02)


@pytest.mark.parametrize("bad", [
    {"frame": "odom", "x": 0.0, "y": 0.0},
    {"frame": "map", "map_name": "m1", "x": 0.0, "y": 0.0},
])
def test_the_seed_refuses_a_pose_that_is_not_in_the_destination_map(bad):
    with pytest.raises(ValueError):
        seed_pose(_cfg(), "2", _wps(co2=bad))


def test_the_seed_refuses_a_waypoint_that_was_never_recorded():
    wps = _wps()
    del wps["co2"]
    with pytest.raises(ValueError, match="not recorded"):
        seed_pose(_cfg(), "2", wps)


# ---------------------------------------------------------------------------------- the gate
def test_a_perfect_fit_measured_with_the_doors_shut_is_refused():
    """THE TEST THIS FILE EXISTS FOR.

    A lift car is geometrically identical on every floor: with the doors shut the scan is four
    walls about a metre away, and it matches floor 1's map exactly as well as floor 2's. So a
    100% fit taken inside a closed car is a confident, well-formed answer to a question nobody
    asked, and treating it as evidence is how the robot drives out onto the wrong floor.
    """
    ok, why = handover_gate(expected_map="m2", loaded_map="m2", fit_pct=100.0,
                            doors_open=False, fit_age_s=0.0)
    assert not ok
    assert "EVERY floor" in why
    assert "not too low" in why      # and the fix is not a lower threshold


def test_nobody_having_looked_at_the_doors_is_also_a_refusal():
    """Never-seen and stale both mean 'not permitted' -- CLAUDE.md's rule for every other gate."""
    ok, _why = handover_gate(expected_map="m2", loaded_map="m2", fit_pct=95.0,
                             doors_open=None, fit_age_s=0.0)
    assert not ok


def test_the_gate_refuses_when_the_wrong_map_is_certified():
    ok, why = handover_gate(expected_map="m2", loaded_map="m1", fit_pct=95.0,
                            doors_open=True, fit_age_s=0.0)
    assert not ok
    assert "DO NOT DRIVE" in why


def test_the_gate_refuses_when_nothing_is_certified_at_all():
    """bringup/floor_swap.py deletes maps/.loaded_map before it touches anything, so a swap that
    died half-way leaves exactly this state -- and it must read as a refusal, not as a fresh start."""
    ok, why = handover_gate(expected_map="m2", loaded_map=None, fit_pct=95.0,
                            doors_open=True, fit_age_s=0.0)
    assert not ok
    assert "did not finish" in why


def test_an_unmeasurable_fit_is_a_refusal_not_a_shrug():
    ok, _why = handover_gate(expected_map="m2", loaded_map="m2", fit_pct=None,
                             doors_open=True, fit_age_s=0.0)
    assert not ok


def test_a_stale_fit_is_refused():
    ok, why = handover_gate(expected_map="m2", loaded_map="m2", fit_pct=95.0,
                            doors_open=True, fit_age_s=MAX_FIT_AGE_S + 1)
    assert not ok
    assert "moment that has passed" in why


def test_a_low_fit_with_the_doors_open_is_a_real_statement_and_refuses():
    ok, why = handover_gate(expected_map="m2", loaded_map="m2", fit_pct=MIN_FIT_PCT - 1,
                            doors_open=True, fit_age_s=0.0)
    assert not ok
    assert "wrong floor" in why


def test_the_gate_passes_with_the_doors_open_and_a_good_fit():
    ok, why = handover_gate(expected_map="m2", loaded_map="m2", fit_pct=88.0,
                            doors_open=True, fit_age_s=1.0)
    assert ok
    assert "doors open" in why


def test_a_fit_between_the_floor_and_the_warning_line_passes_but_says_so():
    ok, why = handover_gate(expected_map="m2", loaded_map="m2",
                            fit_pct=(MIN_FIT_PCT + WARN_FIT_PCT) / 2,
                            doors_open=True, fit_age_s=1.0)
    assert ok
    assert "below" in why


def test_the_in_car_threshold_clears_the_fits_actually_measured_inside_a_car():
    """MEASURED 2026-09-04 on floor 2, robot confirmed correctly localized (position inside the
    mapped car alcove, reached by a 1.39 m straight reverse with 1.0 deg of heading change):

        facing out, doors open   52.1%, 54.5%
        facing the panel wall    55.4%, 69.0%

    The gate originally required 60%, borrowed from elevator_route.sh -- which measures fit
    standing in a LOBBY, where most of the scan lands on mapped wall. Inside a car facing out
    through an open door, much of the scan leaves the room entirely and counts as misses. The
    borrowed number would have refused every one of those correct arrivals."""
    from safety.floor_plan import IN_CAR_MIN_FIT_PCT
    measured = [52.1, 54.5, 55.4, 69.0]
    assert IN_CAR_MIN_FIT_PCT < min(measured), (
        f"{IN_CAR_MIN_FIT_PCT}% would refuse a correct arrival measured at {min(measured)}%")
    assert IN_CAR_MIN_FIT_PCT < MIN_FIT_PCT, "the in-car threshold must be the looser one"
    ok, _ = handover_gate(expected_map="m2", loaded_map="m2", fit_pct=min(measured),
                          doors_open=True, fit_age_s=1.0, min_fit_pct=IN_CAR_MIN_FIT_PCT)
    assert ok


def test_the_verify_path_uses_the_in_car_threshold_not_the_lobby_one():
    src = (REPO / "bringup" / "floor_swap.py").read_text()
    assert "min_fit_pct=IN_CAR_MIN_FIT_PCT" in src, (
        "floor_swap.py --verify scores from inside the car; using the lobby threshold there "
        "refuses correct arrivals")


def test_the_gate_thresholds_are_the_ones_the_single_floor_route_already_uses():
    """bringup/elevator_route.sh dies below 60% and warns below 75%. A multi-floor route holding
    itself to a different standard than the route it forked would be a silent disagreement."""
    src = (REPO / "bringup" / "elevator_route.sh").read_text()
    assert re.search(r"f\s*<\s*60", src) and re.search(r"f\s*<\s*75", src)
    assert (MIN_FIT_PCT, WARN_FIT_PCT) == (60.0, 75.0)


# ---------------------------------------------------------------------------------- wiring
# Same idea as tests/test_stack_wiring.py: the route is a linear bash script, on purpose, and a
# linear script cannot be checked by reading it. These assert the script and plan_ride() agree, so
# reordering one without the other fails here rather than in a lift.
def _script_steps() -> list[tuple[str, str]]:
    """(kind, token) for every executable step in the route, in file order."""
    out: list[tuple[str, str]] = []
    for line in ROUTE_SH.read_text().splitlines():
        s = line.strip()
        if s.startswith("#") or s.startswith("nav()") or s.startswith("press()"):
            continue
        m = re.match(r'^(nav|press)\s+"\$\{?([A-Z_0-9]+)', s)
        if m:
            out.append((m.group(1), m.group(2)))
            continue
        m = re.match(r'^doors\s+"(.*)"', s)
        if m:
            # The script's door prompts are written for a human. "closed" is the one that changes
            # what the software may do, so classify on it and treat everything else as open.
            out.append(("doors", "closed" if "clos" in m.group(1).lower() else "open"))
            continue
        if "floor_swap.py" in s and "--to " in s:
            if not out or out[-1] != ("swap", "TO"):
                out.append(("swap", "TO"))
        elif "floor_swap.py" in s and "--verify " in s:
            if not out or out[-1] != ("verify", "TO"):
                out.append(("verify", "TO"))
    return out


def test_the_route_script_executes_the_planned_steps_in_the_planned_order():
    cfg = yaml.safe_load(FLOORS_YAML.read_text())
    floors = floors_of(cfg)
    a, b = sorted(floors)[0], sorted(floors)[1]
    role_of = {fid: {name: role for role, name in floors[fid].waypoints.items()}
               for fid in (a, b)}

    expected: list[tuple[str, str]] = []
    expect_entry_next = False
    for st in plan_ride(cfg, [a, b]):
        tag = "A" if st.floor == a else "B"
        if st.kind == NAV:
            role = role_of[st.floor][st.arg]
            # The script holds the two entry legs in ENTRY_APPROACH / ENTRY_POSE so ONE script
            # serves both the forward and the reverse shape, resolving them from the config at
            # runtime. plan_ride resolves the same pair eagerly, so the mapping is POSITIONAL, not
            # by role: `car_facing_out` legitimately appears twice in a reverse-entry plan -- once
            # as the entry itself and once as the pre-ride turn to face the doors -- and only the
            # one immediately after the approach is the entry.
            if role in ("door_facing", "door_reverse"):
                var, expect_entry_next = "ENTRY_APPROACH", True
            elif expect_entry_next:
                var, expect_entry_next = "ENTRY_POSE", False
            else:
                var = f"{tag}_{role.upper()}"
            expected.append(("nav", var))
        elif st.kind == PRESS:
            which = "CALL_QUERY" if st.arg == floors[st.floor].call_query else "SELECT_QUERY"
            # The in-car press names the DESTINATION floor's button while standing on the origin
            # floor, so its variable is B_ even though the step's floor is A.
            expected.append(("press", f"{'A' if which == 'CALL_QUERY' else 'B'}_{which}"))
        elif st.kind == DOORS:
            expected.append(("doors", st.arg))
        elif st.kind == SWAP:
            expected.append(("swap", "TO"))
        elif st.kind == VERIFY:
            expected.append(("verify", "TO"))
        # RIDE is not an executable step: it is the interval in which software does nothing.

    assert _script_steps() == expected


def test_the_route_script_never_drives_between_the_swap_and_the_gate():
    steps = _script_steps()
    kinds = [k for k, _ in steps]
    between = kinds[kinds.index("swap") + 1: kinds.index("verify")]
    assert "nav" not in between and "press" not in between


def test_the_route_script_reads_its_waypoint_names_from_the_config():
    """Two floors' worth of names hardcoded in bash is two places to typo and no way to check
    either. safety/floor_plan.check_building() only covers this route if the route reads the
    config it validates."""
    src = ROUTE_SH.read_text()
    assert "config/floors.yaml" in src
    assert "floors_of" in src


def test_the_relaunched_slam_node_lands_in_the_pid_file_session_sh_kills_from():
    """floor_swap.py replaces a process session.sh started. If its pid does not land in the same
    file, `session.sh down` leaves a slam_toolbox running -- and the next `session.sh nav` then
    finds a live /map, believes the stack is already up, and certifies the WRONG map as loaded.
    That is the trap start_nav() spends thirty lines refusing; it must not be reachable from here."""
    swap = (REPO / "bringup" / "floor_swap.py").read_text()
    sess = (REPO / "bringup" / "session.sh").read_text()
    path = re.search(r'PIDS_FILE=(\S+)', sess)
    assert path, "session.sh no longer defines PIDS_FILE"
    assert path.group(1) in swap, (
        f"session.sh kills from {path.group(1)}; floor_swap.py must append the node it starts "
        f"to that same file")


def test_the_marker_publisher_only_draws_waypoints_from_the_LOADED_map():
    """2026-09-04: the operator opened RViz on a freshly built second-floor map and found floor 1's
    five elevator waypoints drawn across it. Nothing was wrong with the map or with SLAM --
    waypoint_markers.py drew the whole flat store in the `map` frame with no map filter, so
    coordinates from a map with an unrelated origin were painted onto this one. It looks entirely
    convincing: the numbers are real and the arrows point correctly.

    The module already refused ODOM-frame waypoints for exactly this reason ("painting it on the
    map would assert a physical location it does not have"). This is that same argument one level
    up, and the display is the last place in this stack that was ignoring provenance."""
    src = (REPO / "bringup" / "waypoint_markers.py").read_text()
    assert "current_map_name" in src, (
        "waypoint_markers.py must consult the loaded map name, or it will draw every map's "
        "waypoints on whichever map happens to be open")
    assert re.search(r"wp_map\s*!=\s*live", src), (
        "waypoint_markers.py must skip waypoints whose map_name is not the loaded map")


def test_the_route_script_folds_the_arm_on_a_failed_run():
    """Inherited from elevator_route.sh: the base is gated on measured joint angles, so an arm
    left extended makes the mux discard every command and the next leg reads as a navigation
    failure a long way from its cause."""
    src = ROUTE_SH.read_text()
    assert "trap _fold_on_exit EXIT" in src
    assert "stow_arm.py" in src


# ---------------------------------------------------------------------------------- shipped cfg
def test_the_shipped_config_parses():
    floors = floors_of(yaml.safe_load(FLOORS_YAML.read_text()))
    assert len(floors) >= 2
    for fid, fl in floors.items():
        assert fl.map and fl.call_query and fl.select_query, fid


def test_the_shipped_config_plans_a_ride_between_its_first_two_floors():
    cfg = yaml.safe_load(FLOORS_YAML.read_text())
    a, b = sorted(floors_of(cfg))[:2]
    assert plan_ride(cfg, [a, b])


def test_floor_one_of_the_shipped_config_matches_the_waypoints_on_disk():
    """Floor 1 is the route that has actually been driven -- its five waypoints were recorded on
    2026-09-01 in map 'elevator'. If check_building() reports a problem with the shipped config it
    must be about a floor nobody has recorded yet, never about floor 1."""
    cfg = yaml.safe_load(FLOORS_YAML.read_text())
    store = yaml.safe_load((REPO / "maps" / "waypoints.yaml").read_text()) or {}
    present = {}
    for ext in ALL_EXTS:
        for p in (REPO / "maps").glob(f"*.{ext}"):
            present.setdefault(p.name[: -(len(ext) + 1)], set()).add(ext)
    ok, why = check_building(cfg, store, present)
    if not ok:
        first = sorted(floors_of(cfg))[0]
        offending = [ln for ln in why.splitlines() if f"floor {first}:" in ln]
        assert not offending, "floor %s is the driven route and must validate:\n%s" % (
            first, "\n".join(offending))
