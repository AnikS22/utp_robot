"""Multi-floor: which map is the robot in, and may it move yet? Pure logic, no ROS, no I/O.

Same split as the rest of safety/: the decision lives here and is unit-tested headlessly,
bringup/floor_swap.py is the plumbing that carries it out.

WHAT A LIFT RIDE ACTUALLY IS
----------------------------
Every other leg in this stack moves the robot continuously through one map, and both halves of the
localization stack are built on that. slam_toolbox correlates each sweep against the last; a Nav2
costmap is a picture of one floor. A lift ride breaks both at once: the robot is carried several
metres vertically into a room the loaded map does not contain, without a single scan in between.
There is no trajectory to follow and nothing to match against.

So a ride is not a navigation problem, and trying to make Nav2 solve it is the wrong shape. It is a
HANDOVER between two maps, and only two questions matter -- which map is live, and has the claim
"we are on floor N" actually been checked.

THE TRAP, STATED BEFORE THE CODE BECAUSE IT IS THE WHOLE REASON THIS MODULE IS FAIL-CLOSED
------------------------------------------------------------------------------------------
A LIFT CAR IS GEOMETRICALLY IDENTICAL ON EVERY FLOOR. With the doors shut the scan sees four walls
about a metre away (docs/MORNING.md measures the car's side walls at 1.00-1.15 m) and nothing else.
That scan matches the car in floor 1's map exactly as well as it matches the car in floor 2's map.

A scan-match fit measured INSIDE A CLOSED CAR is therefore not evidence of anything. It is
confident, well-formed, and answers a different question than the one asked -- the "a device that
answers is not a device that works" failure from docs/AGENT_BRIEF.md, in its purest form. Worse
than useless: it is the kind of number that ends an investigation early.

The floor becomes observable only when the doors open and the lobby comes into view. So
``handover_gate`` REFUSES a fit that was not measured with the doors open, however high it is.

WHY ONE MAP PER FLOOR, AND WHY THE SLAM NODE IS RESTARTED RATHER THAN RE-SEEDED
------------------------------------------------------------------------------
Two floors cannot share a slam_toolbox pose graph: the graph is built from a continuous drive and
there is no drive between floors. So each floor is its own saved map with its own origin, exactly
as unrelated as 'atrium' and 'elevator' already are.

Given that, the handover could be done two ways, and the choice is not cosmetic:

  * call /slam_toolbox/deserialize_map on the running node. The node keeps its identity, so
    pose_source.slam_session_id keeps returning the SAME id -- and safety/map_frame.py, which
    decides whether a stored waypoint is still meaningful, compares exactly that id. Floor 1's
    waypoints would go on validating after the robot reached floor 2. The stack would be wrong in
    the one place it is designed to be right.
  * RESTART the node on the new map. The DDS GID changes, so the session id changes, so every
    floor-1 waypoint is refused the moment the swap happens, by machinery that already exists and
    is already tested. The correct behaviour falls out instead of being added.

The second one. bringup/floor_swap.py restarts, and clears maps/.loaded_map BEFORE it does, so a
swap that dies half-way leaves nothing certified rather than something stale.

WHAT IS NOT SOLVED HERE, SAID PLAINLY
-------------------------------------
Nothing in this module knows the lift arrived, or which floor it stopped at. The car has no sensor
the robot can read and the floor indicator is a VLM question nobody has asked yet. For the demo the
operator declares arrival, exactly as they already hold the doors in bringup/elevator_route.sh --
and then the geometry check below is what has the final word. The operator's claim starts the gate;
it does not pass it.
"""
from __future__ import annotations

from dataclasses import dataclass, field

# ---------------------------------------------------------------------------------------------
# The step kinds a ride decomposes into. Deliberately the same flat vocabulary as
# safety/route_plan.py's GOTO/ACTION/WAIT/CHECK: a ride is a longer list, not a different kind of
# thing, and the moment it becomes a state machine it stops being reviewable before it runs.
# ---------------------------------------------------------------------------------------------
NAV = "nav"          # drive to a waypoint on the CURRENTLY loaded floor's map
PRESS = "press"      # ground and press a control, by query string
DOORS = "doors"      # wait for the doors -- operator, or bringup/doors_open.py
RIDE = "ride"        # the robot is carried; nothing in software is true during this
SWAP = "swap"        # restart localization on the destination floor's map, seeded in the car
VERIFY = "verify"    # score the live scan against the destination map, DOORS OPEN
KINDS = (NAV, PRESS, DOORS, RIDE, SWAP, VERIFY)

# Waypoint roles every floor must define. These are ROLES, not names: the name on floor 2 will not
# be the name on floor 1, because both live in maps/waypoints.yaml and the store is flat.
REQUIRED_ROLES = ("call_button", "door_reverse", "car_facing_out", "car_panel", "exit")

# ...except that door_reverse is only required for a floor that BACKS INTO the car. A floor that
# enters nose-first defines door_facing + car_facing_in instead and never reverses, so it has no
# reverse pose to record. Floor 2 is such a floor (see config/floors.yaml for the two measurements
# that forced forward entry: a 176 deg turn takes 21.2 s and the ADA opener does not hold that
# long, and reversing aims the OS0's near-field artifact into the doorway).
#
# The alternative was to point floor 2's door_reverse at its door_facing pose to satisfy the
# schema. That would have been a lie in a config file that safety code reads, and it is exactly
# how a check stops meaning anything: it would still pass, while naming a pose whose heading is
# 176 deg from the one the name promises.
FORWARD_ENTRY_ROLES = ("door_facing", "car_facing_in")

# A map you can relocalize into is all four files or it is a picture -- the same rule
# bringup/map_persist.sh enforces on the way in, restated here so a config referencing a
# grid-only map is refused before the robot is in a lift.
REQUIRED_MAP_EXTS = ("pgm", "yaml", "posegraph", "data")

# The same numbers bringup/elevator_route.sh already gates its single-floor preflight on, so the
# multi-floor route does not quietly hold itself to a different standard. relocalise.py's own
# docstring is the source: "A fit above ~80% is localized. 50% is lost."
MIN_FIT_PCT = 60.0
WARN_FIT_PCT = 75.0

# THE IN-CAR THRESHOLD IS NOT THE LOBBY THRESHOLD, AND ASSUMING IT WAS WAS A REAL BUG.
#
# The two numbers above are elevator_route.sh's, and they are measured STANDING IN A LOBBY, where
# most of the scan lands on mapped walls. The handover gate scores a robot SITTING IN A CAR WITH
# THE DOORS OPEN, and that is a different beam geometry: the robot is facing out, so a large share
# of the scan leaves through the doorway and lands on free or unmapped cells, which the fit counts
# as misses. It is depressed by the door, not by being lost.
#
# MEASURED 2026-09-04, floor 2, robot correctly localized (position confirmed inside the mapped car
# alcove, arrived by a 1.39 m straight reverse with 1.0 deg of heading change):
#     facing out, doors open   52.1%, 54.5%
#     facing the panel wall    55.4%, 69.0%
# Against a 60% floor the gate would have refused every one of those. 40% clears the measured band
# with margin.
#
# WHAT THIS THRESHOLD IS AND IS NOT. It is calibrated to pass a CORRECT arrival. It is NOT
# calibrated to reject a WRONG-FLOOR arrival, because nobody has ridden the lift and measured one.
# Worse, there is reason to think whole-scan fit discriminates floors poorly no matter where the
# line sits: most of the scan is car wall, and the car is identical on every floor, so the
# informative beams -- the ones that exit the doorway and hit the lobby -- are a minority that the
# majority drowns out. The honest fix is to score only those beams (range beyond the car walls,
# ~1.5 m+); that is not written yet.
#
# So with this value the fit test is close to vacuous and the gate's real work is done by its other
# three conditions: the right map is certified, the doors were observed open, and a fit could be
# measured at all. Those catch a failed or partial swap, which is the likeliest failure. They do
# not catch a lift that stopped on the wrong floor. Until the beam-subset scoring exists, THE
# OPERATOR READING THE FLOOR INDICATOR IS THE REAL CHECK -- see docs/MULTIFLOOR.md.
IN_CAR_MIN_FIT_PCT = 40.0

# A fit is a measurement of a moment. The doors are open for a bounded time and the robot is about
# to drive; a score from a minute ago describes a robot that may since have been nudged, or a
# matcher that has since slid. Stale is refused, like every other gate in this repo.
MAX_FIT_AGE_S = 30.0


@dataclass(frozen=True)
class Step:
    kind: str
    #: waypoint name (NAV), query string (PRESS), floor id (SWAP/VERIFY/RIDE), free text (DOORS)
    arg: str = ""
    #: which floor's map must be loaded for this step to mean anything
    floor: str = ""
    note: str = ""


@dataclass
class Floor:
    id: str
    map: str
    waypoints: dict          # role -> waypoint name
    call_query: str          # the plate OUTSIDE that calls the car to this floor
    select_query: str        # the button INSIDE the car that sends it to this floor
    description: str = ""
    extra: dict = field(default_factory=dict)


# ---------------------------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------------------------
def floors_of(cfg: dict) -> dict[str, Floor]:
    """Parse config/floors.yaml into Floor objects. Raises ValueError on a malformed entry.

    Floor ids are strings throughout, including "1". YAML will happily give an int for an unquoted
    1 and a str for "1", and a dict keyed on a mix of the two looks correct in the file and misses
    every lookup at runtime.
    """
    raw = (cfg or {}).get("floors")
    if not isinstance(raw, dict) or not raw:
        raise ValueError("config has no 'floors' mapping")
    out: dict[str, Floor] = {}
    for fid, spec in raw.items():
        key = str(fid)
        if not isinstance(spec, dict):
            raise ValueError(f"floor '{key}' is not a mapping")
        missing = [k for k in ("map", "waypoints", "call_query", "select_query") if not spec.get(k)]
        if missing:
            raise ValueError(f"floor '{key}' is missing: {', '.join(missing)}")
        wps = spec["waypoints"]
        if not isinstance(wps, dict):
            raise ValueError(f"floor '{key}' waypoints is not a mapping")
        required = list(REQUIRED_ROLES)
        if all(wps.get(r) for r in FORWARD_ENTRY_ROLES):
            required.remove("door_reverse")     # forward entry: nothing ever reverses in
        absent = [r for r in required if not wps.get(r)]
        if absent:
            raise ValueError(f"floor '{key}' has no waypoint for role(s): {', '.join(absent)}")
        out[key] = Floor(id=key, map=str(spec["map"]),
                         waypoints={r: str(wps[r]) for r in wps},
                         call_query=str(spec["call_query"]),
                         select_query=str(spec["select_query"]),
                         description=str(spec.get("description", "")),
                         extra={k: v for k, v in spec.items()
                                if k not in ("map", "waypoints", "call_query", "select_query",
                                             "description")})
    return out


# ---------------------------------------------------------------------------------------------
# Validation -- all of it before anything moves
# ---------------------------------------------------------------------------------------------
def check_building(cfg: dict, waypoints: dict, maps_present: dict) -> tuple[bool, str]:
    """Is this building config drivable AT ALL, given what is on disk?

    waypoints     the loaded maps/waypoints.yaml store: name -> {frame, map_name, x, y, yaw, ...}
    maps_present  map name -> the set of extensions found in maps/, e.g. {'elevator': {'pgm', ...}}

    Checked here rather than at the lift because a typo in a waypoint name is invisible until the
    robot has already ridden a floor -- and by then it is standing in a car, on an unverified map,
    with the doors about to close. This is safety/route_plan.py's argument, one storey up.

    Returns (ok, message). The message lists EVERY problem found, not the first: an operator who
    has to re-record waypoints wants the whole list before walking to the lift, not one per trip.
    """
    try:
        floors = floors_of(cfg)
    except ValueError as e:
        return False, f"config/floors.yaml is malformed: {e}"

    problems: list[str] = []
    for fid, fl in sorted(floors.items()):
        have = set(maps_present.get(fl.map) or ())
        lacks = [e for e in REQUIRED_MAP_EXTS if e not in have]
        if lacks:
            problems.append(
                f"floor {fid}: map '{fl.map}' is missing maps/{fl.map}.{{{','.join(lacks)}}}. "
                f"A .pgm/.yaml pair is a picture -- localization mode deserializes the .posegraph, "
                f"and given only a grid it starts a NEW graph at the robot's feet while looking "
                f"healthy. Re-map and save: bash bringup/map_persist.sh save {fl.map}")

        for role in fl.waypoints:
            name = fl.waypoints.get(role)
            wp = (waypoints or {}).get(name)
            if wp is None:
                problems.append(f"floor {fid}: waypoint '{name}' (role {role}) is not recorded")
                continue
            if (wp.get("frame") or "odom") != "map":
                problems.append(
                    f"floor {fid}: waypoint '{name}' is in the '{wp.get('frame') or 'odom'}' "
                    f"frame. Nav2 needs a map pose, and an odom pose dies with the next "
                    f"ranger_base restart")
                continue
            wp_map = wp.get("map_name")
            if not wp_map:
                problems.append(
                    f"floor {fid}: waypoint '{name}' carries no map name -- recorded against a "
                    f"fresh SLAM session whose origin is wherever the robot booted, so it is not "
                    f"portable to the next session, let alone the next floor")
            elif wp_map != fl.map:
                problems.append(
                    f"floor {fid}: waypoint '{name}' was recorded in map '{wp_map}' but this "
                    f"floor is map '{fl.map}'. Unrelated origins -- the coordinate names a "
                    f"different physical place")

    # Two floors sharing one map is the single most dangerous typo available here: every check
    # downstream compares MAP names, so with one map on two floors the swap becomes a no-op that
    # reports success, and the robot drives floor 1's route while standing on floor 2.
    by_map: dict[str, list[str]] = {}
    for fid, fl in floors.items():
        by_map.setdefault(fl.map, []).append(fid)
    for mp, fids in sorted(by_map.items()):
        if len(fids) > 1:
            problems.append(
                f"floors {', '.join(sorted(fids))} all name map '{mp}'. Every identity check in "
                f"this stack compares map names, so the floor swap would succeed while changing "
                f"nothing. Each floor needs its own saved map")

    # Likewise a waypoint name reused across floors: the store is flat, so the second recording
    # overwrites the first and both floors then drive to whichever survived.
    seen: dict[str, str] = {}
    for fid, fl in sorted(floors.items()):
        for role, name in sorted(fl.waypoints.items()):
            if name in seen and seen[name] != fid:
                problems.append(
                    f"waypoint name '{name}' is used by floor {seen[name]} AND floor {fid}. "
                    f"maps/waypoints.yaml is one flat store: recording the second one overwrote "
                    f"the first. Give every floor its own names")
            seen[name] = fid

    if problems:
        return False, "\n".join("  - " + p for p in problems)
    return True, ""


def check_itinerary(cfg: dict, itinerary) -> tuple[bool, str]:
    """Is this sequence of floors expressible at all? Fail-closed, and before anything moves."""
    try:
        floors = floors_of(cfg)
    except ValueError as e:
        return False, f"config/floors.yaml is malformed: {e}"
    seq = [str(f) for f in (itinerary or [])]
    if len(seq) < 2:
        return False, ("an itinerary needs at least two floors -- where the robot is now, and "
                       "where it is going")
    unknown = [f for f in seq if f not in floors]
    if unknown:
        return False, (f"unknown floor(s) {', '.join(sorted(set(unknown)))}; "
                       f"config/floors.yaml defines {', '.join(sorted(floors))}")
    same = [(a, b) for a, b in zip(seq, seq[1:]) if a == b]
    if same:
        return False, (f"itinerary rides from floor {same[0][0]} to itself. There is no such ride, "
                       f"and the swap it would generate is a restart onto the map already loaded")
    return True, ""


# ---------------------------------------------------------------------------------------------
# The plan
# ---------------------------------------------------------------------------------------------
def plan_ride(cfg: dict, itinerary) -> list[Step]:
    """The whole multi-floor route as a flat, reviewable list of steps.

    This is DATA, not an executor. bringup/multifloor_route.sh is a linear bash script for the
    same reason bringup/elevator_route.sh is -- the door proved a linear script works, and a state
    machine cannot be read before it runs. tests/test_floor_plan.py asserts the script and this
    function agree on the order, so the two cannot drift apart silently.

    WHY THE ROBOT RETURNS TO car_facing_out BEFORE THE RIDE. It has just pressed the panel, so it
    is at car_panel, square to the buttons and about 53 degrees off the doors. Turning is the one
    manoeuvre that hurts here (bringup/settle.py: the matcher cannot follow a brisk rotation at
    this scan rate), and after the swap it would be turning on a map it has only been SEEDED into
    and not yet verified against. Do the turn before the ride, on the map the robot is actually
    localized in, and the post-swap move is a straight drive out through open doors.

    WHY THE SWAP IS BEFORE THE RIDE AND NOT AFTER IT. Two reasons, and they point the same way:

      * It is free there. Restarting the node, deserializing a building-sized pose graph and
        waiting for map->odom is tens of seconds; the ride is tens of seconds during which the
        robot must not move anyway. Put the swap after the doors open on the destination floor and
        that cost lands on the door hold instead -- an ADA opener holds for a bounded time, and
        bringup/doors_open.py's header is explicit that the hold expires while you deliberate.
      * There is nothing to wait for. The seed is the robot's pose WITHIN THE CAR, expressed in
        the destination map's frame. The car has not moved relative to the robot and will not, so
        that pose is as true before the ride as after it. Waiting buys no information.

    WHY IT IS AFTER THE DOORS CLOSE, THOUGH. Not fussiness. While the doors stand open on the
    origin floor the scan reaches out into THAT floor's lobby, and the restarted matcher would be
    handed floor-1 geometry to reconcile with a floor-2 seed. Sealed, the scan is only the car --
    which is the one thing both maps agree about, and the reason the seed survives the ride.
    """
    ok, why = check_itinerary(cfg, itinerary)
    if not ok:
        raise ValueError(why)
    floors = floors_of(cfg)
    seq = [str(f) for f in itinerary]

    steps: list[Step] = []
    for here_id, there_id in zip(seq, seq[1:]):
        here, there = floors[here_id], floors[there_id]
        w = here.waypoints
        steps += [
            Step(NAV, w["call_button"], here_id, "outside the lift, facing the call plate"),
            Step(PRESS, here.call_query, here_id, "call the car to this floor"),
            Step(DOORS, "open", here_id, "hold them; an opener holds for a bounded time"),
            # FORWARD ENTRY when the floor defines door_facing + car_facing_in, else the original
            # reverse. Measured 2026-09-05: backing in needs a ~176 deg turn at the doors that took
            # 21.2 s -- longer than an ADA opener holds -- and it aims the OS0's rear near-field
            # artifact (a ring of returns 0.85-1.20 m behind the robot with nothing there) straight
            # into the doorway it is trying to enter. Nose-first costs a 1.1 deg turn and points
            # the artifact away. Every rotation then happens inside the car, doors shut, no clock.
            Step(NAV, w.get("door_facing", w["door_reverse"]), here_id,
                 "line up square with the doorway"),
            Step(NAV, w.get("car_facing_in", w["car_facing_out"]), here_id,
                 "straight in through the doorway"),
            Step(NAV, w["car_panel"], here_id, "square to the button panel"),
            Step(PRESS, there.select_query, here_id, f"select floor {there_id}"),
            Step(NAV, w["car_facing_out"], here_id, "face the doors BEFORE the ride, while still "
                                                    "localized in a map the robot is really in"),
            Step(DOORS, "closed", here_id, "sealed, the scan is only the car -- the one thing "
                                           "both maps agree about"),
            Step(SWAP, there_id, there_id, f"restart localization on '{there.map}', seeded at "
                                           f"'{there.waypoints['car_facing_out']}'. Overlaps the "
                                           f"ride, so it does not spend the door hold"),
            Step(RIDE, there_id, there_id, "nothing in software is true about the FLOOR until the "
                                           "doors open, however good the fit looks"),
            Step(DOORS, "open", there_id, "the floor is not observable until they do"),
            Step(VERIFY, there_id, there_id, "score the live scan against the destination map, "
                                             "DOORS OPEN. This is the gate"),
            Step(NAV, there.waypoints["exit"], there_id, "out"),
        ]
    return steps


def seed_pose(cfg: dict, floor_id: str, waypoints: dict,
              role: str = "car_facing_out") -> tuple[float, float, float]:
    """(x, y, yaw) to hand slam_toolbox as map_start_pose when swapping ONTO ``floor_id``.

    It is the destination floor's waypoint for ``role``, and it is correct by construction ONLY
    when the robot is physically standing in that spot: the robot does not move relative to the car
    during the ride, the car stops in the same place on every floor, and that waypoint was recorded
    at that spot in that car on that floor's map.

    ROLE IS A PARAMETER BECAUSE THE ROBOT DOES NOT ALWAYS RIDE IN car_facing_out. This defaulted to
    car_facing_out unconditionally, which is right only if the ride begins from the pose the doors
    face. On 2026-09-05 the robot rode down parked at car_panel, 0.48 m and 116 deg away from
    car_facing_out -- seeding at car_facing_out would have handed slam_toolbox a start pose the
    robot was demonstrably not at, inside a car whose blank walls give the matcher nothing to
    correct it with. It would have come up ACTIVE and confident and half a metre wrong, and the
    error would only have surfaced as a failed nav leg after the doors opened.

    Pass the role matching WHERE THE ROBOT ACTUALLY IS when the swap runs. The caller knows this
    and the config does not.

    THIS IS A SEED, NOT A SEARCH, and the difference matters. bringup/relocalise.py's global search
    scores the live scan over every free cell of the map -- which inside a closed car is a scan of
    four blank walls, and it would happily anchor on some other doorway-sized gap metres away and
    report a good fit for it. There is no information in that scan to search WITH. The seed carries
    the information the scan does not, and the doors-open check afterwards is what tests it.

    config/slam_os0.yaml documents the same rule for the single-floor case: without map_start_pose
    slam_toolbox logs one ERROR and comes up ACTIVE anyway on a brand-new empty graph.
    """
    floors = floors_of(cfg)
    if floor_id not in floors:
        raise ValueError(f"unknown floor '{floor_id}'")
    name = floors[floor_id].waypoints.get(role)
    if not name:
        have = ", ".join(sorted(floors[floor_id].waypoints)) or "none"
        raise ValueError(f"floor {floor_id} has no waypoint for role '{role}' (has: {have})")
    wp = (waypoints or {}).get(name)
    if wp is None:
        raise ValueError(f"floor {floor_id}: waypoint '{name}' is not recorded, so there is no "
                         f"pose to seed the swap with")
    if (wp.get("frame") or "odom") != "map":
        raise ValueError(f"floor {floor_id}: waypoint '{name}' is not a map-frame pose")
    if wp.get("map_name") != floors[floor_id].map:
        raise ValueError(f"floor {floor_id}: waypoint '{name}' belongs to map "
                         f"'{wp.get('map_name')}', not '{floors[floor_id].map}'")
    return float(wp["x"]), float(wp["y"]), float(wp.get("yaw", 0.0))


# ---------------------------------------------------------------------------------------------
# The gate
# ---------------------------------------------------------------------------------------------
def handover_gate(*, expected_map: str, loaded_map, fit_pct, doors_open,
                  fit_age_s=None, min_fit_pct: float = MIN_FIT_PCT) -> tuple[bool, str]:
    """May the base move now that the swap has happened? Fail-closed on every unknown.

    expected_map  the destination floor's map name, from config/floors.yaml
    loaded_map    what maps/.loaded_map actually certifies right now, or None if nothing is
    fit_pct       scan-vs-map score in percent, or None if it could not be measured
    doors_open    True only if the doors were OBSERVED open when fit_pct was taken. None means
                  nobody looked, which is a refusal, not a maybe.
    fit_age_s     seconds since fit_pct was measured, or None if unknown

    Returns (may_move, message). Every refusal says what to do about it, because this fires with
    the robot standing in a lift and the operator has seconds, not minutes.

    THE DOORS-OPEN CONDITION IS THE POINT OF THIS FUNCTION. Everything else here is bookkeeping
    that other modules already do. A closed car is four walls a metre away on every floor of the
    building, so the fit is uninformative by construction and a high one is actively misleading.
    See this module's header.
    """
    if not loaded_map:
        return False, ("nothing is certified in maps/.loaded_map, so no map-frame waypoint means "
                       "anything right now. The swap did not finish -- re-run bringup/floor_swap.py")
    if loaded_map != expected_map:
        return False, (f"maps/.loaded_map says '{loaded_map}' but this floor is '{expected_map}'. "
                       f"The swap loaded the wrong map, or did not run. DO NOT DRIVE: those "
                       f"origins are unrelated and the exit waypoint names a different place")
    if doors_open is not True:
        return False, (
            "the doors were not observed open when the fit was scored, so the score is not "
            "evidence about which floor this is: a closed car is four walls at about a metre on "
            "EVERY floor, and it matches every floor's map equally well.\n"
            "  Fix: wait for the doors, then score again. Do not raise the threshold -- the "
            "number is not too low, it is about the wrong thing.")
    if fit_pct is None:
        return False, ("the localization fit could not be scored. An unanswerable question is a "
                       "refusal here, not a shrug -- run: python3 bringup/relocalise.py --check")
    if fit_age_s is not None and fit_age_s > MAX_FIT_AGE_S:
        return False, (f"the fit is {fit_age_s:.0f} s old (limit {MAX_FIT_AGE_S:.0f} s). It "
                       f"describes a moment that has passed. Score it again.")
    if fit_pct < min_fit_pct:
        return False, (
            f"localization fit is {fit_pct:.1f}% against '{expected_map}' (need {min_fit_pct:.0f}%). "
            f"With the doors open this is a real statement and it says the robot is not where the "
            f"seed put it -- wrong floor, or the car stopped somewhere the waypoint does not "
            f"describe.\n"
            f"  Do not drive. Check the floor indicator by eye first; if the floor IS right, "
            f"seed by hand (RViz 2D Pose Estimate) or run python3 bringup/relocalise.py")
    if fit_pct < WARN_FIT_PCT:
        return True, (f"fit {fit_pct:.1f}% -- above the floor but below {WARN_FIT_PCT:.0f}%. Fine "
                      f"if someone is standing near the robot; not fine otherwise")
    return True, f"fit {fit_pct:.1f}% against '{expected_map}', doors open"
