#!/usr/bin/env python3
"""Hand localization from one floor's map to another's, while the robot sits in the lift car.

    python3 bringup/floor_swap.py --check              # validate config/floors.yaml, offline
    python3 bringup/floor_swap.py --plan 1 2           # print the whole ride, offline, no ROS
    python3 bringup/floor_swap.py --to 2               # DRY RUN: says what it would restart
    python3 bringup/floor_swap.py --to 2 --go          # restart localization on floor 2's map
    python3 bringup/floor_swap.py --verify 2 --doors-open   # THE GATE. Run it with doors open.

Read safety/floor_plan.py's header before changing anything here. The one-line version: a lift car
is geometrically identical on every floor, so a scan-match fit taken inside a closed car matches
every floor's map equally well and is not evidence of anything. The swap below is a SEED made on
trust; --verify is the only step that checks it, and it is worthless before the doors open.

WHAT THIS DOES, IN ORDER, AND WHY EACH STEP IS THERE
----------------------------------------------------
  1. Validate the whole building config offline. A typo found here costs thirty seconds; the same
     typo found later is found with the robot inside a lift.
  2. DELETE maps/.loaded_map first. From this instant until step 6 nothing is certified, so a swap
     that dies half-way leaves every map-frame waypoint correctly refused rather than leaving
     floor 1's certification standing while the robot is on floor 2. Fail-closed means the
     failure state is the safe one, and the failure state here is "no map is loaded".
  3. Stop the running slam_toolbox. RESTART, not /slam_toolbox/deserialize_map: the node's DDS GID
     is what pose_source.slam_session_id returns and what safety/map_frame.py compares, so a
     restart invalidates every floor-1 waypoint automatically, through machinery that already
     exists and is already tested. Deserializing in place would keep the id and leave them valid.
  4. Relaunch it on the destination map with map_start_pose = that floor's car_facing_out
     waypoint. config/slam_os0.yaml documents what happens without that param: one ERROR line,
     then ACTIVE anyway on a brand-new empty graph rooted at the robot's feet.
  5. Check the published /map against the destination's saved grid. This catches step 4's silent
     failure directly -- measured 2026-09-01, saved atrium is 772x855 and the un-seeded fresh
     graph published 486x585. Dimensions and origin are cheap and they are conclusive.
  6. Rewrite maps/.loaded_map with the destination map name and the NEW slam session id.
  7. Clear both Nav2 costmaps. The global costmap's static layer is holding floor 1's grid, and
     the local costmap holds observations of a lobby that is now several metres below.

Process killing is scoped by the inherited UTP_ROBOT_STACK marker that bringup/env.sh exports, via
preflight.ours() -- never by a node, topic or frame name. A frame-name match killed 22 of the sim
campaign's TF publishers on 2026-08-18.

OUTPUT CONTRACT. The last stdout line is:

    RESULT {"action": "...", "ok": true, "floor": "...", "map": "...", "detail": "..."}

Exit codes: 0 did what was asked, 2 refused before doing anything, 4 could not complete, 1 crashed.

NOT DEMONSTRATED ON HARDWARE. Nothing in this file has been run on the robot, and the elevator has
never been ridden by anything in this repo. Treat every claim here as a design intent with a test
behind it, not as an observation.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

from safety.floor_plan import (  # noqa: E402
    DOORS, NAV, PRESS, RIDE, SWAP, VERIFY,
    IN_CAR_MIN_FIT_PCT, REQUIRED_MAP_EXTS, check_building, check_itinerary, floors_of,
    handover_gate,
    plan_ride, seed_pose,
)

FLOORS_YAML = Path(os.environ.get("UTP_FLOORS") or (REPO / "config" / "floors.yaml"))
MAPS_DIR = Path(os.environ.get("UTP_MAPS_DIR") or (REPO / "maps"))
LOADED_MAP = Path(os.environ.get("UTP_LOADED_MAP") or (MAPS_DIR / ".loaded_map"))
SLAM_PARAMS = REPO / "config" / "slam_os0.yaml"

# bringup/session.sh records every process it starts here and `session.sh down` kills each process
# GROUP in it. The node relaunched below replaces one session.sh started, so its pid has to land in
# the same file or `down` silently leaves a slam_toolbox running -- and the next `session.sh nav`
# then finds a live /map, believes the stack is already up, and certifies the WRONG map as loaded.
# That is the exact trap start_nav() spends thirty lines refusing; do not make it reachable from
# here. The path is session.sh's, hardcoded there too.
SESSION_PIDS = Path("/tmp/utp_session_pids")

# The node's executable name, used ONLY to narrow a candidate list that has already been filtered
# to processes carrying our UTP_ROBOT_STACK marker. Never as the ownership test itself.
SLAM_EXEC = "localization_slam_toolbox_node"
MAPPING_EXEC = "async_slam_toolbox_node"

MAP_WAIT_S = 45.0
# 30 s WAS TOO SHORT AND IT COST A SWAP. Measured 2026-09-05: after deserializing elevator_f2's
# 24 MB pose graph the node was ACTIVE and publishing /map with the correct grid, but map->odom
# did not appear inside 30 s -- so the swap declared failure, left maps/.loaded_map absent, and the
# transform showed up moments later. The fail-closed behaviour was right; the budget was wrong.
# Deserializing a building-sized graph and matching the first scans against it is not a 30 s job.
TF_WAIT_S = 120.0
# The grid must agree with the saved map to within a cell. Anything larger is not rounding, it is
# a different map -- which is the failure this check exists to catch.
GRID_TOL_M = 0.06


def emit(action: str, ok: bool, floor: str, mapname: str, detail: str) -> None:
    print("RESULT " + json.dumps({"action": action, "ok": bool(ok), "floor": floor,
                                  "map": mapname, "detail": detail}, sort_keys=True), flush=True)


def load_yaml(p: Path):
    import yaml
    with p.open() as f:
        return yaml.safe_load(f)


def load_waypoints() -> dict:
    store = Path(os.environ.get("UTP_WAYPOINTS") or (MAPS_DIR / "waypoints.yaml"))
    if not store.exists():
        return {}
    return load_yaml(store) or {}


def maps_present() -> dict:
    """map name -> set of extensions on disk. What check_building() needs to judge a config."""
    out: dict[str, set] = {}
    for ext in REQUIRED_MAP_EXTS:
        for p in MAPS_DIR.glob(f"*.{ext}"):
            out.setdefault(p.name[: -(len(ext) + 1)], set()).add(ext)
    return out


def pgm_header(pgm: Path) -> tuple[int, int]:
    """(width, height) from a binary P5 header, comments and arbitrary whitespace included."""
    d = pgm.read_bytes()
    tok, i = [], 2
    while len(tok) < 3:
        while i < len(d) and d[i:i + 1].isspace():
            i += 1
        if d[i:i + 1] == b"#":
            while i < len(d) and d[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(d) and not d[j:j + 1].isspace():
            j += 1
        tok.append(d[i:j])
        i = j
    return int(tok[0]), int(tok[1])


def saved_grid(mapname: str) -> tuple[int, int, float, float, float]:
    """(width, height, origin_x, origin_y, resolution) of the SAVED map, read off disk."""
    y = load_yaml(MAPS_DIR / f"{mapname}.yaml")
    res = float(y["resolution"])
    ox, oy = float(y["origin"][0]), float(y["origin"][1])
    w, h = pgm_header(MAPS_DIR / f"{mapname}.pgm")
    return w, h, ox, oy, res


# ---------------------------------------------------------------------------------------------
# Offline modes -- no ROS, runnable with the robot switched off
# ---------------------------------------------------------------------------------------------
def do_check() -> int:
    cfg = load_yaml(FLOORS_YAML)
    ok, why = check_building(cfg, load_waypoints(), maps_present())
    if ok:
        floors = floors_of(cfg)
        print(f"config/floors.yaml: {len(floors)} floors, all drivable as recorded")
        for fid, fl in sorted(floors.items()):
            print(f"  floor {fid}: map '{fl.map}', "
                  f"waypoints {', '.join(fl.waypoints[r] for r in sorted(fl.waypoints))}")
        emit("check", True, "", "", f"{len(floors)} floors validated")
        return 0
    print("config/floors.yaml is NOT drivable as it stands:\n" + why, file=sys.stderr)
    print("\n  This is the expected state until each floor's map is recorded and saved.\n"
          "  Procedure: docs/MULTIFLOOR.md", file=sys.stderr)
    emit("check", False, "", "", "building config not drivable")
    return 2


def do_plan(itinerary: list[str]) -> int:
    cfg = load_yaml(FLOORS_YAML)
    ok, why = check_itinerary(cfg, itinerary)
    if not ok:
        print(f"itinerary refused: {why}", file=sys.stderr)
        emit("plan", False, "", "", why)
        return 2
    floors = floors_of(cfg)
    steps = plan_ride(cfg, itinerary)
    print(f"ride {' -> '.join(str(f) for f in itinerary)}   ({len(steps)} steps)\n")
    for i, s in enumerate(steps, 1):
        mp = floors[s.floor].map if s.floor in floors else "?"
        print(f"  {i:2d}. {s.kind:<6} {s.arg:<34} [floor {s.floor}, map {mp}]")
        if s.note:
            print(f"      {s.note}")
    emit("plan", True, "", "", f"{len(steps)} steps")
    return 0


# ---------------------------------------------------------------------------------------------
# ROS helpers. Imported lazily so --check and --plan work with no ROS on PATH.
# ---------------------------------------------------------------------------------------------
def _run(cmd: list[str], timeout: float = 20.0) -> tuple[int, str]:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        return 1, str(e)


def slam_pids() -> list[tuple[str, str]]:
    """[(pid, cmdline)] for slam_toolbox nodes THIS repo started. Ownership first, name second."""
    from preflight import _procs, ours
    hits = []
    for pid, cmd, _dom, stack in _procs():
        if not ours(cmd, stack):
            continue
        if SLAM_EXEC in cmd or MAPPING_EXEC in cmd:
            hits.append((pid, cmd))
    return hits


def stop_slam(verbose: bool = True) -> int:
    """Stop our slam_toolbox, politely then firmly. Returns how many were stopped."""
    import signal
    pids = slam_pids()
    for pid, cmd in pids:
        if verbose:
            print(f"  stopping slam_toolbox pid {pid}: {cmd[:90]}")
        try:
            os.kill(int(pid), signal.SIGINT)
        except (ProcessLookupError, PermissionError, ValueError):
            pass
    for _ in range(40):
        if not slam_pids():
            return len(pids)
        time.sleep(0.25)
    for pid, _cmd in slam_pids():
        try:
            os.kill(int(pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError, ValueError):
            pass
    time.sleep(1.0)
    return len(pids)


def wait_for_map(timeout: float):
    """Return the latched /map's OccupancyGrid.info, or None. Spins its own short-lived node."""
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    q = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                   reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    started = not rclpy.ok()
    if started:
        rclpy.init()
    n = Node("utp_floor_swap_map")
    got = {}
    n.create_subscription(OccupancyGrid, "/map", lambda m: got.setdefault("m", m), q)
    end = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < end and "m" not in got:
            rclpy.spin_once(n, timeout_sec=0.2)
        return got["m"].info if "m" in got else None
    finally:
        n.destroy_node()
        if started:
            rclpy.shutdown()


def slam_session() -> str | None:
    import rclpy
    from rclpy.node import Node
    from pose_source import slam_session_id
    started = not rclpy.ok()
    if started:
        rclpy.init()
    n = Node("utp_floor_swap_probe")
    try:
        end = time.monotonic() + 4.0
        sid = None
        while time.monotonic() < end and sid is None:
            rclpy.spin_once(n, timeout_sec=0.1)
            sid = slam_session_id(n)
        return sid
    finally:
        n.destroy_node()
        if started:
            rclpy.shutdown()


def have_map_to_odom(timeout: float) -> bool:
    """Poll for map->odom until `timeout`. RETRY, do not take one look.

    tf2_echo is capped at ~25 s per invocation because it never exits on its own; the retry loop
    is what actually spends the budget. A single 25 s look is what failed the 2026-09-05 swap.
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rc, out = _run(["ros2", "run", "tf2_ros", "tf2_echo", "map", "odom"], timeout=20.0)
        if "Translation:" in out:
            return True
        print("  ... still waiting for map->odom", flush=True)
    return False


def measure_fit() -> float | None:
    """Score the live scan against the loaded map, by asking the tool that already does it.

    bringup/relocalise.py --check prints `current (x,y,deg) fit NN.N%` and changes nothing. Parsing
    its output is what bringup/elevator_route.sh already does; a second implementation of the same
    scoring would be a second thing that can disagree.
    """
    rc, out = _run(["python3", str(REPO / "bringup" / "relocalise.py"), "--check"], timeout=90.0)
    m = re.findall(r"fit ([0-9.]+)%", out)
    return float(m[-1]) if m else None


def clear_costmaps() -> str:
    done = []
    for srv, typ in (("/global_costmap/clear_entirely_global_costmap",
                      "nav2_msgs/srv/ClearEntireCostmap"),
                     ("/local_costmap/clear_entirely_local_costmap",
                      "nav2_msgs/srv/ClearEntireCostmap")):
        rc, _out = _run(["ros2", "service", "call", srv, typ, "{}"], timeout=25.0)
        done.append(f"{srv.split('/')[1]}{'' if rc == 0 else ' (FAILED)'}")
    return ", ".join(done)


# ---------------------------------------------------------------------------------------------
# The swap
# ---------------------------------------------------------------------------------------------
def do_swap(floor_id: str, go: bool, seed_role: str = "car_facing_out") -> int:
    cfg = load_yaml(FLOORS_YAML)
    wps = load_waypoints()
    ok, why = check_building(cfg, wps, maps_present())
    if not ok:
        print("REFUSED -- the building config is not drivable:\n" + why, file=sys.stderr)
        emit("swap", False, floor_id, "", "building config not drivable")
        return 2
    floors = floors_of(cfg)
    if floor_id not in floors:
        print(f"unknown floor '{floor_id}'; known: {', '.join(sorted(floors))}", file=sys.stderr)
        emit("swap", False, floor_id, "", "unknown floor")
        return 2
    dest = floors[floor_id]
    try:
        sx, sy, syaw = seed_pose(cfg, floor_id, wps, role=seed_role)
    except ValueError as e:
        print(f"REFUSED -- no seed pose: {e}", file=sys.stderr)
        emit("swap", False, floor_id, dest.map, "no seed pose")
        return 2

    stem = MAPS_DIR / dest.map
    print(f"floor {floor_id}  map '{dest.map}'")
    print(f"  seed  x={sx:+.4f} y={sy:+.4f} yaw={math.degrees(syaw):+.1f} deg "
          f"(waypoint '{dest.waypoints[seed_role]}', role {seed_role})")
    print(f"  this is a SEED, not a search: inside a closed car the scan is four blank walls and")
    print(f"  carries no information to search with. --verify, with the doors open, is the check.")

    if not go:
        print("\nDRY RUN. Would:")
        for pid, cmd in slam_pids():
            print(f"  stop pid {pid}  {cmd[:80]}")
        print(f"  rm {LOADED_MAP}")
        print(f"  ros2 run slam_toolbox {SLAM_EXEC} --ros-args --params-file {SLAM_PARAMS} "
              f"-p mode:=localization -p map_file_name:={stem} "
              f"-p map_start_pose:=[{sx:.4f},{sy:.4f},{syaw:.4f}]")
        print("  then verify the published grid, rewrite .loaded_map, clear the costmaps.")
        print("\nAdd --go to do it.")
        return 0

    # (2) NOTHING IS CERTIFIED FROM HERE. Do this before stopping the node, not after: between the
    # two there is a window in which floor 1's certification would still be standing over a node
    # that is going away, and a crash in that window is exactly when it matters.
    try:
        LOADED_MAP.unlink()
        print(f"  maps/.loaded_map removed -- no map is certified until this finishes")
    except FileNotFoundError:
        print(f"  maps/.loaded_map was already absent")

    # (3)
    n = stop_slam()
    print(f"  {n} slam_toolbox process(es) stopped")

    # (4)
    env = dict(os.environ)
    cmd = ["ros2", "run", "slam_toolbox", SLAM_EXEC, "--ros-args",
           "--params-file", str(SLAM_PARAMS),
           "-p", "use_sim_time:=false",
           "-p", "mode:=localization",
           "-p", f"map_file_name:={stem}",
           "-p", f"map_start_pose:=[{sx:.4f},{sy:.4f},{syaw:.4f}]"]
    print("  " + " ".join(cmd))
    # start_new_session=True is setsid, matching session.sh's bg(): the child leads its own process
    # group so `kill -- -PID` later takes the whole tree, not just the launcher.
    child = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True)
    if SESSION_PIDS.exists():
        with SESSION_PIDS.open("a") as f:
            f.write(f"{child.pid}\n")
        print(f"  pid {child.pid} recorded in {SESSION_PIDS} so `session.sh down` still stops it")
    time.sleep(5.0)
    _run(["ros2", "lifecycle", "set", "/slam_toolbox", "configure"], timeout=25.0)
    _run(["ros2", "lifecycle", "set", "/slam_toolbox", "activate"], timeout=25.0)

    info = wait_for_map(MAP_WAIT_S)
    if info is None:
        print(f"no /map within {MAP_WAIT_S:.0f} s -- slam_toolbox did not activate. "
              f"Check: ros2 lifecycle get /slam_toolbox", file=sys.stderr)
        emit("swap", False, floor_id, dest.map, "no /map after relaunch")
        return 4

    # (5) THE PUBLISHED GRID MUST BE THE SAVED ONE. This is the documented silent failure: given a
    # map it cannot deserialize, or no start pose, slam_toolbox comes up ACTIVE on a brand-new
    # empty graph at the robot's feet and nothing downstream can tell. Dimensions and origin are
    # conclusive and cost nothing.
    try:
        sw, sh, sox, soy, sres = saved_grid(dest.map)
    except Exception as e:  # noqa: BLE001 -- a malformed saved map is a refusal, whatever broke
        print(f"could not read the saved grid maps/{dest.map}.(yaml|pgm): {e}", file=sys.stderr)
        emit("swap", False, floor_id, dest.map, "saved grid unreadable")
        return 4
    lw, lh = info.width, info.height
    lox, loy = info.origin.position.x, info.origin.position.y
    same = (lw == sw and lh == sh
            and abs(lox - sox) <= GRID_TOL_M and abs(loy - soy) <= GRID_TOL_M)
    print(f"  saved grid  {sw}x{sh} at origin ({sox:+.3f}, {soy:+.3f})")
    print(f"  live  /map  {lw}x{lh} at origin ({lox:+.3f}, {loy:+.3f})")
    if not same:
        print("\nTHE SAVED MAP WAS NOT LOADED. slam_toolbox is ACTIVE and publishing, but on a\n"
              "NEW graph rooted at the robot's feet -- not on maps/%s. Every waypoint on this\n"
              "floor is now meaningless and nothing else would have told you.\n"
              "  Usual cause: maps/%s.posegraph or .data is missing or unreadable.\n"
              "  maps/.loaded_map has been left absent, so nothing certifies this."
              % (dest.map, dest.map), file=sys.stderr)
        emit("swap", False, floor_id, dest.map, "published grid is not the saved grid")
        return 4

    if not have_map_to_odom(TF_WAIT_S):
        print("no TF map->odom -- slam_toolbox has not localized. The chassis must be running "
              "too, not just the lidar.", file=sys.stderr)
        emit("swap", False, floor_id, dest.map, "no map->odom after relaunch")
        return 4

    # (6)
    sess = slam_session()
    if not sess:
        print("cannot identify the new slam session (is exactly one node publishing /map?). "
              "maps/.loaded_map left absent, so every map-frame waypoint stays refused.",
              file=sys.stderr)
        emit("swap", False, floor_id, dest.map, "no slam session id")
        return 4
    LOADED_MAP.write_text(f"{dest.map} {sess}\n")
    print(f"  maps/.loaded_map -> {dest.map} [slam {sess[:8]}]")

    # (7)
    print(f"  costmaps cleared: {clear_costmaps()}")

    print("\n  SWAPPED, NOT VERIFIED. The robot is seeded, not localized: the scan it can see is\n"
          "  the inside of a car, which matches every floor. When the doors open, run:\n"
          f"      python3 bringup/floor_swap.py --verify {floor_id} --doors-open")
    emit("swap", True, floor_id, dest.map, f"seeded at ({sx:.3f},{sy:.3f},{syaw:.3f})")
    return 0


# ---------------------------------------------------------------------------------------------
# The gate
# ---------------------------------------------------------------------------------------------
def do_verify(floor_id: str, doors_open: bool, check_doors: bool) -> int:
    cfg = load_yaml(FLOORS_YAML)
    floors = floors_of(cfg)
    if floor_id not in floors:
        print(f"unknown floor '{floor_id}'", file=sys.stderr)
        emit("verify", False, floor_id, "", "unknown floor")
        return 2
    dest = floors[floor_id]

    evidence = "operator" if doors_open else "none"
    if check_doors:
        rc, out = _run(["python3", str(REPO / "bringup" / "doors_open.py"), "--timeout", "20"],
                       timeout=180.0)
        sys.stdout.write(out)
        # doors_open.py: 0 OPEN, 1 SHUT, 2 COULD NOT TELL -- and 2 is documented as "treat as
        # shut". A could-not-tell is exactly the unanswerable question this gate refuses on.
        doors_open = (rc == 0)
        evidence = f"doors_open.py (exit {rc})"

    try:
        live = LOADED_MAP.read_text().split()[0]
    except (OSError, IndexError):
        live = None

    t0 = time.monotonic()
    fit = measure_fit()
    age = time.monotonic() - t0

    # IN_CAR_MIN_FIT_PCT, not MIN_FIT_PCT: this is scored from inside the car with the doors
    # open, where the fit is structurally depressed by beams leaving through the doorway. See the
    # constant's comment -- it is calibrated to pass a correct arrival, NOT to reject a wrong floor.
    ok, msg = handover_gate(expected_map=dest.map, loaded_map=live, fit_pct=fit,
                            doors_open=doors_open if (doors_open or check_doors) else None,
                            fit_age_s=age, min_fit_pct=IN_CAR_MIN_FIT_PCT)
    print(f"  floor {floor_id}  map '{dest.map}'  loaded '{live}'  "
          f"fit {'n/a' if fit is None else f'{fit:.1f}%'}  doors {evidence}")
    if ok:
        print(f"  GATE PASSED: {msg}")
        emit("verify", True, floor_id, dest.map, msg)
        return 0
    print("\nGATE REFUSED. The base must not move.\n  " + msg.replace("\n", "\n  "),
          file=sys.stderr)
    emit("verify", False, floor_id, dest.map, msg.splitlines()[0])
    return 2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true", help="validate config/floors.yaml, offline")
    g.add_argument("--plan", nargs="+", metavar="FLOOR", help="print the ride for an itinerary")
    g.add_argument("--to", metavar="FLOOR", help="swap localization onto this floor's map")
    g.add_argument("--verify", metavar="FLOOR", help="the gate: may the base move on this floor?")
    ap.add_argument("--go", action="store_true", help="with --to: actually do it")
    # WHERE THE ROBOT IS RIDING. The seed is only correct if it names the spot the robot actually
    # occupies; see safety/floor_plan.seed_pose. Default is the pose facing the doors, which is
    # where a robot that drove in and stopped will be -- but a robot that then moved to the panel
    # to press a floor button rides down at car_panel, and must be seeded there.
    ap.add_argument("--seed-role", default="car_facing_out",
                    choices=("car_facing_out", "car_panel", "car_facing_in"),
                    help="role naming where the robot physically is (default: car_facing_out)")
    ap.add_argument("--doors-open", action="store_true",
                    help="with --verify: the OPERATOR states the doors are open")
    ap.add_argument("--check-doors", action="store_true",
                    help="with --verify: ask bringup/doors_open.py instead of taking a word for it")
    a = ap.parse_args()

    if a.check:
        return do_check()
    if a.plan:
        return do_plan([str(f) for f in a.plan])
    if a.to:
        return do_swap(str(a.to), a.go, seed_role=a.seed_role)
    return do_verify(str(a.verify), a.doors_open, a.check_doors)


if __name__ == "__main__":
    raise SystemExit(main())
