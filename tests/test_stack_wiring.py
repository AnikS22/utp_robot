"""The WIRING between files, read as one system. Every component here is individually correct.

WHY THIS FILE EXISTS. On 2026-09-01 a reviewer observed that ~290 tests pass while the stack is
broken on hardware, because every failure that has actually cost time was a CROSS-FILE RUNTIME
INTEGRATION ERROR: each file is self-consistent and consistent with its own tests, and the joint
between two files is wrong. Three that cost hours, all with NO error message anywhere:

  1. Both Nav2 costmaps subscribed to /scan_filtered while the self-occlusion mask published
     /scan. pointcloud_to_laserscan and the costmaps are both BEST_EFFORT, so the WRONG data
     arrived perfectly, at full rate. Nav2 marked the robot's own arm and mast LETHAL around its
     own footprint and refused to plan. (Fixed; tests/test_nav2_scan_source.py pins the topic.)
  2. pointcloud_to_laserscan publishes BEST_EFFORT; slam_toolbox subscribes RELIABLE. Incompatible
     DDS QoS delivers ZERO messages, silently. bringup/scan_relay.py exists only to bridge that,
     and re-pointing any consumer at the wrong side of it kills the chain again with no symptom.
  3. Every waypoint in maps/waypoints.yaml carries map_name: atrium while bringup/session.sh
     defaulted MAP_NAME to atrium2d. Nothing compared them, so a well-formed coordinate could be
     driven into a different map's origin -- a confident arrival at the wrong physical place.

None of those is visible from inside any one file. So this suite does not check files; it builds
an INVENTORY by parsing the configs and scripts, and asserts the graph they jointly describe is
the graph the design intends.

TWO DISCIPLINES, both learned the hard way and both enforced below:

  * NON-VACUITY. Nav2 nests costmaps twice (global_costmap.global_costmap.ros__parameters). A
    fixed-depth walk finds nothing and the assertion built on it PASSES. Every inventory helper
    here recurses, and every test asserts its inventory is NON-EMPTY before asserting anything
    about its contents. An empty inventory is a failure, never a skip.
  * NO ROS. Everything here is static: read files, parse, compare. It must run with the robot on
    charge, on a laptop with nothing sourced.
"""
from __future__ import annotations

import ast
import math
import re
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]

SESSION_SH = REPO / "bringup" / "session.sh"
SAFETY_YAML = REPO / "config" / "safety.yaml"
SLAM_YAML = REPO / "config" / "slam_os0.yaml"
OUSTER_YAML = REPO / "config" / "ouster.yaml"
OUSTER_DRIVER_YAML = REPO / "config" / "ouster_driver.yaml"
NAV_LAUNCH = REPO / "nav2_bringup" / "ranger_nav.launch.py"

# nav2_params_os0_map.yaml is THE ONE THAT RUNS -- session.sh rewrites it into /tmp and launches
# that. The glob is deliberately wider than that one file: ranger_nav.launch.py has its OWN
# default params_file, so any nav2_params*.yaml left in nav2_bringup/ is reachable by
# `ros2 launch <repo>/nav2_bringup/ranger_nav.launch.py` with no params_file, and is therefore
# something a tired operator can start by accident. Archived copies are out of scope.
RUNNING_PARAMS = REPO / "nav2_bringup" / "nav2_params_os0_map.yaml"
ALL_PARAMS = sorted((REPO / "nav2_bringup").glob("nav2_params*.yaml"))

# Topic names are the joints this file is about; spell them once.
T_POINTS = "/ouster/points"
T_RAW_SCAN = "/scan_filtered"       # pointcloud_to_laserscan output. CONTAINS THE ROBOT.
T_SCAN = "/scan"                    # scan_relay.py output: masked AND reliable.
T_CMD_VEL = "/cmd_vel"


# ============================================================================== parsing helpers
def _text(p: Path) -> str:
    return p.read_text(errors="ignore")


def _yaml(p: Path) -> dict:
    return yaml.safe_load(_text(p)) or {}


def _p2l_block() -> str:
    """Just the pointcloud_to_laserscan invocation out of session.sh.

    Parsing the WHOLE script for `range_min:=` would also match the prose that discusses it, and
    the prose has been wrong before (see test_slam_config_quotes_the_range_min_that_runs). Cut the
    block first, then read flags out of it."""
    src = _text(SESSION_SH)
    i = src.index("pointcloud_to_laserscan_node")
    j = src.index("waitfor 20 " + T_RAW_SCAN, i)
    return src[i:j]


def _p2l_flag(name: str) -> str:
    """The value of a single `name:=value` argument to pointcloud_to_laserscan.

    Reads the one flag rather than matching a whole line, so re-wrapping the continuation lines in
    session.sh cannot break this test."""
    block = _p2l_block()
    m = re.search(rf"(?<![\w.]){re.escape(name)}:=(\S+)", block)
    assert m, f"session.sh no longer passes {name}:= to pointcloud_to_laserscan"
    return m.group(1)


def _sh_default(var: str) -> str:
    """`VAR=${VAR:-default}` out of session.sh -- the value used when the operator sets nothing."""
    m = re.search(rf"^{re.escape(var)}=\$\{{{re.escape(var)}:-([^}}]*)\}}",
                  _text(SESSION_SH), re.M)
    assert m, f"session.sh no longer defines a default for {var}"
    return m.group(1)


def _walk(node, path=()):
    """(dotted path, dict) for EVERY dict in a params document, at any depth.

    THE VACUITY GUARD. Nav2 params nest the thing you care about two or three levels below where
    a reader expects it -- global_costmap.global_costmap.ros__parameters.obstacle_layer.scan --
    and a hand-written path that misses by one level yields None, which then satisfies whatever
    assertion was built on it. Recurse, collect everything, and let the tests assert non-empty."""
    if isinstance(node, dict):
        yield ".".join(path), node
        for k, v in node.items():
            yield from _walk(v, path + (str(k),))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from _walk(v, path + (f"[{i}]",))


def _params_key(doc, key):
    """(where, value) for every occurrence of `key` anywhere in a params document."""
    out = []
    for where, d in _walk(doc):
        if key in d and not isinstance(d[key], (dict, list)):
            out.append((f"{where}.{key}" if where else key, d[key]))
    return out


def _costmap_observation_sources(doc):
    """(where, topic, sensor_frame) for every costmap observation source, at any depth."""
    found = []
    for where, d in _walk(doc):
        ob = d.get("obstacle_layer")
        if not isinstance(ob, dict):
            continue
        for src in str(ob.get("observation_sources", "")).split():
            s = ob.get(src)
            if isinstance(s, dict):
                found.append((f"{where}.obstacle_layer.{src}".lstrip("."),
                              s.get("topic"), s.get("sensor_frame")))
    return found


# ============================================================================== 0. NON-VACUITY
def test_the_inventories_this_suite_reasons_over_are_not_empty():
    """THE FIRST TEST, because every other test in this file is worthless without it.

    A parametrized test over an empty glob COLLECTS NOTHING and reports green. A recursive walk
    over a renamed schema returns [] and every `for x in inventory: assert ...` passes. That exact
    mistake was made on 2026-09-01. If the layout moves, this is the test that says so."""
    assert ALL_PARAMS, (
        "no nav2_params*.yaml in nav2_bringup/ — every params test in this file would collect "
        "zero cases and report green")
    assert RUNNING_PARAMS in ALL_PARAMS, \
        f"{RUNNING_PARAMS.name} is gone; session.sh launches it and this suite pins it"
    for p in (SESSION_SH, SAFETY_YAML, SLAM_YAML, OUSTER_YAML, OUSTER_DRIVER_YAML, NAV_LAUNCH):
        assert p.is_file(), f"{p.relative_to(REPO)} is missing; the wiring cannot be read"
    assert _costmap_observation_sources(_yaml(RUNNING_PARAMS)), \
        "the recursive costmap walk found nothing — the params schema changed"
    assert _twist_producers(), "the base-command producer inventory is empty"
    assert _frames_this_robot_has(), "the TF frame inventory is empty"


# ============================================================================== A. TOPIC GRAPH
def test_the_scan_chain_is_wired_end_to_end_in_the_files_that_launch_it():
    """/ouster/points -> pointcloud_to_laserscan -> /scan_filtered -> scan_relay -> /scan.

    Every hop is declared in a DIFFERENT file, and a mismatch at any hop is silent: DDS does not
    complain about a subscriber with no publisher, it just delivers nothing forever. Incident 2
    above is exactly this chain half-connected -- p2l alive at 9.2 Hz, slam_toolbox silent from
    the moment it started, no error anywhere."""
    block = _p2l_block()
    assert f"cloud_in:={T_POINTS}" in block, \
        f"pointcloud_to_laserscan is no longer fed {T_POINTS} (bringup/lidar3d.sh's output)"
    assert f"scan:={T_RAW_SCAN}" in block, \
        f"pointcloud_to_laserscan no longer publishes {T_RAW_SCAN}"

    relay = _text(REPO / "bringup" / "scan_relay.py")
    assert f'IN_TOPIC = "{T_RAW_SCAN}"' in relay and f'OUT_TOPIC = "{T_SCAN}"' in relay, \
        "scan_relay.py no longer bridges the exact topics session.sh sets up"

    src = _text(SESSION_SH)
    assert "bringup/scan_relay.py" in src, (
        "session.sh must LAUNCH the relay. p2l publishes BEST_EFFORT and slam_toolbox subscribes "
        "RELIABLE -- incompatible QoS delivers zero messages with no error anywhere.")


def test_slam_and_both_nav2_costmaps_consume_the_relay_output_not_the_raw_projection():
    """The joint from incident 1, checked from BOTH ends at once.

    tests/test_nav2_scan_source.py pins the costmap side; this pins that slam_toolbox and the
    costmaps agree with each other AND with what scan_relay.py actually publishes, so renaming the
    relay's output topic breaks a test instead of splitting the chain in two."""
    relay_out = re.search(r'OUT_TOPIC = "([^"]+)"',
                          _text(REPO / "bringup" / "scan_relay.py")).group(1)

    slam_topic = _yaml(SLAM_YAML)["slam_toolbox"]["ros__parameters"]["scan_topic"]
    assert slam_topic == relay_out, (
        f"config/slam_os0.yaml consumes {slam_topic} but the relay publishes {relay_out}. "
        f"{T_RAW_SCAN} is BEST_EFFORT and slam_toolbox subscribes RELIABLE: zero messages, "
        f"no error, /map never appears and the node looks merely hung.")

    sources = _costmap_observation_sources(_yaml(RUNNING_PARAMS))
    assert len(sources) >= 2, (
        f"{RUNNING_PARAMS.name}: found {len(sources)} costmap observation sources, expected both "
        f"the global and the local costmap. A walk that finds nothing makes every assertion "
        f"below it pass vacuously.")
    for where, topic, _frame in sources:
        assert topic == relay_out, (
            f"{RUNNING_PARAMS.name}: {where} consumes {topic}, the relay publishes {relay_out}. "
            f"{T_RAW_SCAN} is the RAW projection and contains the robot's own arm and mast at "
            f"0.70-0.85 m -- lethal cells wrapped around the footprint.")


def _raw_scan_consumers():
    """Every live file that CONSUMES /scan_filtered, with why we think it is a consumer.

    A consumer is a subscription (create_subscription(LaserScan, "/scan_filtered", ...)) or a
    consuming default (an env-var/argparse default naming it). Deliberately NOT counted:
      * bringup/scan_relay.py -- it is the masker; consuming the raw topic is its whole job.
      * bringup/session.sh, bringup/lab_gates.sh -- they LAUNCH and liveness-probe the topic.
      * .rviz files -- displaying the raw projection is a legitimate diagnostic view."""
    allowed = {"scan_relay.py", "session.sh", "lab_gates.sh", "_ros_env.py"}
    out = []
    for d in ("bringup", "safety"):
        for p in sorted((REPO / d).rglob("*.py")):
            if "__pycache__" in str(p) or p.name in allowed:
                continue
            src = _text(p)
            for m in re.finditer(r"create_subscription\(\s*LaserScan\s*,\s*([^\n]*)", src):
                arg = m.group(1)
                if T_RAW_SCAN in arg:
                    line = src[:m.start()].count("\n") + 1
                    kind = "env default" if "environ" in arg else "subscription"
                    out.append((f"{p.relative_to(REPO)}:{line}", kind))
    return out


def test_nothing_downstream_of_the_relay_consumes_the_raw_scan():
    """/scan_filtered CONTAINS THE ROBOT and the chain must narrow to /scan at scan_relay.py.

    MEASURED 2026-09-01, 10 scans, stationary, open floor, nobody near: the stowed arm, the mast
    and the chassis rear return at a fixed 0.39-0.85 m across |bearing| 74-180 deg, while the
    forward hemisphere reads 2.87-8.8 m. scan_relay.py masks exactly that arc; /scan is the
    result. Any consumer left on /scan_filtered is looking at the robot and cannot tell -- which
    is incident 1's shape, one layer down from Nav2, in the odom half of the pipeline that was
    not audited when the costmaps were fixed.

    BE PRECISE ABOUT WHICH OF THESE HURTS TODAY, because the fix order depends on it:

      * waypoints.py `anchor` (line ~105, via _collect_scans) is the one that is WRONG NOW, not
        merely fragile. It saves twenty FULL 360-degree sweeps as the relocalization reference and
        safety/scan_anchor.py matches a live sweep against them. The self-returns are fixed in
        base_link, so they appear at identical coordinates in BOTH clouds at EVERY candidate
        transform: they match perfectly wherever the robot is, pull the mean truncated residual
        down toward zero and flatten the margin between the true transform and the runner-up.
        MAX_RESIDUAL_M (0.08) and MIN_MARGIN (0.15) are the gates that refuse a weak match, and
        this pads both with geometry that carries no information about the room. It also inflates
        the point count the module's own docstring reasons from ("~50 usable points a scan").
        Same file, same reasoning as config/ouster.yaml: a return that does not move between
        scans is the robot, not the room.
      * grab_frame.py records scan.json into captures/. Those files are read back as evidence --
        the 0.72 m door analysis that set range_min came from captures/trial_ours_001/scan.json.
        A capture on the raw topic is not the scan any consumer acts on, so the measurement and
        the system disagree by exactly the mask.
      * approach_blockage.py, twopoint.py, face_target.py, and waypoints.py's corridor veto are
        LUCKY, not correct. corridor_blocked tests a 0.90 x +-0.40 m rectangle and nearest_ahead a
        +-15 deg cone; at |bearing| 74 deg a return needs r <= 0.42 m to enter that rectangle and
        the self-returns start at 0.70 m, so today they miss. That margin is an accident of two
        independently tuned numbers. Widen a veto window, or re-measure MASK_MIN_DEG below 74 (it
        has already moved once, from 88), and the robot walks back into its own obstacle check
        with nothing to announce it.

    /scan is a drop-in: scan_relay.py republishes the same message with the same bin count,
    angle_min and angle_increment, masked bins set to +inf -- which every consumer already handles
    on the ~84 empty bins in a typical scan. The fix is the topic name in each file."""
    consumers = _raw_scan_consumers()
    assert not consumers, (
        f"{len(consumers)} live consumers still read the RAW, robot-containing {T_RAW_SCAN} "
        f"instead of the masked {T_SCAN}:\n  "
        + "\n  ".join(f"{w}  ({k})" for w, k in consumers)
        + f"\n\nwaypoints.py's anchor sweeps and grab_frame.py's captures are wrong now; the "
          f"corridor vetoes are one tuning change away. Fix: point them at {T_SCAN} (same bin "
          f"count, same angles, self-returns as +inf). See this test's docstring.")


# ------------------------------------------------------------------ /cmd_vel: one publisher only
def _mux_sources():
    """{name: topic} for every arbitrated command source declared in config/safety.yaml."""
    cfg = _yaml(SAFETY_YAML)
    srcs = {s["name"]: s["topic"] for s in cfg["sources"]}
    assert srcs, "config/safety.yaml declares no sources — did the schema change?"
    return srcs


def _twist_producers():
    """{topic: [where...]} for everything in this repo that PUBLISHES a base command.

    Two mechanisms, because both exist here and both drive the wheels:
      * create_publisher(Twist, ...) in a live .py -- resolved by taking the /cmd_vel* string
        literals in the same file, since the topic reaches the call as a module constant
        (CMD_TOPIC), an argparse default (teleop_keyboard.py) or a config lookup (twist_mux_node).
      * `ros2 topic pub <topic> geometry_msgs/msg/Twist` from a script.
      * launch-file remappings ("/cmd_vel" -> "/cmd_vel_nav") in ranger_nav.launch.py.
    """
    out: dict[str, list[str]] = {}

    def add(topic, where):
        out.setdefault(topic, []).append(where)

    for d in ("bringup", "safety"):
        for p in sorted((REPO / d).rglob("*.py")):
            if "__pycache__" in str(p):
                continue
            src = _text(p)
            publishes_twist = re.search(r"create_publisher\(\s*Twist\b", src) is not None
            pubs_by_cli = re.search(r'"ros2",\s*"topic",\s*"pub"', src) is not None
            if not (publishes_twist or pubs_by_cli):
                continue
            for topic in sorted(set(re.findall(r'"(/cmd_vel[a-z_]*)"', src))):
                add(topic, str(p.relative_to(REPO)))

    # Launch remappings: ("/cmd_vel", "/cmd_vel_nav") on a named Node().
    tree = ast.parse(_text(NAV_LAUNCH))
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        if not (isinstance(call.func, ast.Name) and call.func.id == "Node"):
            continue
        kw = {k.arg: k.value for k in call.keywords}
        name = kw.get("name")
        node_name = name.value if isinstance(name, ast.Constant) else "<node>"
        rem = kw.get("remappings")
        if not isinstance(rem, ast.List):
            continue
        for elt in rem.elts:
            if isinstance(elt, ast.Tuple) and len(elt.elts) == 2 and \
                    all(isinstance(e, ast.Constant) for e in elt.elts):
                frm, to = elt.elts[0].value, elt.elts[1].value
                if frm == T_CMD_VEL:
                    add(to, f"{NAV_LAUNCH.name}:{node_name} (remap {frm} -> {to})")
    return out


def test_cmd_vel_has_exactly_one_publisher_by_design():
    """config/safety.yaml's founding rule: the mux is the ONLY publisher of /cmd_vel.

    Anything else on that topic silently bypasses the E-stop, the arm interlock, the speed
    ceilings and the slew limiter. `ros2 topic info /cmd_vel -v` on 2026-08-20 read publisher
    count 3, all behavior_server -- which is why ranger_nav.launch.py remaps the BEHAVIOR server
    as well as the controller."""
    out_topic = _yaml(SAFETY_YAML)["output_topic"]
    assert out_topic == T_CMD_VEL

    producers = _twist_producers()
    assert producers, "found no base-command producers at all — the inventory is broken, not empty"
    on_cmd_vel = producers.get(T_CMD_VEL, [])
    # bringup/stale_cmd_test.py publishes /cmd_vel DELIBERATELY and says so in its --topic help:
    # it measures whether ranger_base keeps transmitting the last twist on CAN after its commander
    # dies, i.e. it is testing the DRIVER, not the mux, and must not go through the mux.
    unexpected = [w for w in on_cmd_vel
                  if "twist_mux_node.py" not in w and "stale_cmd_test.py" not in w]
    assert not unexpected, (
        f"{T_CMD_VEL} has publishers other than safety/twist_mux_node.py: {unexpected}. "
        f"Each one bypasses every base-motion protection this repo has.")
    assert any("twist_mux_node.py" in w for w in on_cmd_vel), \
        f"nothing publishes {T_CMD_VEL} at all — the mux would not exist and nothing would drive"


def test_nav2_publishes_the_topic_safety_yaml_calls_the_nav_source():
    """The controller is not the only Nav2 node that drives the base.

    BackUp and DriveOnHeading are recovery behaviours running in behavior_server and they emit
    their own /cmd_vel independently of controller_server. Remapping only the controller leaves
    the recoveries publishing straight to /cmd_vel -- verified on 2026-08-20. Both nodes must land
    on exactly the topic config/safety.yaml arbitrates as `nav`."""
    nav_topic = _mux_sources()["nav"]
    producers = _twist_producers()
    remapped = [w for w in producers.get(nav_topic, []) if NAV_LAUNCH.name in w]
    assert len(remapped) >= 2, (
        f"only {len(remapped)} Nav2 node(s) remap {T_CMD_VEL} -> {nav_topic}: {remapped}. "
        f"controller_server AND behavior_server both drive the base; an un-remapped one bypasses "
        f"the mux entirely.")
    assert any("controller_server" in w for w in remapped)
    assert any("behavior_server" in w for w in remapped)


def test_every_mux_source_is_either_produced_or_a_known_unproduced_one():
    """A mux source nothing publishes is a permanently absent input, and the mux CANNOT SAY SO --
    fail-closed treats never-seen exactly like stale, which is correct and also invisible.

    Every configured source now has a producer: waypoints.py deliberately selects servo for the
    campaign's deadman-gated return while retaining teleop as its manual default. This test fails
    if a configured input becomes dead wiring."""
    srcs = _mux_sources()
    producers = _twist_producers()
    unproduced = sorted(t for t in srcs.values() if not producers.get(t))
    assert unproduced == [], (
        f"mux sources with no publisher in this repo: {unproduced}\n"
        f"  producers found: { {k: v for k, v in sorted(producers.items())} }")


def test_no_producer_publishes_a_command_topic_the_mux_does_not_arbitrate():
    """The mirror of the test above. A twist on a topic the mux never reads goes nowhere at all --
    safety/mux_watch.py exists because route_run.py published on a live topic for a full 180 s leg
    and the robot never moved. The failure looks identical to a dead chassis."""
    known = set(_mux_sources().values()) | {T_CMD_VEL}
    producers = _twist_producers()
    orphans = {t: w for t, w in producers.items() if t not in known}
    assert not orphans, (
        f"base commands are published to topics config/safety.yaml does not arbitrate: {orphans}. "
        f"The mux never reads them, so they drive nothing and report nothing.")


# ============================================================================== B. FRAME GRAPH
def _frames_this_robot_has():
    """{frame: which file publishes the edge that creates it}.

    Built from the files that ACTUALLY publish TF, not from a wish list. A costmap whose
    sensor_frame names a frame outside this set has its ObservationBuffer drop every single scan:
    the layer never marks and never clears, so the costmap stays empty, every path looks clear,
    and Nav2 reports nothing wrong. That is the worst possible failure of the three in the
    module docstring, because it fails OPEN."""
    ous = _yaml(OUSTER_YAML)
    drv = _yaml(OUSTER_DRIVER_YAML)["ouster/os_driver"]["ros__parameters"]
    cam = _yaml(REPO / "config" / "camera.yaml")
    slam = _yaml(SLAM_YAML)["slam_toolbox"]["ros__parameters"]
    frames = {
        slam["map_frame"]: "config/slam_os0.yaml (slam_toolbox publishes map->odom)",
        slam["odom_frame"]: "ranger_mini_v3.launch.py publish_odom_tf:=true, via session.sh",
        slam["base_frame"]: "ranger driver / bringup/session.sh",
        ous["parent_frame"]: "config/ouster.yaml parent_frame",
        ous["sensor_frame"]: "bringup/lidar3d.sh static TF from config/ouster.yaml mount",
        drv["lidar_frame"]: "ouster_ros driver (pub_static_tf: true)",
        drv["imu_frame"]: "ouster_ros driver (pub_static_tf: true)",
        f"{cam['namespace']}_link": "bringup/camera.sh static TF from config/camera.yaml mount",
    }
    assert len(frames) >= 5, f"frame inventory came back nearly empty: {frames}"
    return frames


def test_the_rep105_chain_is_the_same_chain_in_every_file_that_names_it():
    """map -> odom -> base_link, spelled identically in slam_os0.yaml and the running Nav2 params.

    base_frame is the one that has already bitten this stack: stock slam_toolbox ships
    base_footprint, this robot has no such frame, and with the stock value slam_toolbox NEVER
    PUBLISHES map->odom while logging nothing about it. /map still appears, the node looks
    healthy, Nav2 downstream is simply blind (measured 2026-08-20). The same trap is called out in
    the amcl block of the params. One disagreement anywhere in this chain reproduces it."""
    slam = _yaml(SLAM_YAML)["slam_toolbox"]["ros__parameters"]
    assert (slam["map_frame"], slam["odom_frame"], slam["base_frame"]) == \
           ("map", "odom", "base_link"), f"slam_os0.yaml breaks REP-105: {slam['map_frame']} -> " \
                                         f"{slam['odom_frame']} -> {slam['base_frame']}"

    doc = _yaml(RUNNING_PARAMS)
    bases = _params_key(doc, "robot_base_frame") + _params_key(doc, "base_frame_id")
    assert len(bases) >= 4, f"{RUNNING_PARAMS.name}: only {len(bases)} base-frame declarations " \
                            f"found; the recursive walk is not reaching the costmaps"
    for where, val in bases:
        assert val == slam["base_frame"], (
            f"{RUNNING_PARAMS.name}: {where} = {val}, but slam_toolbox's base_frame is "
            f"{slam['base_frame']}. This robot has no {val} frame.")

    # The costmaps' global_frame is NOT one value: the global costmap plans in map, the local
    # costmap rolls in odom. Getting these the same way round is the difference between a local
    # window that lurches on every map->odom correction and one that does not.
    globals_ = {w: v for w, v in _params_key(doc, "global_frame")}
    g = [v for w, v in globals_.items() if w.startswith("global_costmap")]
    l = [v for w, v in globals_.items() if w.startswith("local_costmap")]
    assert g and l, f"costmap global_frame walk found global={g} local={l} — schema changed?"
    assert set(g) == {"map"}, f"global costmap must plan in map, got {g}"
    assert set(l) == {"odom"}, f"local costmap must roll in odom, got {l}"


@pytest.mark.parametrize("path", ALL_PARAMS, ids=lambda p: p.name)
def test_no_costmap_names_a_sensor_frame_this_robot_does_not_have(path):
    """THE FAIL-OPEN ONE. Nav2's ObservationBuffer transforms each scan into sensor_frame; if the
    frame does not exist in TF the whole observation is dropped. The obstacle layer then never
    marks and never clears, the costmap stays clean, every path looks free, and NOTHING logs a
    problem -- Nav2 will happily plan straight through a wall it has been told about 6 times a
    second and cannot see.

    On this stack /scan comes from pointcloud_to_laserscan with target_frame:=base_link (set in
    session.sh) and is republished unchanged by scan_relay.py, so the scan's frame IS base_link.
    There is no base_link->lidar_link TF on this robot at all: bringup/lidar3d.sh publishes
    base_link->os_sensor, and health.py has reported lidar_link MISSING since the A1M8 came off."""
    have = _frames_this_robot_has()
    target = _p2l_flag("target_frame")
    sources = _costmap_observation_sources(_yaml(path))
    assert sources, f"{path.name}: no costmap observation source found — did the schema change?"
    for where, _topic, frame in sources:
        assert frame in have, (
            f"{path.name}: {where} sensor_frame = {frame!r}, which nothing on this robot "
            f"publishes. Frames that exist: {sorted(have)}. Every scan would be silently dropped "
            f"and the obstacle layer would never mark or clear.")
        assert frame == target, (
            f"{path.name}: {where} sensor_frame = {frame!r} but session.sh projects the cloud "
            f"with target_frame:={target}, so that is the frame the scan is stamped with.")


def test_the_ouster_frames_agree_between_the_config_that_runs_and_the_config_that_is_read():
    """config/ouster.yaml is what a human reads and what bringup/lidar3d.sh reads to publish the
    mount TF; config/ouster_driver.yaml is what the driver loads. If they name different frames
    the static edge lands on one node of the tree and the point cloud is stamped with another --
    a disconnected TF tree, which presents as 'the lidar works but nothing sees it'."""
    ous = _yaml(OUSTER_YAML)
    drv = _yaml(OUSTER_DRIVER_YAML)["ouster/os_driver"]["ros__parameters"]
    for key in ("sensor_frame", "lidar_frame", "imu_frame"):
        assert ous[key] == drv[key], \
            f"config/ouster.yaml {key}={ous[key]} but the driver runs {key}={drv[key]}"
    assert drv["point_cloud_frame"] in (drv["lidar_frame"], drv["sensor_frame"]), \
        f"the cloud is stamped {drv['point_cloud_frame']}, which the driver does not publish"


# ============================================================================== C. MAP PROVENANCE
def _map_names_on_disk():
    """Map stems in maps/ that have a grid. maps/ also holds waypoints.yaml and site_markers.yaml,
    so `a yaml WITH a .pgm` is the definition -- the same rule bringup/map_persist.sh list uses."""
    return sorted(p.stem for p in (REPO / "maps").glob("*.yaml")
                  if (REPO / "maps" / f"{p.stem}.pgm").is_file())


def _map_completeness(name: str):
    """(missing_extensions) for a map. bringup/map_persist.sh list encodes the rule: grid + pose
    graph = USABLE for a campaign; grid alone = 'CANNOT be relocalized into'."""
    return [e for e in ("pgm", "yaml", "posegraph", "data")
            if not (REPO / "maps" / f"{name}.{e}").is_file()
            or (REPO / "maps" / f"{name}.{e}").stat().st_size == 0]


def _loaded_map_name():
    f = REPO / "maps" / ".loaded_map"
    if not f.is_file() or not _text(f).strip():
        return None
    return _text(f).split()[0]


def _waypoint_map_names():
    wps = _yaml(REPO / "maps" / "waypoints.yaml")
    return sorted({v["map_name"] for v in wps.values()
                   if isinstance(v, dict) and v.get("map_name")})


def test_the_default_map_name_is_the_map_the_waypoints_were_recorded_in():
    """INCIDENT 3, pinned. Every waypoint in maps/waypoints.yaml carries a map_name; session.sh
    defaults MAP_NAME to a map name of its own. Both are well-formed and nothing compared them.
    (At the time this suite was written they read `atrium` and `atrium2d`.)

    What the disagreement costs: `bash bringup/session.sh nav` with no MAP_NAME set localizes into
    map A while every recorded coordinate means something in map B. safety/map_frame.py catches
    the mismatch and REFUSES every waypoint -- correctly -- so the campaign does not drive to the
    wrong place; it simply refuses to drive at all, in the lab, at the start of the session, for a
    reason that lives in two different files and is visible from neither.

    NOT SKIPPED WHEN THEY DISAGREE. Disagreement is the bug this test exists for."""
    if not _map_names_on_disk():
        pytest.skip("maps/ holds no map with a grid yet — nothing to be consistent with")

    default = _sh_default("MAP_NAME")
    wp_names = _waypoint_map_names()

    assert wp_names, ("maps/waypoints.yaml records no map_name at all — every map-frame waypoint "
                      "is nameless and nav2_goto.py refuses all of them")
    assert len(wp_names) == 1, (
        f"waypoints are split across maps {wp_names}; their coordinates are in different frames "
        f"and cannot all be driven in one session")

    assert default == wp_names[0], (
        f"bringup/session.sh defaults MAP_NAME={default!r} but every waypoint in "
        f"maps/waypoints.yaml was recorded in {wp_names[0]!r}. `session.sh nav` with no MAP_NAME "
        f"set localizes into the wrong map and safety/map_frame.py then refuses every waypoint.")


def test_the_loaded_map_if_one_is_loaded_is_the_map_the_waypoints_name():
    """maps/.loaded_map is RUNTIME state -- gitignored, written by session.sh nav and
    map_persist.sh, absent whenever no map is loaded. Its absence is a legitimate state that
    bringup/map_persist.sh list prints as 'no map is loaded', so it is skipped, not failed.

    Its CONTENT is not runtime-variable in the way that matters here: whatever session wrote it
    was localizing into a named map, and if that name is not the one the waypoints were recorded
    in, safety/map_frame.py refuses every one of them. That is the third face of incident 3."""
    loaded = _loaded_map_name()
    if loaded is None:
        pytest.skip("maps/.loaded_map absent — no map is currently loaded (a normal state)")
    wp_names = _waypoint_map_names()
    if not wp_names:
        pytest.skip("no map-frame waypoints recorded yet")
    assert loaded in wp_names, (
        f"maps/.loaded_map says {loaded!r}, the waypoints were recorded in {wp_names}. Those are "
        f"different coordinate frames and the numbers do not transfer.")
    assert loaded == _sh_default("MAP_NAME"), (
        f"maps/.loaded_map says {loaded!r} but session.sh defaults MAP_NAME to "
        f"{_sh_default('MAP_NAME')!r}; the next `session.sh nav` would load a different map.")


def test_a_map_this_stack_names_as_loadable_is_actually_relocalizable_on_disk():
    """A .pgm/.yaml pair is a PICTURE, not a map you can relocalize into.

    slam_toolbox's `mode: localization` deserializes map_file_name as <name>.posegraph +
    <name>.data. Given only the grid it comes up ACTIVE, publishes a /map, and silently starts a
    BRAND NEW graph rooted at the robot's feet -- a fresh-SLAM frame wearing a saved map's name,
    which is the one thing safety/map_frame.py exists to prevent. session.sh guards this at
    runtime; this asserts the guard would not fire on the values the repo ships."""
    on_disk = _map_names_on_disk()
    if not on_disk:
        pytest.skip("maps/ holds no map with a grid yet")

    checked = {}
    for name in {_sh_default("MAP_NAME"), _loaded_map_name(), *_waypoint_map_names()} - {None}:
        checked[name] = _map_completeness(name)
    assert checked, "no map is named by any config — the inventory is broken"

    broken = {n: m for n, m in checked.items() if m}
    assert not broken, (
        "maps named by the live configs are not relocalizable:\n  "
        + "\n  ".join(f"{n}: missing {m}  (named by "
                      + ", ".join(filter(None, [
                          "session.sh MAP_NAME default" if n == _sh_default("MAP_NAME") else "",
                          "maps/.loaded_map" if n == _loaded_map_name() else "",
                          "maps/waypoints.yaml" if n in _waypoint_map_names() else ""])) + ")"
                      for n, m in sorted(broken.items()))
        + "\n\nA grid-only map cannot be relocalized into: slam_toolbox comes up ACTIVE on a "
          "brand-new graph rooted at the robot's feet and nothing downstream can tell. Re-map and "
          "save with bringup/map_persist.sh, which writes the pose graph as well as the grid.")


def test_slam_localization_is_seeded_from_a_map_that_exists():
    """map_start_pose is REQUIRED in localization mode and its absence fails silently-ish -- one
    ERROR line, then ACTIVE anyway on an empty graph (measured 2026-09-01: saved atrium is
    772x855; without the seed it published 486x585 with the robot at (0,0)). The seed is only
    meaningful in the map it was measured in, so that map has to be the one being loaded."""
    slam = _yaml(SLAM_YAML)["slam_toolbox"]["ros__parameters"]
    seed = slam.get("map_start_pose")
    assert seed and len(seed) == 3, \
        "config/slam_os0.yaml lost map_start_pose; localization comes up on an empty graph"
    # Whichever map the next `session.sh nav` would actually deserialize: the one recorded as
    # loaded if there is one, otherwise the script's default.
    target = _loaded_map_name() or _sh_default("MAP_NAME")
    if not _map_names_on_disk():
        pytest.skip("maps/ holds no map with a grid yet")
    assert not _map_completeness(target), (
        f"the map that would be loaded ({target!r}) is incomplete: missing "
        f"{_map_completeness(target)}. slam_toolbox would come up ACTIVE on a brand-new graph "
        f"rooted at the robot's feet, wearing this map's name.")


# ============================================================================== D. PARAMS FILE
def test_spin_is_absent_from_the_behavior_plugins_that_run():
    """The base rolled 172 degrees when Nav2's spin recovery fired at a closed door: high CoM,
    xArm6 on top, 4WS. Asserted on the PLUGIN LIST, not the file text -- the comment directly
    above it reads `NO "spin"`, so a whole-file substring check passes on the documentation."""
    doc = _yaml(RUNNING_PARAMS)
    lists = [(w, d["behavior_plugins"]) for w, d in _walk(doc) if "behavior_plugins" in d]
    assert lists, f"{RUNNING_PARAMS.name}: no behavior_plugins list found"
    for where, plugins in lists:
        assert "spin" not in plugins, (
            f"{RUNNING_PARAMS.name}: {where} = {plugins}. Spin recovery flipped this robot "
            f"(roll 172 deg).")
    assert "no_spin" in _text(RUNNING_PARAMS), "the no-spin behaviour trees must be selected"


def test_enable_stamped_cmd_vel_is_false_wherever_a_node_drives_the_base():
    """Jazzy DEFAULTS this to true, i.e. TwistStamped. safety/twist_mux_node.py subscribes
    geometry_msgs/Twist. Mismatched message types on one topic is another silent-zero-delivery
    failure: Nav2 publishes happily, the mux receives nothing, the robot does not move and no
    layer reports a fault. Both driving nodes declare it, so both are checked."""
    doc = _yaml(RUNNING_PARAMS)
    decls = _params_key(doc, "enable_stamped_cmd_vel")
    assert len(decls) >= 2, (
        f"{RUNNING_PARAMS.name}: found {len(decls)} enable_stamped_cmd_vel declarations, expected "
        f"controller_server and behavior_server. Both drive the base.")
    for where, val in decls:
        assert val is False, (
            f"{RUNNING_PARAMS.name}: {where} = {val}. The mux consumes unstamped Twist; "
            f"TwistStamped delivers nothing, silently.")
    wheres = " ".join(w for w, _ in decls)
    assert "controller_server" in wheres and "behavior_server" in wheres, \
        f"a base-driving node no longer declares enable_stamped_cmd_vel: {wheres}"


def _bt_xml_keys():
    return ("default_nav_to_pose_bt_xml", "default_nav_through_poses_bt_xml")


def test_both_behaviour_tree_paths_are_rewritten_before_launch():
    """nav2_params_os0_map.yaml hard-codes both tree paths into ANOTHER developer's home
    directory. On this laptop they do not exist; bt_navigator fails to load its tree, the
    lifecycle manager aborts the bringup, and Nav2 comes up looking healthy while
    navigate_to_pose never works -- the silent half-failure docs/NAV2.md warns about.

    Two independent rewrites exist and BOTH must cover BOTH keys: session.sh seds the params into
    /tmp before launching, and ranger_nav.launch.py overrides them from __file__. This asserts
    the params really are unresolvable (so the rewrite is load-bearing, not decorative), that
    session.sh rewrites both AND verifies the rewrite, and that this repo ships both trees."""
    doc = _yaml(RUNNING_PARAMS)
    declared = {}
    for key in _bt_xml_keys():
        found = _params_key(doc, key)
        assert found, f"{RUNNING_PARAMS.name} no longer declares {key}"
        declared[key] = found[0][1]

    ses = _text(SESSION_SH)
    for key in _bt_xml_keys():
        assert f"{key}:" in ses, (
            f"session.sh does not rewrite {key}. The params ship it as {declared[key]!r}, which "
            f"does not exist here — bt_navigator would load nothing.")
    assert "behaviour-tree path rewrite failed" in ses, \
        "the rewrite must be VERIFIED — a failed sed would silently launch the original paths"

    launch_src = _text(NAV_LAUNCH)
    for key in _bt_xml_keys():
        assert key in launch_src, \
            f"{NAV_LAUNCH.name} no longer overrides {key} from __file__"

    for name in ("navigate_to_pose_no_spin.xml", "navigate_through_poses_no_spin.xml"):
        p = REPO / "nav2_bringup" / "behavior_trees" / name
        assert p.is_file(), f"this repo must ship its own {name}; the rewrite targets it"
        assert "<Spin" not in _text(p), \
            f"{name} still calls Spin — the recovery that rolled the base 172 degrees"
        assert name in ses, (
            f"session.sh's rewrite does not name {name}; it must point the params at THIS "
            f"repo's copy of the tree, wherever the repo happens to live")

    # NOTE, deliberately not an assertion: on this laptop both shipped paths are unresolvable
    # (they point into /home/minghanwei/...), which is what makes the rewrite load-bearing. On a
    # machine where they happen to resolve the rewrite is still required — it is what guarantees
    # the NO-SPIN trees are the ones loaded — so the checks above are unconditional.


def test_session_sh_launches_the_params_file_that_these_tests_pin():
    """The whole of section D is about ONE file. If session.sh launches a different one, every
    assertion above is about a file nothing runs."""
    ses = _text(SESSION_SH)
    assert RUNNING_PARAMS.name in ses, (
        f"session.sh no longer launches {RUNNING_PARAMS.name} — the params these tests pin are "
        f"not the params that run")
    assert "localization:=slam" in ses, \
        "Nav2 must not start map_server/AMCL: exactly one source may own /map and map->odom"


# ============================================================================== E. MIRRORS
# A number written in two files is a bug waiting for one of them to be edited. The repo documents
# several such mirrors on purpose (config/ouster.yaml's scan_slice mirrors session.sh's flags, and
# its self_mask mirrors scan_relay.py's constants) because the config is what a human reads while
# the script is what runs. Existing tests pin some. These are the ones nothing pinned.

def test_the_scan_slice_angular_resolution_matches_the_flag_that_runs():
    """config/ouster.yaml documents the slice in DEGREES; session.sh passes RADIANS. The four
    height/range values of this block are already pinned by tests/test_map_persistence.py; the
    angular one was not, and it is the one that ties the projection to the sensor: 0.35 deg is
    ~1024 rays over 360 deg, matching the OS0's native columns, and it is also the number
    config/ouster_driver.yaml's lidar_mode comment reasons about when it argues 512x10 leaves
    half the scan's bins empty."""
    documented_deg = _yaml(OUSTER_YAML)["scan_slice"]["angle_increment_deg"]
    running_rad = float(_p2l_flag("angle_increment"))
    assert math.degrees(running_rad) == pytest.approx(documented_deg, abs=0.005), (
        f"config/ouster.yaml documents {documented_deg} deg/ray but session.sh passes "
        f"{running_rad} rad = {math.degrees(running_rad):.4f} deg. Nothing reads the config at "
        f"runtime, so the next person to tune the slice edits the file that does nothing.")


def test_the_slam_localization_seed_is_the_start_waypoint_it_claims_to_be():
    """config/slam_os0.yaml says of map_start_pose: 'These are the coordinates of the start
    waypoint recorded in maps/waypoints.yaml at the moment atrium was saved, i.e. the robot's
    parking spot.' That is a number written twice, in two files, with no link between them.

    It matters more than most: a seed that is metres wrong is WORSE than none, because the scan
    matcher converges confidently to the wrong corridor. If someone re-records `start` and does
    not edit slam_os0.yaml, localization is seeded at the old parking spot and every subsequent
    pose is offset by however far the robot has been moved since."""
    seed = _yaml(SLAM_YAML)["slam_toolbox"]["ros__parameters"]["map_start_pose"]
    wps = _yaml(REPO / "maps" / "waypoints.yaml")
    start = wps.get("start")
    if not isinstance(start, dict):
        pytest.skip("no `start` waypoint recorded yet")
    assert [start["x"], start["y"], start["yaw"]] == pytest.approx(seed, abs=1e-4), (
        f"config/slam_os0.yaml map_start_pose = {seed} but maps/waypoints.yaml `start` is "
        f"[{start['x']}, {start['y']}, {start['yaw']}]. Localization would be seeded at a pose "
        f"the robot is not standing at.")


def test_slam_config_quotes_the_range_min_that_actually_runs():
    """config/slam_os0.yaml explains, in prose a human will act on, what keeps the chassis out of
    the map: 'What ACTUALLY does that is pointcloud_to_laserscan in session.sh: range_min:=0.50'.

    session.sh does not pass 0.50. It was DELIBERATELY lowered to 0.30 (config/ouster.yaml records
    why: at 0.70 a glass door that first appeared at 0.72 m was two centimetres from vanishing
    from the scan entirely, and the angular problem was moved to scan_relay.py's sector mask
    where it belongs). The prose was not updated, so the file that explains the safety property
    now cites a value that has not run since the day it was written -- and it is the file someone
    reads when deciding whether it is safe to change the mask."""
    running = float(_p2l_flag("range_min"))
    quoted = re.findall(r"range_min:=([0-9.]+)", _text(SLAM_YAML))
    assert quoted, "config/slam_os0.yaml no longer cites the projection's range_min"
    for q in set(quoted):
        assert float(q) == running, (
            f"config/slam_os0.yaml prose cites range_min:={q} but session.sh passes "
            f"range_min:={running}. The file that explains what keeps the robot out of its own "
            f"map is quoting a value that does not run.")


def test_nav2_speed_ceilings_fit_under_the_mux_ceilings():
    """The controller plans trajectories the mux will not let it execute.

    safety/twist_mux_node.py applies config/safety.yaml's limits AFTER arbitration, so anything
    Nav2 asks for above them is clamped. MPPI does not know that: it samples and scores full
    trajectories against its own vx_max/wz_max, commits to the best one, and then a different
    twist reaches the wheels. The result is not a crash -- it is a controller whose model of its
    own actuation is wrong, which presents as persistent overshoot and corner-cutting that looks
    like tuning and is not.

    safety.yaml is explicit that max_wz is the binding one: 'the Ranger spins about its centre,
    and with the arm out the tool tip sweeps ~0.88 m radius through space the costmap believes is
    empty.'"""
    lim = _yaml(SAFETY_YAML)["limits"]
    doc = _yaml(RUNNING_PARAMS)
    ctrl = [(w, d) for w, d in _walk(doc) if "vx_max" in d and "wz_max" in d]
    assert ctrl, f"{RUNNING_PARAMS.name}: no controller speed block found (vx_max/wz_max)"
    bad = []
    for where, d in ctrl:
        if float(d["vx_max"]) > float(lim["max_vx"]):
            bad.append(f"{where}.vx_max={d['vx_max']} > safety max_vx={lim['max_vx']}")
        if float(d["wz_max"]) > float(lim["max_wz"]):
            bad.append(f"{where}.wz_max={d['wz_max']} > safety max_wz={lim['max_wz']}")
        # vy only matters while the motion model actually emits it. DiffDrive does not, and the
        # Ranger firmware DROPS angular.z whenever linear.y is non-zero, which is why the model
        # was changed. Check it only if someone puts Omni back.
        if str(d.get("motion_model", "")).lower().startswith("omni") and \
                float(d.get("vy_max", 0)) > float(lim["max_vy"]):
            bad.append(f"{where}.vy_max={d['vy_max']} > safety max_vy={lim['max_vy']}")
    assert not bad, (
        "Nav2 plans against speeds the safety mux will clamp:\n  " + "\n  ".join(bad)
        + "\n\nMPPI scores trajectories it cannot execute, so the twist that reaches the wheels "
          "is not the one it committed to. Lower the controller, or raise the ceiling "
          "deliberately and say why in config/safety.yaml.")


def test_the_costmap_footprint_is_the_robot_in_hardware_specs():
    """The footprint polygon is the chassis dimensions written a second time, as four corners, in
    two costmaps. docs/HARDWARE_SPECS.md is the measurement of record (0.720 x 0.500 m). An
    understated footprint threads gaps the robot does not fit through; an overstated one refuses
    doorways it does. The inflation_radius comment reasons from the inscribed radius derived from
    exactly these numbers, so they cannot drift independently."""
    specs = _text(REPO / "docs" / "HARDWARE_SPECS.md")
    m = re.search(r"Dimensions\s*\|\s*([0-9.]+)\s*[x×]\s*([0-9.]+)", specs)
    assert m, "docs/HARDWARE_SPECS.md no longer states the chassis dimensions"
    length, width = float(m.group(1)), float(m.group(2))

    doc = _yaml(RUNNING_PARAMS)
    prints = _params_key(doc, "footprint")
    assert len(prints) >= 2, (
        f"{RUNNING_PARAMS.name}: found {len(prints)} footprints, expected the global and local "
        f"costmap. A walk that finds one is not checking both.")
    for where, val in prints:
        pts = yaml.safe_load(val)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        assert max(xs) - min(xs) == pytest.approx(length, abs=1e-3), \
            f"{where}: footprint length {max(xs) - min(xs)} != HARDWARE_SPECS {length}"
        assert max(ys) - min(ys) == pytest.approx(width, abs=1e-3), \
            f"{where}: footprint width {max(ys) - min(ys)} != HARDWARE_SPECS {width}"


def test_the_stow_pose_is_the_same_pose_in_the_script_and_the_gate():
    """bringup/stow_arm.py DRIVES the arm to STOW_DEG; safety/arm_monitor_node.py reads
    config/safety.yaml's stow_pose_deg and publishes the /safety/arm_stowed gate from it, within
    joint_tolerance_deg. If they disagree by more than the tolerance, stow_arm.py 'succeeds' and
    the gate stays False forever: the nav and servo mux sources are dead, teleop still works
    (allows_arm_override), and the robot looks alive while refusing every autonomous command.
    stow_arm.py names the config key in a comment; this makes the comment enforceable."""
    cfg = _yaml(SAFETY_YAML)["arm_monitor"]["xarm"]
    declared = [float(v) for v in cfg["stow_pose_deg"]]
    tol = float(cfg["joint_tolerance_deg"])
    m = re.search(r"^STOW_DEG\s*=\s*(\[[^\]]*\])", _text(REPO / "bringup" / "stow_arm.py"), re.M)
    assert m, "bringup/stow_arm.py no longer defines STOW_DEG"
    driven = [float(v) for v in ast.literal_eval(m.group(1))]
    assert len(driven) == len(declared), \
        f"stow pose lengths differ: script {driven}, config {declared}"
    off = [abs(a - b) for a, b in zip(driven, declared)]
    assert max(off) < tol, (
        f"bringup/stow_arm.py drives {driven} but config/safety.yaml gates on {declared} within "
        f"{tol} deg (worst joint off by {max(off)}). The arm would stow and the arm_stowed gate "
        f"would stay False, killing the nav and servo mux sources with no error.")


def test_the_lidar_lever_arm_is_the_same_number_in_the_config_and_the_envelope():
    """safety/reach_envelope.py hard-codes LIDAR_FORWARD_M and cites config/lidar.yaml for it. It
    is a LEVER ARM: an error there barely shows while driving straight and swings every return
    during rotation, which is exactly when mapping was observed to lose its pose. The envelope
    uses it to convert a scan range into a base standoff, so a drift between the two files moves
    every press standoff by the difference."""
    declared = float(_yaml(REPO / "config" / "lidar.yaml")["mount"]["x_m"])
    m = re.search(r"^LIDAR_FORWARD_M\s*=\s*([0-9.]+)",
                  _text(REPO / "safety" / "reach_envelope.py"), re.M)
    assert m, "safety/reach_envelope.py no longer defines LIDAR_FORWARD_M"
    assert float(m.group(1)) == declared, (
        f"safety/reach_envelope.py uses {m.group(1)} m; config/lidar.yaml mount.x_m is "
        f"{declared} m")
