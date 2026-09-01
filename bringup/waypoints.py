#!/usr/bin/env python3
"""Record places by driving to them, then drive back to them on odometry alone.

    python3 bringup/waypoints.py record door_a     # stand the robot here, save this spot
    python3 bringup/waypoints.py list
    python3 bringup/waypoints.py where             # where am I relative to each waypoint
    python3 bringup/waypoints.py goto door_a       # DRY RUN: prints what it would do
    python3 bringup/waypoints.py goto door_a --go  # THE ROBOT MOVES

WHY ODOMETRY AND NOT A MAP. On 2026-08-25 slam_toolbox could not hold a pose in this building --
in a corridor a ~100-point scan matches almost equally well at several positions and the estimate
flips between them. The experiment does not need a map. It needs the base to arrive with the ADA
plate in the camera frame; approach_target.py servos visually from there and hand-eye is good to
2.96 mm RMS. Odometry drift over a 15-20 m leg is well inside what the servo absorbs.

WHAT THAT COSTS YOU, STATED PLAINLY. Odometry is dead reckoning. It drifts, it never recovers, and
NOTHING here detects that it has. A waypoint recorded at the far end of a long drive is only as
good as the odometry that got you there. Record from a KNOWN START, keep legs short, and treat
every arrival as approximate until the camera confirms it.

  * `record` is passive -- it reads /odom and writes a file. It cannot move anything.
  * `goto` publishes to /cmd_vel_teleop, so it goes through the safety mux like everything else
    and is subject to every gate. It is NOT a bypass. The mux must be running.
  * A lidar corridor check vetoes motion; see safety/waypoint_drive.corridor_blocked.
  * A watchdog stops the robot if this process dies -- the chassis coasts 1.26 s on a lost
    commander (EXPERIMENT_LOG 2026-08-21d), so the parting zero matters.
"""
from __future__ import annotations

import argparse
import os
import math
import sys
import time
from pathlib import Path

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parent))
from _ros_env import require_ros
require_ros()

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from odom_session import odom_session_id  # noqa: E402
from pose_source import PoseSource, current_map_name, mola_session_id  # noqa: E402
from safety.map_frame import (FRAME_KEY, MAP_NAME_KEY, MOLA_SESSION_KEY,  # noqa: E402
                              FRAME_MAP, FRAME_ODOM)
from safety.waypoint_frame import SESSION_KEY, check_session  # noqa: E402
from safety.waypoint_drive import Limits, corridor_blocked, plan_step, to_goal, wrap  # noqa: E402

# UTP_WAYPOINTS: alternate store, so a SIM run (domain 42) can never read or clobber the
# hardware waypoints -- sim poses in the real building would drive the robot into a wall.
STORE = Path(os.environ.get("UTP_WAYPOINTS", "")) if os.environ.get("UTP_WAYPOINTS") \
    else REPO / "maps" / "waypoints.yaml"
CMD_TOPIC = "/cmd_vel_teleop"
RATE_HZ = 20.0
ODOM_STALE_S = 0.5


def yaw_of(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def load() -> dict:
    if not STORE.exists():
        return {}
    return yaml.safe_load(STORE.read_text()) or {}


def save(d: dict) -> None:
    STORE.parent.mkdir(parents=True, exist_ok=True)
    STORE.write_text(
        "# Waypoints. The 'frame' field says which frame each one is in; ABSENT MEANS ODOM,\n"
        "# because every waypoint recorded before map support existed is an odom waypoint.\n"
        "#\n"
        "# frame: odom -- only meaningful within one continuous run of the ranger\n"
        "# driver: restarting it re-zeros odom and silently invalidates every entry here.\n"
        "# The 'odom_session' field is how you tell: the DDS GID of the /odom publisher, a new\n"
        "# value for every driver instance. goto and route_run REFUSE a waypoint whose session\n"
        "# does not match the running one. 'odom_epoch' is wall-clock provenance only -- nothing\n"
        "# reads it. See safety/waypoint_frame.py.\n"
        + yaml.safe_dump(d, sort_keys=True))


class Pose(Node):
    """Just enough node to read one pose sample -- from odom, or from a SLAM map frame."""

    def __init__(self, name: str = "utp_waypoints", frame: str = "auto"):
        super().__init__(name)
        self.pose = None
        self.stamp = 0.0
        self.scan = None
        # PoseSource fills self.pose/self.stamp exactly as the old /odom callback did. It
        # resolves lazily, on first wait_for_pose(), so `list` pays nothing for TF settling.
        self.src = PoseSource(self, frame)
        self.create_subscription(LaserScan, "/scan_filtered", self._scan, qos_profile_sensor_data)

    def _scan(self, m: LaserScan) -> None:
        self.scan = m

    def wait_for_pose(self, timeout: float = 5.0) -> bool:
        return self.src.wait_for_pose(timeout)

    def fresh(self) -> bool:
        return self.src.fresh()


def cmd_record(n: Pose, a) -> int:
    # --at lets a waypoint be marked by POINTING AT THE MAP instead of driving the robot there.
    # `record` normally captures where the robot IS, which is the right default -- the pose is
    # measured, not estimated, and for anything the arm must reach (the ADA plate at a 0.68 m
    # standoff) it is the only acceptable way.
    #
    # But `door` and `outside` are NAVIGATION targets: Nav2 plans to them and the goal checker
    # accepts arrival within its own tolerance, so a coordinate read off the map to +-15 cm is
    # entirely good enough -- and it does not require driving the robot to a spot that may be
    # behind the very closed door you are trying to test. Recorded exactly like a driven one,
    # with the same map-name provenance, so nothing downstream can tell or needs to.
    #
    # The heading, when not given, faces from where the ROBOT IS NOW toward the point -- the
    # direction it will be travelling when it arrives.
    if getattr(a, "at", None):
        if n.src.frame != FRAME_MAP:
            print("--at records a MAP-frame coordinate; run with --frame map", file=sys.stderr)
            return 1
        gx, gy = float(a.at[0]), float(a.at[1])
        if len(a.at) > 2:
            th = math.radians(float(a.at[2]))
        else:
            if not n.wait_for_pose():
                print("need the robot's pose to derive a heading; pass a third --at value "
                      "(degrees) to set it explicitly", file=sys.stderr)
                return 1
            rx, ry, _ = n.pose
            th = math.atan2(gy - ry, gx - rx)
        x, y = gx, gy
        d = load()
        mola = mola_session_id(n)
        if mola is None:
            print("cannot identify the SLAM pose publisher. Refusing to record a waypoint that "
                  "cannot be validated later.", file=sys.stderr)
            return 1
        entry = {"x": round(x, 4), "y": round(y, 4), "yaw": round(th, 4),
                 "odom_epoch": round(time.time()), FRAME_KEY: FRAME_MAP,
                 MOLA_SESSION_KEY: mola, "marked_on_map": True}
        mp = current_map_name(n)
        if not mp:
            print("NO SAVED MAP IS LOADED. A map-frame coordinate means nothing without one -- "
                  "refusing.", file=sys.stderr)
            return 1
        entry[MAP_NAME_KEY] = mp
        d[a.name] = entry
        save(d)
        print(f"marked '{a.name}' ON THE MAP: x={x:+.3f} y={y:+.3f} "
              f"yaw={math.degrees(th):+.1f} deg [frame=map, map='{mp}']")
        print("  NOT a measured robot pose -- fine for a navigation goal, NOT for an arm target.")
        print(f"  -> {STORE}")
        return 0

    if not n.wait_for_pose():
        print(f"no pose: {n.src.description or 'nothing publishing'}", file=sys.stderr)
        return 1
    print(f"  {n.src.description}")
    x, y, th = n.pose
    d = load()

    if n.src.frame == FRAME_MAP:
        # MAP FRAME. What makes these portable between sessions is the NAME of the saved map,
        # not the existence of a map frame -- a fresh MOLA has one too, with an arbitrary origin.
        # Both are recorded so safety/map_frame.py can tell them apart later.
        mola = mola_session_id(n)
        if mola is None:
            print("cannot identify the MOLA pose publisher (absent, or more than one). Refusing "
                  "to record a waypoint that cannot be validated later.", file=sys.stderr)
            return 1
        entry = {"x": round(x, 4), "y": round(y, 4), "yaw": round(th, 4),
                 "odom_epoch": round(time.time()), FRAME_KEY: FRAME_MAP,
                 MOLA_SESSION_KEY: mola}
        mp = current_map_name(n)
        if mp:
            entry[MAP_NAME_KEY] = mp
        d[a.name] = entry
        save(d)
        print(f"recorded '{a.name}': x={x:+.3f} y={y:+.3f} yaw={math.degrees(th):+.1f} deg "
              f"[frame=map]")
        if mp:
            print(f"  anchored to saved map '{mp}' -- valid in any future session localized "
                  f"in that map.")
        else:
            print("  NO SAVED MAP LOADED, so this is valid only while THIS MOLA instance keeps "
                  "running. To make it survive a restart: save a map "
                  "(bash bringup/map_persist.sh save <name>), load it, and re-record.")
        print(f"  -> {STORE}")
        return 0

    sess = odom_session_id(n)
    if sess is None:
        print("cannot identify the /odom publisher (absent, or more than one -- run "
              "bringup/health.py). Refusing to record a waypoint that cannot be validated later.",
              file=sys.stderr)
        return 1
    d[a.name] = {"x": round(x, 4), "y": round(y, 4), "yaw": round(th, 4),
                 "odom_epoch": round(time.time()), FRAME_KEY: FRAME_ODOM, SESSION_KEY: sess}
    save(d)
    print(f"recorded '{a.name}': x={x:+.3f} y={y:+.3f} yaw={math.degrees(th):+.1f} deg "
          f"[frame=odom]")
    print(f"  -> {STORE}")
    return 0


def cmd_list(n: Pose, a) -> int:
    d = load()
    if not d:
        print("no waypoints yet")
        return 0
    for k, v in sorted(d.items()):
        print(f"  {k:<20} x={v['x']:+8.3f} y={v['y']:+8.3f} yaw={math.degrees(v['yaw']):+7.1f} deg")
    return 0


def cmd_where(n: Pose, a) -> int:
    if not n.wait_for_pose():
        print(f"no pose: {n.src.description or 'nothing publishing'}",
              file=sys.stderr)
        return 1
    x, y, th = n.pose
    print(f"now: x={x:+.3f} y={y:+.3f} yaw={math.degrees(th):+.1f} deg")
    for k, v in sorted(load().items()):
        dist, bear = to_goal(x, y, th, v["x"], v["y"])
        print(f"  {k:<20} {dist:6.2f} m away, bearing {math.degrees(bear):+7.1f} deg")
    return 0


def cmd_rebase(n: Pose, a) -> int:
    """Re-express every waypoint in a NEW odom frame, given where the robot was in the OLD one.

    WHY THIS EXISTS. Waypoints live in the odom frame and `ranger_base` zeroes odom every time it
    starts. It has restarted twice on 2026-08-26 -- once from a crash when the CAN adapter
    re-enumerated, once for a driver rebuild -- and each time every recorded waypoint silently
    became wrong. Re-driving the whole route to re-record is a twenty-minute tax on a five-second
    event.

    It is recoverable because the robot does not MOVE across a driver restart. The new origin is
    the robot's current physical position, so if you note its pose in the old frame first, the two
    frames differ by exactly that rigid transform. Apply the inverse and the waypoints are valid
    again -- same places, new numbers.

    THE ONE RULE: read the old pose while the OLD driver is still running, and do not move the
    robot in between. If it rolls even slightly, every waypoint inherits that error silently.
    """
    ox, oy, oyaw = a.from_x, a.from_y, math.radians(a.from_yaw_deg)
    d = load()
    if not d:
        print("no waypoints to rebase", file=sys.stderr)
        return 1
    new_session = odom_session_id(n)
    if new_session is None:
        print("cannot identify the /odom publisher -- start ranger_bringup first, so the rebased "
              "waypoints can be stamped with the session they are valid in.", file=sys.stderr)
        return 1
    c, s_ = math.cos(oyaw), math.sin(oyaw)
    out = {}
    for k, v in d.items():
        dx, dy = v["x"] - ox, v["y"] - oy
        # Re-stamp with the CURRENT session. Rebasing that left the old id in place would
        # produce waypoints that are correct and still refused; dropping it, as this did before,
        # produced waypoints that are correct and unverifiable.
        out[k] = {"x": round(dx*c + dy*s_, 4),
                  "y": round(-dx*s_ + dy*c, 4),
                  "yaw": round(wrap(v["yaw"] - oyaw), 4),
                  "odom_epoch": round(time.time()), SESSION_KEY: new_session}
        print(f"  {k:<18} ({v['x']:+7.3f},{v['y']:+7.3f},{math.degrees(v['yaw']):+7.1f}) -> "
              f"({out[k]['x']:+7.3f},{out[k]['y']:+7.3f},{math.degrees(out[k]['yaw']):+7.1f})")
    if not a.go:
        print("\nDRY RUN. Add --go to write.")
        return 0
    save(out)
    print(f"\nrebased {len(out)} waypoints -> {STORE}")
    return 0


def cmd_goto(n: Pose, a) -> int:
    d = load()
    if a.name not in d:
        print(f"unknown waypoint '{a.name}'. Known: {sorted(d) or 'none'}", file=sys.stderr)
        return 2
    goal = d[a.name]
    if not n.wait_for_pose():
        print(f"no pose: {n.src.description or 'nothing publishing'}",
              file=sys.stderr)
        return 1

    ok, why = check_session(d, odom_session_id(n), names={a.name})
    if not ok:
        print(f"\nSTALE WAYPOINT -- not driving.\n  {why}", file=sys.stderr)
        if not a.force:
            return 1
        print("  --force given: driving anyway. The coordinate may be meaningless.",
              file=sys.stderr)

    lim = Limits()
    pub = n.create_publisher(Twist, CMD_TOPIC, 10)
    if not a.go:
        x, y, th = n.pose
        dist, bear = to_goal(x, y, th, goal["x"], goal["y"])
        step = plan_step(dist, bear, wrap(goal["yaw"] - th), False, lim)
        print(f"DRY RUN. {dist:.2f} m away, bearing {math.degrees(bear):+.1f} deg")
        print(f"  first action: {step.state}  vx={step.twist.vx:.3f} wz={step.twist.wz:.3f}")
        print(f"  would publish to {CMD_TOPIC} at {RATE_HZ:.0f} Hz. Re-run with --go to move.")
        return 0

    print(f"DRIVING to '{a.name}'. Ctrl-C stops. E-stop is faster.")
    deadline = time.monotonic() + a.timeout
    last_state = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(n, timeout_sec=1.0/RATE_HZ)
            if not n.fresh():
                pub.publish(Twist())
                continue
            x, y, th = n.pose
            dist, bear = to_goal(x, y, th, goal["x"], goal["y"])
            blocked = False
            if n.scan is not None:
                blocked = corridor_blocked(n.scan.ranges, n.scan.angle_min,
                                           n.scan.angle_increment)
            step = plan_step(dist, bear, wrap(goal["yaw"] - th), blocked, lim,
                             prev_state=last_state or "")
            t = Twist(); t.linear.x = step.twist.vx; t.angular.z = step.twist.wz
            pub.publish(t)
            if step.state != last_state:
                print(f"  [{step.state}] {dist:5.2f} m, bearing {math.degrees(bear):+6.1f} deg")
                last_state = step.state
            if step.state == "arrived":
                break
        else:
            print("TIMEOUT -- stopping", file=sys.stderr)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # The chassis coasts 1.26 s on a lost commander. An explicit zero is a COMMAND and stops
        # it now; letting the watchdog expire is 18 cm of uncommanded travel.
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.02)
        print("stopped (zero published)")
    return 0


ANCHORS = REPO / "maps"


def _collect_scans(n: Pose, count: int, timeout: float = 15.0) -> list:
    """`count` distinct /scan_filtered sweeps from the current (stationary) pose."""
    seen = set()
    out = []
    end = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < end and len(out) < count:
        rclpy.spin_once(n, timeout_sec=0.1)
        s = n.scan
        if s is None:
            continue
        key = (s.header.stamp.sec, s.header.stamp.nanosec)
        if key in seen:
            continue
        seen.add(key)
        out.append({"ranges": [float(r) for r in s.ranges], "angle_min": float(s.angle_min),
                    "angle_increment": float(s.angle_increment)})
    return out


def cmd_anchor(n: Pose, a) -> int:
    """Save a dense lidar reference at the current pose, so this frame can be re-found later.

    WHY. Odometry drifts; session ids only catch a driver restart. Measured 2026-08-29: from one
    recorded 'button' two runs landed 1.6-1.7 m from the plate. The operator's requirement is
    "make sure the coordinates for each run are good so I don't have to re-record before every
    run". The lidar measures the static world, so a scan saved HERE, now, is a landmark that
    `relocalize` can match against on every later run -- and it takes twenty sweeps, not one,
    because this A1M8 returns on ~13% of beams and a single sweep is too sparse to match to.
    """
    if not n.wait_for_pose():
        print(f"no pose: {n.src.description or 'nothing publishing'}",
              file=sys.stderr)
        return 1
    scans = _collect_scans(n, count=20)
    if len(scans) < 10:
        print(f"only {len(scans)} scans in 15 s -- is bringup/lidar3d.sh + the scan chain running? (bash bringup/session.sh up)", file=sys.stderr)
        return 1
    x, y, th = n.pose
    sid = odom_session_id(n)
    f = ANCHORS / f"anchor_{a.name}.json"
    f.write_text(json.dumps({"name": a.name, "pose": [x, y, th], "odom_session": sid,
                             "odom_epoch": round(time.time()), "scans": scans}))
    nvalid = sum(1 for s_ in scans for r in s_["ranges"] if r == r and 0.25 < r < 12.0)
    print(f"anchored '{a.name}' at x={x:+.3f} y={y:+.3f} yaw={math.degrees(th):+.1f} deg: "
          f"{len(scans)} sweeps, {nvalid} usable returns -> {f}")
    return 0


def cmd_relocalize(n: Pose, a) -> int:
    """Match a live scan to a saved anchor and re-express every waypoint in the CURRENT frame.

    Park the robot roughly where the anchor was taken (within ~1 m and ~40 deg) and run this
    before a route. It refuses -- and says why -- when the match is weak, so a bad alignment is
    never silently written over good coordinates.
    """
    from safety.scan_anchor import accumulate, apply_transform, relocalize, scan_to_points
    f = ANCHORS / f"anchor_{a.name}.json"
    if not f.exists():
        print(f"no anchor '{a.name}' -- record one with: waypoints.py anchor {a.name}",
              file=sys.stderr)
        return 2
    anc = json.loads(f.read_text())
    if not n.wait_for_pose():
        print(f"no pose: {n.src.description or 'nothing publishing'}",
              file=sys.stderr)
        return 1
    live_scans = _collect_scans(n, count=3)
    if not live_scans:
        print("no /scan_filtered -- is bringup/lidar3d.sh + the scan chain running? (bash bringup/session.sh up)", file=sys.stderr)
        return 1
    ref = accumulate([s_["ranges"] for s_ in anc["scans"]], anc["scans"][0]["angle_min"],
                     anc["scans"][0]["angle_increment"])
    live = accumulate([s_["ranges"] for s_ in live_scans], live_scans[0]["angle_min"],
                      live_scans[0]["angle_increment"])
    m, T = relocalize(tuple(anc["pose"]), live, ref, tuple(n.pose))
    print(f"match: dx={m.dx:+.3f} m dy={m.dy:+.3f} m dyaw={math.degrees(m.dyaw):+.1f} deg   "
          f"residual {m.residual_m*100:.1f} cm  margin {m.margin*100:.0f}%  "
          f"(live {m.n_live} pts, ref {m.n_ref} pts)")
    if T is None:
        print(f"NOT APPLIED: {m.why_not()}", file=sys.stderr)
        return 1
    tx, ty, tyaw = T
    print(f"frame correction: shift ({tx:+.3f}, {ty:+.3f}) m, rotate {math.degrees(tyaw):+.2f} deg")
    d = load()
    sid = odom_session_id(n)
    out = {}
    for k, v in d.items():
        x2, y2, yaw2 = apply_transform(T, (v["x"], v["y"], v["yaw"]))
        out[k] = {**v, "x": round(x2, 4), "y": round(y2, 4), "yaw": round(yaw2, 4),
                  "odom_epoch": round(time.time()), SESSION_KEY: sid}
        print(f"  {k:<12} ({v['x']:+7.3f},{v['y']:+7.3f},{math.degrees(v['yaw']):+7.1f}) -> "
              f"({x2:+7.3f},{y2:+7.3f},{math.degrees(yaw2):+7.1f})")
    if not a.go:
        print("\nDRY RUN. Add --go to write.")
        return 0
    save(out)
    print(f"\nrelocalized {len(out)} waypoints -> {STORE}")
    return 0


def cmd_derive(n: Pose, a) -> int:
    """A waypoint at an EXISTING one's spot, turned to face ANOTHER one. No driving, no /odom.

    Why this exists: the pose for LOOKING at the doors and the pose for PRESSING the plate are
    the same floor spot with different headings (measured 2026-08-26: 'button' is at the doors,
    yawed ~99 deg off the direction of travel to face the plate's wall). Recording both means
    piloting to the same spot twice; deriving one from the other cannot disagree about position.
    """
    d = load()
    missing = [k for k in (a.at, a.facing) if k not in d]
    if missing:
        print(f"unknown waypoint(s) {missing}; known: {sorted(d)}", file=sys.stderr)
        return 1
    src, tgt = d[a.at], d[a.facing]
    dx, dy = tgt["x"] - src["x"], tgt["y"] - src["y"]
    if math.hypot(dx, dy) < 0.30:
        print(f"'{a.at}' and '{a.facing}' are {math.hypot(dx, dy):.2f} m apart -- too close to "
              f"define a heading. Pick a farther 'facing' waypoint.", file=sys.stderr)
        return 1
    yaw = math.atan2(dy, dx)
    d[a.name] = {"x": src["x"], "y": src["y"], "yaw": round(yaw, 4),
                 "odom_epoch": src.get("odom_epoch", 0),
                 SESSION_KEY: src.get(SESSION_KEY)}
    save(d)
    print(f"derived '{a.name}': at '{a.at}' (x={src['x']:+.3f} y={src['y']:+.3f}), "
          f"facing '{a.facing}' -> yaw={math.degrees(yaw):+.1f} deg")
    print(f"  -> {STORE}")
    return 0


def cmd_project(n: Pose, a) -> int:
    """A waypoint N metres straight ahead of an existing one, same heading. No driving.

    For a destination the robot cannot be piloted to before the run -- e.g. 'outside' when the
    doors are closed and only the autonomous run will open them. The leg to it is dead
    reckoning through unseen space, so it stays short and the corridor veto stays on.
    """
    d = load()
    if a.src not in d:
        print(f"unknown waypoint '{a.src}'; known: {sorted(d)}", file=sys.stderr)
        return 1
    if not (0.3 <= a.forward <= 6.0):
        print(f"--forward {a.forward} m is outside 0.3..6.0 -- a projected goal is a guess, "
              f"and a long guess through unseen space is not a plan.", file=sys.stderr)
        return 1
    src = d[a.src]
    d[a.name] = {"x": round(src["x"] + a.forward * math.cos(src["yaw"]), 4),
                 "y": round(src["y"] + a.forward * math.sin(src["yaw"]), 4),
                 "yaw": src["yaw"], "odom_epoch": src.get("odom_epoch", 0),
                 SESSION_KEY: src.get(SESSION_KEY)}
    save(d)
    v = d[a.name]
    print(f"projected '{a.name}': {a.forward:.2f} m ahead of '{a.src}' -> "
          f"x={v['x']:+.3f} y={v['y']:+.3f} yaw={math.degrees(v['yaw']):+.1f} deg")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # --frame goes on every subcommand rather than on the top parser, so it can be typed where
    # it reads naturally: `waypoints.py record button --frame map`.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--frame", choices=["auto", "map", "odom"], default="auto",
                        help="which frame to record/read in. auto (default) prefers the SLAM map "
                             "when MOLA is publishing a usable one, and always says which it "
                             "chose rather than deciding silently.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record", parents=[common]); r.add_argument("name")
    r.add_argument("--at", nargs="+", metavar=("X", "Y"),
                   help="mark a MAP coordinate instead of capturing the robot's pose: "
                        "--at X Y [YAW_DEG]. For navigation targets only, never arm targets.")
    r.set_defaults(fn=cmd_record)
    sub.add_parser("list", parents=[common]).set_defaults(fn=cmd_list)
    sub.add_parser("where", parents=[common]).set_defaults(fn=cmd_where)
    rb = sub.add_parser("rebase", parents=[common], help="re-express waypoints after a ranger_base restart")
    rb.add_argument("--from-x", type=float, required=True,
                    help="robot x in the OLD odom frame, read before the restart")
    rb.add_argument("--from-y", type=float, required=True)
    rb.add_argument("--from-yaw-deg", type=float, required=True)
    rb.add_argument("--go", action="store_true")
    rb.set_defaults(fn=cmd_rebase)
    dv = sub.add_parser("derive", parents=[common], help="new waypoint at one waypoint's spot, facing another")
    dv.add_argument("name", help="name for the derived waypoint, e.g. doors")
    dv.add_argument("--at", required=True, help="take x,y from this waypoint")
    dv.add_argument("--facing", required=True, help="point the heading at this waypoint")
    dv.set_defaults(fn=cmd_derive)
    pj = sub.add_parser("project", parents=[common], help="new waypoint N metres straight ahead of another")
    pj.add_argument("name"); pj.add_argument("--from", dest="src", required=True)
    pj.add_argument("--forward", type=float, required=True, help="metres ahead, 0.3..6.0")
    pj.set_defaults(fn=cmd_project)
    an = sub.add_parser("anchor", parents=[common], help="save a lidar landmark at the current pose")
    an.add_argument("name"); an.set_defaults(fn=cmd_anchor)
    rl = sub.add_parser("relocalize", parents=[common], help="match the live scan to an anchor and re-express "
                                            "every waypoint in the current odom frame")
    rl.add_argument("name"); rl.add_argument("--go", action="store_true")
    rl.set_defaults(fn=cmd_relocalize)
    g = sub.add_parser("goto", parents=[common]); g.add_argument("name")
    g.add_argument("--force", action="store_true",
                   help="drive even if the waypoint is from a dead odom session (it will be wrong)")
    g.add_argument("--go", action="store_true", help="actually move the robot")
    g.add_argument("--timeout", type=float, default=120.0)
    g.set_defaults(fn=cmd_goto)
    a = ap.parse_args()

    rclpy.init()
    n = Pose(frame=a.frame)
    try:
        return a.fn(n, a)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
