#!/usr/bin/env python3
"""utp.pipeline.interfaces.World, implemented against the real robot.

    ~/unlocking-the-path/env/.venv/bin/python bringup/ros_world.py --selftest
    ~/unlocking-the-path/env/.venv/bin/python bringup/ros_world.py --goal through_door --dry-run

Runs under the PIPELINE VENV, because the FSM and the grounder need torch. Every ROS-side action
is a SUBPROCESS -- rclpy is not importable here and torch is not importable there, so the boundary
between perception and hardware is a process and a file, exactly as grab_frame/detect_frame
already work. Subprocess startup is ~1 s; every call here drives seconds-to-minutes of robot, so
that cost is irrelevant and the isolation is worth having.

WHAT THIS BUYS. The FSM's loop is navigate -> detect_blockage -> reason -> ground -> execute ->
verify. In simulation the World is Isaac and the blockage comes from GROUND TRUTH. Implement the
same seven methods against the robot and the SAME FSM, the SAME reasoner and the SAME grounder run
on hardware -- which is the difference between "we drove a route and pressed a button" and "the
system reasoned about a blocked door", and only the second is a result.

WHAT IS DELIBERATELY NOT HERE. The three gt_* methods are ground truth for benchmark scoring.
There is no ground truth on a real corridor. They return empty, and any metric derived from them
is meaningless on hardware -- which is correct, not a gap: reasoning_correct cannot be scored
against an answer key that does not exist. Score hardware trials on what actually happened
(did the door open) and keep the answer-key metrics for the sim campaign.

THE BLOCKAGE IS PERCEIVED, NOT KNOWN. current_blockage() asks the VLM what is in the way
(bringup/ask_blockage.py) and that call is deliberately restricted to DESCRIBING. It never
proposes an action, because the reasoner's choice of action is the thing being measured.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SIM = Path.home() / "unlocking-the-path"
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(REPO))          # `import safety.*` -- see below
sys.path.insert(0, str(REPO / "bringup"))
# REPO ITSELF WAS MISSING FROM THE PATH. Run as a script, sys.path[0] is bringup/, not the repo
# root, so every `from safety.<x> import ...` in this file (reach_envelope, look_policy, and now
# blockage_fusion) resolved only because run_trial.py / run_campaign.py happen to insert REPO
# before importing this module. `bringup/ros_world.py --selftest`, and any test that imports this
# module directly, hit ModuleNotFoundError instead. A file that imports a package must put that
# package on the path itself rather than relying on whoever imported it.

import numpy as np  # noqa: E402

from utp.pipeline.types import (BlockageEvent, Detection, ExecResult, NavOutcome,  # noqa: E402
                                Observation, Plan, Pose)
from ask_blockage import ask as ask_blockage  # noqa: E402

VENV = SIM / "env" / ".venv" / "bin" / "python"
ROS_PY = "python3"
LEG_TIMEOUT_S = 180
# How close the robot must be to its goal before a camera blockage counts as "blocked at it".
#
# 3.0 m, and the number is geometric, not a guess. On this route the recorded 'door' pose sits
# 1.95 m short of 'outside', and the robot parks ~0.6 m off 'door' after its leg -- so standing
# correctly in front of the closed doors puts it 2.59 m from 'outside'. At 2.5 m the check would
# have missed by 9 cm and the robot would have driven at glass instead of reasoning about it.
# Set this from the gap between the approach pose and the through-the-doorway goal, not by feel.
NEAR_GOAL_M = 3.0

# ---------------------------------------------------------------- backing off a confirmed blockage
#
# WHERE THE ROBOT ENDS UP WHEN IT DISCOVERS IT IS BLOCKED. Measured 2026-09-01, trial_ours_001:
# 0.72 m of forward range at closed glass doors -- the lidar had 85 returns inside +-20 deg and
# the nearest was 0.72 m. That is 0.72 m from the LIDAR, which sits 0.318 m ahead of base_link and
# 0.057 m behind the bumper (safety/reach_envelope.py), so the chassis was ~0.66 m off the glass.
# From there the robot cannot survey, and the operator's sequence starts with BACK UP for a
# reason -- so backing up is an explicit, named, bounded step here, not a side effect of an
# approach helper and emphatically not Nav2's `backup` recovery, which on that same trial reversed
# about 4 m with no idea what it was backing away from or why.
#
# HOW FAR, AND WHY 1.40 m. The number is the survey standoff (safety/reach_envelope.SURVEY_STANDOFF_M)
# and it is chosen by camera geometry, not by feel:
#
#   * the D435 colour FOV is 70.2 x 43.2 deg MEASURED -- not the 90 x 65 the datasheet reading
#     assumed -- so the horizontal half-angle is 35.1 deg, and anything further off-axis than that
#     is simply not in the picture no matter what the reasoner is asked;
#   * from a door-facing pose at 5.51 m range the ADA plate bore +25.1 deg, which puts it
#     5.51*sin(25.1) = 2.34 m to the SIDE of the door-facing axis, on the flanking wall, roughly in
#     the plane of the doors. That lateral offset is a property of the building and does not change
#     when the robot moves;
#   * so the plate's bearing from a door-facing pose d metres out is atan(2.34/d):
#         d = 0.72 m  ->  72.9 deg      d = 1.40 m  ->  59.1 deg      d = 2.00 m  ->  49.5 deg
#   * against SCAN_BEARINGS_DEG and a 35.1 deg half-frame, the plate is IN FRAME when
#     |bearing - rung| <= 35.1:
#         from 0.72 m : +80 rung -> 7.1 deg off boresight (in);  +45 rung -> 27.9 deg (barely in,
#                       at the frame edge, which is exactly where the 2026-08-29 trial lost it:
#                       "from 0.54 m the plate was at the extreme left edge, half cut off");
#         from 1.40 m : +80 rung -> 20.9 deg (in);               +45 rung -> 14.1 deg (in);
#         from 2.00 m : +80 rung -> 30.5 deg (in, at the edge);  +45 rung -> 4.5 deg (centred).
#
# So backing off to 1.40 m does one specific thing: it turns a sweep in which only ONE rung can
# frame the plate into one in which TWO independent rungs can. fsm.py takes the FIRST rung that
# moves the base and allows max_recovery_attempts of them, so a ladder with one viable rung is a
# ladder that gets one chance. Going further than 1.40 m starts costing pixels for no extra rung
# (~50 px at 1.40 m already, versus 81x88 px at ~1.0 m, and the grounder ranked two wall signs
# above the 50 px version), which is why this is the SURVEY standoff and not simply "as far back
# as the reverse is allowed to go".
BACKOFF_STANDOFF_M = 1.40
# Do not reverse for a few centimetres. 0.15 m is the same slack approach_blockage.py uses to
# decide it is "too close to survey", and it stops the robot twitching backwards every time the
# lidar reads 1.38 instead of 1.41.
BACKOFF_TRIGGER_SLACK_M = 0.15
# approach_blockage.reverse() refuses anything outside 0.05-1.00 m, and it refuses it for a good
# reason: the rear lidar sector is filtered out because it is the robot seeing ITSELF, so there is
# no obstacle check astern at all. One bounded reverse per confirmed blockage, never a free run.
MAX_BACKOFF_M = 1.00
# The forward window the standoff is measured in. +-20 deg, because that is the window the
# 2026-09-01 measurement was taken in (85 returns, nearest 0.72 m) and it is narrow enough to be
# about what is straight ahead rather than about the flanking walls.
FORWARD_HALF_ANGLE_DEG = 20.0

# ------------------------------------------------------- supervising the leg while it is driving
#
# THE LEG USED TO BE ONE BLIND BLOCKING CALL. The camera blockage was evaluated ONCE, at the pose
# the leg started from, and then `nav2_goto.py --go` ran to completion inside a single
# subprocess.run(). So the comment below claiming "within NEAR_GOAL_M the camera check runs
# exactly as before" described something that was never implemented: between the start of the leg
# and its end NOTHING looked. Measured 2026-09-01: the pre-leg check said "the goal is 8.1 m away,
# drive the leg first", the leg then ran uninterrupted, and the operator stopped the robot by hand
# as it closed on the glass doors. Enlarging NEAR_GOAL_M would only have shortened the blind leg.
#
# THE FIX IS A STAGED APPROACH, not a supervised one. The leg is driven as a sequence of BOUNDED
# stages: send the goal with a short --timeout, let nav2_goto cancel it and exit when that expires,
# perceive from the stopped pose, and only then send the next stage. It composes with the rest of
# the sequence being built here (blocked -> reverse -> look -> press -> resume), because every
# stage already ends in a perception step, and it needs no concurrency at all.
#
# WHY NOT Popen + poll. It buys nothing here and costs the one failure that actually bit this
# project: a detached run_trial/nav2_goto survived an interrupt and kept the robot driving,
# because it had been launched under setsid. Everything below stays inside subprocess.run(), so a
# stage's process is a child that dies with its parent, and nav2_goto's exit-code contract (0 real
# outcome, 6 timeout, 2-5 cannot serve, 1 crashed) is honoured unchanged -- a stage boundary is
# just a 6, produced by nav2_goto's own cancel-and-exit path rather than by us killing it.
#
# HOW LONG A STAGE IS. Not a fixed number: the robot must never drive for longer than it would
# take, at Nav2's own configured top speed, to close the gap between the nearest thing the lidar
# can see ahead RIGHT NOW and the range at which the leg would be aborted anyway. Anything the
# lidar can see therefore cannot be crossed unobserved.
#
#     stage_s = clamp((nearest_ahead_m - LEG_ABORT_RANGE_M) / NAV_VX_MAX, MIN, MAX)
#
# When NOTHING resolves ahead the stage is the MINIMUM, not the maximum -- an empty forward sector
# is the open corridor AND it is the glass door at range (2026-08-26: the lidar read 6.98 m
# straight through closed glass doors), and the least informed case gets the shortest leash. That
# costs time on an open corridor and it is the right way round: the failure being fixed is driving
# at glass, not arriving slowly.
NAV_VX_MAX = 0.6            # nav2_bringup/nav2_params_os0_map.yaml, MPPI FollowPath vx_max
# Below this the per-stage overhead (subprocess start, rclpy init, action handshake, one capture,
# one VLM call -- order 10 s) dominates the ~1 m of driving it supervises.
# 5.0, not 2.0. The pacing rule below was backwards for the case that dominates a real run: an
# OPEN corridor resolves nothing ahead, took the shortest stage, and therefore paid the most
# perception stops -- 8 m of empty floor cost 3-5 grab_frame + VLM pauses of ~4 s each, and the
# operator's first question after watching it was "why is it going so slowly". Short stages buy
# nothing where nothing is close: glass at range is invisible to both sensors anyway, so the
# supervision that matters starts when something DOES resolve, which shortens the stage on its
# own via the formula below.
LEG_STAGE_MIN_S = 5.0
# 8 s is ~4 m at cruise. The camera picks a glass door out at 8 m (that is the whole reason
# NEAR_GOAL_M exists), so even the longest stage leaves the robot at least one look at a door with
# room to stop.
LEG_STAGE_MAX_S = 12.0
# ABORT THE LEG WHEN SOMETHING SOLID IS THIS CLOSE AHEAD AND THE FUSED VERDICT SAYS BLOCKED --
# whatever the distance to the goal. The NEAR_GOAL_M gate exists to stop the camera calling a door
# it can see from 8 m "the blockage at my goal"; it is not a licence to ignore a surface a metre in
# front of the bumper. 1.20 m of lidar range puts the bumper 1.14 m clear (the lidar sits 0.318 m
# ahead of base_link and the bumper 0.375 m, so the bumper leads the sensor by 0.057 m), which is
# ample against Nav2's own 0.5 m/s and -2.5 m/s^2, and it is tighter than BACKOFF_STANDOFF_M so an
# abort and the back-off that follows it do not fight each other.
#
# BOTH conditions are required, deliberately. Proximity alone fires at every corner and doorway a
# corridor has; the fused verdict is what distinguishes "a wall I am turning past" from "the thing
# in my way".
LEG_ABORT_RANGE_M = 1.20


def _ros(args: list[str], timeout: int = 120) -> subprocess.CompletedProcess:
    """Run a ROS-side tool. env.sh is sourced in a login shell because rclpy only exists there."""
    cmd = f"source {REPO}/bringup/env.sh >/dev/null 2>&1 && " + " ".join(
        f"'{a}'" for a in args)
    return subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=timeout)


def _nav_result(stdout: str) -> dict | None:
    """Parse nav2_goto's final machine-readable result; never infer status from prose."""
    for line in reversed((stdout or "").splitlines()):
        if not line.startswith("RESULT "):
            continue
        try:
            value = json.loads(line[len("RESULT "):])
        except (TypeError, ValueError):
            return None
        return value if isinstance(value, dict) and isinstance(value.get("status"), str) else None
    return None


def _read_scan(capture_dir) -> dict | None:
    """The LaserScan that grab_frame.py saved beside rgb.png, or None.

    THE SCAN IS ALREADY ON DISK AND THAT IS THE POINT. rclpy is not importable in this process
    (see the module docstring: the perception/hardware boundary is a process and a file), so the
    honest way to get "the lidar at the instant of this camera frame" is not a new subscription
    from a new subprocess -- that would be a scan from a DIFFERENT instant, and the whole failure
    this fixes is about two sensors disagreeing about ONE moment. grab_frame.py already writes
    scan.json into the capture directory for the lidar lift, with exactly the three fields the
    fusion contract wants: {"frame", "angle_min", "angle_increment", "ranges"}.

    Returns None rather than raising: a capture with no scan means the camera is on its own, and
    the caller must say so out loud instead of pretending it fused something.
    """
    if capture_dir is None:
        return None
    try:
        d = json.loads((Path(capture_dir) / "scan.json").read_text())
    except Exception:
        return None
    if not isinstance(d, dict) or not d.get("ranges"):
        return None
    try:
        return {"ranges": [float(r) for r in d["ranges"]],
                "angle_min": float(d["angle_min"]),
                "angle_increment": float(d["angle_increment"]),
                "frame": str(d.get("frame", ""))}
    except (TypeError, ValueError, KeyError):
        return None


def _nearest_ahead_m(scan: dict | None,
                     half_deg: float = FORWARD_HALF_ANGLE_DEG) -> float | None:
    """Nearest RANGE in the corridor ahead, for DESCRIBING how close something is. Or None.

    ONE GEOMETRY. On 2026-09-01 three modules answered "how far ahead is it" with three different
    shapes: a +-15 deg cone in approach_blockage, a +-20 deg cone here, and the rectangle in
    safety/blockage_fusion. At 2 m a +-20 deg cone is 0.73 m half-width; at 0.5 m it is 0.18 m, so
    the same scan gave different numbers depending which module asked, and which one answered
    depended on whether an optional import had succeeded. This now delegates to the rectangle, so
    the number in a description always refers to the returns that made the blocked decision.

    THIS IS A RANGE. Do not reverse from it -- use _clearance_ahead_m(). See its docstring.
    """
    parsed = _scan_fields(scan)
    if parsed is None:
        return None
    ranges, a0, inc = parsed
    try:
        from safety.blockage_fusion import (DEFAULT_HALF_WIDTH_M, DEFAULT_LOOK_AHEAD_M,
                                            _nearest_in_corridor)
        return _nearest_in_corridor(ranges, a0, inc, DEFAULT_HALF_WIDTH_M, DEFAULT_LOOK_AHEAD_M)
    except ImportError:
        return _cone_min(ranges, a0, inc, half_deg, along_track=False)


def _clearance_ahead_m(scan: dict | None) -> float | None:
    """ALONG-TRACK clearance to the nearest thing in the corridor. THE ONE TO REVERSE FROM.

    A range is not a clearance, and the back-off was computed from a range. For an off-axis return
    the range overstates how much room there is: measured on captures/trial_ours_001/scan.json --
    the real capture taken at the closed glass doors -- the nearest range in the corridor is
    0.701 m while the along-track clearance is 0.578 m. Backing off "to 1.40 m" from the range
    leaves the robot 12.2 cm closer than intended, and the error is spent in the one direction
    that matters: toward the thing being backed away from.

    safety/blockage_fusion._nearest_in_corridor says this in its own docstring -- "use this to
    TELL somebody how close the thing is, never as a clearance budget" -- and the caller was doing
    exactly that. Two quantities, two names, so it cannot happen silently again.
    """
    parsed = _scan_fields(scan)
    if parsed is None:
        return None
    ranges, a0, inc = parsed
    try:
        from safety.blockage_fusion import clearance_ahead_m
        return clearance_ahead_m(ranges, a0, inc)
    except ImportError:
        return _cone_min(ranges, a0, inc, FORWARD_HALF_ANGLE_DEG, along_track=True)


def _scan_fields(scan):
    """(ranges, angle_min, angle_increment) or None. A malformed scan is not a measurement."""
    if not scan:
        return None
    try:
        return scan["ranges"], float(scan["angle_min"]), float(scan["angle_increment"])
    except (KeyError, TypeError, ValueError):
        return None


def _cone_min(ranges, a0: float, inc: float, half_deg: float, *, along_track: bool):
    """Fallback for when safety.blockage_fusion is unavailable. A CONE, so a different shape --
    deliberately narrow, so it under-reports clearance and fails toward stopping sooner."""
    best = None
    for i, r in enumerate(ranges):
        try:
            r = float(r)
        except (TypeError, ValueError):
            continue
        if r != r or r <= 0.0 or r == float("inf"):
            continue
        a = a0 + i * inc
        if abs(math.degrees(math.atan2(math.sin(a), math.cos(a)))) <= half_deg:
            v = r * math.cos(a) if along_track else r
            if v > 0.0:
                best = v if best is None else min(best, v)
    return best


def _fused_verdict(camera: dict, scan: dict | None) -> dict:
    """One blockage verdict from the camera dict and the saved scan, via safety.blockage_fusion.

    THE FAILURE THIS EXISTS FOR, measured 2026-09-01, captures/trial_ours_001. The robot was
    0.72 m from CLOSED GLASS DOORS. The VLM, asked about the camera frame, answered
    blocked=False, "an open walkway with pillars" -- and it was not hallucinating, it was
    describing what the picture shows, because the picture shows the corridor THROUGH the glass.
    The lidar in the same instant had 85 returns inside +-20 deg with the nearest at 0.72 m. The
    camera-only verdict drove the robot at the door and the operator stopped it by hand.

    Neither sensor is trusted alone, in either direction: the camera is the one that can read a
    door as a door (navigate_to_goal's docstring records the mirror-image failure -- lidar seeing
    6.98 m THROUGH the same kind of glass) and the lidar is the one that cannot be talked out of
    a surface 0.72 m away. Combining them is safety/blockage_fusion.py's job and its verdict is
    taken as given here.

    IF THE FUSION MODULE IS NOT ON DISK, this falls back to the CAMERA-ONLY verdict and prints
    the failure it is reinstating, because a hardware run that silently reverts to the behaviour
    of 2026-09-01 is worse than one that refuses -- but a file that will not import at all is
    worse still, and this module has to survive being read while blockage_fusion.py is being
    written next to it.
    """
    cam = dict(camera or {})
    base = {"blocked": bool(cam.get("blocked", True)),
            "kind": cam.get("kind", "") or "",
            "description": cam.get("description", "") or "",
            # "camera" until a fuser says otherwise: this dict IS the camera-only verdict, and it
            # is what every fallback below returns. Never label it "both".
            "evidence": "camera",
            "nearest_ahead_m": _nearest_ahead_m(scan)}
    if scan is None:
        return base
    try:
        from safety.blockage_fusion import fuse
    except ImportError:
        print("[ros_world] WARNING: safety/blockage_fusion.py is not importable, so this "
              "blockage verdict is CAMERA-ONLY. That is exactly the configuration that, on "
              "2026-09-01, called closed glass doors 0.72 m ahead 'an open walkway with "
              f"pillars' and drove at them. Nearest return ahead right now: "
              f"{'%.2f m' % base['nearest_ahead_m'] if base['nearest_ahead_m'] else 'unknown'}.")
        return base
    try:
        res = fuse(cam, scan["ranges"], scan["angle_min"], scan["angle_increment"])
    except Exception as e:                      # noqa -- the fuser must not be able to end a trial
        print(f"[ros_world] WARNING: safety.blockage_fusion.fuse raised "
              f"{type(e).__name__}: {e} -- falling back to the CAMERA-ONLY verdict, which is the "
              f"glass-door failure of 2026-09-01. Fix the fuser; do not run trials like this.")
        return base
    if not isinstance(res, dict) or "blocked" not in res:
        print(f"[ros_world] WARNING: safety.blockage_fusion.fuse returned {res!r}, which is not "
              f"the agreed contract -- falling back to the CAMERA-ONLY verdict.")
        return base
    out = dict(base)
    out.update({k: res[k] for k in
                ("blocked", "kind", "description", "evidence", "nearest_ahead_m") if k in res})
    out["blocked"] = bool(out["blocked"])
    return out


class RosWorld:
    """The World protocol, on hardware. Construct with the goal waypoint name."""

    # NOT world_kind = "ros". fsm.py stamps TrialRecord with getattr(world, "world_kind",
    # "mock"), but schema.py validates that field to mock|graph|isaac and raises on anything
    # else, so a hardware trial is logged as world="mock" -- a hardware result filed as a
    # simulation, which is the worst kind of wrong in a results table because nothing about it
    # looks broken. Adding "ros" to the schema is a one-line change in the PIPELINE repo and is
    # not mine to make. bringup/run_trial.py therefore stamps its own record instead, and the
    # `world` field inside a hardware TrialRecord must be read as meaningless, not as "mock".

    def __init__(self, goal: str = "", dry_run: bool = True, capture_prefix: str = "fsm",
                 hints=None) -> None:
        self.goal = goal
        self.hints = hints          # LookHints written by SteeredReasoner; None for other methods
        self.dry_run = dry_run
        self.capture_prefix = capture_prefix
        self._scan_i = 0            # next bearing in SCAN_BEARINGS_DEG
        self._scan_offset = 0.0     # degrees currently turned away from the approach heading
        self._looks: list = []      # audit trail for last_look_info()
        self._widened = False       # widen_view is once per blockage; see the note there
        self._n = 0
        self._last_nav = "reached"
        self._last_capture: Path | None = None
        self._blockage: BlockageEvent | None = None
        # The forward range and the evidence class behind the CURRENT blockage verdict, from the
        # fused result. Per-trial, and reset with everything else: a stale 0.72 m from the last
        # trial would send the next one reversing before it has looked at anything.
        self._nearest_ahead_m: float | None = None
        self._evidence = ""
        self._backed_off = False

    # -------------------------------------------------------------- lifecycle
    def reset(self, scene_type: str = "", seed: int = 0) -> None:
        self._n = 0
        self._last_nav = "reached"
        self._last_capture = None
        self._blockage = None
        self._nearest_ahead_m = None
        self._evidence = ""
        self._backed_off = False
        # THE LOOK LADDER IS PER-TRIAL STATE AND MUST RESET WITH IT (2026-08-31).
        # run_trial.py builds a fresh RosWorld per invocation, so this never mattered there. A
        # multi-trial campaign (bringup/run_campaign.py) reuses ONE world across N trials, and
        # without this every trial after the first would start with _scan_i already past the end
        # of SCAN_BEARINGS_DEG: the survey would be inert, the reasoner would abstain with nothing
        # new to look at, and 49 of 50 trials would fail identically for a reason that has nothing
        # to do with the robot. Same class as the sim's _scan_i/_strafe_i reset in
        # IsaacWorldClient.reset(), which exists for exactly this reason.
        self._scan_i = 0
        self._scan_offset = 0.0
        self._looks = []
        self._widened = False

    # ------------------------------------------------------------- perception
    def get_observation(self) -> Observation:
        """One aligned RGB-D frame, via bringup/grab_frame.py."""
        self._n += 1
        name = f"{self.capture_prefix}_{self._n:03d}"
        r = _ros([ROS_PY, str(REPO / "bringup" / "grab_frame.py"),
                  "--name", name, "--timeout", "45"], timeout=90)
        cap = REPO / "captures" / name
        if r.returncode != 0 or not (cap / "rgb.png").exists():
            # An Observation with rgb=None is what the grounder sees when it cannot see. That is
            # a real state the pipeline already handles -- do not fabricate an empty image.
            #
            # AND FORGET THE OLD CAPTURE. It used to be left in place, so a grab that failed
            # partway through a leg left current_blockage() fusing the frame AND the scan.json
            # from the pose before -- a verdict about where the robot USED to be, delivered as if
            # it were about here. Dropping it makes the next current_blockage() fail closed
            # ("path blocked; no camera frame available") instead of confidently describing a
            # place the robot has left.
            self._last_capture = None
            return Observation()
        self._last_capture = cap
        from PIL import Image
        rgb = np.array(Image.open(cap / "rgb.png").convert("RGB"))
        depth = np.load(cap / "depth.npy") if (cap / "depth.npy").exists() else None
        info = json.loads((cap / "cam.json").read_text()) if (cap / "cam.json").exists() else {}
        return Observation(rgb=rgb, depth=depth, cam_info=info, robot_pose=self._pose())

    def _pose(self) -> Pose:
        r = _ros([ROS_PY, str(REPO / "bringup" / "waypoints.py"), "where"], timeout=40)
        for line in r.stdout.splitlines():
            if line.startswith("now:"):
                try:
                    parts = line.replace("now:", "").split()
                    x = float(parts[0].split("=")[1]); y = float(parts[1].split("=")[1])
                    yaw = math.radians(float(parts[2].split("=")[1]))
                    return Pose(x=x, y=y, yaw=yaw)
                except Exception:
                    break
        return Pose()

    def _goal_waypoint(self) -> dict | None:
        """The stored record for self.goal, or None. Reads the file; touches no hardware."""
        try:
            import yaml
            store = Path(os.environ.get("UTP_WAYPOINTS") or (REPO / "maps" / "waypoints.yaml"))
            wp = (yaml.safe_load(store.read_text()) or {}).get(self.goal)
            return wp if isinstance(wp, dict) else None
        except Exception:
            return None

    def _distance_to_goal(self) -> float | None:
        """Metres from here to the goal waypoint, or None if it cannot be resolved.

        BOTH POSES MUST BE IN THE SAME FRAME, and getting that wrong is not a rounding error.
        `waypoints.py where` with no --frame resolves `auto`, which on this stack returns the ODOM
        pose. Subtracting a map-frame waypoint from an odom-frame position produces a number with
        no meaning at all: MEASURED 2026-09-01, robot genuinely 2.59 m from 'outside', this
        returned 5.09 m -- odom happened to read (4.96, 2.93) while the map pose was (5.35, 5.59).
        The gate then let the leg run when it should have reported blocked, and the robot set off
        in a direction that made no sense from outside. Two origins, one subtraction.

        So the frame is stated explicitly, and a pose that does not come back in the map frame
        returns None rather than a plausible-looking wrong number.

        None means "do not gate on distance" -- an unreadable pose must not silently disable the
        glass check, so the caller treats None as near, which fails toward stopping.
        """
        try:
            wp = self._goal_waypoint()
            if not wp:
                return None
            if wp.get("frame") != "map":
                return None          # an odom waypoint has no map-frame distance to give
            r = _ros([ROS_PY, str(REPO / "bringup" / "waypoints.py"), "where", "--frame", "map"],
                     timeout=40)
            for line in r.stdout.splitlines():
                if line.startswith("now:"):
                    parts = line.replace("now:", "").split()
                    x = float(parts[0].split("=")[1]); y = float(parts[1].split("=")[1])
                    return math.hypot(wp["x"] - x, wp["y"] - y)
            return None
        except Exception:
            return None

    # ------------------------------------------------------------- navigation
    def navigate_to_goal(self) -> NavOutcome:
        """Drive one leg toward the goal waypoint.

        THE LIDAR IS NOT THE BLOCKAGE DETECTOR. It was written that way and that was WRONG.
        Measured 2026-08-26, robot parked in front of closed glass doors:

            camera  : "closed glass double doors labeled Da Vinci Room"
            lidar   : nearest return straight ahead 6.98 m
            corridor_blocked() fired on 5/73 scans -- noise, not a detection

        A 2D lidar sees THROUGH glass, and glass is exactly what the doors in this building are
        made of. Relying on the corridor veto would drive the robot into a door at full speed and
        never report blocked, so the FSM would never reason, never ground, never act -- the whole
        experiment silently degrades into "drove into a window".

        So the CAMERA is checked BEFORE the wheels turn, and the lidar corridor veto stays on as
        a safety backstop for the opaque things the camera might miss. Two sensors, two failure
        modes, and neither one trusted alone.

        AND NEITHER ONE IS ASKED ONLY ONCE. That sentence used to describe a check that ran at the
        starting pose and then never again for the whole blocking leg; the leg is now driven in
        bounded stages with a fused camera+lidar look between them. See the block above
        LEG_ABORT_RANGE_M for why staging, and not a Popen-and-poll supervisor, is the shape of
        that fix.
        """
        if not self.goal:
            return NavOutcome(status="reached")
        # Camera first -- this is the one that sees glass. BUT ONLY ONCE WE ARE NEAR THE GOAL.
        #
        # WHY THE DISTANCE GATE EXISTS. Without it this check fired from wherever the robot
        # happened to be standing: the camera can see a glass door from 8 m, so current_blockage()
        # returned blocked BEFORE the wheels ever turned, this method returned, and the Nav2 leg
        # below -- the whole reason the saved map exists -- was never reached. Measured on hardware
        # 2026-09-01: three consecutive live trials recorded path_length_m 0.0 with the goal 8 m
        # away, then approach_blockage crept the robot forward along whatever heading it already
        # had. From outside it looked like navigation; the robot had simply never navigated.
        #
        # THE RULE, and it is the operator's: the VLM is triggered when NAV2, ON ITS PATH TO THE
        # GOAL, discovers it is blocked -- not when a camera notices a door somewhere in frame.
        # You cannot be "blocked at a door" you are 8 m from and have not driven toward.
        #
        # The glass safety this replaces is not lost: within NEAR_GOAL_M the camera check runs
        # exactly as before, which is the range at which driving into glass is the actual risk --
        # and it now runs again at EVERY stage boundary of the leg, not only at the start.
        b = self._perceive_blockage()
        stop, why = self._leg_should_stop(b)
        if stop:
            # LATCH THE STATUS. This early return used to leave _last_nav at whatever it was --
            # "reached", straight out of __init__/reset -- so at_goal() answered True while the
            # robot stood in front of a shut door, and fsm.py credits `trace.reached_goal` from
            # exactly that call the moment a blockage clears. A trial that never navigated could
            # be recorded as having arrived. The status is a claim about the world and it is set
            # on every path that makes one.
            self._last_nav = "blocked"
            print(f"[ros_world] leg not started: {why}")
            return NavOutcome(status="blocked", blockage=b)
        if self.dry_run:
            return NavOutcome(status="reached")
        # NAV BACKEND. `nav2` plans over the SAVED MAP; `waypoints` dead-reckons on odom.
        # Default is nav2 when a map-frame goal is usable, because odom waypoints drift
        # continuously and die outright when ranger_base restarts -- both fatal across 50 trials.
        # UTP_NAV_BACKEND=waypoints forces the old path (the A1M8-era behaviour) if Nav2 is down.
        # Only the LEG moves to the map: approach_blockage, the look ladder and the press chain
        # stay on odom, because a map->odom correction mid-press moves the target under the arm
        # (docs/NAV2.md). Nav2 gets us to the door; vision closes the last metre.
        backend = os.environ.get("UTP_NAV_BACKEND", "nav2").strip().lower()
        if backend not in ("nav2", "waypoints"):
            raise ValueError("UTP_NAV_BACKEND must be 'nav2' or 'waypoints'")
        if backend == "nav2":
            # ONE UNINTERRUPTED GOAL BY DEFAULT. Staging breaks the leg into bounded stages and
            # cancels the Nav2 goal at every boundary, so the controller re-plans from wherever it
            # stopped. Measured on hardware 2026-09-01: `nav2_goto.py button --go` as a single goal
            # ARRIVED in 21.8 s, 0.19 m off; `outside --go` arrived in 28.0 s, 0.24 m off. The same
            # legs under staging stuttered and wandered, because repeated cancel-and-replan is not
            # the same manoeuvre as one plan followed to completion.
            #
            # Staging was added to stop the robot driving blind at glass for a whole leg. That risk
            # is real, but it is paid for at the ENDS: navigate_to_goal already runs a fused
            # camera+lidar blockage check before the leg, and again when it finishes. Set
            # UTP_NAV_STAGED=1 to get mid-leg supervision back when approaching something the
            # lidar cannot see and the camera can.
            if os.environ.get("UTP_NAV_STAGED", "0").strip() in ("1", "true", "yes"):
                return self._drive_leg_staged()
            return self._drive_leg_single()
        return self._drive_leg_odom(LEG_TIMEOUT_S)

    def _drive_leg_single(self) -> NavOutcome:
        """One Nav2 goal, driven to completion -- exactly what `nav2_goto.py <name> --go` does by
        hand, which is the path proven on hardware."""
        deadline = time.monotonic() + LEG_TIMEOUT_S
        r = _ros([ROS_PY, str(REPO / "bringup" / "nav2_goto.py"), self.goal, "--go"],
                 timeout=LEG_TIMEOUT_S + 30)
        if r.returncode in (1, 2, 3, 4, 5):
            return self._nav2_unavailable(r, deadline)
        out = r.stdout or ""
        if "arrived" in out:
            self._last_nav = "reached"
            return NavOutcome(status="reached")
        if "blocked" in out:
            self._last_nav = "blocked"
            return NavOutcome(status="blocked", blockage=self._perceive_blockage())
        self._last_nav = "timeout"
        return NavOutcome(status="timeout")

    # -- the leg itself ------------------------------------------------------------------------
    def _stage_seconds(self, budget_left_s: float) -> float:
        """How long the next stage of the leg may drive for. See the LEG_STAGE_* block above."""
        near = self._nearest_ahead_m
        if near is None:
            # Nothing resolves in the forward window: an open corridor, or glass at range. The
            # least informed case gets the shortest leash.
            want = LEG_STAGE_MIN_S
        else:
            want = (near - LEG_ABORT_RANGE_M) / NAV_VX_MAX
        return max(LEG_STAGE_MIN_S, min(LEG_STAGE_MAX_S, min(want, budget_left_s)))

    def _drive_leg_staged(self) -> NavOutcome:
        """Nav2, one bounded stage at a time, perceiving from the stopped pose between stages.

        Every stage is a plain blocking subprocess.run() with a `--timeout` shorter than its own
        watchdog, so nav2_goto always ends through ITS OWN cancel-and-exit path (rc 6) rather than
        by us killing it -- the exit-code contract is unchanged and there is nothing to orphan.
        The total driving time is still bounded by LEG_TIMEOUT_S across all stages, so supervision
        cannot turn a bounded leg into an unbounded one.
        """
        deadline = time.monotonic() + LEG_TIMEOUT_S
        stages = 0
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                self._last_nav = "timeout"
                return NavOutcome(status="timeout")
            stage = self._stage_seconds(left)
            stages += 1
            print(f"[ros_world] leg stage {stages}: up to {stage:.1f} s "
                  f"(nearest ahead "
                  f"{'%.2f m' % self._nearest_ahead_m if self._nearest_ahead_m else 'unknown'})")
            r = _ros([ROS_PY, str(REPO / "bringup" / "nav2_goto.py"), self.goal, "--go",
                      "--timeout", f"{stage:.1f}"], timeout=int(stage) + 60)
            # nav2_goto's exit codes are a contract:
            #   0    real outcome (arrived / blocked) -> parse stdout
            #   6    the STAGE's timeout: the goal was cancelled, the base is stopped -> perceive
            #   2-5  cannot serve this request (odom-frame waypoint, no map name / wrong map name,
            #        Nav2 down, goal rejected)
            #   1    crashed; a dead backend must never be recorded as a navigation timeout,
            #        which is a claim about the WORLD.
            result = _nav_result(r.stdout or "")
            status = result.get("status") if result else None
            if status in ("cancelled", "error"):
                # Exit 4 is shared with no_server for compatibility, but RESULT is not lossy.
                # A cancellation is an instruction to stop, never a reason to start another driver.
                self._last_nav = "timeout"
                print(f"[ros_world] Nav2 {status}; stopping without fallback: "
                      f"{result.get('detail', '')}")
                return NavOutcome(status="timeout")
            if r.returncode in (1, 2, 3, 4, 5):
                return self._nav2_unavailable(r, deadline)
            if status == "arrived":
                self._last_nav = "reached"
                return NavOutcome(status="reached")
            if status == "blocked":
                # ABORTED is a control-plane result, not proof of an obstacle: TF, planner and
                # controller faults abort too. Only perception may promote it to physical blocked.
                b = self._perceive_blockage()
                stop, why = self._leg_should_stop(b)
                if stop:
                    self._last_nav = "blocked"
                    return NavOutcome(status="blocked", blockage=b)
                detail = (result or {}).get("detail", "Nav2 aborted")
                print(f"[ros_world] Nav2 aborted but perception did not confirm a blockage: "
                      f"{detail}; {why}")
                self._last_nav = "unreachable"
                return NavOutcome(status="unreachable")
            if status != "timeout" or r.returncode != 6:
                # 130 (interrupted) or anything unclassified. Do not keep sending stages at a
                # robot whose last stage ended in a way this code does not understand.
                print(f"[ros_world] leg stage ended with rc={r.returncode}; stopping the leg. "
                      f"{(r.stderr or '').strip()[:160]}")
                self._last_nav = "timeout"
                return NavOutcome(status="timeout")
            # STAGE BOUNDARY. The base is stopped and the goal is cancelled: look, then decide.
            b = self._perceive_blockage()
            stop, why = self._leg_should_stop(b)
            if stop:
                self._last_nav = "blocked"
                print(f"[ros_world] leg stopped after stage {stages}: {why}")
                return NavOutcome(status="blocked", blockage=b)

    def _nav2_unavailable(self, r, deadline: float) -> NavOutcome:
        """Nav2 could not serve the goal. Fall back ONLY if the odom driver could actually help.

        THE OLD FALLBACK WAS UNCONDITIONAL AND EVERY WAYPOINT ON THIS ROBOT IS MAP-FRAME
        (maps/waypoints.yaml: button, door, outside, start are all frame: map). `waypoints.py goto`
        drives on ODOM -- it takes the live odom pose and subtracts the stored coordinate -- so
        handing it a map-frame waypoint is the same two-origins-one-subtraction mistake
        _distance_to_goal was fixed for, except this one has the wheels turning: measured
        2026-09-01, an odom pose of (4.96, 2.93) against a map pose of (5.35, 5.59), 2.5 m of pure
        fiction, and the robot sets off in a direction that makes no sense from outside. The trial
        then records a `timeout`, i.e. a statement about the WORLD, for what was a configuration
        error in this process.

        So: an odom-frame goal falls back, because that is what the odom driver is for. A
        map-frame goal returns `unreachable` and prints what nav2_goto actually said -- Nav2 down,
        goal rejected, no map name, or the waypoint's map_name not matching maps/.loaded_map,
        which nav2_goto now also refuses. Naming the real problem is worth more than a leg.
        """
        # NO DEFAULT. _goal_waypoint() swallows every exception and answers None -- store
        # missing, unreadable, or being rewritten by waypoints.py save() in another process. The
        # old default was "odom", which is the value that AUTHORISES the odom fallback: an
        # unreadable store became permission to dead-reckon. CLAUDE.md's rule for gates is that
        # never-seen and stale both mean NOT PERMITTED, and this is a gate on driving.
        #
        # Today this is masked by an accident -- every waypoint on disk happens to be map-frame --
        # not by the design. An unknown frame now blocks the fallback, which costs a leg and
        # names the real fault instead of dead-reckoning toward a coordinate nobody resolved.
        wp = self._goal_waypoint()
        frame = wp.get("frame") if isinstance(wp, dict) else None
        why = (r.stderr or r.stdout or "").strip().splitlines()
        why = why[-1][:200] if why else f"rc={r.returncode}"
        # THE PERMISSIVE BRANCH REQUIRES THE PERMISSIVE VALUE, EXPLICITLY. `frame != "map"` reads
        # as "not a map waypoint, so odom is fine", but it is satisfied by None -- an unreadable
        # store, a mid-write store, a waypoint with no frame field at all. Asking "is it NOT the
        # thing I refuse" grants permission by default; asking "is it THE thing I allow" does not.
        if frame == "odom":
            print(f"[ros_world] nav2 backend unavailable (rc={r.returncode}); '{self.goal}' is an "
                  f"odom-frame waypoint, so falling back to the odom driver. {why}")
            return self._drive_leg_odom(max(1.0, deadline - time.monotonic()))
        if frame == "map":
            print(f"[ros_world] nav2 cannot serve '{self.goal}' (rc={r.returncode}) and it is a "
                  f"MAP-frame waypoint, so the odom driver cannot serve it either -- it would "
                  f"drive to a meaningless coordinate. NOT falling back. {why}")
        else:
            print(f"[ros_world] nav2 cannot serve '{self.goal}' (rc={r.returncode}) and its frame "
                  f"could not be read ({frame!r}). Refusing to guess: the odom driver would "
                  f"dead-reckon toward a coordinate nobody resolved. NOT falling back. {why}")
        self._last_nav = "unreachable"
        return NavOutcome(status="unreachable")

    def _drive_leg_odom(self, budget_s: float) -> NavOutcome:
        """The odom waypoint driver, in one call. Deliberately NOT staged.

        It does not need staging: it runs its own control loop at 20 Hz with corridor_blocked()
        live on every tick (safety/waypoint_drive.py), so unlike a blocking Nav2 action it is
        already looking while it drives. What it cannot do is see glass -- which is why it is the
        degraded path and why it is only ever reached for an odom-frame goal.
        """
        r = _ros([ROS_PY, str(REPO / "bringup" / "waypoints.py"), "goto", self.goal, "--go"],
                 timeout=int(budget_s) + 30)
        out = r.stdout or ""
        if "arrived" in out:
            self._last_nav = "reached"
            return NavOutcome(status="reached")
        if "blocked" in out:
            self._last_nav = "blocked"
            return NavOutcome(status="blocked", blockage=self._perceive_blockage())
        self._last_nav = "timeout"
        return NavOutcome(status="timeout")

    def _perceive_blockage(self) -> BlockageEvent | None:
        """A FRESH frame from where the robot is standing now, and the fused verdict on it.

        The freshness is the point. current_blockage() re-uses self._last_capture when it has one,
        which is right for the FSM (it asks about the blockage it has just been handed) and wrong
        for a navigation decision -- the old code asked the post-leg question of the PRE-leg frame
        and the pre-leg scan, i.e. it described a pose the robot had already driven away from.
        """
        self.get_observation()
        return self.current_blockage()

    def _leg_should_stop(self, b: BlockageEvent | None) -> tuple[bool, str]:
        """May the leg start / continue? Returns (stop, why), and `why` is printed either way.

        TWO INDEPENDENT REASONS TO STOP, and they answer different questions.

        1. BLOCKED AT THE GOAL -- the fused verdict says blocked and the goal is within
           NEAR_GOAL_M. This is the FSM's reasoning trigger and it keeps the operator's rule: you
           cannot be "blocked at a door" you are 8 m from and have not driven toward. Distance
           unknown counts as near, because an unreadable pose must not silently disable the check.

        2. SOMETHING SOLID IS RIGHT THERE -- the fused verdict says blocked AND the nearest return
           ahead is inside LEG_ABORT_RANGE_M. That is a statement about the robot's next metre,
           not about its goal, so the goal gate has no business vetoing it. Without this a glass
           door standing 20 m short of the goal is driven at with nothing empowered to stop it,
           which is the hole the distance gate opened when it was added.
        """
        if b is None or not b.blocked:
            return False, "not blocked"
        near = self._distance_to_goal()
        if near is None or near <= NEAR_GOAL_M:
            where = "distance to goal unreadable" if near is None else f"{near:.1f} m from goal"
            return True, (f"blocked at the goal ({where}, evidence={self._evidence or '?'}): "
                          f"{b.description[:120]}")
        ahead = self._nearest_ahead_m
        if ahead is not None and ahead <= LEG_ABORT_RANGE_M:
            return True, (f"blocked with something solid {ahead:.2f} m ahead -- inside the "
                          f"{LEG_ABORT_RANGE_M:.2f} m abort range, so the {NEAR_GOAL_M:.1f} m "
                          f"goal gate does not apply: {b.description[:120]}")
        print(f"[ros_world] blockage reported but the goal '{self.goal}' is {near:.1f} m away and "
              f"the nearest thing ahead is "
              f"{'%.2f m' % ahead if ahead is not None else 'out of lidar range'} -- driving on, "
              f"as Nav2 owns the approach")
        return False, "blocked, but not at the goal and nothing close ahead"

    # ---- back up ------------------------------------------------------------------------------
    def _back_off_from_blockage(self) -> bool:
        """Reverse to the survey standoff before anything looks around. True only if it moved.

        THE STEP THE OPERATOR ASKED FOR, AND IT IS FIRST FOR A REASON. Measured 2026-09-01, the
        robot ended a leg 0.72 m from closed glass doors -- chassis ~0.66 m off the glass. From
        there nothing downstream can work: the survey standoff is 1.40 m, the reasoner's prompt
        forbids it from naming a control it cannot see, and an ADA plate BESIDE the door swings
        FURTHER off-axis the closer the base gets (2026-08-29: from 0.54 m it was half cut off at
        the frame edge and both the VLM and the grounder missed it). Every rung of the look ladder
        then reports honestly that it found nothing, and the trial dies of a geometry problem
        wearing a perception problem's clothes.

        WHAT THIS IS NOT. Nav2 ran its own `backup` recovery on that same trial and reversed about
        four metres -- an unbounded retreat with no idea what it was retreating from, chosen by a
        planner that had already decided it was stuck. This is one reverse, to a stated distance,
        for a stated reason, and only when the measured range says it is needed.

        THREE THINGS ARE REQUIRED BEFORE IT MOVES:

        * A MEASURED RANGE. self._nearest_ahead_m comes from the fused verdict, i.e. from the
          scan.json saved beside the frame this blockage was judged on. If nothing resolved ahead
          the robot does NOT reverse: approach_blockage.reverse() has no obstacle check astern at
          all (the rear lidar sector is filtered out because it is the robot seeing itself at
          0.17 m), so reversing on a guess is the one thing that must not happen here.
        * ROOM WORTH TAKING. Inside BACKOFF_TRIGGER_SLACK_M of the standoff there is nothing to
          gain and a twitch to lose.
        * A HEADING WORTH REVERSING ALONG. recentre_view() first, so the reverse runs back down
          the approach heading rather than along whatever sweep bearing the ladder left the base
          at. On the first blockage that is a no-op; after a re-navigation it is not.
        """
        # CLEARANCE, NOT RANGE. A standoff is an along-track distance, and reversing from a range
        # under-reverses by however far the nearest return sits off the axis. Measured on the real
        # closed-door capture: range 0.701 m, clearance 0.578 m -- 12.2 cm short, spent toward the
        # door. self._nearest_ahead_m stays the number we PRINT, because it is what the fused
        # verdict reported and what a reader will compare against the log.
        near = _clearance_ahead_m(_read_scan(self._last_capture))
        if near is None:
            near = self._nearest_ahead_m          # fused verdict's range: worse, but not nothing
        if near is None:
            print("[ros_world] blocked, but nothing resolves in the forward lidar window -- NOT "
                  "reversing: there is no obstacle check behind this robot and a reverse on a "
                  "guess is not a recovery.")
            return False
        if near >= BACKOFF_STANDOFF_M - BACKOFF_TRIGGER_SLACK_M:
            print(f"[ros_world] blocked at {near:.2f} m clearance, already at or beyond the "
                  f"{BACKOFF_STANDOFF_M:.2f} m survey standoff -- no back-up needed")
            return False
        want = min(MAX_BACKOFF_M, BACKOFF_STANDOFF_M - near)
        self.recentre_view()
        if self.dry_run:
            self._looks.append({"rung": "back_off", "from_m": near, "want_m": want,
                                "moved": False, "note": "dry run"})
            print(f"[ros_world] DRY RUN: would back up {want:.2f} m, from {near:.2f} m to the "
                  f"{BACKOFF_STANDOFF_M:.2f} m survey standoff")
            return False
        r = _ros([ROS_PY, str(REPO / "bringup" / "approach_blockage.py"),
                  "--back", f"{want:.2f}"], timeout=90)
        # approach_blockage exits 0 only when reverse() reports the base actually moved (it
        # returns gone > 0.05 even on its own 40 s timeout), so the exit code IS the moved test.
        moved = r.returncode == 0
        detail = (r.stdout or r.stderr or "").strip().splitlines()[-1:]
        self._looks.append({"rung": "back_off", "from_m": near, "want_m": want,
                            "moved": moved, "detail": detail})
        print(f"[ros_world] back up {want:.2f} m from {near:.2f} m  moved={moved}  "
              f"{detail[0][:120] if detail else ''}")
        if moved:
            self._backed_off = True
            # RE-ARM THE LOOK LADDER. This is the bug that would have made the back-up pointless.
            # _scan_i and _widened are cleared only by reset(), which fsm.py calls once per TRIAL
            # -- so on the second and later blockages of a trial (press fails, leg is re-driven,
            # the door blocks again) the cursor was already past the end of SCAN_BEARINGS_DEG.
            # scan_view() would recentre and return False without turning, widen_view() would
            # return False without moving, and the survey would be inert from a pose the robot had
            # never surveyed from. The base has physically moved to a new viewpoint; every bearing
            # is unvisited again, and the ladder must say so.
            self._scan_i = 0
            self._widened = False
            # _scan_offset is already 0 from recentre_view() above, and _looks is NOT cleared:
            # it is the audit trail behind last_look_info() and erasing it would hide the
            # back-up itself from the trial record.
        return moved

    def approach_blockage(self) -> None:
        """Close the gap to the obstruction BEFORE perception and the arm reach.

        fsm.py:421 calls this when the world provides it. isaac_world does; this did not, so the
        hasattr check silently skipped it on hardware and grounding ran from wherever navigation
        stopped. The sim version states the cost: "Depth is invalid/noisy at 8-10 m -- grounding
        from there lifts the target to a garbage 3D point and the arm IKs out of reach."

        Measured here 2026-08-29 at ~9 m: the camera read "closed double glass doors" correctly,
        and the reasoner then abstained -- "I cannot see a button or a card reader on or near the
        glass double doors." Correct, and useless. From 9 m the plate is a few pixels.

        This also removes the last operator-supplied advantage in the hardware trial. Without it,
        getting within arm reach needs a PRE-RECORDED `button` waypoint -- a human pointing at the
        control -- which hollows out the grounding claim entirely. Approaching geometrically needs
        no waypoint and no map, and it approaches the BLOCKAGE, not the button: what the control is
        and where it sits stays the grounder's job, from a frame taken here.

        AND SOMETIMES THE GAP IS ALREADY NEGATIVE. fsm.py calls this on the blocked path only, and
        the blocked path now ends with the robot right against the obstruction: 0.72 m of forward
        range at the glass doors, measured 2026-09-01. The operator's sequence is BACK UP, look
        around, see the button, go to it, press it, come back out -- so the first thing that
        happens here is the back-up, explicitly and by name, and the forward approach runs only
        when there is actually a gap left to close.
        """
        if self._back_off_from_blockage():
            # Do NOT then creep forward again. Reversing to the survey standoff and immediately
            # re-approaching it is the stop/back/forward oscillation fsm.py warns about, and from
            # outside it reads as a robot that cannot make up its mind.
            return
        # Stop at the SURVEY standoff, not the press standoff. approach_blockage exists so the
        # reasoner can SEE the blockage; closing all the way to 0.55 m puts a side-mounted plate
        # at the frame edge and the reasoner -- forbidden by its prompt from naming a control it
        # cannot see -- abstains. Driving the last metre is face_target's job, and only once
        # there is a grounded target to drive at.
        from safety.reach_envelope import SURVEY_STANDOFF_M
        args = [ROS_PY, str(REPO / "bringup" / "approach_blockage.py"),
                "--stop-at", f"{SURVEY_STANDOFF_M:.2f}",
                "--dry-run" if self.dry_run else "--go"]
        r = _ros(args, timeout=150)
        tail = (r.stdout or r.stderr or "").strip().splitlines()
        if tail:
            print(f"[ros_world] {tail[-1][:160]}")

    # ---- look-around ladder -----------------------------------------------------------------
    # fsm.py walks (strafe_view, scan_view, widen_view) when the reasoner abstains because the
    # target is not visible, taking the FIRST that returns True, and each rung must return True
    # ONLY if the base actually moved so a wedged base falls through to the next.
    #
    # WHY THIS IS THE BLOCKER ON HARDWARE. Measured 2026-08-29 at the FAU atrium doors, twice,
    # with the base already approached to ~0.97 m:
    #   "I can see the closed glass double doors labeled 'The Atrium', but I cannot see a button,
    #    card reader, or handle to interact with them. I NEED TO LOOK CLOSER to find the control
    #    mechanism."   -> planned_action none, failure_detail target_offscreen
    # The D435 is ~69 deg wide and the ADA plate is on the wall BESIDE the doors, so from a pose
    # square to them it is simply not in the picture. The reasoner was right to abstain; there was
    # nothing in the frame to reason about. With no rung implemented the FSM's survey was inert
    # and every trial ended there.
    # isaac_world.SCAN_BEARINGS_RAD is (+/-45, +/-22.5). Those are right for the SIM, where the
    # button sits on or near the door frame, close to the door normal. THIS BUILDING IS NOT THAT.
    #
    # MEASURED 2026-08-29 at the FAU atrium doors. Surveying from 1.40 m and turning +45 deg, the
    # ADA plate was still HALF CUT OFF at the extreme left edge of the frame. Working back from
    # the D435's ~69 deg horizontal FOV, that puts it about +79 deg from the door-facing heading:
    # the plate is on the PERPENDICULAR wall beside the doorway, not on the door frame. A +-45 deg
    # sweep cannot reach it, which is why three trials abstained with the reasoner correctly
    # reporting it could not see a control.
    #
    # Wide bearings FIRST, because that is where the evidence says the control actually is, and
    # because fsm.py takes the first rung that returns True with only max_recovery_attempts of
    # them -- so a bearing late in the list is a bearing that never gets tried.
    SCAN_BEARINGS_DEG = (80.0, -80.0, 45.0, -45.0)

    def strafe_view(self) -> bool:
        """The VLM-STEERED look. fsm.py tries this rung first; True only if the base moved.

        On this robot "strafe" is not a sideways slide -- it is the reasoner saying where to look
        (params.look = left | right | closer | back, set by SteeredReasoner when it abstains) and
        the world carrying that out as ONE bounded motion from safety/look_policy.py. No hint, or a
        hint that cannot be honoured, returns False and the FSM falls through to the blind sweep
        in scan_view -- which is all that heuristic and passive ever get, since their reasoners
        emit no hints. See look_policy.py for why a VLM hint is reasoning, not leaked ground truth.
        """
        from safety.look_policy import decide_look
        hint = self.hints.take() if self.hints is not None else None
        if hint is None:
            return False
        mv = decide_look(hint, self._scan_offset, None)
        if mv is None:
            self._looks.append({"rung": "strafe_view", "hint": hint, "moved": False,
                                "note": "hint refused by policy (cap or no room)"})
            return False
        if self.dry_run:
            self._looks.append({"rung": "strafe_view", "hint": hint, "moved": False,
                                "note": "dry run"})
            return False

        moved = False
        detail = ""
        if mv.kind == "turn":
            r = _ros([ROS_PY, str(REPO / "bringup" / "turn_by.py"), "--deg", f"{mv.amount:.1f}"],
                     timeout=60)
            moved = r.returncode == 0
            if moved:
                self._scan_offset += mv.amount
            detail = (r.stdout or r.stderr or "").strip().splitlines()[-1:]
        elif mv.kind == "closer":
            r = _ros([ROS_PY, str(REPO / "bringup" / "approach_blockage.py"),
                      "--stop-at", f"{mv.amount:.2f}"], timeout=150)
            out = (r.stdout or r.stderr or "").strip()
            moved = r.returncode == 0 and "after 0.00 m" not in out
            detail = out.splitlines()[-1:]
        elif mv.kind == "back":
            moved = self.widen_view(mv.amount)
            detail = ["via widen_view"]
        self._looks.append({"rung": "strafe_view", "hint": hint, "kind": mv.kind,
                            "amount": mv.amount, "moved": moved, "detail": detail})
        print(f"[ros_world] strafe_view (steered) -> {hint}: {mv.kind} {mv.amount:+.2f}  "
              f"moved={moved}")
        return moved

    def scan_view(self) -> bool:
        """Rotate in place to the next unvisited bearing. True only if the base actually turned.

        Rotation, not translation, because the plate sits BESIDE the door: sweeping the camera
        across the flanking wall is what brings it into frame, and backing off (widen_view) only
        makes an already-small target smaller. The sweep is bounded and returns to the starting
        heading when exhausted, so the run ends facing the obstruction rather than somewhere
        arbitrary.
        """
        if self._scan_i >= len(self.SCAN_BEARINGS_DEG):
            # isaac_world._end_scan: "the original heading is restored so the next nav leg does
            # not start pointing somewhere arbitrary". Trial 4 left the robot at -80 deg because
            # this was missing. Only on EXHAUSTION -- a sweep that ends in a commitment leaves the
            # robot facing the control, which is what face_target then drives from.
            self.recentre_view()
            return False
        want = self.SCAN_BEARINGS_DEG[self._scan_i]
        self._scan_i += 1
        delta = want - self._scan_offset
        if self.dry_run:
            self._scan_offset = want
            self._looks.append({"rung": "scan_view", "bearing_deg": want, "moved": False,
                                "note": "dry run"})
            return False
        # SCAN_BUDGET_S = 12.0 in the sim: a hard ceiling PER bearing, so a wedged base costs
        # seconds rather than the trial budget. turn_by has its own 60 s internal timeout; this
        # bounds the subprocess as well.
        r = _ros([ROS_PY, str(REPO / "bringup" / "turn_by.py"), "--deg", f"{delta:.1f}"],
                 timeout=30)
        moved = r.returncode == 0
        if moved:
            self._scan_offset = want
        self._looks.append({"rung": "scan_view", "bearing_deg": want, "moved": moved,
                            "detail": (r.stdout or r.stderr or "").strip().splitlines()[-1:]})
        print(f"[ros_world] scan_view -> {want:+.0f} deg  moved={moved}")
        return moved

    def widen_view(self, delta_m: float = 0.35) -> bool:
        """Back the base off so the camera frames more of the blockage. True only if it moved.

        fsm.py's third rung, and on this robot a real one rather than a fallback. The reasoner
        cannot name a control it cannot see (its prompt forbids it, deliberately -- at temperature
        0 it once invented an ADA button on a sealed door), and an ADA plate BESIDE a door goes
        FURTHER off-axis the closer the base gets. Measured 2026-08-29: from 0.54 m the plate was
        half out of frame and the VLM and the grounder both missed it; from further back the same
        plate was 81x88 px and grounded at 0.489.

        Bounded to one use per blockage: reversing repeatedly walks the robot away from its own
        mission, and the sweep is the better tool once the distance is right.
        """
        if self._widened or self.dry_run:
            return False
        self._widened = True
        r = _ros([ROS_PY, str(REPO / "bringup" / "approach_blockage.py"),
                  "--back", f"{delta_m:.2f}"], timeout=90)
        moved = r.returncode == 0
        self._looks.append({"rung": "widen_view", "delta_m": delta_m, "moved": moved,
                            "detail": (r.stdout or r.stderr or "").strip().splitlines()[-1:]})
        print(f"[ros_world] widen_view -> back {delta_m:.2f} m  moved={moved}")
        return moved

    def recentre_view(self) -> None:
        """Undo the sweep. Called after a survey so the robot ends facing the obstruction."""
        if self._scan_offset and not self.dry_run:
            _ros([ROS_PY, str(REPO / "bringup" / "turn_by.py"),
                  "--deg", f"{-self._scan_offset:.1f}"], timeout=90)
        self._scan_offset = 0.0

    def last_look_info(self) -> dict:
        """What the survey actually did, for row["look_around"] in the trial record.

        A recovery that CLAIMS a new viewpoint must be checkable against the record rather than
        taken on faith -- isaac_world makes the same point about its widen-that-did-not-move.
        """
        return {"rungs_taken": len(self._looks), "offset_deg": self._scan_offset,
                "bearings": list(self.SCAN_BEARINGS_DEG), "history": self._looks[-6:],
                # The back-up is part of the survey's provenance: a sweep run from 1.40 m and a
                # sweep run from 0.72 m are different experiments, and the record has to say which
                # one produced the answer. `bearings_left` makes an inert ladder visible as an
                # inert ladder instead of as a robot that looked and saw nothing.
                "backed_off": self._backed_off,
                "nearest_ahead_m": self._nearest_ahead_m,
                "bearings_left": max(0, len(self.SCAN_BEARINGS_DEG) - self._scan_i)}

    def current_blockage(self) -> BlockageEvent | None:
        """What is in the way, from the CAMERA AND THE LIDAR together. Describes only.

        The VLM half (bringup/ask_blockage.py) is unchanged and still deliberately restricted to
        describing -- the reasoner's choice of action is the thing being measured, so nothing here
        may propose one. What changed is that its answer is no longer the verdict on its own.

        MEASURED 2026-09-01, captures/trial_ours_001, the robot 0.72 m from CLOSED GLASS DOORS:

            camera : blocked=False, "an open walkway with pillars"
            lidar  : 85 returns inside +-20 deg forward, nearest 0.72 m

        The camera was describing the picture accurately; the picture is of the corridor THROUGH
        the glass. Acting on it alone, the robot drove on and the operator stopped it by hand.
        Note that this is the MIRROR of the 2026-08-26 failure recorded in navigate_to_goal's
        docstring, where the lidar read 6.98 m through the same kind of doors and the camera was
        the only sensor that saw them. Both are true and they are not in conflict: a glass return
        depends on range and incidence, so the lidar sees these doors from 0.72 m and not from
        7 m. That is exactly why the verdict is fused rather than assigned to a favourite sensor.

        The scan comes from scan.json in the capture directory -- the same instant as the frame,
        already on disk, written by grab_frame.py. See _read_scan for why it is read rather than
        subscribed to.
        """
        if self._last_capture is None:
            self.get_observation()
        if self._last_capture is None:
            self._nearest_ahead_m = None
            self._evidence = "neither"
            return BlockageEvent(blocked=True, kind="",
                                 description="path blocked; no camera frame available")
        # ask_blockage owns the camera+saved-scan fusion boundary.  Do not read scan.json and
        # fuse its already-fused result again here: besides being redundant, doing so turns the
        # fused camera/lidar verdict into the "camera" input of a second fusion and corrupts the
        # evidence provenance.  RosWorld only translates the returned transport dict into the
        # pipeline's BlockageEvent.
        d = ask_blockage(self._last_capture)
        if "evidence" not in d:
            print(f"[ros_world] no scan.json in {self._last_capture} -- this verdict is "
                  f"CAMERA-ONLY, which is the configuration that missed the glass doors on "
                  f"2026-09-01. Is grab_frame.py seeing {os.environ.get('UTP_SCAN_TOPIC', '/scan')}?")
        self._nearest_ahead_m = d.get("nearest_ahead_m")
        self._evidence = str(d.get("evidence", "camera") or "camera")
        self._blockage = BlockageEvent(blocked=bool(d.get("blocked", True)),
                                       kind=d.get("kind", ""),
                                       description=d.get("description", ""))
        print(f"[ros_world] blockage: blocked={self._blockage.blocked} "
              f"evidence={self._evidence or '?'} "
              f"nearest_ahead="
              f"{'%.2f m' % self._nearest_ahead_m if self._nearest_ahead_m else 'n/a'} "
              f"kind={self._blockage.kind!r} desc={self._blockage.description[:100]!r}")
        return self._blockage

    # ------------------------------------------------------------------- act
    # Below this the FSM's grounding is treated as "seen something, not yet localised": the base
    # closes in and grounds again before anything is driven at. From the 1.40 m survey standoff the
    # plate is ~50 px and GDINO ranked two wall signs above it (0.405, 0.370, plate not in the top
    # five); from ~1.0 m it was 81x88 px and won at 0.489. The threshold sits between those.
    WEAK_SCORE = 0.42
    WEAK_SIDE_PX = 60
    CLOSE_FOR_GROUNDING_M = 0.85

    def _reground(self, query: str, label: str):
        """Fresh frame from where the robot stands NOW, grounded for ``query`` and vetoed.

        Returns the capture dir holding detection.json, or None. None means: do not drive at it and
        do not reach for it. Every exit is fail-closed.
        """
        self.get_observation()
        cap = self._last_capture
        if cap is None:
            print(f"[ros_world] {label}: no frame")
            return None
        r = subprocess.run([sys.executable, str(REPO / "bringup" / "detect_frame.py"), str(cap),
                            "--query", query], capture_output=True, text=True, timeout=300)
        det = Path(cap) / "detection.json"
        if r.returncode != 0 or not det.exists():
            print(f"[ros_world] {label}: grounder found nothing for {query!r}")
            return None
        d = json.loads(det.read_text())
        if not d.get("point3d_cam_m"):
            print(f"[ros_world] {label}: no 3D point")
            return None
        # NEVER PRESS A FIRE ALARM. The plate and the pull station are ~10 cm apart on this wall,
        # and the detector has returned the alarm as "the accessible door push button" before.
        v = subprocess.run([sys.executable, str(REPO / "bringup" / "check_press_safe.py"),
                            str(cap)], capture_output=True, text=True, timeout=300)
        if v.returncode != 0:
            tail = (v.stdout or v.stderr or "").strip().splitlines()
            print(f"[ros_world] {label}: REFUSED -- {tail[0][:160] if tail else 'veto'}")
            return None
        b = d.get("bbox_px") or [0, 0, 0, 0]
        print(f"[ros_world] {label}: score {d.get('score', 0):.3f}  "
              f"box {int(b[2]-b[0])}x{int(b[3]-b[1])} px  "
              f"range {float(d['point3d_cam_m'][2]):.2f} m")
        return cap

    def act(self, plan: Plan, detection: Detection | None) -> ExecResult:
        """Drive to the grounded control, ground it again from there, press it, stow.

        This is isaac_world.act() on hardware: "POSITION the base within arm reach of the target,
        then let the arm IK make the final reach." Two additions the sim does not need:

        CLOSE IN BEFORE TRUSTING THE GROUNDING. The sim grounds fine at its survey standoff. Here
        the plate is ~50 px at 1.40 m and loses to the wall signs; at ~1.0 m it wins. A weak or
        small first detection means "the reasoner saw a control, the detector has not localised
        it yet", so the base steps toward the wall it is facing and grounds again.

        RE-GROUND FROM THE PRESS POSE. An earlier version of this function carried a comment
        promising exactly that and then handed the arm the ORIGINAL 3D point, measured before the
        base moved -- the sim's own recorded failure ("base yawed 0.69 rad between observing and
        pressing ... the arm reached confidently at blank wall"), reproduced by the comment that
        cited it. The point the arm aims at is now always measured from where the arm is.
        """
        if plan.abstain or plan.action_type in ("", "none"):
            return ExecResult(False, "reasoner abstained; nothing executed")
        if detection is None or not getattr(detection, "point3d", None):
            # Refusing here is the point: on 2026-08-25 a hardcoded fallback target sent the arm
            # 223 mm wide with total confidence. No 3D point means no motion.
            return ExecResult(False, "no grounded 3D point; refusing to move the arm")
        if self._last_capture is None:
            return ExecResult(False, "no capture to act from")
        query = (plan.target_description or "").strip() or "the accessible door push button"

        # 1. IS THE FIRST GROUNDING GOOD ENOUGH TO DRIVE AT?
        x0, y0, x1, y1 = detection.bbox
        weak = (float(detection.score or 0.0) < self.WEAK_SCORE
                or min(x1 - x0, y1 - y0) < self.WEAK_SIDE_PX)
        cap = self._last_capture
        if weak:
            print(f"[ros_world] first grounding weak (score {detection.score:.3f}, "
                  f"{int(x1-x0)}x{int(y1-y0)} px) -- closing in before trusting it")
            if not self.dry_run:
                r = _ros([ROS_PY, str(REPO / "bringup" / "approach_blockage.py"),
                          "--stop-at", f"{self.CLOSE_FOR_GROUNDING_M:.2f}"], timeout=150)
                ln = (r.stdout or r.stderr or "").strip().splitlines()
                if ln:
                    print(f"[ros_world] {ln[-1]}")
            cap = self._reground(query, "re-grounded closer")
            if cap is None:
                return ExecResult(False, "control not localised after closing in; arm not moved")
        else:
            v = subprocess.run([sys.executable, str(REPO / "bringup" / "check_press_safe.py"),
                                str(cap)], capture_output=True, text=True, timeout=300)
            if v.returncode != 0:
                return ExecResult(False, "refused: the grounded target reads as a fire alarm / "
                                         "emergency control")

        # 2. POSITION THE BASE -- isaac_world._approach_press_pose, ported (face_target.py): face
        #    the grounded control, step in to PRESS_STANDOFF, accept dist <= standoff + 0.18 and
        #    yaw_err <= 0.25, target-relative on odometry and never on the lidar.
        pos = _ros([ROS_PY, str(REPO / "bringup" / "face_target.py"), str(cap)]
                   + (["--dry-run"] if self.dry_run else []), timeout=180)
        for ln in (pos.stdout or "").strip().splitlines()[-3:]:
            print(f"[ros_world] {ln}")
        if pos.returncode != 0 and not self.dry_run:
            tail = (pos.stderr or pos.stdout or "").strip().splitlines()
            return ExecResult(False, "could not position within arm reach: "
                                     + (tail[-1][:180] if tail else "face_target failed"))

        # SIM: the arm is the trial server's IK, driven by sim/sim_press.py over /arm_reach.
        # Same grab -> ground -> refuse-without-a-3D-point as hardware, then the server presses
        # and physically opens the door -- so a wrong point FAILS here exactly as it would there.
        # No READY/STOW: those are xArm SDK calls with no counterpart in the sim.
        if os.environ.get("UTP_SIM") == "1":
            r = _ros([ROS_PY, str(REPO / "sim" / "sim_press.py"), "--query", query]
                     + (["--dry-run"] if self.dry_run else []), timeout=300)
            tail = (r.stdout or r.stderr or "").strip().splitlines()
            for ln in tail[-4:]:
                print(f"[ros_world] {ln}")
            return ExecResult(r.returncode == 0,
                              tail[-1][:200] if tail else ("ok" if r.returncode == 0 else "failed"))

        # 3. RE-GROUND FROM THE PRESS POSE. The arm aims at a point measured from HERE.
        #    HARDWARE ORDER MATTERS: raise the arm to READY first -- with it STOWED the folded arm
        #    fills the lower-centre of the mast camera's view, exactly where a plate 0.7 m ahead
        #    sits (measured 2026-08-29: grounder returned the fire alarm, veto refused it; with
        #    the arm up the same camera saw the plate plainly). press_run.sh has the same order.
        ARM_PY = str(REPO / ".venv-arm" / "bin" / "python")
        if not self.dry_run:
            rr = _ros([ARM_PY, str(REPO / "bringup" / "stow_arm.py"), "--ready", "--go"],
                      timeout=180)
            if rr.returncode != 0:
                return ExecResult(False, "could not reach the press-ready wrist pose; not approaching")
            cap = self._reground(query, "re-grounded at the press pose, arm up")
            if cap is None:
                _ros([ARM_PY, str(REPO / "bringup" / "stow_arm.py"), "--go"], timeout=180)
                return ExecResult(False, "control not localised from the press pose; arm not moved")

        # 4. READY -> REACH -> STOW, the three steps press_run.sh does. Stow and press are
        #    different wrist orientations ("approaching straight out of stow reaches at the stow
        #    angle and skids off a round button"), and without the trailing stow every later leg
        #    is refused with arm_not_stowed, which reads as a navigation fault after a success.
        mode = "--dry-run" if self.dry_run else "--go"
        r = _ros([ROS_PY, str(REPO / "bringup" / "approach_target.py"),
                  "--capture", str(cap), "--min-standoff", "60", mode], timeout=300)
        ok = r.returncode == 0

        if not self.dry_run:
            sr = _ros([ARM_PY, str(REPO / "bringup" / "stow_arm.py"), "--go"], timeout=180)
            if sr.returncode != 0:
                print("[ros_world] STOW FAILED after the press -- the base will refuse to move.")
        tail = (r.stdout or r.stderr or "").strip().splitlines()
        return ExecResult(ok, tail[-1][:200] if tail else ("ok" if ok else "failed"))

    # ----------------------------------------------------------------- state
    def get_scene_state(self) -> dict:
        p = self._pose()
        return {"robot_x": p.x, "robot_y": p.y, "robot_yaw": p.yaw,
                "last_nav": self._last_nav,
                "blockage_kind": self._blockage.kind if self._blockage else ""}

    def at_goal(self) -> bool:
        return self._last_nav == "reached"

    # ------------------------------------- ground truth: absent on hardware
    def gt_expected_actions(self) -> list:
        return []

    def gt_current_target(self):
        return None

    def gt_interactions_required(self) -> int:
        return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", default="")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--selftest", action="store_true",
                    help="check this class satisfies the World protocol; touches no hardware")
    a = ap.parse_args()

    if a.selftest:
        from utp.pipeline.interfaces import World
        w = RosWorld()
        missing = [m for m in ("reset", "get_observation", "navigate_to_goal", "current_blockage",
                               "act", "get_scene_state", "at_goal", "gt_expected_actions",
                               "gt_current_target", "gt_interactions_required")
                   if not callable(getattr(w, m, None))]
        print(f"World protocol methods missing: {missing or 'none'}")
        print(f"isinstance(RosWorld(), World) = {isinstance(w, World)}")
        return 1 if missing else 0

    w = RosWorld(goal=a.goal, dry_run=a.dry_run)
    w.reset()
    print("observation ..."); obs = w.get_observation()
    print(f"  rgb={None if obs.rgb is None else obs.rgb.shape}  pose=({obs.robot_pose.x:.2f},"
          f" {obs.robot_pose.y:.2f}, {math.degrees(obs.robot_pose.yaw):.0f} deg)")
    print("blockage ..."); b = w.current_blockage()
    print(f"  blocked={b.blocked} kind={b.kind!r} desc={b.description!r}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
