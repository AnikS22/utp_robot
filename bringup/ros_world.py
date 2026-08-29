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
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SIM = Path.home() / "unlocking-the-path"
sys.path.insert(0, str(SIM))
sys.path.insert(0, str(REPO / "bringup"))

import numpy as np  # noqa: E402

from utp.pipeline.types import (BlockageEvent, Detection, ExecResult, NavOutcome,  # noqa: E402
                                Observation, Plan, Pose)
from ask_blockage import ask as ask_blockage  # noqa: E402

VENV = SIM / "env" / ".venv" / "bin" / "python"
ROS_PY = "python3"
LEG_TIMEOUT_S = 180


def _ros(args: list[str], timeout: int = 120) -> subprocess.CompletedProcess:
    """Run a ROS-side tool. env.sh is sourced in a login shell because rclpy only exists there."""
    cmd = f"source {REPO}/bringup/env.sh >/dev/null 2>&1 && " + " ".join(
        f"'{a}'" for a in args)
    return subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=timeout)


class RosWorld:
    """The World protocol, on hardware. Construct with the goal waypoint name."""

    # NOT world_kind = "ros". fsm.py stamps TrialRecord with getattr(world, "world_kind",
    # "mock"), but schema.py validates that field to mock|graph|isaac and raises on anything
    # else, so a hardware trial is logged as world="mock" -- a hardware result filed as a
    # simulation, which is the worst kind of wrong in a results table because nothing about it
    # looks broken. Adding "ros" to the schema is a one-line change in the PIPELINE repo and is
    # not mine to make. bringup/run_trial.py therefore stamps its own record instead, and the
    # `world` field inside a hardware TrialRecord must be read as meaningless, not as "mock".

    def __init__(self, goal: str = "", dry_run: bool = True, capture_prefix: str = "fsm") -> None:
        self.goal = goal
        self.dry_run = dry_run
        self.capture_prefix = capture_prefix
        self._scan_i = 0            # next bearing in SCAN_BEARINGS_DEG
        self._scan_offset = 0.0     # degrees currently turned away from the approach heading
        self._looks: list = []      # audit trail for last_look_info()
        self._n = 0
        self._last_nav = "reached"
        self._last_capture: Path | None = None
        self._blockage: BlockageEvent | None = None

    # -------------------------------------------------------------- lifecycle
    def reset(self, scene_type: str = "", seed: int = 0) -> None:
        self._n = 0
        self._last_nav = "reached"
        self._last_capture = None
        self._blockage = None

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
        """
        if not self.goal:
            return NavOutcome(status="reached")
        # Camera first -- this is the one that sees glass.
        self.get_observation()
        b = self.current_blockage()
        if b is not None and b.blocked:
            return NavOutcome(status="blocked", blockage=b)
        if self.dry_run:
            return NavOutcome(status="reached")
        r = _ros([ROS_PY, str(REPO / "bringup" / "waypoints.py"), "goto", self.goal, "--go"],
                 timeout=LEG_TIMEOUT_S + 30)
        out = r.stdout
        if "arrived" in out:
            self._last_nav = "reached"
            return NavOutcome(status="reached")
        if "blocked" in out:
            # The lidar backstop fired mid-leg: something opaque the camera check missed.
            self._last_nav = "blocked"
            return NavOutcome(status="blocked", blockage=self.current_blockage())
        self._last_nav = "timeout"
        return NavOutcome(status="timeout")

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
        """
        args = [ROS_PY, str(REPO / "bringup" / "approach_blockage.py"),
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
    # isaac_world.SCAN_BEARINGS_RAD = (0.785, -0.785, 0.393, -0.393) -- "+/-45 deg then
    # +/-22.5 deg, in call order". Same values, same order: wide first because a flank-mounted
    # plate is usually well off axis, then narrow to catch anything just past the frame edge.
    SCAN_BEARINGS_DEG = (45.0, -45.0, 22.5, -22.5)

    def scan_view(self) -> bool:
        """Rotate in place to the next unvisited bearing. True only if the base actually turned.

        Rotation, not translation, because the plate sits BESIDE the door: sweeping the camera
        across the flanking wall is what brings it into frame, and backing off (widen_view) only
        makes an already-small target smaller. The sweep is bounded and returns to the starting
        heading when exhausted, so the run ends facing the obstruction rather than somewhere
        arbitrary.
        """
        if self._scan_i >= len(self.SCAN_BEARINGS_DEG):
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
                "bearings": list(self.SCAN_BEARINGS_DEG), "history": self._looks[-6:]}

    def current_blockage(self) -> BlockageEvent | None:
        """Ask the VLM what is in the way. Describes only; the reasoner decides the action."""
        if self._last_capture is None:
            self.get_observation()
        if self._last_capture is None:
            return BlockageEvent(blocked=True, kind="",
                                 description="path blocked; no camera frame available")
        d = ask_blockage(self._last_capture)
        self._blockage = BlockageEvent(blocked=bool(d.get("blocked", True)),
                                       kind=d.get("kind", ""),
                                       description=d.get("description", ""))
        return self._blockage

    # ------------------------------------------------------------------- act
    def act(self, plan: Plan, detection: Detection | None) -> ExecResult:
        """Reach the grounded target with the arm. Only grounded actions can execute."""
        if plan.abstain or plan.action_type in ("", "none"):
            return ExecResult(False, "reasoner abstained; nothing executed")
        if detection is None or not getattr(detection, "point3d", None):
            # Refusing here is the point: on 2026-08-25 a hardcoded fallback target sent the arm
            # 223 mm wide with total confidence. No 3D point means no motion.
            return ExecResult(False, "no grounded 3D point; refusing to move the arm")
        if self._last_capture is None:
            return ExecResult(False, "no capture to act from")
        x, y, z = detection.point3d

        # POSITION THE BASE FIRST -- isaac_world.act() has done this since it was written:
        #   "POSITION the base within arm reach of the target, then let the arm IK make the final
        #    reach. The target sits on/at an obstacle (a button on the door / wall), so the exact
        #    press standoff is usually unreachable by a strict drive..."
        # via _approach_press_pose, which FACES the grounded button and steps in to
        # PRESS_STANDOFF_X. This went straight from detection to the arm, so on hardware the base
        # never repositioned at all: 2026-08-29 the grounder found the ADA plate correctly at
        # 1.97 m, the base ended 1.23 m away, and a 0.88 m arm was commanded at it and faulted
        # with ControllerError 21.
        #
        # bringup/face_target.py is that algorithm on this robot -- same constants, same
        # acceptance test (dist <= standoff + 0.18, yaw_err <= 0.25), same stall evidence, same
        # budget, and target-relative on odometry rather than on the lidar for the reason the sim
        # states: the self-hit filter cannot guard the close range, so the standoff is what keeps
        # the chassis clear.
        pos = _ros([ROS_PY, str(REPO / "bringup" / "face_target.py"),
                    str(self._last_capture)] + (["--dry-run"] if self.dry_run else []),
                   timeout=180)
        for ln in (pos.stdout or "").strip().splitlines()[-3:]:
            print(f"[ros_world] {ln}")
        if pos.returncode != 0 and not self.dry_run:
            tail = (pos.stderr or pos.stdout or "").strip().splitlines()
            return ExecResult(False, "could not position within arm reach: "
                                     + (tail[-1][:180] if tail else "face_target failed"))

        # RE-GROUND FROM THE NEW POSE. The 3D point above was measured from where the robot WAS.
        # isaac_world lifts it with the OBSERVATION pose for the same reason -- it records the
        # base yawing 0.69 rad between observing and pressing, swinging a 1 m target half a metre,
        # so the arm reached at blank wall and the trial was booked as a GROUNDING failure though
        # the detector had been right. Hardware can do better than re-projecting: take a fresh
        # frame from where the arm actually is.
        # READY -> REACH -> STOW, the three steps press_run.sh does. act() did only the middle
        # one, and both ends matter:
        #
        #   READY. "Stow and press are different orientations: stow folds the wrist to J5=90 so
        #   the tool points up out of the way; a press needs it pointing AT the wall (J5 ~ 2.5).
        #   approach_target.py holds whatever orientation the arm starts in, so approaching
        #   straight out of stow reaches at the stow angle and skids off a round button."
        #   The pose is the OPERATOR'S, captured with stow_arm.py --save-ready, not invented here.
        #
        #   STOW. approach_target retreats to its START pose -- wherever the arm happened to be --
        #   not to stow. config/safety.yaml gates ALL base motion on measured joint angles, so
        #   without this the FSM's next navigate is refused with blocked_by="arm_not_stowed" and
        #   the failure looks like a navigation fault immediately after a SUCCESSFUL press.
        ARM_PY = str(REPO / ".venv-arm" / "bin" / "python")
        mode = "--dry-run" if self.dry_run else "--go"
        rr = _ros([ARM_PY, str(REPO / "bringup" / "stow_arm.py"), "--ready"]
                  + ([] if self.dry_run else ["--go"]), timeout=180)
        if rr.returncode != 0 and not self.dry_run:
            return ExecResult(False, "could not reach the press-ready wrist pose; not approaching")

        args = [ROS_PY, str(REPO / "bringup" / "approach_target.py"),
                "--capture", str(self._last_capture),
                "--target-cam", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                "--min-standoff", "60", mode]
        r = _ros(args, timeout=300)
        ok = r.returncode == 0

        if not self.dry_run:
            sr = _ros([ARM_PY, str(REPO / "bringup" / "stow_arm.py"), "--go"], timeout=180)
            if sr.returncode != 0:
                # Say so loudly: the base is now gated off and every later leg will be refused
                # with arm_not_stowed, which reads as a navigation problem.
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
