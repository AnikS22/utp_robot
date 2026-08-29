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

        # 3. RE-GROUND FROM THE PRESS POSE. The arm aims at a point measured from HERE.
        if not self.dry_run:
            cap = self._reground(query, "re-grounded at the press pose")
            if cap is None:
                return ExecResult(False, "control not localised from the press pose; arm not moved")

        # 4. READY -> REACH -> STOW, the three steps press_run.sh does. Stow and press are
        #    different wrist orientations ("approaching straight out of stow reaches at the stow
        #    angle and skids off a round button"), and without the trailing stow every later leg
        #    is refused with arm_not_stowed, which reads as a navigation fault after a success.
        ARM_PY = str(REPO / ".venv-arm" / "bin" / "python")
        mode = "--dry-run" if self.dry_run else "--go"
        rr = _ros([ARM_PY, str(REPO / "bringup" / "stow_arm.py"), "--ready"]
                  + ([] if self.dry_run else ["--go"]), timeout=180)
        if rr.returncode != 0 and not self.dry_run:
            return ExecResult(False, "could not reach the press-ready wrist pose; not approaching")

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
