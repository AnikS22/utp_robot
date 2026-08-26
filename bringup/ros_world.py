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

    def __init__(self, goal: str = "", dry_run: bool = True, capture_prefix: str = "fsm") -> None:
        self.goal = goal
        self.dry_run = dry_run
        self.capture_prefix = capture_prefix
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
        args = [ROS_PY, str(REPO / "bringup" / "approach_target.py"),
                "--capture", str(self._last_capture),
                "--target-cam", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}",
                "--min-standoff", "60", "--dry-run" if self.dry_run else "--go"]
        r = _ros(args, timeout=300)
        ok = r.returncode == 0
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
