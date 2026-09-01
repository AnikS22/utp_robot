#!/usr/bin/env python3
"""Run N real-robot trials back-to-back, returning to the start pose between each one.

    python3 bringup/run_campaign.py --trials 50 --method ours --start start
    python3 bringup/run_campaign.py --trials 50 --method ours --start start --resume
    python3 bringup/run_campaign.py --trials 4  --method ours --start start --dry-run

YOU HOLD THE DEADMAN AND YOU WATCH IT. This is not an unattended script. It is a script that does
the repetitive part correctly so that your attention is spent on the robot instead of on a
terminal, and so that a campaign is not silently invalidated halfway through.

WHY THIS EXISTS AND WHY IT IS NOT A `for` LOOP AROUND run_trial.py
------------------------------------------------------------------
Three properties of THIS robot make naive repetition produce unusable data:

1. ODOM DRIFT COMPOUNDS. `config/routes.yaml` states it plainly: waypoints live in the ODOM
   frame. A single trial's drift is inside the visual servo's budget; fifty trials of it are not.
   So every trial ends by driving back to the start waypoint and MEASURING the residual against
   the pose recorded at trial 1. When that residual passes --max-drift the campaign STOPS, because
   from that point on the waypoints no longer mean what they meant when they were recorded.

2. RESTARTING THE RANGER DRIVER RE-ZEROES ODOM and silently invalidates every waypoint. So this
   script never restarts it, and it aborts rather than continuing through a driver bounce
   (detected as an impossible jump in reported pose).

3. A TRIAL THAT DOES NOT WRITE A TrialRecord IS NOT A DATA POINT. Records are appended, one JSON
   object per line, flushed and fsync'd per trial, so a crash at trial 37 costs trial 37 and
   nothing else. --resume continues from what is on disk.

WHAT COUNTS AS A REASON TO STOP THE WHOLE CAMPAIGN (not just fail one trial)
---------------------------------------------------------------------------
Anything that makes SUBSEQUENT trials unmeasurable, rather than merely unsuccessful:
  * return-to-start residual over budget      -> the frame the waypoints live in has moved
  * a collision                               -> the robot's state and possibly the scene changed
  * /safety/enable stopped publishing         -> the deadman was released; nothing will move anyway
  * the reasoning endpoint became unreachable -> `ours` and `direct_vlm` silently degrade
  * an odom discontinuity                     -> the driver restarted under us
A trial that simply FAILS (door did not open, target never found) is data and the campaign
continues -- that is the experiment, not an error.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PIPELINE = Path(os.environ.get("UTP_PIPELINE_REPO", Path.home() / "unlocking-the-path"))
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))
sys.path.insert(0, str(PIPELINE))

STOP = {"now": False}


def _sigint(_sig, _frm):
    # Finish the trial in flight, then stop cleanly. A second Ctrl-C is the real emergency exit,
    # but the RC transmitter and the E-stop are both faster and are what you should actually use.
    if STOP["now"]:
        print("\n[campaign] second interrupt — exiting immediately", flush=True)
        sys.exit(130)
    STOP["now"] = True
    print("\n[campaign] interrupt received — will stop after the current trial", flush=True)


def topic_alive(topic: str, timeout: float = 6.0) -> bool:
    """True if `topic` produced at least one message inside `timeout`."""
    try:
        r = subprocess.run(["ros2", "topic", "echo", topic, "--once"],
                           capture_output=True, timeout=timeout)
        return r.returncode == 0
    except Exception:
        return False


def llm_reachable() -> bool:
    script = REPO / "bringup" / "check_llm.sh"
    if not script.is_file():
        return True                      # nothing to check against; do not invent a failure
    try:
        return subprocess.run(["bash", str(script)], capture_output=True,
                              timeout=30).returncode == 0
    except Exception:
        return False


def pose_err(a, b) -> tuple[float, float]:
    """(planar metres, |yaw| radians) between two (x, y, yaw) poses."""
    d = math.hypot(a[0] - b[0], a[1] - b[1])
    dy = abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2])))
    return d, dy


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, required=True)
    ap.add_argument("--method", default="ours")
    ap.add_argument("--start", required=True,
                    help="waypoint name the robot returns to between trials "
                         "(bringup/waypoints.py record <name>)")
    ap.add_argument("--scene", default="button_door")
    ap.add_argument("--goal", default="")
    ap.add_argument("--dry-run", action="store_true",
                    help="every stage runs; nothing on the robot moves. USE THIS FIRST.")
    ap.add_argument("--max-drift", type=float, default=0.30,
                    help="metres of return-to-start residual before the campaign stops (default 0.30)")
    ap.add_argument("--max-yaw-drift", type=float, default=0.20, help="radians (default 0.20)")
    ap.add_argument("--settle", type=float, default=3.0, help="seconds to hold between trials")
    ap.add_argument("--out", type=Path, default=REPO / "captures" / "campaign.jsonl")
    ap.add_argument("--resume", action="store_true", help="continue from what is already in --out")
    ap.add_argument("--config", type=Path,
                    default=Path(os.environ.get("UTP_CONFIG_DIR", REPO / "config" / "pipeline")))
    a = ap.parse_args()
    signal.signal(signal.SIGINT, _sigint)

    a.out.parent.mkdir(parents=True, exist_ok=True)
    done = 0
    if a.resume and a.out.is_file():
        done = sum(1 for ln in a.out.open() if ln.strip())
        print(f"[campaign] resuming: {done} trial(s) already on disk")
    if done >= a.trials:
        print(f"[campaign] nothing to do — {done} >= {a.trials}")
        return 0

    # ---- preconditions, checked ONCE and then re-checked cheaply between trials -------------
    print("[campaign] preconditions")
    if not a.dry_run:
        if not topic_alive("/safety/enable"):
            print("  FAIL /safety/enable is silent — hold the deadman (bringup/deadman.py). "
                  "Without it every autonomous command is discarded and the robot will look dead "
                  "while behaving exactly as designed.", file=sys.stderr)
            return 2
        print("  ok   /safety/enable is publishing")
    if a.method in ("ours", "direct_vlm") and not llm_reachable():
        print("  FAIL reasoning endpoint unreachable — campus network / VPN?", file=sys.stderr)
        return 2
    print("  ok   reasoning endpoint" if a.method in ("ours", "direct_vlm") else "  --   no VLM needed")

    from waypoints import load as load_waypoints
    wps = load_waypoints()
    if a.start not in wps:
        print(f"  FAIL start waypoint '{a.start}' not recorded. Known: {sorted(wps)}", file=sys.stderr)
        return 2
    print(f"  ok   start waypoint '{a.start}'")

    from ros_world import RosWorld
    from steered_reasoner import LookHints, SteeredReasoner
    from utp.common.config import Config
    from utp.pipeline.fsm import run_trial
    from utp.pipeline.registry import build_modules

    if not (a.config / "methods.yaml").is_file():
        print(f"  FAIL config dir has no methods.yaml: {a.config}", file=sys.stderr)
        return 2
    cfg = Config.load(a.config)
    print(f"  ok   config dir {a.config}")
    method = cfg.method(a.method)
    cfg.data.setdefault("runtime", {})["world"] = "ros"
    cfg.data["runtime"]["max_recovery_attempts"] = max(
        4, int(cfg.data["runtime"].get("max_recovery_attempts", 2)))

    hints = LookHints()
    world = RosWorld(goal=a.goal, dry_run=a.dry_run, capture_prefix=f"camp_{a.method}", hints=hints)
    modules = build_modules(cfg, method, world)
    if method.get("reasoning") == "vlm":
        modules.reasoner = SteeredReasoner(cfg.data["methods"].get("vlm", {}), hints)

    anchor = None            # pose at the start of trial 1 — every later residual is against THIS
    t_campaign = time.time()
    n_ok = n_fail = 0

    for i in range(done, a.trials):
        if STOP["now"]:
            print("[campaign] stopping on request"); break
        n = i + 1
        print(f"\n=========== trial {n}/{a.trials}  ({a.method}) ===========", flush=True)

        # -- cheap between-trial re-checks: these are the things that go stale mid-campaign
        if not a.dry_run and not topic_alive("/safety/enable", timeout=4.0):
            print("[campaign] STOP: deadman released mid-campaign", file=sys.stderr); break

        here = world._pose()
        pose_now = (here.x, here.y, here.yaw)
        if anchor is None:
            anchor = pose_now
            print(f"[campaign] anchor pose x={anchor[0]:.3f} y={anchor[1]:.3f} yaw={anchor[2]:.3f}")

        t0 = time.time()
        try:
            rec = run_trial(cfg, world, modules, a.scene, i, a.method)
        except Exception as e:                       # a crashed trial must not kill the campaign
            print(f"[campaign] trial raised {type(e).__name__}: {e}", file=sys.stderr)
            rec = None
        dt = time.time() - t0

        # -- return to start, then MEASURE how far off we are from the anchor -----------------
        returned = drift = yaw_drift = None
        if not a.dry_run:
            print(f"[campaign] returning to '{a.start}' ...", flush=True)
            # `waypoints.py goto` is a DRY RUN unless --go is passed; without it the robot would
            # never actually return and every residual below would be measured against a robot that
            # had not moved -- a silently perfect-looking campaign of invalid trials.
            rc = subprocess.run([sys.executable, str(REPO / "bringup" / "waypoints.py"),
                                 "goto", a.start, "--go"], cwd=str(REPO)).returncode
            returned = (rc == 0)
            time.sleep(a.settle)
            back = world._pose()
            drift, yaw_drift = pose_err((back.x, back.y, back.yaw), anchor)
            print(f"[campaign] return: {'ok' if returned else 'FAILED'}  "
                  f"residual {drift:.3f} m / {yaw_drift:.3f} rad")

        d = rec if isinstance(rec, dict) else getattr(rec, "__dict__", {}) if rec else {}
        d = dict(d)
        d.update(world="ros_hardware", hardware=True, dry_run=bool(a.dry_run),
                 campaign_index=n, campaign_method=a.method, trial_wall_s=round(dt, 1),
                 returned_to_start=returned,
                 return_drift_m=None if drift is None else round(drift, 4),
                 return_drift_yaw_rad=None if yaw_drift is None else round(yaw_drift, 4),
                 anchor_pose=None if anchor is None else [round(v, 4) for v in anchor],
                 timestamp=time.strftime("%Y-%m-%dT%H:%M:%S%z"))
        with a.out.open("a") as fh:                  # append + fsync: a crash costs ONE trial
            fh.write(json.dumps(d, default=str) + "\n"); fh.flush(); os.fsync(fh.fileno())

        succeeded = bool(d.get("success"))
        n_ok += succeeded; n_fail += (not succeeded)
        print(f"[campaign] trial {n}: {'SUCCESS' if succeeded else 'fail'} "
              f"({d.get('failure_category')})  {dt:.0f}s   running {n_ok}/{n_ok+n_fail}")

        # -- campaign-level stop conditions ---------------------------------------------------
        if not a.dry_run:
            if d.get("collided") or (d.get("n_collisions") or 0) > 0:
                print("[campaign] STOP: collision recorded — inspect before continuing",
                      file=sys.stderr); break
            if drift is not None and drift > a.max_drift:
                print(f"[campaign] STOP: return residual {drift:.3f} m > {a.max_drift} m. The odom "
                      f"frame the waypoints were recorded in has moved; re-record them "
                      f"(bringup/waypoints.py record) before trusting further trials.",
                      file=sys.stderr); break
            if yaw_drift is not None and yaw_drift > a.max_yaw_drift:
                print(f"[campaign] STOP: yaw residual {yaw_drift:.3f} rad > {a.max_yaw_drift}",
                      file=sys.stderr); break
            if returned is False:
                print("[campaign] STOP: could not return to start", file=sys.stderr); break

    mins = (time.time() - t_campaign) / 60.0
    print(f"\n=========== campaign done: {n_ok} success / {n_ok + n_fail} scored "
          f"in {mins:.0f} min ===========")
    print(f"records: {a.out}")
    print("Resume with the same command plus --resume.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
