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

WHAT IT REFUSES TO START AT ALL
-------------------------------
Before the first trial, and before --dry-run gets to skip anything, this reads the mux config
(config/safety.yaml) and refuses unless the deadman still gates the autonomous sources. See
"THE CAMPAIGN INTERLOCK PREFLIGHT" below for why a campaign needs that when a supervised drive
does not, and for the one flag that overrides it -- which stamps every trial record it writes.
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


# ---------------------------------------------------------------------------------------------
# THE CAMPAIGN INTERLOCK PREFLIGHT  (added 2026-09-01 after a code review)
# ---------------------------------------------------------------------------------------------
# config/safety.yaml's mux is the ONLY publisher of /cmd_vel, and `requires_enable: true` on the
# autonomous sources is what makes a lost commander a BOUNDED event: bringup/deadman.py publishes
# /safety/enable at 20 Hz only while a human is physically holding a button, and the arbiter
# discards every autonomous command within timeouts.gate_s (0.2 s) of that hold ending --
# safety/arbiter.py, `if chosen.requires_enable and not enable: blocked("deadman")`.
#
# On 2026-09-01 the operator set `requires_enable: false` on the `nav` source, deliberately and
# with a real argument, written out in full above the setting: holding a browser deadman costs
# the hand that would otherwise be on the chassis E-stop, and the E-stop cuts MOTOR POWER, which
# is strictly stronger than a software gate that only stops commands reaching the driver (the
# chassis still coasts ~1.26 s / ~18 cm afterwards, measured 2026-08-21). That trade is sound for
# a SUPERVISED DRIVE with a human standing at the robot. The same comment says, in as many words,
# that it is NOT valid for the 50-trial campaign and must be set back to true first.
#
# NOTHING ENFORCED THAT. A comment is not an interlock, and this script would have run 50
# autonomous trials with the gate stood down. What makes a campaign different is not the risk per
# command, it is the exposure and where the human is looking: 50 trials is about an hour of
# autonomous base motion during which the operator is at a laptop scoring trials rather than
# standing over the E-stop, and the deadman check further down only re-runs BETWEEN trials -- a
# released deadman is therefore noticed up to a whole trial's worth of motion late. With
# requires_enable:true it does not need to be noticed at all: the base stops in 0.2 s because the
# mux stops passing commands. That property is the reason a campaign is allowed to be run by one
# person, and it is what this preflight checks is still true.
#
# FAIL CLOSED. A missing, unparseable, or unrecognisable config is a REFUSAL, not a pass. A
# preflight that waves through what it could not read is worse than no preflight at all, because
# after the first time it "passes" nobody opens the file themselves.
#
# The return-to-start leg invokes waypoints.py with --deadman-gated, routing it through the servo
# source checked below.  Manual `waypoints.py goto` remains on teleop by default; campaign code
# must never omit that flag.
SAFETY_CONFIG = Path(os.environ.get("UTP_SAFETY_CONFIG", REPO / "config" / "safety.yaml"))

# Typed in full, every time, on purpose. `--force` gets typed by habit within a week; this does
# not, and it is long enough to be visible in the shell history that ends up in EXPERIMENT_LOG.md.
UNSAFE_OVERRIDE_FLAG = "--i-accept-an-unsafe-campaign"

# The one source a human is holding themselves. safety/arbiter.py: teleop sets requires_enable
# False because "a human already has their hand on the control", and it is the ONLY source that
# may carry allows_arm_override -- the recovery path for a stuck arm, not an operating mode.
HUMAN_SOURCE = "teleop"

# Sources the campaign's autonomous motion actually comes out of. If the config does not describe
# both of them it is not the mux config this robot runs, and we do not get to guess which.
REQUIRED_SOURCES = ("nav", "servo")

_ABSENT = object()


def mux_safety_violations(path) -> list[str]:
    """Reasons `path` is not a safe mux config to run a CAMPAIGN against. Empty list == safe.

    Every return path that is not "I read this file, I understood it, and it holds" produces a
    violation. Unreadable, unparseable, wrong shape, right shape with a non-boolean where a
    boolean belongs -- all refusals. The question this answers is not "is anything obviously
    wrong" but "can I PROVE the deadman still bounds autonomous motion", and no evidence is not
    proof. Same discipline as every gate in safety/arbiter.py: never-seen and stale both mean no.
    """
    path = Path(path)
    try:
        import yaml
    except Exception as e:                            # pragma: no cover - PyYAML is a hard dep
        return [f"PyYAML will not import ({type(e).__name__}: {e}), so {path} cannot be read. "
                f"The mux config is unverifiable, which is a refusal, not a pass."]
    try:
        raw = path.read_text()
    except OSError as e:
        return [f"cannot read the mux config {path} ({type(e).__name__}: {e}). Nothing here can "
                f"say whether the deadman still gates autonomous motion."]
    try:
        doc = yaml.safe_load(raw)
    except Exception as e:
        return [f"{path} is not parseable YAML ({type(e).__name__}: {e}). Note the mux itself "
                f"would fail to load it too -- fix the file, do not run around it."]
    if not isinstance(doc, dict):
        return [f"{path} does not contain a YAML mapping (got {type(doc).__name__}); this is not "
                f"the twist-mux config."]

    sources = doc.get("sources", _ABSENT)
    if sources is _ABSENT:
        return [f"{path} has no `sources:` list, so it declares no mux sources at all. Either it "
                f"is the wrong file or the mux is unconfigured; both are refusals."]
    if not isinstance(sources, list) or not sources:
        return [f"{path}: `sources:` is {type(sources).__name__} and must be a non-empty list of "
                f"mux sources."]

    bad: list[str] = []
    names: list[str] = []
    for i, s in enumerate(sources):
        if not isinstance(s, dict):
            bad.append(f"{path}: sources[{i}] is {type(s).__name__}, not a mapping — this file "
                       f"is not shaped like a mux config.")
            continue
        name = s.get("name")
        if not isinstance(name, str) or not name.strip():
            bad.append(f"{path}: sources[{i}] has no usable `name:` ({name!r}); an unnamed source "
                       f"cannot be reasoned about.")
            continue
        name = name.strip()
        names.append(name)

        # ---- requires_enable: the deadman gate -------------------------------------------------
        if name != HUMAN_SOURCE:
            req = s.get("requires_enable", _ABSENT)
            if req is _ABSENT:
                # twist_mux_node.py defaults this to True, which is the safe direction -- but the
                # campaign will not take a safety property on loan from a default in another file
                # that a refactor can change without touching this one. State it here.
                bad.append(f"source '{name}' does not state `requires_enable` at all. The mux "
                           f"defaults it to true, but a campaign must not infer its one bound on "
                           f"a lost commander from a default: write `requires_enable: true` under "
                           f"`- name: {name}` in {path}.")
            elif not isinstance(req, bool):
                bad.append(f"source '{name}' has `requires_enable: {req!r}`, which is not a "
                           f"boolean. YAML will hand the mux something truthy and this preflight "
                           f"cannot tell what was meant. Write `requires_enable: true`.")
            elif req is False:
                bad.append(f"source '{name}' has `requires_enable: false` — the deadman gate is "
                           f"STOOD DOWN for an autonomous source. Autonomous commands would reach "
                           f"the base whether or not a human is holding /safety/enable. Fix: in "
                           f"{path}, under `- name: {name}`, set `requires_enable: true`.")

        # ---- allows_arm_override: driving with the arm out -------------------------------------
        arm = s.get("allows_arm_override", _ABSENT)
        if arm is _ABSENT:
            pass                                   # absent == false in the mux, and false is safe
        elif not isinstance(arm, bool):
            bad.append(f"source '{name}' has `allows_arm_override: {arm!r}`, which is not a "
                       f"boolean. Write true or false.")
        elif arm is True and name != HUMAN_SOURCE:
            bad.append(f"source '{name}' has `allows_arm_override: true`. That lets it drive the "
                       f"base with the arm EXTENDED, and the tool tip then sweeps a ~0.88 m "
                       f"radius through space the costmap believes is empty (config/safety.yaml, "
                       f"limits.max_wz). Only '{HUMAN_SOURCE}' may have it — that is a human's "
                       f"deliberate recovery path for a stuck arm, not something autonomous. Fix: "
                       f"in {path}, under `- name: {name}`, set `allows_arm_override: false`.")

    for req_name in REQUIRED_SOURCES:
        if req_name not in names:
            bad.append(f"{path} declares no '{req_name}' source (found: {names or 'none'}). The "
                       f"campaign's autonomous motion comes out of nav and servo; a config that "
                       f"does not describe them is not the one governing this robot, and a "
                       f"preflight that cannot find what it was asked to check must refuse.")
    dupes = sorted({n for n in names if names.count(n) > 1})
    if dupes:
        bad.append(f"{path} declares duplicate source name(s) {dupes}; which one the mux keeps is "
                   f"not something this check should be guessing at.")
    return bad


def print_refusal(path, violations: list[str]) -> None:
    """Loud, specific, and it names the edit. Written for someone standing up at the robot."""
    w = sys.stderr
    print("\n" + "=" * 94, file=w)
    print("[campaign] REFUSING TO START — the base-motion mux is not campaign-safe.", file=w)
    print(f"           config checked: {path}", file=w)
    print("=" * 94, file=w)
    for i, v in enumerate(violations, 1):
        print(f"  {i}) {v}", file=w)
    print(f"""
WHY THIS IS A CAMPAIGN RULE AND NOT A GENERAL ONE
  A supervised drive has a human at the robot with a hand on the chassis E-stop, which
  cuts motor power and is stronger than any software gate. A campaign does not: it is
  ~an hour of autonomous base motion with the operator at a laptop scoring trials, and
  this script only re-checks the deadman BETWEEN trials — so a commander lost mid-trial
  keeps driving until that trial ends. `requires_enable: true` bounds it to 0.2 s with
  nobody having to notice anything. Restore it, then start the campaign.

IF YOU MEANT IT (supervised, hand on the E-stop, watching every trial)
  re-run with {UNSAFE_OVERRIDE_FLAG} . It is deliberately long to type, and
  EVERY trial record written will carry unsafe_campaign_override: true plus the reasons
  above — so this dataset can never be mistaken for one run with the gate up.""", file=w)
    print("=" * 94 + "\n", file=w)


def print_override_warning(path, violations: list[str]) -> None:
    """The override is allowed to exist; it is not allowed to be quiet."""
    w = sys.stderr
    print("\n" + "!" * 94, file=w)
    print(f"[campaign] RUNNING WITH THE INTERLOCK DOWN — {UNSAFE_OVERRIDE_FLAG} was given.", file=w)
    print(f"           config: {path}", file=w)
    for i, v in enumerate(violations, 1):
        print(f"  {i}) {v}", file=w)
    print("  Nothing stops the base when the deadman is released except you and the E-stop.", file=w)
    print("  Stay at the robot. Every trial record is stamped unsafe_campaign_override: true.",
          file=w)
    print("!" * 94 + "\n", file=w)


def main() -> int:
    # allow_abbrev=False, and it is the interlock that needs it. argparse accepts any unambiguous
    # PREFIX of a long option, so with abbreviation on, `--i-accept` silently means
    # `--i-accept-an-unsafe-campaign` -- and the whole point of that flag is that it costs
    # something to type. Found by tests/test_campaign_safety.py, which asserts the shorthands do
    # not work. The cost is that `--tri 50` no longer means `--trials 50`; every caller in this
    # repo (bringup/session.sh) spells its flags out anyway.
    ap = argparse.ArgumentParser(description=__doc__, allow_abbrev=False,
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
    ap.add_argument("--safety-config", type=Path, default=SAFETY_CONFIG,
                    help="mux config the interlock preflight reads "
                         "(default $UTP_SAFETY_CONFIG or config/safety.yaml). Whatever is "
                         "checked is recorded in every trial record.")
    # No --force. A short flag becomes muscle memory in a week and then the interlock is
    # decoration; this one has to be meant, and it shows up in shell history and in the records.
    ap.add_argument(UNSAFE_OVERRIDE_FLAG, dest="accept_unsafe", action="store_true",
                    help="run even though the mux config is not campaign-safe. Prints why, and "
                         "stamps unsafe_campaign_override on EVERY trial record.")
    a = ap.parse_args()
    signal.signal(signal.SIGINT, _sigint)

    # ---- interlock preflight: before the output file exists, before --resume, before --dry-run.
    # --dry-run is INSIDE this on purpose. A dry run that skips the check is how people learn the
    # check is optional, and the dry run is exactly where you want to be told that safety.yaml
    # still needs restoring -- while nothing is moving, not three minutes later with the robot
    # already out on the floor.
    violations = mux_safety_violations(a.safety_config)
    if violations and not a.accept_unsafe:
        print_refusal(a.safety_config, violations)
        return 2
    unsafe_override = bool(violations)          # only reachable True with the flag typed in full
    if unsafe_override:
        print_override_warning(a.safety_config, violations)
    else:
        print(f"[campaign] mux interlock ok — deadman gates the autonomous sources "
              f"({a.safety_config})")

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

        # EVIDENCE MUST NOT OVERWRITE ITSELF. RosWorld names captures
        # f"{capture_prefix}_{self._n:03d}" and reset() puts _n back to 0 every trial, so a fixed
        # prefix means trial 2 writes over trial 1's frames and a 50-trial campaign keeps only the
        # last one. The records would still look complete while pointing at the wrong images.
        world.capture_prefix = f"camp_{a.method}_t{n:03d}"

        t0 = time.time()
        try:
            # seed=i, so make_trial_id (which hashes scene/seed/method) differs per trial and the
            # records are distinguishable.
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
            # NAV2 FOR A MAP-FRAME START, waypoints.py only for an odom one. `waypoints.py goto`
            # is the ODOM driver: it subtracts a stored coordinate from a live odom pose, which is
            # meaningless for a map-frame waypoint and drives to an arbitrary place. Every waypoint
            # recorded on 2026-09-01 is map-frame, so the return leg would have dead-reckoned home
            # and then measured drift against wherever it ended up -- a campaign of invalid trials
            # that looks fine in the record, which is the exact failure this file exists to avoid.
            import yaml as _yaml
            _store = Path(os.environ.get("UTP_WAYPOINTS") or (REPO / "maps" / "waypoints.yaml"))
            try:
                _frame = (_yaml.safe_load(_store.read_text()) or {}).get(a.start, {}).get("frame")
            except Exception:
                _frame = None
            if _frame == "map":
                _cmd = [sys.executable, str(REPO / "bringup" / "nav2_goto.py"), a.start, "--go"]
            elif _frame == "odom":
                _cmd = [sys.executable, str(REPO / "bringup" / "waypoints.py"),
                        "goto", a.start, "--go", "--deadman-gated"]
            else:
                print(f"[campaign] '{a.start}' has frame {_frame!r} -- refusing to guess how to "
                      f"drive home; a wrong return invalidates every later trial.", flush=True)
                _cmd = None
            rc = subprocess.run(_cmd, cwd=str(REPO)).returncode if _cmd else 1
            returned = (rc == 0)
            time.sleep(a.settle)
            back = world._pose()
            drift, yaw_drift = pose_err((back.x, back.y, back.yaw), anchor)
            print(f"[campaign] return: {'ok' if returned else 'FAILED'}  "
                  f"residual {drift:.3f} m / {yaw_drift:.3f} rad")

        d = rec if isinstance(rec, dict) else getattr(rec, "__dict__", {}) if rec else {}
        d = dict(d)
        # THE INTERLOCK IS PART OF THE PROVENANCE OF THE DATA, exactly like `hardware`. A trial
        # run with the deadman gate stood down is not the same measurement as one run with it up,
        # and a year from now the only place that difference survives is this record.
        d.update(world="ros_hardware", hardware=True, dry_run=bool(a.dry_run),
                 deadman_interlock_verified=(not unsafe_override),
                 unsafe_campaign_override=unsafe_override,
                 unsafe_campaign_reasons=(list(violations) if unsafe_override else None),
                 mux_config=str(a.safety_config),
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
