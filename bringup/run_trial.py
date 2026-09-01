#!/usr/bin/env python3
"""Run ONE trial of the real pipeline on the real robot. The FSM, not a component of it.

    ~/unlocking-the-path/env/.venv/bin/python bringup/run_trial.py --method ours --goal finish
    ... --dry-run              # every stage runs; nothing on the robot moves
    ... --method heuristic     # same route, same everything, different reasoner

Runs under the PIPELINE VENV.

WHY THIS FILE EXISTS. utp/runner/batch.py's make_world() knows mock, graph and isaac. It does not
know RosWorld, which lives in this repo -- so there has been NO WAY to drive the pipeline's own
reason -> ground -> act -> verify loop against the hardware, and every "hardware test" so far has
been a component exercised on its own: route_run's escalation is my loop, and reach_control is the
grounder with a query I typed. Neither is the system in the paper.

This closes that gap without touching the pipeline repo: it builds the SAME Modules from the SAME
config/methods.yaml row via the SAME registry, and hands them the SAME run_trial the sim campaign
uses, with RosWorld as the world. What differs between `ours`, `direct_vlm`, `heuristic` and
`passive` is exactly the method row, which is the point of a method row.

WHAT YOU GET THAT route_run CANNOT GIVE. run_trial is the loop with the VERIFY step, the
look-around ladder, the recovery budget, observe_failure, the negative-control scoring and a
TrialRecord written to the campaign's own logger. A trial that does not produce a TrialRecord is
not a data point.

The three gt_* methods return empty on hardware, by design: there is no answer key on a real
corridor, so reasoning_correct cannot be scored here. Score hardware trials on what happened --
did the door open -- and keep the answer-key metrics for the sim campaign. ros_world.py says the
same in more detail.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PIPELINE = Path(os.environ.get("UTP_PIPELINE_REPO", Path.home() / "unlocking-the-path"))
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))
sys.path.insert(0, str(PIPELINE))


def load_env() -> None:
    """The reasoner reads OPENAI_* from the environment and nothing here loads the gitignored
    .env -- the same gap that made reasoning=vlm report a missing key while perception worked."""
    f = PIPELINE / ".env"
    if not f.exists():
        return
    for line in f.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--method", default="ours", help="row from the pipeline's config/methods.yaml")
    ap.add_argument("--goal", default="", help="waypoint name the robot is trying to reach")
    ap.add_argument("--scene", default="button_door",
                    help="scene key from the pipeline config. button_door is the door-with-a-"
                         "control case; it selects the scoring, not the physical scene.")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dry-run", action="store_true",
                    help="run every stage; move nothing on the robot")
    ap.add_argument("--out", type=Path, default=REPO / "captures" / "trials.jsonl")
    # THE ROVER-LAPTOP CONFIG MUST BE PASSED EXPLICITLY (2026-08-31). Config.load() with no
    # argument resolves config_dir as `Path(utp/common/config.py).parents[2]/"config"` -- i.e. the
    # SIM repo's config -- so config/pipeline/ was dead: its detectors.yaml cuda:0 override (this
    # laptop has ONE GPU; a literal cuda:1 raises at .to()) and its `keyfree` matrix were both
    # silently ignored. config/pipeline/README.md already says the dir is "passed via
    # --config <this dir>"; nothing passed it.
    ap.add_argument("--config", type=Path, default=Path(os.environ.get("UTP_CONFIG_DIR",
                    REPO / "config" / "pipeline")),
                    help="pipeline config dir (default: this repo's rover-laptop override copy)")
    a = ap.parse_args()

    load_env()
    from ros_world import RosWorld
    from utp.common.config import Config
    from utp.pipeline.fsm import run_trial
    from utp.pipeline.registry import build_modules

    if not (a.config / "methods.yaml").is_file():
        print(f"config dir has no methods.yaml: {a.config}", file=sys.stderr)
        return 2
    cfg = Config.load(a.config)
    print(f"[run_trial] config dir: {a.config}")
    try:
        method = cfg.method(a.method)
    except KeyError as e:
        print(e, file=sys.stderr)
        return 2

    # runtime.world drives is_mock inside the registry. It must NOT say "mock" here or the
    # registry hands back mock modules and the whole run is a simulation wearing a robot's name.
    cfg.data.setdefault("runtime", {})["world"] = "ros"
    # The survey needs enough attempts to WALK the bearing list. fsm.py spends one recovery
    # attempt per look, and the campaign default is 2 -- so with four bearings the last two were
    # never tried, and on this building the control is at ~80 deg, which is exactly where the
    # untried ones point. Raised for hardware only; the sim campaign keeps its own default.
    cfg.data["runtime"]["max_recovery_attempts"] = max(
        4, int(cfg.data["runtime"].get("max_recovery_attempts", 2)))

    # The one slot through which the VLM may say where to look next. The reasoner writes it, the
    # world reads it (RosWorld.strafe_view). Reasoners that never write it -- heuristic, passive --
    # leave the world on the blind sweep, so what varies across methods is still the reasoner.
    from steered_reasoner import LookHints, SteeredReasoner
    hints = LookHints()
    world = RosWorld(goal=a.goal, dry_run=a.dry_run, capture_prefix=f"trial_{a.method}",
                     hints=hints)
    modules = build_modules(cfg, method, world)
    if method.get("reasoning") == "vlm":
        # Same model, same system prompt, same parser as the pipeline's GPT5Reasoner; two
        # hardware-only changes in the USER text (see steered_reasoner.py). Substituted here
        # rather than in the registry so the pipeline repo stays untouched.
        modules.reasoner = SteeredReasoner(cfg.data["methods"].get("vlm", {}), hints)
        print("reasoner : SteeredReasoner (fresh second look + VLM-steered look-around)")
    print(f"method   : {method.get('label', a.method)}  "
          f"(reasoning={method.get('reasoning')} grounding={method.get('grounding')} "
          f"execution={method.get('execution')} verification={method.get('verification')})")
    print(f"goal     : {a.goal or '(none -- blockage handling only)'}")
    print(f"dry run  : {a.dry_run}\n")

    rec = run_trial(cfg, world, modules, a.scene, a.seed, a.method)

    d = rec if isinstance(rec, dict) else getattr(rec, "__dict__", {"record": str(rec)})
    # The TrialRecord's own `world` field says "mock" on hardware -- schema.py permits only
    # mock|graph|isaac, so RosWorld cannot declare itself. Overwrite it here, in OUR record, or a
    # real-robot trial sits in the results indistinguishable from a simulated one.
    d["world"] = "ros_hardware"
    d["hardware"] = True
    d["dry_run"] = bool(a.dry_run)
    d["goal_waypoint"] = a.goal
    # There is no answer key on a real corridor, so the gt_* fields and anything derived from
    # them (reasoning_correct, grounding_iou) are null or meaningless here BY DESIGN. Say so in
    # the record rather than letting a null be read as a zero.
    d["scored_against_answer_key"] = False
    a.out.parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "a") as fh:
        fh.write(json.dumps(d, default=str) + "\n")

    print("\n--- TRIAL ---")
    for k in ("trial_id", "method_name", "scene_type", "success", "steps_taken",
              "interactions_completed", "failure_reason"):
        if k in d:
            print(f"  {k:24s} {d[k]}")
    print(f"  record appended to      {a.out}")
    return 0 if d.get("success") else 1


if __name__ == "__main__":
    sys.exit(main())
