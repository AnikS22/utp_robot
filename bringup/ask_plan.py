#!/usr/bin/env python3
"""Put a perceived blockage to the PIPELINE'S REASONER and return the action it chooses.

    ~/unlocking-the-path/env/.venv/bin/python bringup/ask_plan.py <capture_dir> \
        --blockage <json> --method ours --json

Runs under the PIPELINE VENV, not ROS's python.

WHY THIS EXISTS, and it is the difference between an experiment and a demonstration.

route_run's `--on-blocked <route>` names the route to run when the way is blocked. That is fine
for getting a robot through a door. It is USELESS AS EVIDENCE, because the action is chosen by
whoever typed the command line. The paper's claim is about the REASONER: given a described
obstruction and a bounded tool list, does it pick the right tool? Hardcoding `press_and_pass`
answers that question in the operator's favour before the robot has seen anything, and it makes
the comparison against the heuristic baseline meaningless -- with the action fixed, every method
is the same method.

So this hands the blockage to the reasoner the METHOD ROW selects (config/methods.yaml):

    passive      reasoning: none        -- never acts. The floor of the comparison.
    heuristic    reasoning: heuristic   -- rule-based action selection
    direct_vlm   reasoning: vlm         -- same reasoner, different grounding downstream
    ours         reasoning: vlm         -- the full method

and returns the Plan{action_type, target_description, rationale, abstain} it produced. What
route_run does with that action is a FIXED table, identical for every method, so the only thing
that varies across a comparison is the module under test.

The reasoner never sees geometry: Observation.candidates stays empty and no bbox or 3D point is
passed. It gets the description and the tool list, exactly as it does in simulation.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

PIPELINE = Path(os.environ.get("UTP_PIPELINE_REPO", Path.home() / "unlocking-the-path"))
sys.path.insert(0, str(PIPELINE))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture", type=Path, help="captures/<name> from grab_frame.py")
    ap.add_argument("--blockage", required=True, help="JSON from ask_blockage.py")
    ap.add_argument("--method", default="ours", help="method key from config/methods.yaml")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    # The reasoner reads OPENAI_* from the process environment, and nothing in this path loads
    # the gitignored .env. ask_blockage.py has its own load_env() for exactly this reason -- which
    # is why perception worked while reasoning=vlm reported the key "missing or still the
    # placeholder". Same source, same precedence (setdefault: a real env var still wins).
    envf = PIPELINE / ".env"
    if envf.exists():
        for line in envf.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))

    try:
        import numpy as np
        from utp.common.config import Config
        from utp.pipeline.registry import _build_reasoner
        from utp.pipeline.types import BlockageEvent, Observation
    except Exception as e:
        print(json.dumps({"error": f"pipeline import failed: {type(e).__name__}: {e}"}))
        return 1

    b = json.loads(a.blockage)
    blockage = BlockageEvent(blocked=bool(b.get("blocked")), kind=b.get("kind", "") or "",
                             description=b.get("description", "") or "")

    rgb = depth = None
    cam = {}
    try:
        from PIL import Image
        rgb = np.asarray(Image.open(a.capture / "rgb.png").convert("RGB"))
        depth = np.load(a.capture / "depth.npy")
        cam = json.loads((a.capture / "cam.json").read_text())
    except Exception:
        pass    # the reasoner is not entitled to geometry anyway; a missing frame is not fatal

    # candidates stays EMPTY on purpose. In simulation it is the mock perception proxy, and
    # handing a real reasoner a list of pre-identified interactables would be exactly the
    # shortcut the decoupling claim is about.
    obs = Observation(rgb=rgb, depth=depth, cam_info=cam, candidates=[])

    try:
        cfg = Config.load()
        method = cfg.method(a.method)      # applies defaults (grounding_backend, navigation)
    except Exception as e:
        print(json.dumps({"error": f"could not load method '{a.method}': "
                                   f"{type(e).__name__}: {e}"}))
        return 1

    try:
        reasoner = _build_reasoner(cfg, method, None, False)
        plan = reasoner.plan(obs, blockage, [])
    except Exception as e:
        # Fail closed and SAY SO. A reasoner that errored is not a reasoner that abstained, and
        # scoring them the same would quietly flatter whichever method crashed.
        print(json.dumps({"error": f"{type(e).__name__}: {e}", "method": a.method,
                          "reasoning": method.get("reasoning")}))
        return 1

    out = {"action_type": plan.action_type, "target_description": plan.target_description,
           "params": plan.params, "rationale": plan.rationale, "abstain": bool(plan.abstain),
           "method": a.method, "reasoning": method.get("reasoning"),
           "label": method.get("label", a.method)}
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
