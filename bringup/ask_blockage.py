#!/usr/bin/env python3
"""Ask the VLM what is blocking the robot. Perception, not reasoning.

    ~/unlocking-the-path/env/.venv/bin/python bringup/ask_blockage.py captures/ada_live_02
    ... --json          # machine-readable only, for RosWorld

Runs under the PIPELINE VENV (needs openai), not ROS's python.

WHAT THIS IS FOR. utp/pipeline/interfaces.py's World protocol needs current_blockage() to return a
BlockageEvent{blocked, kind, description}. In simulation that comes from GROUND TRUTH -- the scene
knows a door is there. On hardware there is no ground truth, so it has to be perceived.

THE LINE THIS MUST NOT CROSS. It reports WHAT IS THERE. It does not decide WHAT TO DO. The
reasoner picks the action, and the paper's claim is about the reasoner: if this call returned
"press the button beside the door", the reasoning would have already happened here and
reasoning_correct would be measuring a prompt written by us. So the prompt asks for a description
of the obstruction and nothing else, and the word "press" does not appear in it.

For the same reason the prompt does NOT ask "is this a door?" -- naming the expected answer inside
the question is how you get it back. It asks what is in front of the robot and whether the robot
can pass without operating something.

FAIL CLOSED. Endpoint down, malformed reply, unparseable JSON -> kind="" and the raw text as the
description. Never invent "door": a wrong kind sends the reasoner looking for a control that is
not there, and the trial records a reasoning failure that was really a perception failure.

THE CAMERA ALONE IS NOT ENOUGH, AND THAT IS NOT THE VLM's FAULT. 2026-09-01, on hardware: with
the robot 0.72 m from CLOSED GLASS DOORS, this script returned blocked=False, "an open walkway
with pillars". The picture genuinely shows an open walkway -- glass is transparent to a camera,
so a correct reading of the image was a wrong reading of the world (captures/trial_ours_001).
The lidar scan captured with that same frame had 39 returns inside the drive corridor, the
nearest at 0.70 m. So when the capture directory also holds a scan.json, the camera verdict is
fused with it through safety/blockage_fusion.py, which ORs the two: blocked if EITHER sensor says
blocked, because the two fail on opposite physics and requiring agreement means requiring both to
succeed on the case each is worst at. The reasoning for that lives in that module's docstring.

No scan.json in the directory -> unchanged behaviour, camera only. The printed and JSON keys
`blocked`, `kind` and `description` keep their existing meanings (bringup/ros_world.py and the
archived route_run parse them); fusing only ever adds `evidence` and `nearest_ahead_m`.
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from safety.blockage_fusion import fuse   # noqa: E402  -- pure logic, numpy/stdlib only

SIM = Path.home() / "unlocking-the-path"
KINDS = ("door", "elevator", "")

SYSTEM = (
    "You are the forward camera of a wheeled robot that has stopped because its path is "
    "obstructed. Report only what you can see. Do not suggest actions, do not mention buttons "
    "unless one is physically visible in the image, and do not speculate about what the robot "
    "should do next. Answer strictly as JSON with keys: "
    '"obstruction" (a short factual description of what is in the robot\'s way), '
    '"passable_without_operating_something" (true or false), '
    '"category" (one of "door", "elevator", "other").'
)
USER = ("This is the robot's forward view. What is in the robot's way, and could the robot pass "
        "it without operating anything? Reply with the JSON object only.")


def load_env() -> None:
    """Read the gitignored .env in the sim repo, the same source check_llm.sh uses."""
    f = SIM / ".env"
    if not f.exists():
        return
    for line in f.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        os.environ.setdefault(k.strip(), v.strip().strip('"').strip("'"))


def rgb_data_url(path: Path) -> str:
    from PIL import Image
    im = Image.open(path).convert("RGB")
    im.thumbnail((1024, 1024))
    buf = io.BytesIO()
    im.save(buf, format="JPEG", quality=85)
    return "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode("ascii")


def parse(raw: str) -> dict:
    """Pull the JSON object out of whatever the model said. Never raise."""
    txt = raw.strip()
    if "```" in txt:
        txt = txt.split("```")[1]
        txt = txt[4:] if txt.lower().startswith("json") else txt
    i, j = txt.find("{"), txt.rfind("}")
    if i < 0 or j <= i:
        return {"kind": "", "description": raw.strip()[:400], "blocked": True,
                "note": "model did not return JSON"}
    try:
        d = json.loads(txt[i:j+1])
    except Exception:
        return {"kind": "", "description": raw.strip()[:400], "blocked": True,
                "note": "JSON did not parse"}
    cat = str(d.get("category", "")).lower().strip()
    kind = cat if cat in KINDS else ""
    passable = d.get("passable_without_operating_something")
    return {"kind": kind,
            "description": str(d.get("obstruction", "")).strip()[:400],
            # Only a definite True clears the blockage. Absent or unparseable stays blocked --
            # driving on because a model omitted a field is the wrong way to be wrong.
            "blocked": not (passable is True),
            "note": ""}


def ask_camera(capture_dir: Path) -> dict:
    """The VLM call on its own. Callers almost certainly want ask(), which also uses the lidar."""
    rgb = capture_dir / "rgb.png"
    if not rgb.exists():
        return {"kind": "", "description": f"no rgb.png in {capture_dir}", "blocked": True,
                "note": "no image"}
    load_env()
    base_url = os.environ.get("OPENAI_BASE_URL")
    api_key = os.environ.get("OPENAI_API_KEY")
    model = os.environ.get("UTP_VLM_MODEL", "openai/gemma4-vibe")
    if not (base_url and api_key):
        return {"kind": "", "description": "no OPENAI_BASE_URL / OPENAI_API_KEY", "blocked": True,
                "note": "no credentials"}
    try:
        from openai import OpenAI
        client = OpenAI(base_url=base_url, api_key=api_key)
        r = client.chat.completions.create(
            model=model, temperature=0.0, max_tokens=300,
            messages=[{"role": "system", "content": SYSTEM},
                      {"role": "user", "content": [
                          {"type": "text", "text": USER},
                          {"type": "image_url",
                           "image_url": {"url": rgb_data_url(rgb)}}]}])
        out = parse(r.choices[0].message.content or "")
        out["model"] = model
        return out
    except Exception as e:
        return {"kind": "", "description": f"VLM call failed: {type(e).__name__}: {e}"[:400],
                "blocked": True, "note": "call failed"}


def _f(v) -> float:
    """A scan field as a float, or NaN. NaN is what blockage_fusion reads as "no usable
    geometry", which is the honest answer for a field that is missing or is not a number."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def ask(capture_dir: Path) -> dict:
    """The camera verdict, fused with the lidar scan saved beside it if there is one.

    This is the entry point bringup/ros_world.py imports, so the fusion applies to the live
    pipeline and not only to this CLI. It runs on EVERY path, including the fail-closed ones (no
    rgb.png, no credentials, VLM call failed), so `evidence` means the same thing every time a
    scan is present rather than appearing only when the VLM happened to answer.

    A scan.json that is missing or unreadable leaves the camera verdict standing -- exactly what
    happened before this existed. That is deliberate and it is not a fail-open: the lidar can only
    ADD a blockage here, never clear one, so losing it can never turn a blocked into a clear.
    """
    res = ask_camera(capture_dir)
    scan_f = capture_dir / "scan.json"
    if not scan_f.exists():
        return res
    try:
        sc = json.loads(scan_f.read_text())
        if not isinstance(sc, dict):
            sc = {}
        scan_note = ""
    except Exception as e:
        # Reported on its own key, NOT folded into `note`: `note` is this script's fail-closed
        # marker and printing "failed closed" next to a camera verdict of clear would be a lie.
        # An unreadable scan simply means the lidar contributed nothing, and evidence says so.
        sc, scan_note = {}, f"scan.json unreadable: {type(e).__name__}"
    out = dict(res)   # keep `note` and `model`; fuse() owns the five contract keys
    out.update(fuse(res, sc.get("ranges"), _f(sc.get("angle_min")),
                    _f(sc.get("angle_increment"))))
    if scan_note:
        out["scan_note"] = scan_note
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--json", action="store_true", help="print only the JSON object")
    a = ap.parse_args()
    res = ask(Path(a.capture))
    if a.json:
        print(json.dumps(res))
        return 0
    print(f"blocked     : {res['blocked']}")
    print(f"kind        : {res['kind'] or '(unclassified)'}")
    print(f"description : {res['description']}")
    if "evidence" in res:
        # Only printed when a scan was actually fused, so the old output is byte-identical when
        # there is no scan.json. "neither" next to blocked=True means nothing could see at all.
        near = res.get("nearest_ahead_m")
        extra = "" if near is None else f"  (nearest {near:.2f} m ahead in the corridor)"
        if res.get("scan_note"):
            extra += f"  ({res['scan_note']})"
        print(f"evidence    : {res['evidence']}{extra}")
    if res.get("note"):
        print(f"note        : {res['note']}  <- failed closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
