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
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import os
import sys
from pathlib import Path

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


def ask(capture_dir: Path) -> dict:
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
    if res.get("note"):
        print(f"note        : {res['note']}  <- failed closed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
