#!/usr/bin/env python3
"""Watch for the ADA doors to open after a press, and say so within a bounded time.

    python3 bringup/doors_open.py                 # wait up to 12 s, exit 0 the moment they open
    python3 bringup/doors_open.py --timeout 20
    python3 bringup/doors_open.py --once          # single look, no waiting

EXIT CODES ARE THE POINT -- this is called from a script that must decide whether to drive:
    0  OPEN      -- go, and go now: an ADA opener holds for a limited time and then closes
    1  STILL SHUT
    2  COULD NOT TELL (no frame, no reasoner, unparseable answer) -- treat as SHUT

WHY THE CAMERA AND NOT THE LIDAR. The doors are glass. Measured 2026-09-01 from the door pose:
the camera looked straight through them and reported "an open walkway with pillars" while they
were CLOSED, and on another frame the lidar had 85 returns at 0.72 m where the camera saw nothing.
Neither sensor is reliable alone -- which is why safety/blockage_fusion.py exists and why this
asks the same fused question rather than inventing a third opinion.

WHY IT POLLS INSTEAD OF ASKING ONCE. An opener takes a second or two to swing and then holds.
A single look timed badly reports SHUT on a door that is opening, the run gives up, and the hold
expires while the robot is deciding. Looking repeatedly costs a VLM call each time and is worth it.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
ROS_PY = "python3"
VENV = Path.home() / "unlocking-the-path" / "env" / ".venv" / "bin" / "python"


def look(tag: str) -> tuple[bool | None, str]:
    """(open?, description). None means could-not-tell, which the caller treats as shut."""
    cap = REPO / "captures" / tag
    r = subprocess.run([ROS_PY, str(REPO / "bringup" / "grab_frame.py"),
                        "--name", tag, "--timeout", "20"],
                       capture_output=True, text=True, timeout=60)
    if r.returncode != 0 or not (cap / "rgb.png").exists():
        return None, f"no frame ({(r.stderr or '').strip()[-80:]})"
    r = subprocess.run([str(VENV), str(REPO / "bringup" / "ask_blockage.py"), str(cap)],
                       capture_output=True, text=True, timeout=120)
    out = r.stdout or ""
    blocked = None
    desc = ""
    for line in out.splitlines():
        if line.startswith("blocked"):
            v = line.split(":", 1)[1].strip().lower()
            blocked = True if v.startswith("true") else (False if v.startswith("false") else None)
        elif line.startswith("description"):
            desc = line.split(":", 1)[1].strip()
    if blocked is None:
        return None, f"unparseable verdict: {out.strip()[-80:]}"
    # "not blocked" is the doors being open. The fused verdict already combines camera and lidar,
    # so a door that is open to the camera but still returning lidar hits reads as SHUT -- which
    # is the safe direction.
    return (not blocked), desc


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timeout", type=float, default=12.0)
    ap.add_argument("--once", action="store_true")
    a = ap.parse_args()

    t0 = time.time()
    n = 0
    last = "no look completed"
    while True:
        n += 1
        opened, desc = look(f"doors_{time.strftime('%H%M%S')}_{n}")
        last = desc
        if opened is True:
            print(f"OPEN after {time.time() - t0:.1f} s -- {desc}")
            return 0
        print(f"  look {n}: {'SHUT' if opened is False else 'could not tell'} -- {desc}")
        if a.once or time.time() - t0 >= a.timeout:
            break
    if opened is None:
        print(f"COULD NOT TELL after {time.time() - t0:.1f} s -- {last}", file=sys.stderr)
        return 2
    print(f"STILL SHUT after {time.time() - t0:.1f} s -- {last}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
