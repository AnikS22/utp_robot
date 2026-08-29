"""Turn `ModuleNotFoundError: rclpy` into an instruction.

ROS 2 lives in a shell environment, not in the interpreter: `rclpy` is only importable after
/opt/ros/jazzy/setup.bash has been sourced. A python script cannot source it for you -- sourcing
mutates the PARENT shell, and by the time this module runs the interpreter has already started.

So the best it can do is say so precisely. A bare traceback reads as "the tool is broken"; it is
not, the shell just is not set up, and the fix is one line. This matters most at the robot, where
the person reading the error is standing up holding an E-stop.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

_ENV = Path(__file__).resolve().parent / "env.sh"


def require_domain() -> None:
    """ROS_DOMAIN_ID must be set EXPLICITLY, because the default is silently wrong.

    An unset ROS_DOMAIN_ID means domain 0. Hardware is 9 and the sim is 42, so a node started
    without it comes up perfectly healthy on an empty graph: it finds no /odom, no /scan and no
    mux, and reports "no /odom -- is ranger_bringup running?". The driver IS running. The node is
    just listening on a different DDS partition, and nothing in the error says so.

    rclpy imports fine in that state, so the check above cannot catch it. This is the same
    silent-mismatch shape as the RC switch and the missing /scan_filtered: everything looks
    healthy and the two halves cannot hear each other."""
    if os.environ.get("ROS_DOMAIN_ID"):
        return
    print(f"ROS_DOMAIN_ID is not set, so this would run on domain 0 -- an EMPTY graph.\n"
          f"Hardware is domain 9, the sim is 42. Nothing would be found and the error would\n"
          f"blame the driver.\n"
          f"\n"
          f"    source {_ENV}\n"
          f"    python3 {' '.join(sys.argv)}\n", file=sys.stderr)
    raise SystemExit(1)


def require_ros() -> None:
    """Import-check rclpy and the domain, or exit with the command that fixes it."""
    try:
        import rclpy  # noqa: F401
    except ModuleNotFoundError as e:
        if e.name not in ("rclpy", "sensor_msgs", "nav_msgs", "geometry_msgs", "tf2_ros"):
            raise
        print(f"ROS 2 is not on this shell's environment (no module '{e.name}').\n"
              f"\n"
              f"    source {_ENV}\n"
              f"    python3 {' '.join(sys.argv)}\n"
              f"\n"
              f"env.sh sources /opt/ros/jazzy, the workspace overlay, and sets ROS_DOMAIN_ID -- "
              f"which must match\nthe rest of the stack or the node will start, find nothing, and "
              f"look like a hardware fault.", file=sys.stderr)
        raise SystemExit(1)
    require_domain()
