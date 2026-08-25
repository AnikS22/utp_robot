"""Turn `ModuleNotFoundError: rclpy` into an instruction.

ROS 2 lives in a shell environment, not in the interpreter: `rclpy` is only importable after
/opt/ros/jazzy/setup.bash has been sourced. A python script cannot source it for you -- sourcing
mutates the PARENT shell, and by the time this module runs the interpreter has already started.

So the best it can do is say so precisely. A bare traceback reads as "the tool is broken"; it is
not, the shell just is not set up, and the fix is one line. This matters most at the robot, where
the person reading the error is standing up holding an E-stop.
"""
from __future__ import annotations

import sys
from pathlib import Path

_ENV = Path(__file__).resolve().parent / "env.sh"


def require_ros() -> None:
    """Import-check rclpy, or exit with the command that fixes it."""
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
