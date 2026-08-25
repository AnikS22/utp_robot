#!/usr/bin/env python3
"""Measure, apply, and use a fail-closed xArm Cartesian safety envelope.

Boundary collection is READ-ONLY. Move the arm with the manufacturer pendant/teach mode while a
person holds the arm E-stop, then record the current TCP at each safe extreme:

    python3 bringup/arm_workspace.py status
    python3 bringup/arm_workspace.py record x_min   # nearest safe point to the rear screen
    python3 bringup/arm_workspace.py record x_max
    python3 bringup/arm_workspace.py record y_min
    python3 bringup/arm_workspace.py record y_max
    python3 bringup/arm_workspace.py record z_min
    python3 bringup/arm_workspace.py record z_max
    python3 bringup/arm_workspace.py apply          # preview only
    python3 bringup/arm_workspace.py apply --go     # controller-enforced reduced/fence mode
    python3 bringup/arm_workspace.py jog --go       # 5 mm keyboard steps, after verified apply

The recorded point is pulled inward by margin_mm. The boundary constrains the configured TCP
origin, not every point on an unknown tool. Set the real TCP/tool payload before close work, and
choose safe extremes with enough clearance for the complete gripper, cables, and stopping distance.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = REPO / "config" / "arm_workspace.yaml"
SDK_PATH = REPO / ".venv-arm" / "lib" / "python3.12" / "site-packages"
LABELS = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")
AXIS = {"x": 0, "y": 1, "z": 2}
ERRORS = {
    1: "arm-controller emergency stop is pressed",
    2: "control-box emergency input is triggered",
    3: "three-state switch emergency stop is pressed",
    19: "collision detected",
    21: "kinematic error",
    22: "self-collision",
    23: "joint angle exceeds limit",
    24: "speed exceeds limit",
    31: "collision caused abnormal current; check the payload setting",
    35: "SAFETY BOUNDARY LIMIT: controller stopped the arm at the configured fence",
}


def load(path: Path) -> dict:
    cfg = yaml.safe_load(path.read_text())
    cfg.setdefault("bounds_mm", {})
    return cfg


def effective_boundary(cfg: dict) -> list[int]:
    """Return SDK order [xmax,xmin,ymax,ymin,zmax,zmin], pulled inward by margin."""
    b = cfg["bounds_mm"]
    missing = [k for k in LABELS if b.get(k) is None]
    if missing:
        raise ValueError("missing measured bounds: " + ", ".join(missing))
    m = float(cfg.get("margin_mm", 20.0))
    mins = {a: float(b[f"{a}_min"]) + m for a in "xyz"}
    maxs = {a: float(b[f"{a}_max"]) - m for a in "xyz"}
    bad = [a for a in "xyz" if mins[a] >= maxs[a]]
    if bad:
        raise ValueError(f"margin leaves an empty envelope on axis: {', '.join(bad)}")
    return [round(maxs["x"]), round(mins["x"]), round(maxs["y"]), round(mins["y"]),
            round(maxs["z"]), round(mins["z"])]


def contains(boundary: list[int], xyz: list[float]) -> bool:
    xmax, xmin, ymax, ymin, zmax, zmin = boundary
    return xmin <= xyz[0] <= xmax and ymin <= xyz[1] <= ymax and zmin <= xyz[2] <= zmax


def connect(ip: str):
    sys.path.insert(0, str(SDK_PATH))
    from xarm.wrapper import XArmAPI
    arm = XArmAPI(ip, is_radian=False, do_not_open=False)
    if not arm.connected:
        raise RuntimeError(f"xArm at {ip} did not connect")
    return arm


def pose(arm) -> list[float]:
    code, p = arm.get_position(is_radian=False)
    if code != 0 or not p:
        raise RuntimeError(f"get_position failed, code={code}")
    return [float(v) for v in p]


def write_bound(path: Path, cfg: dict, label: str, p: list[float]) -> None:
    cfg["bounds_mm"][label] = round(p[AXIS[label[0]]], 1)
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))


def checked(code, operation: str) -> None:
    if code != 0:
        raise RuntimeError(f"{operation} failed with SDK code {code}")


def arm_error(arm) -> str:
    code = int(arm.error_code or 0)
    return f"{code} ({ERRORS.get(code, 'unmapped controller error')})"


def apply_limits(arm, cfg: dict, boundary: list[int]) -> None:
    # SDK documentation requires toggling reduced mode for changed values to take effect.
    checked(arm.set_reduced_mode(False), "disable reduced mode for configuration")
    checked(arm.set_reduced_tcp_boundary(boundary), "set TCP boundary")
    # Preserve the controller's factory ranges except for explicitly measured one-sided limits.
    code, prior = arm.get_reduced_states(is_radian=False)
    checked(code, "read joint ranges")
    joint_range = list(prior[4])
    if cfg.get("joint_limits_deg", {}).get("j2_min") is not None:
        joint_range[2] = float(cfg["joint_limits_deg"]["j2_min"])
    checked(arm.set_reduced_joint_range(joint_range, is_radian=False), "set reduced joint range")
    checked(arm.set_reduced_max_tcp_speed(float(cfg["max_tcp_speed_mm_s"])), "set TCP speed")
    checked(arm.set_reduced_max_joint_speed(float(cfg["max_joint_speed_deg_s"]), is_radian=False),
            "set joint speed")
    checked(arm.set_collision_sensitivity(int(cfg["collision_sensitivity"])),
            "set collision sensitivity")
    checked(arm.set_self_collision_detection(True), "enable self-collision detection")
    checked(arm.set_fence_mode(True), "enable fence mode")
    checked(arm.set_reduced_mode(True), "enable reduced mode")
    code, states = arm.get_reduced_states(is_radian=False)
    checked(code, "read back reduced state")
    if not states or not bool(states[0]) or [int(v) for v in states[1]] != boundary:
        raise RuntimeError(f"controller readback does not match request: {states}")
    if abs(float(states[4][2]) - float(joint_range[2])) > 0.02:
        raise RuntimeError(f"controller J2 readback does not match request: {states[4][2]}")


def jog(arm, cfg: dict, boundary: list[int], step: float) -> None:
    p = pose(arm)
    if not contains(boundary, p[:3]):
        raise RuntimeError(f"current TCP {p[:3]} is outside the configured envelope; not enabling")
    arm.motion_enable(enable=True)
    checked(arm.set_mode(0), "set position mode")
    checked(arm.set_state(0), "set ready state")
    print("Commands: x+/x- y+/y- z+/z- (one 5 mm move), p (pose), q (stop).")
    try:
        while True:
            cmd = input("arm> ").strip().lower()
            if cmd in ("q", "quit", "exit"):
                break
            if cmd == "p":
                print("TCP mm:", [round(v, 2) for v in pose(arm)])
                continue
            if len(cmd) != 2 or cmd[0] not in AXIS or cmd[1] not in "+-":
                print("invalid command")
                continue
            cur = pose(arm)
            target = cur[:]
            target[AXIS[cmd[0]]] += step * (1 if cmd[1] == "+" else -1)
            if not contains(boundary, target[:3]):
                print("BLOCKED: requested TCP is outside the safety envelope")
                continue
            code = arm.set_position(x=target[0], y=target[1], z=target[2], roll=target[3],
                                    pitch=target[4], yaw=target[5], speed=min(20.0, float(
                                    cfg["max_tcp_speed_mm_s"])), mvacc=50.0,
                                    is_radian=False, wait=True, timeout=10)
            if code != 0 or arm.error_code:
                arm.set_state(4)
                raise RuntimeError(f"move stopped: SDK return={code}, controller error="
                                   f"{arm_error(arm)}, state={arm.state}")
            print("TCP mm:", [round(v, 2) for v in pose(arm)[:3]])
    finally:
        arm.set_state(4)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=("status", "record", "apply", "jog"))
    ap.add_argument("label", nargs="?", choices=LABELS)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--ip", default=os.environ.get("UTP_XARM_IP", "192.168.1.221"))
    ap.add_argument("--go", action="store_true", help="required for any controller change or motion")
    ap.add_argument("--step-mm", type=float, default=5.0)
    a = ap.parse_args()
    cfg = load(a.config)
    arm = connect(a.ip)
    try:
        p = pose(arm)
        print(f"TCP mm: x={p[0]:.1f} y={p[1]:.1f} z={p[2]:.1f}  "
              f"rpy={p[3]:.1f},{p[4]:.1f},{p[5]:.1f}")
        print(f"state={arm.state} mode={arm.mode} error={arm.error_code}")
        if a.command == "status":
            print("configured bounds:", cfg["bounds_mm"])
            code, states = arm.get_reduced_states(is_radian=False)
            print("controller reduced state:", states if code == 0 else f"read failed {code}")
            return 0
        if a.command == "record":
            if not a.label:
                raise ValueError("record requires one of: " + ", ".join(LABELS))
            write_bound(a.config, cfg, a.label, p)
            print(f"recorded {a.label}; READ-ONLY, no motion was enabled")
            return 0
        boundary = effective_boundary(cfg)
        print("effective SDK boundary [xmax,xmin,ymax,ymin,zmax,zmin]:", boundary)
        if not a.go:
            print("preview only; pass --go to change the controller or enable jogging")
            return 0
        if arm.error_code:
            raise RuntimeError(f"arm has error {arm_error(arm)}; inspect it before clearing")
        apply_limits(arm, cfg, boundary)
        print("verified: reduced mode + fence boundary active in controller")
        if a.command == "jog":
            if not 0 < a.step_mm <= 10:
                raise ValueError("--step-mm must be >0 and <=10")
            jog(arm, cfg, boundary, a.step_mm)
        return 0
    finally:
        arm.disconnect()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as e:
        print(f"STOP: {e}", file=sys.stderr)
        raise SystemExit(1)
