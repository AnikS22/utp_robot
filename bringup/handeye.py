#!/usr/bin/env python3
"""Rigid-transform solve for the mast camera — CALIBRATION.md item 8. Pure numpy, no hardware.

The camera is bolted to a mast on the CHASSIS, not to the arm. So this is a base-to-sensor
calibration, not classic eye-in-hand: nothing about the transform changes when the arm moves. The
arm is only a way to put a marker at points whose position we already know exactly.

FRAMES -- get this wrong and every press is off by the riser height
-------------------------------------------------------------------
The xArm SDK's get_position() returns the tool tip in the ARM's OWN base frame (`link_base`), which
sits on top of the riser. The robot's `base_link` is on the chassis deck, below it. So the solve
here produces

    link_base <- mast_cam_optical         (call it T_arm_cam)

and the transform the stack actually consumes is

    base_link <- mast_cam_optical  =  (base_link <- link_base) @ T_arm_cam

where `base_link <- link_base` is CALIBRATION.md item 1, the riser, and is a pure translation. Doing
item 8 before item 1 does not fail -- it silently folds the unmeasured riser height into the camera
extrinsic, which then looks fine on the calibration points and is wrong everywhere else. compose()
below keeps the two separate on purpose.

WHY KABSCH AND NOT cv2.calibrateHandEye
---------------------------------------
`cv2.calibrateHandEye` solves AX=XB for a camera rigidly attached to a MOVING gripper. Our camera
does not move with the arm, so that formulation does not apply. What we have is two sets of the same
physical points expressed in two frames, which is the Kabsch/Umeyama problem and has a closed-form
SVD solution with no initial guess and no iteration.

DEGENERACY -- the failure this file exists to catch
---------------------------------------------------
CALIBRATION.md warns that coplanar points "give a solution that looks good and extrapolates badly".
That is not a soft warning: with all points on a plane the rotation about the plane normal is poorly
constrained, RMS on the calibration set stays small, and error explodes off it. A calibration that
reports 4 mm RMS and misses a real plate by 10 cm is worse than no calibration, because it is
believed. spread() measures it and solve() refuses to stay quiet about it.
"""
from __future__ import annotations

import numpy as np

MIN_POINTS = 4          # 3 is the algebraic minimum; 4 is the minimum that can show residual
PLANARITY_WARN = 0.02   # smallest PCA extent, metres, below which the set is effectively coplanar


def kabsch(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares rigid transform mapping src onto dst. Returns (R, t) with dst ~= R @ src + t.

    Rotation only -- NO scale. Umeyama with scaling would happily absorb a systematic depth-scale
    error into `s` and report a beautiful residual while every metric distance stayed wrong. Both
    frames here are metric by construction, so a fitted scale could only ever hide a bug.
    """
    src = np.asarray(src, float).reshape(-1, 3)
    dst = np.asarray(dst, float).reshape(-1, 3)
    if src.shape != dst.shape:
        raise ValueError(f"point sets differ in shape: {src.shape} vs {dst.shape}")
    if len(src) < 3:
        raise ValueError(f"need at least 3 point pairs, got {len(src)}")

    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    # The det correction is what keeps this a ROTATION. Without it, a noisy or near-degenerate set
    # can produce a reflection: residuals look fine and the arm drives to mirrored points.
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, cd - R @ cs


def residuals(R: np.ndarray, t: np.ndarray, src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Per-point Euclidean error in metres, after applying (R, t) to src."""
    src = np.asarray(src, float).reshape(-1, 3)
    dst = np.asarray(dst, float).reshape(-1, 3)
    return np.linalg.norm((R @ src.T).T + t - dst, axis=1)


def spread(pts: np.ndarray) -> np.ndarray:
    """PCA extents (metres, descending). The third value near zero means a coplanar set."""
    pts = np.asarray(pts, float).reshape(-1, 3)
    if len(pts) < 3:
        return np.zeros(3)
    return np.sqrt(np.linalg.eigvalsh(np.cov((pts - pts.mean(0)).T))[::-1]) * 2.0


def homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def compose(T_base_arm: np.ndarray, T_arm_cam: np.ndarray) -> np.ndarray:
    """base_link <- mast_cam_optical, from the riser and the solved camera transform.

    Kept as an explicit step rather than folded into solve(): the riser is measured with a tape
    (item 1) and the camera transform is solved from data (item 8). Two different provenances, two
    different error bars, and merging them hides which one is wrong when a press misses.
    """
    return np.asarray(T_base_arm, float) @ np.asarray(T_arm_cam, float)


def rotation_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """Fixed-axis XYZ (roll, pitch, yaw) in radians -- the convention ROS static TF publishers use."""
    sy = -R[2, 0]
    if abs(sy) > 0.999999:                       # gimbal lock: yaw is arbitrary, fold it into roll
        return float(np.arctan2(-R[1, 2], R[1, 1])), float(np.arcsin(np.clip(sy, -1, 1))), 0.0
    return (float(np.arctan2(R[2, 1], R[2, 2])),
            float(np.arcsin(np.clip(sy, -1, 1))),
            float(np.arctan2(R[1, 0], R[0, 0])))


def solve(cam_pts, arm_pts, *, accept_rms_m: float = 0.02, accept_max_m: float = 0.04) -> dict:
    """Solve link_base <- mast_cam_optical and judge it against CALIBRATION.md item 8.

    cam_pts : marker positions in the camera optical frame, metres
    arm_pts : the SAME physical points from arm forward kinematics, in link_base, metres

    Returns a dict carrying the transform, the residuals, and an explicit pass/fail with reasons.
    It never raises on a bad calibration -- a bad result is data and must be recorded, not thrown.
    """
    cam = np.asarray(cam_pts, float).reshape(-1, 3)
    arm = np.asarray(arm_pts, float).reshape(-1, 3)

    R, t = kabsch(cam, arm)
    res = residuals(R, t, cam, arm)
    ext = spread(cam)

    reasons: list[str] = []
    if len(cam) < MIN_POINTS:
        reasons.append(f"only {len(cam)} point pairs; want 8-10 well spread")
    rms = float(np.sqrt((res ** 2).mean()))
    if rms > accept_rms_m:
        reasons.append(f"RMS {rms*100:.1f} cm > {accept_rms_m*100:.0f} cm")
    if res.max() > accept_max_m:
        reasons.append(f"worst point {res.max()*100:.1f} cm > {accept_max_m*100:.0f} cm")
    if ext[2] < PLANARITY_WARN:
        reasons.append(f"points are near-coplanar (thinnest extent {ext[2]*100:.1f} cm) — "
                       "rotation about the plane normal is weakly constrained; vary DEPTH")

    return {
        "R": R,
        "t": t,
        "T_arm_cam": homogeneous(R, t),
        "rpy_rad": rotation_to_rpy(R),
        "residuals_m": res,
        "rms_m": rms,
        "max_m": float(res.max()),
        "n": int(len(cam)),
        "spread_m": ext,
        "passed": not reasons,
        "reasons": reasons,
    }


def report(sol: dict) -> str:
    R, t = sol["R"], sol["t"]
    rpy = np.degrees(sol["rpy_rad"])
    L = [
        f"n points        : {sol['n']}",
        f"translation (m) : x={t[0]:+.4f}  y={t[1]:+.4f}  z={t[2]:+.4f}",
        f"rotation (deg)  : roll={rpy[0]:+.2f}  pitch={rpy[1]:+.2f}  yaw={rpy[2]:+.2f}",
        f"point spread (m): {sol['spread_m'][0]:.3f} / {sol['spread_m'][1]:.3f} / {sol['spread_m'][2]:.3f}",
        f"RMS residual    : {sol['rms_m']*100:.2f} cm     (accept < 2 cm)",
        f"worst residual  : {sol['max_m']*100:.2f} cm     (accept < 4 cm)",
        "",
        "per-point residual (cm): " + " ".join(f"{r*100:.1f}" for r in sol["residuals_m"]),
        "",
        f"VERDICT: {'PASS' if sol['passed'] else 'FAIL'}",
    ]
    L += [f"  - {r}" for r in sol["reasons"]]
    if sol["passed"]:
        L.append("  RMS residual IS the press error budget. Record it in EXPERIMENT_LOG.md.")
    return "\n".join(L)
