"""Hand-eye solver tests — CALIBRATION.md item 8. Synthetic data only, no hardware.

The solver's output IS the press error budget, so it gets the same treatment as the safety arbiter:
the part that decides whether the robot hits the plate is the part that is tested. Every case below
builds points from a KNOWN transform and checks the solve recovers it, because a calibration routine
that is subtly wrong reports a small residual and misses in the real world — the failure mode is a
confident wrong answer, not a crash.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "bringup"))
from handeye import (  # noqa: E402
    compose, homogeneous, kabsch, residuals, rotation_to_rpy, solve, spread,
)


def rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch),
                              np.sin(pitch), np.cos(yaw), np.sin(yaw))
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def volumetric(n: int = 12, seed: int = 0) -> np.ndarray:
    """Well-spread, non-coplanar points — what CALIBRATION.md asks the operator to collect."""
    rng = np.random.default_rng(seed)
    return rng.uniform([-0.4, -0.4, 0.5], [0.4, 0.4, 1.3], size=(n, 3))


# ---- exact recovery ---------------------------------------------------------------------------
def test_recovers_known_transform_exactly():
    R_true, t_true = rot(0.1, -0.2, 0.3), np.array([0.15, -0.25, 1.10])
    cam = volumetric()
    arm = (R_true @ cam.T).T + t_true

    sol = solve(cam, arm)
    assert np.allclose(sol["R"], R_true, atol=1e-9)
    assert np.allclose(sol["t"], t_true, atol=1e-9)
    assert sol["rms_m"] < 1e-9
    assert sol["passed"]


def test_identity_transform():
    cam = volumetric()
    sol = solve(cam, cam.copy())
    assert np.allclose(sol["R"], np.eye(3), atol=1e-9)
    assert np.allclose(sol["t"], 0.0, atol=1e-9)


def test_pure_translation_has_no_rotation():
    cam = volumetric()
    sol = solve(cam, cam + np.array([0.3, -0.1, 0.9]))
    assert np.allclose(sol["R"], np.eye(3), atol=1e-9)
    assert np.allclose(sol["t"], [0.3, -0.1, 0.9], atol=1e-9)


# ---- the reflection trap ----------------------------------------------------------------------
def test_result_is_a_rotation_not_a_reflection():
    """det(R) must be +1. A reflection fits mirrored data with a small residual and drives the
    arm to mirrored points — the determinant correction in kabsch() is what prevents it."""
    rng = np.random.default_rng(7)
    for seed in range(6):
        cam = volumetric(8, seed)
        arm = (rot(*rng.uniform(-np.pi, np.pi, 3)) @ cam.T).T + rng.uniform(-1, 1, 3)
        assert np.linalg.det(solve(cam, arm)["R"]) == pytest.approx(1.0, abs=1e-9)


def test_mirrored_data_does_not_fit_well():
    """Mirroring one axis is NOT a rigid motion, so the solve must show it as a large residual
    rather than silently returning a reflection that fits."""
    cam = volumetric(10)
    sol = solve(cam, cam * np.array([1.0, 1.0, -1.0]))
    assert sol["rms_m"] > 0.05
    assert not sol["passed"]


# ---- noise and the acceptance criteria --------------------------------------------------------
def test_small_noise_still_passes_and_stays_near_truth():
    R_true, t_true = rot(0.05, 0.1, -0.15), np.array([0.2, 0.0, 1.05])
    cam = volumetric(12, seed=3)
    rng = np.random.default_rng(11)
    arm = (R_true @ cam.T).T + t_true + rng.normal(0, 0.003, (12, 3))   # 3 mm sensor noise

    sol = solve(cam, arm)
    assert sol["passed"], sol["reasons"]
    assert sol["rms_m"] < 0.02
    assert np.linalg.norm(sol["t"] - t_true) < 0.01


def test_gross_outlier_is_reported_not_hidden():
    """One badly-detected marker must fail the max-residual criterion. Kabsch is least-squares and
    has no outlier rejection: the guard is that the operator SEES it, not that it is absorbed."""
    R_true, t_true = rot(0.0, 0.0, 0.2), np.array([0.1, 0.1, 1.0])
    cam = volumetric(10, seed=5)
    arm = (R_true @ cam.T).T + t_true
    arm[4] += np.array([0.25, 0.0, 0.0])        # 25 cm blunder on one point

    sol = solve(cam, arm)
    assert not sol["passed"]
    assert sol["max_m"] > 0.04
    assert any("worst point" in r for r in sol["reasons"])
    assert int(np.argmax(sol["residuals_m"])) == 4


# ---- degeneracy: the documented trap ----------------------------------------------------------
def test_coplanar_points_are_flagged_even_when_residual_is_tiny():
    """The case CALIBRATION.md warns about: a planar target gives a near-zero residual and a
    solution that extrapolates badly. Residual alone cannot detect it; the spread check can."""
    rng = np.random.default_rng(2)
    cam = np.column_stack([rng.uniform(-0.3, 0.3, 12),
                           rng.uniform(-0.3, 0.3, 12),
                           np.full(12, 0.9)])                 # all at one depth
    R_true, t_true = rot(0.1, 0.1, 0.1), np.array([0.1, 0.2, 1.0])
    arm = (R_true @ cam.T).T + t_true

    sol = solve(cam, arm)
    assert sol["rms_m"] < 1e-9                                # fits perfectly...
    assert not sol["passed"]                                  # ...and is still rejected
    assert any("coplanar" in r for r in sol["reasons"])


def test_spread_separates_planar_from_volumetric():
    planar = np.column_stack([np.random.default_rng(1).uniform(-0.3, 0.3, (20, 2)), np.full(20, 0.8)])
    assert spread(planar)[2] < 0.02
    assert spread(volumetric(20, seed=4))[2] > 0.05


def test_too_few_points_flagged():
    cam = volumetric(3, seed=9)
    sol = solve(cam, cam.copy())
    assert not sol["passed"]
    assert any("point pairs" in r for r in sol["reasons"])


# ---- frame composition: the riser ------------------------------------------------------------
def test_compose_adds_the_riser_height():
    """base_link <- mast_cam_optical must include the riser. Skipping item 1 folds the riser into
    the camera extrinsic, which is exactly the silent error compose() exists to keep visible."""
    riser = 0.187
    T_base_arm = homogeneous(np.eye(3), np.array([0.0, 0.0, riser]))
    T_arm_cam = homogeneous(rot(0, -0.1745, 0), np.array([-0.25, 0.0, 0.963]))

    T = compose(T_base_arm, T_arm_cam)
    assert T[2, 3] == pytest.approx(0.963 + riser, abs=1e-12)
    assert np.allclose(T[:3, :3], T_arm_cam[:3, :3])          # rotation untouched by a translation


def test_compose_is_not_commutative_when_rotation_present():
    A = homogeneous(rot(0.0, 0.3, 0.0), np.array([0.0, 0.0, 0.2]))
    B = homogeneous(rot(0.1, 0.0, 0.0), np.array([0.4, 0.0, 0.0]))
    assert not np.allclose(compose(A, B), compose(B, A))


# ---- rpy round-trip ---------------------------------------------------------------------------
@pytest.mark.parametrize("rpy", [(0, 0, 0), (0.1, -0.2, 0.3), (-0.5, 0.4, 1.2), (0, -0.1745, 0)])
def test_rpy_round_trip(rpy):
    assert np.allclose(rot(*rotation_to_rpy(rot(*rpy))), rot(*rpy), atol=1e-9)


def test_rpy_gimbal_lock_is_finite():
    r, p, y = rotation_to_rpy(rot(0.0, np.pi / 2, 0.0))
    assert all(np.isfinite(v) for v in (r, p, y))
    assert p == pytest.approx(np.pi / 2, abs=1e-6)


# ---- misuse -----------------------------------------------------------------------------------
def test_mismatched_shapes_raise():
    with pytest.raises(ValueError, match="differ in shape"):
        kabsch(volumetric(5), volumetric(6))


def test_two_points_raise():
    with pytest.raises(ValueError, match="at least 3"):
        kabsch(np.zeros((2, 3)), np.zeros((2, 3)))


def test_residuals_match_solve():
    cam, arm = volumetric(8, seed=6), volumetric(8, seed=8)
    sol = solve(cam, arm)
    assert np.allclose(residuals(sol["R"], sol["t"], cam, arm), sol["residuals_m"])
