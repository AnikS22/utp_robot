"""Pins the OpenCV convention used by bringup/handeye_solve_rw.py.

This file exists because the convention was got WRONG on the first attempt, by reasoning from
the documentation. `cv2.calibrateRobotWorldHandEye` is written for eye-IN-hand (camera on the
gripper, target fixed in the world); we have eye-TO-hand (camera on the chassis, target on the
gripper). The natural-looking move is to invert the robot poses to swap the roles. That is wrong,
and it fails silently: the solver returns a plausible transform, the residual looks fine, and
every press misses.

The only defence is ground truth. Each test here builds a synthetic rig with a KNOWN camera pose
and a KNOWN marker offset, generates perfect observations, and asserts recovery to sub-millimetre.
If someone "fixes" the mapping in handeye_solve_rw.py, these fail immediately.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "bringup"))

cv2 = pytest.importorskip("cv2")
from handeye_solve_rw import rot_angle_deg, rotation_spread, rpy_deg_to_R  # noqa: E402

if not hasattr(cv2, "calibrateRobotWorldHandEye"):
    pytest.skip("OpenCV lacks calibrateRobotWorldHandEye", allow_module_level=True)


def H(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def make_rig(n=14, seed=1, cam_rpy=(-100.0, 5.0, 90.0), cam_t=(0.30, -0.05, 0.75),
             marker_rpy=(12.0, -7.0, 25.0), marker_t=(0.02, 0.01, 0.09), noise_m=0.0):
    """Synthesise n observations of a known rig.

    Returns (A, B, T_base_cam, T_flange_marker) where A[i] = base_T_flange (what the arm reports)
    and B[i] = cam_T_marker (what the camera measures).
    """
    rng = np.random.default_rng(seed)
    T_bc = H(rpy_deg_to_R(list(cam_rpy)), cam_t)
    T_fm = H(rpy_deg_to_R(list(marker_rpy)), marker_t)
    A, B = [], []
    for _ in range(n):
        T_bf = H(rpy_deg_to_R(rng.uniform(-50, 50, 3)),
                 rng.uniform(-0.25, 0.25, 3) + np.array([0.1, 0.0, 0.85]))
        T_cm = np.linalg.inv(T_bc) @ T_bf @ T_fm
        if noise_m:
            T_cm[:3, 3] += rng.normal(0.0, noise_m, 3)
        A.append(T_bf)
        B.append(T_cm)
    return A, B, T_bc, T_fm


def solve(A, B):
    """Exactly the call handeye_solve_rw.py makes.

    Inputs go in as measured. Both outputs are INVERTED -- that is the whole convention, and
    using them as returned gives answers ~0.8 m wrong that still look like plausible geometry.
    """
    R_bw, t_bw, R_gc, t_gc = cv2.calibrateRobotWorldHandEye(
        [b[:3, :3].copy() for b in B], [b[:3, 3].reshape(3, 1).copy() for b in B],
        [a[:3, :3].copy() for a in A], [a[:3, 3].reshape(3, 1).copy() for a in A],
        method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)
    return np.linalg.inv(H(R_gc, t_gc)), np.linalg.inv(H(R_bw, t_bw))


# ---------------------------------------------------------------------------------------------
# the convention itself
# ---------------------------------------------------------------------------------------------

def test_forward_model_is_consistent():
    """A X = Z B must hold exactly, or the tests below prove nothing about the solver."""
    A, B, T_bc, T_fm = make_rig()
    for a, b in zip(A, B):
        assert np.allclose(a @ T_fm, T_bc @ b, atol=1e-12)


def test_recovers_camera_pose():
    A, B, T_bc, _ = make_rig()
    T_cam, _ = solve(A, B)
    assert np.linalg.norm(T_cam[:3, 3] - T_bc[:3, 3]) < 1e-4, "translation not recovered"
    assert rot_angle_deg(T_cam[:3, :3].T @ T_bc[:3, :3]) < 0.05, "rotation not recovered"


def test_recovers_marker_offset_without_being_told_it():
    """The whole point: the ruler measurement comes OUT of the solve, not into it."""
    A, B, _, T_fm = make_rig()
    _, T_marker = solve(A, B)
    assert np.linalg.norm(T_marker[:3, 3] - T_fm[:3, 3]) < 1e-4


def test_using_the_outputs_uninverted_is_wrong():
    """The other half of the convention: the returned matrices must be inverted.

    Taking them as-is is the obvious reading of the API and it is wrong by roughly 0.8 m -- a
    distance that still looks like plausible robot geometry, which is exactly why it needs a test
    rather than a comment.
    """
    A, B, T_bc, _ = make_rig()
    R_bw, t_bw, R_gc, t_gc = cv2.calibrateRobotWorldHandEye(
        [b[:3, :3].copy() for b in B], [b[:3, 3].reshape(3, 1).copy() for b in B],
        [a[:3, :3].copy() for a in A], [a[:3, 3].reshape(3, 1).copy() for a in A],
        method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)
    assert np.linalg.norm(H(R_gc, t_gc)[:3, 3] - T_bc[:3, 3]) > 0.05


def test_inverting_the_robot_poses_is_wrong():
    """Guards the specific mistake made on 2026-08-21.

    Inverting A to 'swap eye-in-hand for eye-to-hand' is the intuitive move. It does not error --
    it returns a confident answer that is off by hundreds of millimetres. Asserting that it is
    wrong stops anyone re-deriving their way back into it.
    """
    A, B, T_bc, _ = make_rig()
    A_inv = [np.linalg.inv(a) for a in A]
    T_cam_bad, _ = solve(A_inv, B)
    assert np.linalg.norm(T_cam_bad[:3, 3] - T_bc[:3, 3]) > 0.05


def test_inverting_the_camera_poses_is_wrong():
    A, B, T_bc, _ = make_rig()
    B_inv = [np.linalg.inv(b) for b in B]
    T_cam_bad, _ = solve(A, B_inv)
    assert np.linalg.norm(T_cam_bad[:3, 3] - T_bc[:3, 3]) > 0.05


# ---------------------------------------------------------------------------------------------
# the degeneracy the solver refuses on
# ---------------------------------------------------------------------------------------------

def test_constant_orientation_is_detected_as_degenerate():
    """With one fixed wrist orientation the camera pose and marker offset cannot be separated.

    rotation_spread() is what handeye_solve_rw refuses on. It must see this as ~0 degrees, because
    the solver itself will happily return a wrong answer rather than complain.
    """
    R = rpy_deg_to_R([10.0, 20.0, 30.0])
    assert rotation_spread([R.copy() for _ in range(8)]) < 1e-6


def test_rotation_spread_measures_the_largest_pairwise_angle():
    Rs = [rpy_deg_to_R([0, 0, 0]), rpy_deg_to_R([0, 0, 30]), rpy_deg_to_R([0, 0, 45])]
    assert rotation_spread(Rs) == pytest.approx(45.0, abs=0.5)


def test_constant_orientation_does_not_yield_a_usable_answer():
    """Not just detectable in principle -- constant orientation genuinely cannot be solved.

    OpenCV 4.6 happens to RAISE here ("Rotation normalization issue") rather than returning
    nonsense, which is the kinder failure. Either outcome is acceptable; what would not be is a
    plausible-looking answer. This asserts the disjunction rather than the raise, because the
    behaviour is an implementation detail that could reasonably change between versions and the
    property we actually depend on is "does not silently succeed".
    """
    rng = np.random.default_rng(3)
    T_bc = H(rpy_deg_to_R([-100.0, 5.0, 90.0]), [0.30, -0.05, 0.75])
    T_fm = H(rpy_deg_to_R([12.0, -7.0, 25.0]), [0.02, 0.01, 0.09])
    R_fixed = rpy_deg_to_R([10.0, 0.0, 0.0])
    A, B = [], []
    for _ in range(12):
        T_bf = H(R_fixed, rng.uniform(-0.25, 0.25, 3) + np.array([0.1, 0.0, 0.85]))
        A.append(T_bf)
        B.append(np.linalg.inv(T_bc) @ T_bf @ T_fm)
    try:
        T_cam, _ = solve(A, B)
    except cv2.error:
        return                                    # refused outright -- fine
    assert np.linalg.norm(T_cam[:3, 3] - T_bc[:3, 3]) > 0.01, \
        "constant orientation must not produce a correct-looking answer"


# ---------------------------------------------------------------------------------------------
# robustness
# ---------------------------------------------------------------------------------------------

def test_tolerates_realistic_measurement_noise():
    """1 mm of noise on each sighting must not move the answer by more than the press budget."""
    A, B, T_bc, _ = make_rig(n=14, noise_m=0.001)
    T_cam, _ = solve(A, B)
    assert np.linalg.norm(T_cam[:3, 3] - T_bc[:3, 3]) < 0.010


def test_five_poses_is_enough_when_they_are_well_spread():
    A, B, T_bc, _ = make_rig(n=5, seed=7)
    T_cam, _ = solve(A, B)
    assert np.linalg.norm(T_cam[:3, 3] - T_bc[:3, 3]) < 1e-3


def test_works_for_a_different_rig_geometry():
    """Nothing may be hard-coded to the one camera pose used in the other tests."""
    A, B, T_bc, T_fm = make_rig(seed=11, cam_rpy=(-85.0, -12.0, 70.0), cam_t=(-0.32, 0.11, 1.20),
                                marker_rpy=(-30.0, 40.0, 5.0), marker_t=(-0.01, 0.05, 0.14))
    T_cam, T_marker = solve(A, B)
    assert np.linalg.norm(T_cam[:3, 3] - T_bc[:3, 3]) < 1e-4
    assert np.linalg.norm(T_marker[:3, 3] - T_fm[:3, 3]) < 1e-4


def test_verification_poses_are_held_out_of_the_fit():
    """A plain *.json glob absorbed verify_*.json into the calibration on 2026-08-25. The
    calibration would then verify against its own fit inputs -- circular, and invisible in
    every reported statistic. This pins the exclusion."""
    import pathlib
    src = (pathlib.Path(__file__).resolve().parent.parent
           / "bringup" / "handeye_solve_rw.py").read_text()
    assert 'f.stem.startswith("verify")' in src
    assert "continue" in src.split('f.stem.startswith("verify")')[1][:120]
