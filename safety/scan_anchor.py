"""Anchor the odometry frame to the world with a lidar scan, so waypoints survive drift.

Pure geometry, no ROS. The ROS side is `waypoints.py anchor` / `waypoints.py relocalize`.

THE PROBLEM THIS SOLVES. Waypoints live in the odom frame and odom drifts continuously -- every
metre, every turn, worst of all every wheel rotation that does not move the body. Measured
2026-08-29: from one recorded 'button' waypoint two runs landed 1.58 m and 1.72 m from the plate
at +30 deg, and the operator reasonably asked whether everything must be re-recorded before every
run. Session ids catch a driver RESTART; nothing catches drift.

THE IDEA. The lidar measures the STATIC WORLD in the robot's frame. Save the scan taken at the
moment 'start' was recorded, together with the odom pose at that moment. Later, park the robot
roughly at start again (a metre and 30 deg of slop is fine), take a scan, and find the rigid
motion (dx, dy, dyaw) that best lays the live scan onto the saved one. That motion is the robot's
displacement from the recorded pose IN THE WORLD, from a sensor with no stake in what odom
believes. Compose it with the saved pose and you have the robot's pose in the RECORDING frame;
odom gives its pose in the CURRENT frame; one correspondence fixes the rigid transform between
the two frames, and every waypoint is re-expressed through it. Record once, relocalize per run.

WHAT LIMITS IT, and each is reported rather than hidden:
  * The A1M8 returns on ~13% of beams here -- ~50 usable points a scan. Enough to fix three
    degrees of freedom against a wall-and-door scene, not enough to be casual about it, so the
    match reports its residual and its margin over the runner-up and REFUSES to apply a weak one.
  * A featureless scene (a bare corridor) matches equally well along its length. Reported as a
    weak margin. Anchor at a spot with structure -- a doorway, a corner.
  * People. A person standing in the scan is a cluster the reference does not have. The cost is
    a TRUNCATED nearest-point distance, so an outlier cluster costs a bounded amount instead of
    dragging the fit.
  * Search window. +-1.0 m and +-40 deg. Park inside that. Outside it, the answer is to
    re-record, and the tool says so.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TRUNC_M = 0.30            # per-point cost cap: a person-sized outlier cluster cannot drag the fit
MAX_RESIDUAL_M = 0.08     # mean truncated distance above this -> not a match, refuse
                          # (against a DENSE reference; a single sparse scan floors at ~15 cm)
MIN_MARGIN = 0.15         # runner-up must be this fraction worse, or the scene is ambiguous
MIN_POINTS = 25           # live sweep floor; the reference should be far denser (accumulate)


def accumulate(scans, angle_min: float, angle_increment: float) -> np.ndarray:
    """Stack several scans taken from ONE stationary pose into a dense reference cloud.

    WHY A SINGLE SCAN IS NOT A REFERENCE. This lidar returns on ~13% of beams -- about 30 points
    a sweep here. Two independent sparse samplings of the same wall do not land on the same
    spots, so point-to-point nearest-neighbour distance between them is ~the spacing of the
    points, not the alignment error: on synthetic scans the residual at the TRUE transform was
    15 cm, above any sane acceptance threshold. Twenty sweeps from one pose put ~600 points on the
    same walls and the floor drops to centimetres. Accumulate the reference; match one live sweep
    against it.
    """
    pts = [scan_to_points(r, angle_min, angle_increment) for r in scans]
    return np.vstack([p for p in pts if len(p)]) if pts else np.zeros((0, 2))


def scan_to_points(ranges, angle_min: float, angle_increment: float,
                   r_min: float = 0.25, r_max: float = 12.0) -> np.ndarray:
    """Valid returns as an (N, 2) array in the robot frame. NaN/inf/self-hits dropped."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_increment
    ok = np.isfinite(r) & (r > r_min) & (r < r_max)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def _transform(pts: np.ndarray, dx: float, dy: float, dyaw: float) -> np.ndarray:
    c, s = math.cos(dyaw), math.sin(dyaw)
    x = pts[:, 0] * c - pts[:, 1] * s + dx
    y = pts[:, 0] * s + pts[:, 1] * c + dy
    return np.stack([x, y], axis=1)


def _cost(moved: np.ndarray, ref: np.ndarray) -> float:
    """Mean truncated nearest-neighbour distance from moved points to the reference."""
    d = np.sqrt(((moved[:, None, :] - ref[None, :, :]) ** 2).sum(-1)).min(axis=1)
    return float(np.minimum(d, TRUNC_M).mean())


@dataclass(frozen=True)
class Match:
    dx: float
    dy: float
    dyaw: float
    residual_m: float        # MEDIAN nearest-neighbour distance at the winner (outlier-proof)
    margin: float            # (runner-up cost - best cost) / best cost; higher is more decisive
    n_live: int
    n_ref: int

    @property
    def ok(self) -> bool:
        return (self.residual_m <= MAX_RESIDUAL_M and self.margin >= MIN_MARGIN
                and self.n_live >= MIN_POINTS and self.n_ref >= MIN_POINTS)

    def why_not(self) -> str:
        if self.n_live < MIN_POINTS or self.n_ref < MIN_POINTS:
            return (f"too few lidar returns to fix three degrees of freedom (live {self.n_live}, "
                    f"reference {self.n_ref}; need {MIN_POINTS}). Anchor somewhere with more "
                    f"structure in view.")
        if self.residual_m > MAX_RESIDUAL_M:
            return (f"best alignment still leaves a {self.residual_m*100:.0f} cm mean residual "
                    f"(limit {MAX_RESIDUAL_M*100:.0f}). The robot is probably outside the "
                    f"+-1 m / +-40 deg search window, or the scene has changed. Park closer to "
                    f"the recorded start, or re-record.")
        if self.margin < MIN_MARGIN:
            return (f"ambiguous: the runner-up alignment is only {self.margin*100:.0f}% worse. "
                    f"A featureless scene matches equally well in several places. Anchor at a "
                    f"spot with a corner or doorway in view.")
        return ""


def match_scans(live: np.ndarray, ref: np.ndarray,
                *, xy_span: float = 1.0, yaw_span_deg: float = 40.0) -> Match:
    """Rigid (dx, dy, dyaw) that best lays ``live`` onto ``ref``. Coarse grid, then refine.

    The transform maps live-frame points into the reference (recording) frame: a point the robot
    sees now at p is the same world point the anchor scan saw at T(p). So T is the robot's pose
    NOW expressed in the frame it had at recording -- which is exactly the displacement we want.
    """
    n_live, n_ref = len(live), len(ref)
    if n_live < MIN_POINTS or n_ref < MIN_POINTS:
        return Match(0.0, 0.0, 0.0, float("inf"), 0.0, n_live, n_ref)

    def search(cx, cy, cyaw, xy_half, yaw_half, xy_step, yaw_step):
        best = None
        second = float("inf")
        xs = np.arange(cx - xy_half, cx + xy_half + 1e-9, xy_step)
        ys = np.arange(cy - xy_half, cy + xy_half + 1e-9, xy_step)
        yaws = np.arange(cyaw - yaw_half, cyaw + yaw_half + 1e-9, yaw_step)
        for yaw in yaws:
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.stack([live[:, 0] * c - live[:, 1] * s,
                            live[:, 0] * s + live[:, 1] * c], axis=1)
            for x in xs:
                for y in ys:
                    cost = _cost(rot + np.array([x, y]), ref)
                    if best is None or cost < best[0]:
                        if best is not None and (abs(x - best[1]) > 0.25 or abs(y - best[2]) > 0.25
                                                 or abs(yaw - best[3]) > math.radians(8)):
                            second = min(second, best[0])
                        best = (cost, x, y, yaw)
                    elif (abs(x - best[1]) > 0.25 or abs(y - best[2]) > 0.25
                          or abs(yaw - best[3]) > math.radians(8)):
                        second = min(second, cost)
        return best, second

    ys_ = math.radians(yaw_span_deg)
    coarse, second = search(0.0, 0.0, 0.0, xy_span, ys_, 0.10, math.radians(2.0))
    fine, _ = search(coarse[1], coarse[2], coarse[3], 0.12, math.radians(2.5), 0.02,
                     math.radians(0.5))
    cost, dx, dy, dyaw = fine
    margin = (second - cost) / cost if cost > 1e-6 and math.isfinite(second) else 0.0
    # The SEARCH uses the truncated mean (a smooth landscape). The ACCEPTANCE uses the median
    # nearest-neighbour distance at the winner: a person standing in the scan is a dozen points
    # each paying the full 0.30 m cap, which lifts a correct alignment's mean from 2 cm to 9 cm
    # and would have it refused. The median does not move for a minority cluster.
    d = np.sqrt(((_transform(live, dx, dy, dyaw)[:, None, :] - ref[None, :, :]) ** 2)
                .sum(-1)).min(axis=1)
    residual = float(np.median(d))
    return Match(float(dx), float(dy), float(dyaw), residual, float(margin), n_live, n_ref)


def compose(pose_a, delta) -> tuple[float, float, float]:
    """pose_a (x, y, yaw) then a motion `delta` (dx, dy, dyaw) expressed in pose_a's frame."""
    x, y, yaw = pose_a
    dx, dy, dyaw = delta
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * dx - s * dy, y + s * dx + c * dy, _wrap(yaw + dyaw))


def frame_transform(pose_in_rec, pose_in_cur):
    """The rigid map T with pose_in_cur = T(pose_in_rec), as (tx, ty, tyaw) -- i.e. how to
    re-express anything recorded in the RECORDING odom frame into the CURRENT odom frame."""
    xr, yr, tr = pose_in_rec
    xc, yc, tc = pose_in_cur
    tyaw = _wrap(tc - tr)
    c, s = math.cos(tyaw), math.sin(tyaw)
    tx = xc - (c * xr - s * yr)
    ty = yc - (s * xr + c * yr)
    return (tx, ty, tyaw)


def apply_transform(T, pose) -> tuple[float, float, float]:
    tx, ty, tyaw = T
    x, y, yaw = pose
    c, s = math.cos(tyaw), math.sin(tyaw)
    return (tx + c * x - s * y, ty + s * x + c * y, _wrap(yaw + tyaw))


def relocalize(anchor_pose, live_pts, ref_pts, odom_pose_now) -> tuple[Match, tuple | None]:
    """Everything above in one call.

    anchor_pose    : odom pose at the moment the anchor scan was taken (recording frame)
    live_pts/ref   : scans as (N,2) robot-frame points
    odom_pose_now  : the robot's pose right now in the CURRENT odom frame
    Returns the match and, if it is trustworthy, the transform to apply to every waypoint.
    """
    m = match_scans(live_pts, ref_pts)
    if not m.ok:
        return m, None
    pose_now_in_rec = compose(anchor_pose, (m.dx, m.dy, m.dyaw))
    return m, frame_transform(pose_now_in_rec, odom_pose_now)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))
