"""Lift an image box to a 3D point from lidar range along its ray, when depth has nothing.

Pure numpy, no ROS. detect_frame.py calls this after the depth lift fails; grab_frame.py saves
the scan and transforms it needs beside every frame.

WHEN DEPTH HAS NOTHING. Two cases, both measured 2026-08-29:
  * Glass. The atrium doors return no usable depth, while the lidar sees the tape on them.
  * The Isaac sim on this laptop. Its depth topic publishes 100% inf on every frame -- the SDG
    pipeline drops the depth sync edge ("Illegal cycle connection ... WriterSyncGate ignored") --
    while the RTX lidar is fine.

HOW. Move the lidar returns into the CAMERA optical frame through the real transforms (the camera
here is 0.65 m behind and 1.1 m above the lidar, so a shared-origin approximation is not good
enough), keep the returns whose horizontal bearing matches the bbox-centre ray, and take their
median depth along the optical axis as the ray's depth. The result is the point on the pixel ray
at that depth -- exact for a wall-mounted control, up to the lidar's planar sampling.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

BEARING_TOL_RAD = math.radians(1.5)     # tight window: median depth (dense scans, e.g. RTX lidar)
WIDE_TOL_RAD = math.radians(15.0)       # wide window: LINE FIT (sparse scans -- the A1M8 returns on
                                        # ~13% of beams, ~7 deg apart, so a tight window is usually
                                        # empty; a wall is a line, and a line needs only a few points)
MIN_RETURNS = 2
MIN_LINE_POINTS = 3
MIN_Z_M = 0.2
MAX_LINE_RESID_M = 0.025     # a real wall fits to ~5 mm with A1M8 noise; a 0.2 m-wide blob fits to ~3 cm


@dataclass(frozen=True)
class Lift:
    point3d: tuple           # (x, y, z) in the camera optical frame, metres
    depth_m: float           # median depth along the optical axis used
    n_returns: int


def scan_to_cam(ranges, angle_min: float, angle_increment: float,
                T_cam_lidar: np.ndarray, r_min: float = 0.25, r_max: float = 12.0) -> np.ndarray:
    """Valid lidar returns as (N, 3) points in the camera optical frame."""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + np.arange(len(r)) * angle_increment
    ok = np.isfinite(r) & (r > r_min) & (r < r_max)
    n = int(ok.sum())
    if n == 0:
        return np.zeros((0, 3))
    pl = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), np.zeros(n), np.ones(n)])
    return (np.asarray(T_cam_lidar, dtype=float) @ pl)[:3].T


def lift_bbox(bbox, K, pts_cam: np.ndarray, *, tol_rad: float = BEARING_TOL_RAD,
              min_returns: int = MIN_RETURNS) -> Lift | None:
    """3D point for the centre of ``bbox`` from lidar returns already in the camera frame.

    Two estimators, tried in order:
      1. MEDIAN of returns within +-1.5 deg of the ray (dense scans).
      2. LINE FIT to returns within +-15 deg, intersected with the ray (sparse scans, and
         oblique walls, where a wide-window median would be biased). The wall the control is
         mounted on is a line in the horizontal plane; three returns pin it.
    """
    K = np.asarray(K, dtype=float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x0, y0, x1, y1 = bbox
    u, v = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    bearing = math.atan2(ray[0], ray[2])
    ahead = pts_cam[pts_cam[:, 2] > MIN_Z_M] if len(pts_cam) else pts_cam
    if len(ahead) == 0:
        return None
    b = np.arctan2(ahead[:, 0], ahead[:, 2])
    d = np.abs(np.arctan2(np.sin(b - bearing), np.cos(b - bearing)))

    near = ahead[d < tol_rad]
    if len(near) >= min_returns:
        z = float(np.median(near[:, 2]))
        return Lift(tuple(float(c) for c in ray * z), z, int(len(near)))

    wide = ahead[d < WIDE_TOL_RAD]
    if len(wide) < MIN_LINE_POINTS:
        return None
    # Horizontal-plane line through the returns: total least squares on (x, z).
    xz = wide[:, [0, 2]]
    c = xz.mean(axis=0)
    _, _, vt = np.linalg.svd(xz - c)
    direction = vt[0]                      # unit vector along the wall
    normal = np.array([-direction[1], direction[0]])
    resid = float(np.abs((xz - c) @ normal).mean())
    if resid > MAX_LINE_RESID_M:
        return None                        # not a wall: a person, a corner, clutter
    # Intersect the ray r(t) = t * (ray_x, ray_z) with the line n . (p - c) = 0.
    r2 = np.array([ray[0], ray[2]])
    denom = float(normal @ r2)
    if abs(denom) < 1e-6:
        return None
    t = float(normal @ c) / denom
    if t <= 0:
        return None
    z = t * 1.0                            # ray_z is 1
    if z < MIN_Z_M:
        return None
    return Lift(tuple(float(cc) for cc in ray * z), z, int(len(wide)))
