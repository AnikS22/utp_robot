"""Lifting a box from the lidar must recover the wall's depth along that pixel's ray."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.lidar_lift import lift_bbox, scan_to_cam

# Camera intrinsics like the D435 colour stream.
K = [[910.0, 0.0, 640.0], [0.0, 910.0, 360.0], [0.0, 0.0, 1.0]]

# Lidar frame: x forward, y left, z up. Camera optical: x right, y down, z forward, and the
# camera sits 0.65 m BEHIND and 1.1 m ABOVE the lidar (the real mount).
R_opt = np.array([[0.0, -1.0, 0.0],      # opt x = -lidar y
                  [0.0, 0.0, -1.0],      # opt y = -lidar z
                  [1.0, 0.0, 0.0]])      # opt z =  lidar x
T = np.eye(4)
T[:3, :3] = R_opt
T[:3, 3] = R_opt @ np.array([0.65, 0.0, -1.1])          # p_cam = R (p_lidar - c), c = (-0.65, 0, 1.1)


def wall_scan(dist_from_lidar: float, n: int = 360, keep: float = 0.15, seed: int = 0):
    """A wall perpendicular to +x at the given distance, sparse like the A1M8."""
    rng = np.random.default_rng(seed)
    amin, inc = -math.pi, 2 * math.pi / n
    out = []
    for i in range(n):
        a = amin + i * inc
        if math.cos(a) > 0.05 and rng.random() < keep:
            out.append(dist_from_lidar / math.cos(a) + rng.normal(0, 0.005))
        else:
            out.append(float("nan"))
    return out, amin, inc


def test_recovers_wall_depth_on_the_camera_ray():
    """Wall 2.0 m ahead of the lidar is 2.65 m ahead of the camera (0.65 m further back)."""
    ranges, amin, inc = wall_scan(2.0)
    pts = scan_to_cam(ranges, amin, inc, T)
    lift = lift_bbox((600, 380, 680, 460), K, pts)      # box near image centre
    assert lift is not None
    assert abs(lift.depth_m - 2.65) < 0.05, lift
    assert lift.n_returns >= 2


def test_off_centre_box_lands_on_the_ray_not_the_axis():
    """A box 300 px left of centre must lift to a point with negative optical x."""
    ranges, amin, inc = wall_scan(1.5)
    pts = scan_to_cam(ranges, amin, inc, T)
    lift = lift_bbox((300, 380, 380, 460), K, pts)
    assert lift is not None
    x, y, z = lift.point3d
    assert x < -0.5 and abs(z - 2.15) < 0.06


def test_no_returns_refuses():
    assert lift_bbox((600, 380, 680, 460), K, np.zeros((0, 3))) is None


def test_sparse_a1m8_scan_still_lifts_via_the_line_fit():
    """~13% of beams, ~7 deg apart: a 1.5 deg window is empty; the wall is still a line."""
    ranges, amin, inc = wall_scan(2.0, keep=0.13, seed=5)
    pts = scan_to_cam(ranges, amin, inc, T)
    lift = lift_bbox((600, 380, 680, 460), K, pts)
    assert lift is not None and abs(lift.depth_m - 2.65) < 0.05, lift


def test_a_person_cluster_is_not_mistaken_for_a_wall():
    """A blob 0.4 m across 1 m ahead fits no line: residual over the limit -> refuse."""
    # Off the ray (bearings 3-12 deg) so the tight-window median cannot fire and the line fit
    # has to judge it. A blob ON the ray legitimately returns its depth: that is what the pixel
    # sees, and a detector would have boxed the person, not the wall behind them.
    rng = np.random.default_rng(1)
    z = rng.uniform(0.9, 1.3, 12)
    x = z * np.tan(np.radians(rng.uniform(3.0, 12.0, 12)))
    blob = np.stack([x, np.full(12, 1.1), z], axis=1)
    assert lift_bbox((600, 380, 680, 460), K, blob) is None


def test_returns_behind_the_camera_are_ignored():
    pts = np.array([[0.0, 0.0, -1.0], [0.0, 0.0, -2.0]])
    assert lift_bbox((600, 380, 680, 460), K, pts) is None
