import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "safety"))
from scan_temporal_filter import NearFieldConfirm


def test_flickering_near_return_never_reaches_nav():
    f = NearFieldConfirm(confirmations=3)
    seen = []
    for value in (2.5, .48, 2.5, .46, 2.5, .51, 2.5):
        out, _ = f.filter([value]); seen.append(out[0])
    assert all(v == 2.5 or math.isinf(v) for v in seen)


def test_real_near_obstacle_is_admitted_after_confirmation():
    f = NearFieldConfirm(confirmations=3)
    a, _ = f.filter([.50]); b, _ = f.filter([.51]); c, _ = f.filter([.49])
    assert math.isinf(a[0]) and math.isinf(b[0]) and c[0] == .49


def test_neighboring_bins_track_small_angular_jitter():
    f = NearFieldConfirm(confirmations=3, neighbor_bins=1)
    f.filter([.5, math.inf, math.inf])
    f.filter([math.inf, .51, math.inf])
    out, _ = f.filter([math.inf, math.inf, .49])
    assert out[2] == .49


def test_clearing_and_far_ranges_pass_without_delay():
    f = NearFieldConfirm(confirmations=3)
    assert f.filter([math.inf, 2.0, 0.0])[0] == [math.inf, 2.0, 0.0]


def test_measured_rear_phantom_ring_is_always_removed():
    f = NearFieldConfirm()
    for value in (.85, 1.0, 1.20, 1.30):
        out, n = f.filter([value], angle_min=math.pi, angle_increment=.01)
        assert math.isinf(out[0]) and n == 1


def test_real_rear_surface_beyond_phantom_radius_survives():
    f = NearFieldConfirm()
    out, n = f.filter([2.55], angle_min=math.pi, angle_increment=.01)
    assert out == [2.55] and n == 0


def test_close_forward_obstacle_is_not_geometrically_masked():
    f = NearFieldConfirm(confirmations=1)
    out, _ = f.filter([.50], angle_min=0.0, angle_increment=.01)
    assert out == [.50]
