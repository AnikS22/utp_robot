import math
from safety.scan_filter import filtered_ranges, KEEP_HALF_ANGLE_DEG


def _scan():
    """One sample per degree, -179..180, like the measured A1M8 scan."""
    out = filtered_ranges([1.0]*360, math.radians(-179), math.radians(1))
    return lambda deg: out[deg + 179]


def test_forward_and_sides_kept():
    at = _scan()
    assert at(0) == 1.0
    assert at(-90) == 1.0 and at(90) == 1.0


def test_rear_sector_removed():
    """The robot itself lives around 180 deg: arm riser, battery, mast. Measured 0.16-0.19 m."""
    at = _scan()
    assert math.isnan(at(-179)) and math.isnan(at(180))
    assert math.isnan(at(-160)) and math.isnan(at(160))


def test_rejected_beams_are_nan_never_inf():
    """inf would mean 'observed empty to range_max' and would let the chassis clear real
    obstacles behind it out of the costmap. NaN means 'no observation'."""
    at = _scan()
    assert math.isnan(at(180))
    assert not math.isinf(at(180))


def test_the_band_105_to_148_is_kept():
    """The regression this file exists for. At 105 deg the filter discarded ~85 deg of live
    scan -- measured 2-7 m returns, no self-hits -- on a sensor already returning on only ~23%
    of its beams. CAD puts the scan plane above the deck, so the chassis never occludes here."""
    at = _scan()
    for deg in (-145, -120, -106, 106, 120, 145):
        assert at(deg) == 1.0, f"{deg} deg must survive the filter"


def test_guard_band_inside_the_nearest_measured_self_hit():
    """Nearest self-hit measured at -150 deg. Keep a margin rather than sitting on the edge:
    the mount pose carries cm-level uncertainty and the robot rocks on its suspension."""
    assert 140.0 <= KEEP_HALF_ANGLE_DEG <= 149.0


def test_boundaries_are_inclusive():
    out = filtered_ranges([1.0]*3, math.radians(-KEEP_HALF_ANGLE_DEG),
                          math.radians(KEEP_HALF_ANGLE_DEG))
    assert out == [1.0, 1.0, 1.0]
