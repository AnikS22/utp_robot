"""The self-occlusion mask in bringup/scan_relay.py: what it must remove, and what it must NEVER.

WHY THIS FILE EXISTS. The OS0 sits on a mast at base_link (-0.375, 0, 1.146), behind the stowed
xArm6, and pointcloud_to_laserscan flattens the 0.20-1.20 m band -- so the robot's own arm, mast
and chassis rear are IN the 2D scan. Nav2's obstacle layer marked them LETHAL wrapped around the
footprint, the planner concluded the robot was standing inside an obstacle, and the robot accepted
goals and never moved with 4.10 m clear ahead (2026-09-01).

The first fix was to raise pointcloud_to_laserscan's range_min from 0.50 to 0.70. It silenced the
self-returns and it was WRONG: range_min is a GLOBAL cutoff over every bearing. What that cost is
measurable and is the last test in this file -- captures/trial_ours_001/scan.json, taken at closed
glass doors, has 85 returns within +-20 deg forward whose NEAREST is 0.72 m. The door was two
centimetres above the cutoff. One step closer and it would not have been "seen and ignored", it
would have been GONE from the scan, and the corridor veto, Nav2's obstacle layer and
approach_blockage would all have been looking at an empty corridor with a door in it.

So: a sector mask, and the two failures it sits between.
  * mask too little  -> the robot is an obstacle to itself and cannot plan.
  * mask too much    -> a real obstacle disappears silently, which is the worse one, because the
                        stack looks healthy right up until it drives into something.

A NOTE ON HOW THIS TESTS. scan_relay.py used to do the masking inline in its rclpy callback, which
could only be tested by grepping its source. The masking is now a module-level pure function,
mask_self_returns(ranges, angle_min, angle_increment), sitting ABOVE soft ROS imports -- so these
tests call the code that actually runs on the robot, with no ROS environment and no live graph.
A string-matching test would have passed just as happily with the bearing normalisation inverted.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

from scan_relay import (MASK_MAX_DEG, MASK_MAX_M, MASK_MIN_DEG,  # noqa: E402
                        mask_self_returns)

# The REAL scan geometry on this robot, taken from captures/trial_ours_001/scan.json:
# 1031 bins, angle_min -pi, angle_increment 0.0061 rad. Spans 359.99 deg, so the last bin lands
# just short of +180 -- the wrap-around case the normalisation has to get right.
N = 1031
AMIN = -math.pi
AINC = 0.0061
CAPTURE = REPO / "captures" / "trial_ours_001" / "scan.json"


def _bin(deg: float) -> int:
    """Index of the bin nearest `deg` in the real scan geometry."""
    i = round((math.radians(deg) - AMIN) / AINC)
    assert 0 <= i < N
    return i


def _scan(*, at: dict[float, float], background: float = 5.0) -> list[float]:
    """A synthetic 1031-bin scan: open room at `background`, with returns placed by bearing."""
    r = [background] * N
    for deg, rng in at.items():
        r[_bin(deg)] = rng
    return r


def _deg(i: int) -> float:
    return (math.degrees(AMIN + i * AINC) + 180.0) % 360.0 - 180.0


# --------------------------------------------------------------------------- what must be removed
def test_a_close_return_astern_is_masked():
    """+120 deg at 0.50 m is the stowed arm. MEASURED 2026-09-01, 10 stationary scans on open
    floor with nobody nearby: the +120..+135 sector read min 0.70 / median 0.72, pinned across
    all ten. A return that does not move between scans, only ever appears astern, and sits at a
    fixed radius is the ROBOT."""
    out, n = mask_self_returns(_scan(at={120.0: 0.50}), AMIN, AINC)
    assert out[_bin(120.0)] == float("inf")
    assert n == 1, "exactly one bin should have changed"


def test_the_mask_is_symmetric_about_the_nose():
    """Both rear quarters carry structure, so the test is on |bearing|. Measured minima:
    -120..-105 gave 0.73 and -105..-90 gave 0.79, mirroring +105..+120's 0.70. Masking only the
    left would leave the right-hand half of the robot in the costmap."""
    for deg in (-120.0, -95.0, -160.0, 95.0, 160.0, 179.0):
        out, n = mask_self_returns(_scan(at={deg: 0.50}), AMIN, AINC)
        assert out[_bin(deg)] == float("inf"), f"{deg} deg at 0.50 m survived the mask"
        assert n == 1


def test_the_rear_arc_runs_all_the_way_to_180():
    """THE 155 REGRESSION. MASK_MAX_DEG was 155, from a sweep taken while range_min was 0.50 --
    which had already deleted everything closer than 0.50 m before anything was measured. Those
    sectors then read min 1.02-1.10 m and looked clean. With range_min at 0.30 the same bearings
    on the same stationary robot showed -180..-135 min 0.39 and +135..+180 min 0.55: the chassis
    rear, always there, hidden by the cutoff.

    A CUTOFF THAT HIDES A SELF-RETURN ALSO HIDES THE FACT THAT YOU NEEDED A MASK."""
    assert MASK_MAX_DEG == 180.0
    for deg in (-179.9, -170.0, 160.0, 178.0):
        out, _ = mask_self_returns(_scan(at={deg: 0.45}), AMIN, AINC)
        assert out[_bin(deg)] == float("inf"), \
            f"{deg} deg at 0.45 m survived -- the 155 deg limit is back and the chassis rear " \
            f"is a lethal obstacle wrapped around the footprint again"


# ------------------------------------------------------------------------------ what must survive
def test_a_return_straight_ahead_is_kept():
    """The whole point. The forward hemisphere measured 3.0-8.8 m with the robot stationary --
    it is completely clean of self-returns, so anything seen there is the world."""
    out, n = mask_self_returns(_scan(at={0.0: 0.50}), AMIN, AINC)
    assert out[_bin(0.0)] == 0.50
    assert n == 0


def test_the_forward_arc_up_to_the_boundary_is_kept():
    """MASK_MIN_DEG is 74, fixed by a 5-degree sweep of the forward-left boundary: +75..+90 is
    BIMODAL (median 1.81-1.88 m of real room over a hard floor at 0.71-0.72, spread 1.1-1.3),
    while +70..+75 reads min 1.91 median 1.95 spread 0.09 -- one population, and it is room."""
    assert MASK_MIN_DEG == 74.0
    for deg in (0.0, 45.0, -45.0, 70.0, -70.0, 73.0):
        out, n = mask_self_returns(_scan(at={deg: 0.50}), AMIN, AINC)
        assert out[_bin(deg)] == 0.50, f"a 0.50 m obstacle at {deg} deg was masked away"
        assert n == 0


def test_a_distant_return_astern_is_kept():
    """Beyond MASK_MAX_M the same bearings see the real room, and the mask must not touch it.
    This is why the mask has a radius at all: at +120 deg the median was 0.72 m (structure) but
    at +90..+105 it was 1.67 m (room seen past the arm). Masking the whole bearing would delete
    the wall behind the robot from the map."""
    out, n = mask_self_returns(_scan(at={120.0: 2.0}), AMIN, AINC)
    assert out[_bin(120.0)] == 2.0
    assert n == 0
    assert MASK_MAX_M == 0.90


def test_the_radius_boundary_is_inclusive_and_tight():
    """A bin exactly at MASK_MAX_M is structure; one hair beyond it is room."""
    out, _ = mask_self_returns(_scan(at={120.0: MASK_MAX_M}), AMIN, AINC)
    assert out[_bin(120.0)] == float("inf")
    out, _ = mask_self_returns(_scan(at={120.0: MASK_MAX_M + 0.01}), AMIN, AINC)
    assert out[_bin(120.0)] == MASK_MAX_M + 0.01


def test_only_the_targeted_bins_change():
    """An open room at 5 m must come out of the mask completely untouched -- nothing masked, and
    the same 5.0 in every bin, including every bin inside the masked arc."""
    out, n = mask_self_returns([5.0] * N, AMIN, AINC)
    assert n == 0
    assert out == [5.0] * N


# ---------------------------------------------------------------------------- the message contract
def test_the_bin_count_never_changes():
    """+inf, NOT deletion. Bin i means angle_min + i*angle_increment to every subscriber -- SLAM,
    the costmap, the corridor veto. Dropping entries would shift every bearing after the masked
    arc and silently rotate the geometry, with nothing anywhere reporting an error."""
    src = _scan(at={120.0: 0.5, 130.0: 0.6, 0.0: 0.7})
    out, n = mask_self_returns(src, AMIN, AINC)
    assert len(out) == len(src) == N
    assert n == 2


def test_masked_bins_are_inf_and_never_nan():
    """session.sh passes use_inf:=true, so +inf is already this chain's 'no return' -- a typical
    scan carries 84 of them and every consumer handles them. NaN would be a second, different
    'nothing here' for the same chain to disagree about."""
    out, _ = mask_self_returns(_scan(at={120.0: 0.5}), AMIN, AINC)
    v = out[_bin(120.0)]
    assert math.isinf(v) and v > 0
    assert not math.isnan(v)


def test_the_input_list_is_not_mutated():
    """The callback hands this a live message's ranges. Masking in place would edit the message
    another callback may still be holding."""
    src = _scan(at={120.0: 0.5})
    out, _ = mask_self_returns(src, AMIN, AINC)
    assert src[_bin(120.0)] == 0.5
    assert out is not src


# ------------------------------------------------------------------------------- must not raise
def test_nan_and_inf_inputs_do_not_raise_and_pass_through():
    """A real scan is 8% +inf (84 of 1031 bins in the capture) and a driver hiccup can emit NaN.
    This runs in an rclpy callback at 6-10 Hz: one exception here kills the relay, /scan stops,
    and slam_toolbox goes silent looking exactly like the QoS bug this node exists to fix."""
    r = [float("nan")] * N
    r[_bin(120.0)] = float("inf")
    r[_bin(130.0)] = float("-inf")
    r[_bin(140.0)] = float("nan")
    r[_bin(0.0)] = float("inf")
    out, n = mask_self_returns(r, AMIN, AINC)
    assert n == 0, "there is nothing to mask in a scan of non-observations"
    assert math.isinf(out[_bin(120.0)]) and out[_bin(120.0)] > 0
    assert math.isinf(out[_bin(130.0)]) and out[_bin(130.0)] < 0
    assert math.isnan(out[_bin(140.0)])
    assert len(out) == N


def test_a_mixed_scan_of_nan_inf_and_real_returns_is_handled_bin_by_bin():
    r = _scan(at={120.0: 0.5, 0.0: 0.72})
    r[_bin(125.0)] = float("nan")
    r[_bin(150.0)] = float("inf")
    out, n = mask_self_returns(r, AMIN, AINC)
    assert n == 1
    assert out[_bin(120.0)] == float("inf")
    assert math.isnan(out[_bin(125.0)])
    assert out[_bin(0.0)] == 0.72


def test_degenerate_geometry_does_not_raise():
    """An empty scan, a zero increment, or a NaN angle_min are all nonsense the relay must survive
    rather than die on -- the mask is not the right place to decide the sensor is broken."""
    assert mask_self_returns([], AMIN, AINC) == ([], 0)
    assert mask_self_returns([0.5] * 10, AMIN, 0.0) == ([0.5] * 10, 0)
    assert mask_self_returns([0.5] * 10, float("nan"), AINC) == ([0.5] * 10, 0)
    assert mask_self_returns([0.5] * 10, AMIN, float("inf")) == ([0.5] * 10, 0)


# ---------------------------------------------------------------------- bearing normalisation
def test_bearings_are_normalised_before_they_are_compared():
    """THE TRAP a source-grep would never catch. pointcloud_to_laserscan does not have to be
    given angle_min = -pi; left at its default it emits a 0..2pi scan. Then bin bearings run to
    +270, and an un-normalised abs() would test |270| against 74..180, decide it is outside the
    mask, and keep the arm. Straight ahead in that convention is 0 AND 360.

    Same scan, same physical bearings, expressed 0..2pi: the answers must be identical."""
    n = 720
    inc = math.radians(0.5)
    r = [5.0] * n
    r[240] = 0.5        # 120 deg  -> structure
    r[0] = 0.5          #   0 deg  -> forward, real
    r[600] = 0.5        # 300 deg == -60 deg -> forward-right, real
    r[400] = 0.5        # 200 deg == -160 deg -> structure
    out, masked = mask_self_returns(r, 0.0, inc)
    assert out[240] == float("inf"), "120 deg not masked in a 0..2pi scan"
    assert out[400] == float("inf"), "200 deg (== -160) not masked in a 0..2pi scan"
    assert out[0] == 0.5, "a forward obstacle was masked because 0 deg was misread"
    assert out[600] == 0.5, "300 deg (== -60) is forward-right and must be kept"
    assert masked == 2


def test_the_full_360_wrap_bin_is_handled():
    """The real scan's last bin sits at +179.99 and its first at -180.00 -- the same physical
    bearing, one either side of the wrap. Both are dead astern and both must mask."""
    assert abs(_deg(0)) > 179.9 and abs(_deg(N - 1)) > 179.9
    r = [5.0] * N
    r[0] = 0.45
    r[N - 1] = 0.45
    out, n = mask_self_returns(r, AMIN, AINC)
    assert out[0] == float("inf") and out[N - 1] == float("inf")
    assert n == 2


# ------------------------------------------------------------------------------- REAL DATA
def test_the_glass_door_capture_survives_the_mask():
    """THE FAILURE THIS MASK MUST NOT CAUSE, checked against a real scan off the robot.

    captures/trial_ours_001/scan.json was taken facing closed glass doors. It carries 85 returns
    within +-20 deg forward and the nearest is 0.7222 m -- the doors. That is 2 cm above the 0.70
    range_min that the first attempt at this fix installed, and it is the concrete reason the fix
    is an angular mask and not a global cutoff.

    Every one of those 85 must come through untouched. If this fails, the robot has just been made
    blind to a door it is about to hit, and nothing else in the stack will notice.

    2026-09-05: captures/ is gitignored, local scratch data, and this exact path has since been
    overwritten by an unrelated later capture (an open corridor, nearest return >3.9 m) -- almost
    certainly a diagnostic run reusing the trial_ours_001 name. The original glass-door capture is
    not recoverable from git. Rather than assert stale numbers against the wrong scene (which
    would test nothing) or silently pass, this checks the file still holds ITS OWN documented
    signature before trusting it, and skips loudly, by name, if it does not -- see
    docs/TESTING.md for how to re-capture it."""
    if not CAPTURE.exists():
        pytest.skip(f"{CAPTURE.relative_to(REPO)} not on disk")
    d = json.loads(CAPTURE.read_text())
    assert d["frame"] == "base_link"
    r, a0, ai = d["ranges"], d["angle_min"], d["angle_increment"]

    def bearing(i):
        return (math.degrees(a0 + i * ai) + 180.0) % 360.0 - 180.0

    fwd = [i for i in range(len(r))
           if abs(bearing(i)) <= 20.0 and math.isfinite(r[i])]
    min_fwd = min((r[i] for i in fwd), default=None)
    if len(fwd) != 85 or min_fwd is None or abs(min_fwd - 0.7222) > 1e-3:
        pytest.skip(
            f"{CAPTURE.relative_to(REPO)} no longer holds the glass-door capture this test needs: "
            f"expected 85 forward returns nearest 0.7222 m, found {len(fwd)} nearest "
            f"{min_fwd if min_fwd is None else round(min_fwd, 4)} m. The file has been overwritten "
            f"by a later, unrelated capture reusing this trial name -- re-capture a scan facing "
            f"closed glass doors at this path (see docs/TESTING.md) to restore this regression "
            f"check.")

    out, masked = mask_self_returns(r, a0, ai)
    assert len(out) == len(r), "the mask changed the bin count of a real scan"
    for i in fwd:
        assert out[i] == r[i], (
            f"the DOOR was masked: bin {i} at {bearing(i):.1f} deg, {r[i]:.3f} m. A global "
            f"range_min of 0.70 nearly deleted this return; the mask must not finish the job.")


def test_the_mask_actually_does_work_on_the_real_scan():
    """The other half: the same capture must lose its self-returns, or the test above is passing
    vacuously. This scan was recorded while range_min was 0.70 -- its global minimum is 0.7002 --
    so the 0.39 m chassis returns are not in it; what is in it is the arm and mast at 0.70-0.99 m
    astern, which is exactly the cluster Nav2 was marking lethal.

    2026-09-05: same fixture-loss note as test_the_glass_door_capture_survives_the_mask above --
    this skips loudly, by name, if the capture no longer contains a maskable self-return cluster,
    instead of asserting against a scene that can no longer exercise the property."""
    if not CAPTURE.exists():
        pytest.skip(f"{CAPTURE.relative_to(REPO)} not on disk")
    d = json.loads(CAPTURE.read_text())
    r, a0, ai = d["ranges"], d["angle_min"], d["angle_increment"]
    global_min = min((x for x in r if math.isfinite(x)), default=None)
    if global_min is None or global_min > MASK_MAX_M:
        pytest.skip(
            f"{CAPTURE.relative_to(REPO)} no longer contains a self-occlusion cluster (nearest "
            f"finite return is {global_min} m, MASK_MAX_M is {MASK_MAX_M} m). The file has been "
            f"overwritten by a later, unrelated capture reusing this trial name -- re-capture a "
            f"scan with the arm/mast self-returns present at this path (see docs/TESTING.md) to "
            f"restore this regression check.")
    out, masked = mask_self_returns(r, a0, ai)
    assert masked > 100, f"only {masked} bins masked on a real scan -- the mask is not biting"
    changed = [i for i in range(len(r)) if out[i] != r[i]]
    assert len(changed) == masked, "the reported count and the bins actually changed disagree"
    for i in changed:
        deg = (math.degrees(a0 + i * ai) + 180.0) % 360.0 - 180.0
        assert MASK_MIN_DEG <= abs(deg) <= MASK_MAX_DEG and r[i] <= MASK_MAX_M


# ------------------------------------------------------------------- config <-> code agreement
def test_ouster_yaml_documents_the_mask_constants_that_actually_run():
    """Same rule, and same reason, as test_the_scan_slice_config_matches_what_session_sh_actually
    _passes in test_map_persistence.py: config/ouster.yaml is what a human READS and nothing reads
    it at runtime, so it can drift into being confidently wrong. scan_slice had already drifted
    once (range_min_m said 0.40 while session.sh used 0.50). The mask block carries the
    measurements, so if it disagrees with the code the measurements are attached to the wrong
    numbers."""
    import yaml
    cfg = yaml.safe_load((REPO / "config" / "ouster.yaml").read_text())
    m = cfg["self_mask"]
    assert m["min_deg"] == MASK_MIN_DEG
    assert m["max_deg"] == MASK_MAX_DEG
    assert m["max_range_m"] == MASK_MAX_M


def test_the_config_does_not_still_claim_the_155_limit():
    """The stale number is worse than no number: it is the one that looked measured."""
    txt = (REPO / "config" / "ouster.yaml").read_text()
    assert "74-155" not in txt and "74..155" not in txt
    assert "A CUTOFF THAT HIDES A SELF-RETURN" in txt, \
        "the reason 155 was wrong must stay written down, or it will be re-derived"


def test_range_min_is_045_in_both_places():
    """The mask only works because range_min is LOW. At 0.70 there is nothing left astern to mask
    and nothing left at 0.72 forward either. These two files must agree AND must agree on 0.30.

    SETTLED AT 0.45 ON 2026-09-01, after both ends were measured on the robot:
      0.70 -- put the cutoff 2 cm under a real door at 0.72 m, so the door vanished.
      0.30 -- exposed the PACKED ARM, which sits at 0.31-0.36 m, straight into the map.
      0.45 -- clears the packed arm and still sees the door.
    This test asserted 0.30 until then and had gone red against two config files that already
    agreed on 0.45; the constant it was guarding had moved and it had not. 0.45 also satisfies
    tests/test_map_persistence.py's range_min >= 0.4, so the contradiction noted here is gone."""
    import yaml
    cfg = yaml.safe_load((REPO / "config" / "ouster.yaml").read_text())
    assert cfg["scan_slice"]["range_min_m"] == 0.45

    src = (REPO / "bringup" / "session.sh").read_text()
    line = next(l for l in src.splitlines() if "range_min:=" in l)
    assert float(line.split("range_min:=")[1].split()[0]) == 0.45
