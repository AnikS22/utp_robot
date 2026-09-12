"""The reach test must use the distance the ARM covers, not the floor plan.

face_target used hypot(x, y) from base_link; approach_target uses the 3D distance from link_base,
0.74 m up the riser. For a control at head height those disagree by more than the shortfall they
are arguing about, and the base gets waved through while the arm refuses to reach.

Numbers are the real campus ADA plate, 2026-09-11.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from safety.reach_envelope import ARM_REACH_M, shortfall  # noqa: E402

RISER_M = 0.74
PLATE = (0.826, -0.007, 1.201)        # target in base_link, as measured


def reach_range(p, riser=RISER_M):
    return math.sqrt(p[0]**2 + p[1]**2 + (p[2] - riser)**2)


def test_the_horizontal_range_is_not_the_reach_range():
    horiz = math.hypot(PLATE[0], PLATE[1])
    assert abs(horiz - 0.826) < 0.01
    assert horiz <= ARM_REACH_M, "horizontally this looks reachable"
    assert reach_range(PLATE) > ARM_REACH_M, "in 3D from the shoulder it is NOT"


def test_the_disagreement_is_bigger_than_the_shortfall():
    """This is why one tool advanced the base and the other refused to reach."""
    horiz = math.hypot(PLATE[0], PLATE[1])
    gap = reach_range(PLATE) - horiz
    assert gap > 0.10, f"only {gap:.3f} m apart"
    assert shortfall(reach_range(PLATE)) > 0.0


def test_a_plate_at_shoulder_height_agrees_with_the_horizontal_range():
    """The two measures coincide only when the target is level with the shoulder."""
    level = (0.826, -0.007, RISER_M)
    assert abs(reach_range(level) - math.hypot(level[0], level[1])) < 1e-9


def test_closing_the_base_in_makes_it_reachable():
    """After the creep the plate was 0.62 m horizontally; that must clear the envelope in 3D too."""
    after = (0.622, 0.0, 1.201)
    assert reach_range(after) < ARM_REACH_M
    assert shortfall(reach_range(after)) == 0.0
