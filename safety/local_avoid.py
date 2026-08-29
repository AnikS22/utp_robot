"""Steer around what is in the way, using only the live scan and the bearing to the goal.

Pure geometry, no ROS, so it is testable headlessly -- same split as arbiter.py / scan_filter.py.

WHY THIS IS TRACTABLE HERE, when route planning was not. Planning a path around an obstacle needs
a map, and a map needs localisation, and localisation is exactly what failed in this building
(2026-08-25: a ~100-point scan matches almost equally well at many positions along a corridor).

Local avoidance needs neither. The obstacle is already in the scan frame -- measured, this
instant, relative to the robot -- and the goal bearing comes from odometry, which is accurate
over the few metres of a single leg. Nothing here integrates, nothing accumulates, and there is
no map to be wrong. It is the same trade the rest of this stack makes: odometry for where we are
going, live perception for what is actually there.

METHOD: follow-the-gap.
  1. Take the forward arc only. Behind us cannot obstruct forward motion.
  2. Every return closer than `horizon_m` blocks not just its own bearing but an angular WEDGE
     either side, because the robot is 0.5 m wide and a point obstacle at 1 m still stops a body
     that wide. The wedge is asin(half_width / r) -- wider when the obstacle is closer, which is
     the correct behaviour and falls out of the geometry rather than being tuned.
  3. What survives is the set of free directions. Keep the contiguous runs wide enough to fit the
     robot, and steer down whichever one points closest to the goal.
  4. If nothing fits, say so and let the caller stop. Refusing is a real answer.

WHAT IT CANNOT DO, and none of this is fixable by tuning:
  * LOCAL MINIMA. It has no memory and no map. A U-shape or a dead end will trap it -- it will
    steer into the pocket, find no gap, and stop. Stopping is safe; escaping needs a map.
  * GLASS, and anything else a 2D lidar cannot see. Already the top site risk (S1).
  * SPARSE RETURNS. This A1M8 gives valid returns on ~13-40% of beams, so an unseen beam is
    treated as free -- it must be, or every scan would be a wall. A low-reflectivity obstacle is
    therefore invisible here exactly as it is to the corridor veto.
  * DRIFT. Every detour adds turns, and turns are where 4WS odometry degrades. A long way round
    makes the goal coordinate itself less trustworthy. Avoidance is not free.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

FORWARD_ARC_RAD = math.radians(100.0)   # consider +-100 deg; behind cannot block forward motion
HALF_WIDTH_M = 0.40                     # robot half-width plus clearance (chassis is ~0.5 m wide)
HORIZON_M = 2.0                         # returns beyond this do not constrain the next move
STEP_RAD = math.radians(2.0)            # angular resolution of the free/blocked decision


@dataclass(frozen=True)
class Choice:
    bearing_rad: float | None   # steer here; None means nothing fits, caller should stop
    reason: str
    free_width_rad: float = 0.0


def _blocked_mask(ranges, angle_min, angle_increment, *, half_width_m, horizon_m, arc_rad,
                  step_rad):
    """Sample the forward arc into slots and mark each blocked or free.

    An unseen beam (NaN/inf) is FREE. It has to be: this lidar returns on a minority of its
    beams, so treating silence as an obstacle would make every scan a wall. The cost is that a
    non-reflective obstacle is invisible -- the same blind spot the corridor veto has.
    """
    n_slots = int(2 * arc_rad / step_rad) + 1
    blocked = [False] * n_slots

    def slot_of(a: float) -> float:
        return (a + arc_rad) / step_rad

    for i, r in enumerate(ranges):
        if r != r or abs(r) == float("inf") or r <= 0.0:
            continue
        if r > horizon_m:
            continue
        a = angle_min + i * angle_increment
        a = math.atan2(math.sin(a), math.cos(a))
        if abs(a) > arc_rad:
            continue
        # Angular half-width this obstacle subtends for a body of half_width_m. At r below the
        # half-width the robot cannot pass on either side at any angle: block the whole arc.
        if r <= half_width_m:
            return [True] * n_slots
        half = math.asin(min(1.0, half_width_m / r))
        lo = max(0, int(math.floor(slot_of(a - half))))
        hi = min(n_slots - 1, int(math.ceil(slot_of(a + half))))
        for s in range(lo, hi + 1):
            blocked[s] = True
    return blocked


def choose_heading(ranges, angle_min, angle_increment, goal_bearing_rad, *,
                   half_width_m: float = HALF_WIDTH_M,
                   horizon_m: float = HORIZON_M,
                   arc_rad: float = FORWARD_ARC_RAD,
                   step_rad: float = STEP_RAD,
                   prev_bearing_rad: float | None = None,
                   hysteresis_rad: float = math.radians(12.0)) -> Choice:
    """Pick a free direction as close to the goal bearing as the obstacles allow.

    ``prev_bearing_rad`` is last cycle's choice. Keeping it when it is still free stops the robot
    dithering between two gaps of nearly equal merit -- the same failure the turn hysteresis in
    waypoint_drive.py exists for, and on this 4WS chassis each flip is a physical re-steer of all
    four wheels, not just a changed number.
    """
    blocked = _blocked_mask(ranges, angle_min, angle_increment, half_width_m=half_width_m,
                            horizon_m=horizon_m, arc_rad=arc_rad, step_rad=step_rad)
    n_slots = len(blocked)
    if all(blocked):
        return Choice(None, "no free direction in the forward arc")

    def bearing_of(slot: int) -> float:
        return slot * step_rad - arc_rad

    # Contiguous free runs, and the best (goal-closest) bearing inside each.
    runs = []
    s = 0
    while s < n_slots:
        if blocked[s]:
            s += 1
            continue
        e = s
        while e + 1 < n_slots and not blocked[e + 1]:
            e += 1
        runs.append((s, e))
        s = e + 1

    best = None
    for (s, e) in runs:
        width = (e - s + 1) * step_rad
        lo, hi = bearing_of(s), bearing_of(e)
        # Aim at the goal if it lies inside this run, otherwise at the nearest edge -- but keep
        # off the edge itself, since the edge is where the inflation says we just fit.
        margin = min(step_rad * 2, width / 4.0)
        aim = min(max(goal_bearing_rad, lo + margin), hi - margin)
        cost = abs(aim - goal_bearing_rad)
        if prev_bearing_rad is not None and lo <= prev_bearing_rad <= hi:
            cost -= hysteresis_rad          # stickiness, not a free pass
        if best is None or cost < best[0]:
            best = (cost, aim, width)

    if best is None:
        return Choice(None, "no free run wide enough for the robot")
    _, aim, width = best
    if abs(aim - goal_bearing_rad) < step_rad:
        return Choice(aim, "clear toward the goal", width)
    return Choice(aim, f"steering {math.degrees(aim - goal_bearing_rad):+.0f} deg around an "
                       f"obstruction", width)
