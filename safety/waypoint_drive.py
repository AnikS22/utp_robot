"""Pure waypoint-following logic. No ROS, no hardware, so it is testable headlessly.

WHY THIS EXISTS. SLAM could not hold a pose in this building (2026-08-25: scan matching flips
between solutions in a corridor with ~100-point scans). But the experiment does not need a map.
It needs the base to arrive close enough that the ADA plate is in the camera frame; the visual
servo in bringup/approach_target.py closes the rest, and hand-eye is good to 2.96 mm RMS. So we
drive on ODOMETRY ALONE and let the servo absorb the drift.

THE CHASSIS SHAPES THE CONTROLLER. The Ranger CAN protocol takes one body twist and the firmware
picks a mode from it (docs/HARDWARE_SPECS.md):

    linear.y != 0        -> PARALLEL, and angular.z is DROPPED
    small turn radius    -> SPINNING, and linear.x is DROPPED
    otherwise            -> DUAL_ACKERMAN, and linear.y is DROPPED

So a twist that mixes strafe and yaw is silently truncated. This controller therefore never emits
linear.y at all, and only ever produces either a pure rotation or an Ackermann arc.

EVERY GUARD IS FAIL-CLOSED. Unknown, stale and NaN all mean stop -- `not (x <= t)` is used rather
than `x > t` precisely so NaN takes the stop branch instead of sailing through a comparison.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Twist3:
    vx: float = 0.0
    wz: float = 0.0

    def is_zero(self, eps: float = 1e-9) -> bool:
        return abs(self.vx) <= eps and abs(self.wz) <= eps


ZERO = Twist3()


@dataclass(frozen=True)
class Limits:
    v_max: float = 0.25          # m/s -- docs/MAPPING.md speed, gentle on the high CoM
    w_max: float = 0.40          # rad/s
    turn_tol_rad: float = 0.15   # ~8.6 deg: closer than this, stop turning and drive
    pos_tol_m: float = 0.15      # arrival radius; the visual servo covers the rest
    k_ang: float = 1.2
    slow_radius_m: float = 0.60  # start easing off inside this
    v_min: float = 0.06          # below this the chassis stalls rather than creeps


@dataclass(frozen=True)
class Step:
    twist: Twist3
    state: str


def wrap(a: float) -> float:
    """Wrap to (-pi, pi]. NaN propagates rather than silently becoming 0."""
    if a != a:
        return float("nan")
    return math.atan2(math.sin(a), math.cos(a))


def _finite(*vals: float) -> bool:
    return all(v == v and abs(v) != float("inf") for v in vals)


def plan_step(dist_m: float,
              bearing_err_rad: float,
              final_heading_err_rad: float | None,
              blocked: bool,
              limits: Limits = Limits()) -> Step:
    """One control tick. Returns the twist to publish and why.

    dist_m             distance to the waypoint
    bearing_err_rad    angle between where we point and where the waypoint is
    final_heading_err  desired heading at the waypoint, or None to not care
    blocked            lidar says the path ahead is obstructed
    """
    if not _finite(dist_m, bearing_err_rad):
        return Step(ZERO, "bad_input")
    if blocked:
        return Step(ZERO, "blocked")

    if not (dist_m <= limits.pos_tol_m):
        # Far from the waypoint. Point at it before driving at it: a large bearing error driven
        # as an arc sweeps a wide curve through space the lidar has not cleared.
        if not (abs(bearing_err_rad) <= limits.turn_tol_rad):
            w = max(-limits.w_max, min(limits.w_max, limits.k_ang * bearing_err_rad))
            return Step(Twist3(0.0, w), "turn_to_bearing")
        # Aimed. Ackermann arc: vx and wz together is the ONE mix the firmware keeps intact.
        v = limits.v_max
        if dist_m < limits.slow_radius_m:
            v = max(limits.v_min, limits.v_max * dist_m / limits.slow_radius_m)
        w = max(-limits.w_max, min(limits.w_max, limits.k_ang * bearing_err_rad))
        return Step(Twist3(v, w), "drive")

    # Arrived. Optionally settle onto the recorded heading.
    if final_heading_err_rad is not None:
        if not _finite(final_heading_err_rad):
            return Step(ZERO, "bad_input")
        if not (abs(final_heading_err_rad) <= limits.turn_tol_rad):
            w = max(-limits.w_max, min(limits.w_max, limits.k_ang * final_heading_err_rad))
            return Step(Twist3(0.0, w), "final_heading")
    return Step(ZERO, "arrived")


def to_goal(cur_x: float, cur_y: float, cur_yaw: float,
            goal_x: float, goal_y: float) -> tuple[float, float]:
    """Distance and bearing error from a pose to a goal, both in the odom frame."""
    dx, dy = goal_x - cur_x, goal_y - cur_y
    return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - cur_yaw)


def corridor_blocked(ranges, angle_min, angle_increment, *,
                     half_width_m: float = 0.40,
                     look_ahead_m: float = 0.90,
                     min_hits: int = 3) -> bool:
    """True if enough returns fall inside the rectangle we are about to drive through.

    A rectangle, not a cone: the robot is 0.5 m wide whatever the range, and a cone either clears
    a real obstacle at distance or vetoes on a harmless wall beside us up close.

    min_hits exists because this lidar returns on only ~30-40% of its beams and produces isolated
    spurious points; one stray return must not latch the robot in place. Three in the box is a
    thing, not a speckle.
    """
    hits = 0
    for i, r in enumerate(ranges):
        if r != r or abs(r) == float("inf"):
            continue
        a = angle_min + i*angle_increment
        x, y = r*math.cos(a), r*math.sin(a)
        if 0.0 < x <= look_ahead_m and abs(y) <= half_width_m:
            hits += 1
            if hits >= min_hits:
                return True
    return False
