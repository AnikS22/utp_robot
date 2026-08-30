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
    # HYSTERESIS. Once a turn has started it runs until the bearing is well inside tolerance,
    # not merely at it. Without this the controller re-plans at 20 Hz and flips between
    # turn_to_bearing and drive around the threshold -- and on a 4WS chassis each flip is a MODE
    # CHANGE the firmware answers by physically re-steering all four wheels (HARDWARE_SPECS: a
    # small turn radius selects SPINNING, an arc selects DUAL_ACKERMAN). The wheels then spend
    # their time re-orienting and the body never commits to moving. Observed on 2026-08-26 as
    # "wheels are just rotating, robot isn't actually moving".
    turn_exit_tol_rad: float = 0.05   # ~2.9 deg
    pos_tol_m: float = 0.15      # arrival radius; the visual servo covers the rest
    k_ang: float = 1.2
    slow_radius_m: float = 0.60  # start easing off inside this
    v_min: float = 0.06          # below this the chassis stalls rather than creeps
    # THE SAME STALL FLOOR APPLIES TO ROTATION, and until 2026-08-30 it was missing. The 4WS
    # wheels must physically re-steer before the body turns -- characterise_twist.py waits
    # SETTLE_S = 1.5 s for exactly this -- so a small commanded w is spent entirely on
    # re-steering and the body never rotates. Proportional control walks straight into it:
    # k_ang * turn_exit_tol is 0.06 rad/s, and even at the ENTRY tolerance only 0.18, both under
    # the 0.20 rad/s that was actually measured to turn the robot (angular scale 0.59-0.80 at
    # wz=0.20, 2026-08-29). Every heading correction was therefore silently discarded, which is
    # the "wheels are just rotating, robot isn't actually moving" report from 2026-08-26.
    w_min: float = 0.20          # rad/s -- smallest rotation the chassis is known to execute


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


def _turn_rate(err_rad: float, limits: Limits) -> float:
    """Rotation rate for an IN-PLACE turn, floored to what the chassis will actually execute.

    Proportional-down-to-zero does not work here: under Limits.w_min the command is absorbed by
    re-steering the wheels and the body does not move. Clamp the magnitude UP -- this is only
    called when a turn is wanted, and the turn_exit_tol_rad hysteresis decides when to stop, so
    flooring the rate cannot prevent the turn from ever ending.
    """
    mag = min(limits.w_max, max(limits.w_min, abs(limits.k_ang * err_rad)))
    return math.copysign(mag, err_rad)


def _arc_rate(err_rad: float, limits: Limits) -> float:
    """Yaw to mix with forward motion in an Ackermann arc. NOT floored, deliberately.

    Flooring would be wrong twice over. w_min against v_max is a ~1.2 m turn radius, and
    HARDWARE_SPECS says a small radius makes the firmware select SPINNING and DROP linear.x --
    so the robot would stop advancing. And below the floor the arc correction does nothing
    anyway. Emit an honest 0.0 and let the drive/turn hysteresis hand the correction to a real
    in-place turn once the error grows past turn_tol_rad.
    """
    w = max(-limits.w_max, min(limits.w_max, limits.k_ang * err_rad))
    return w if abs(w) >= limits.w_min else 0.0


def plan_step(dist_m: float,
              bearing_err_rad: float,
              final_heading_err_rad: float | None,
              blocked: bool,
              limits: Limits = Limits(),
              prev_state: str = "") -> Step:
    """One control tick. Returns the twist to publish and why.

    dist_m             distance to the waypoint
    bearing_err_rad    angle between where we point and where the waypoint is
    final_heading_err  desired heading at the waypoint, or None to not care
    blocked            lidar says the path ahead is obstructed
    prev_state         the state this returned last tick, for turn hysteresis. Threading it is
                       the caller's job so this stays a pure function of its arguments.
    """
    if not _finite(dist_m, bearing_err_rad):
        return Step(ZERO, "bad_input")

    # Arrival hysteresis, same idea as the turn band: the boundary between "at the waypoint"
    # and "not yet" must be wider to LEAVE than to enter. At dist ~= pos_tol the bearing to a
    # point 15 cm away swings tens of degrees on millimetre drift, and without this the
    # controller chattered turn/drive/settle ~20 cycles at the goal edge (sim, 2026-08-27).
    settled = prev_state in ("final_heading", "arrived")
    pos_tol = limits.pos_tol_m * (1.6 if settled else 1.0)

    if not (dist_m <= pos_tol):
        # Far from the waypoint. Point at it before driving at it: a large bearing error driven
        # as an arc sweeps a wide curve through space the lidar has not cleared.
        # Wider band to LEAVE a turn than to enter one -- see Limits.turn_exit_tol_rad.
        tol = (limits.turn_exit_tol_rad if prev_state == "turn_to_bearing"
               else limits.turn_tol_rad)
        if not (abs(bearing_err_rad) <= tol):
            # An in-place turn is permitted even when the corridor ahead is blocked: the
            # footprint does not advance, and the goal may be BEHIND us (measured in sim
            # 2026-08-27: robot parked 0.17 m past the waypoint, facing a closed door, could
            # never turn around because the veto keyed on the front rays it was leaving).
            return Step(Twist3(0.0, _turn_rate(bearing_err_rad, limits)), "turn_to_bearing")
        if blocked:
            # About to move FORWARD into the obstruction -- that, the veto stops.
            return Step(ZERO, "blocked")
        # Aimed. Ackermann arc: vx and wz together is the ONE mix the firmware keeps intact.
        v = limits.v_max
        if dist_m < limits.slow_radius_m:
            v = max(limits.v_min, limits.v_max * dist_m / limits.slow_radius_m)
        return Step(Twist3(v, _arc_rate(bearing_err_rad, limits)), "drive")

    # Arrived. Optionally settle onto the recorded heading.
    if final_heading_err_rad is not None:
        if not _finite(final_heading_err_rad):
            return Step(ZERO, "bad_input")
        ftol = (limits.turn_exit_tol_rad if prev_state == "final_heading"
                else limits.turn_tol_rad)
        if not (abs(final_heading_err_rad) <= ftol):
            return Step(Twist3(0.0, _turn_rate(final_heading_err_rad, limits)),
                        "final_heading")
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
