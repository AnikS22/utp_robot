"""Is this target inside the arm's envelope, and if not, how much closer must the base get?

Pure geometry, no ROS. Same split as the rest of safety/.

WHY THIS EXISTS -- the geometry that made the first real press attempt fail, 2026-08-29.

  the grounder found the ADA plate at   1.97 m   (correctly: it beat the FIRE alarm beside it)
  the base repositioned and stopped at  1.23 m
  the xArm6 envelope with the riser is  0.88 m
  -> the arm was commanded past its reach and faulted with ControllerError 21

And it was not going to end otherwise, because the base COULD NOT get closer. The final approach
was governed by corridor_blocked(), whose 0.90 m look-ahead is measured from the LIDAR -- which
sits 0.318 m FORWARD of base_link. So the driving veto halts the chassis with the wall about
1.21 m from base_link, permanently outside a 0.88 m arm. The safety rule that keeps the robot off
walls while driving also guarantees it can never reach anything mounted on one.

Those are different jobs and they need different rules. Driving: never close on an unknown
obstacle. Pressing: close deliberately, slowly, on a target that has been grounded and vetoed, to
a distance chosen from the ARM's geometry rather than the chassis's. The veto stays as a hard
floor; it just stops being the thing that decides when to stop.
"""
from __future__ import annotations

ARM_REACH_M = 0.88          # xArm6 + riser, from HARDWARE_SPECS
LIDAR_FORWARD_M = 0.318     # base_link -> lidar_link x, CAD-derived and cross-validated
PRESS_STANDOFF_M = 0.55     # base standoff proven for the press pose, 2026-08-25
MIN_LIDAR_RANGE_M = 0.20    # never command the base closer than this to anything, ever.
                            # The chassis front is ~0.375 m forward of base_link and the lidar
                            # ~0.318 m, so the bumper leads the sensor by ~0.06 m: a 0.20 m lidar
                            # reading puts the chassis ~0.14 m off the wall. Any higher and it
                            # clamps the 0.55 m press standoff (0.55-0.318 = 0.232) and the base
                            # stops short of arm reach again -- which is the bug this file exists
                            # for, reintroduced by a safety margin that sounded prudent.


def in_reach(range_m: float, *, reach_m: float = ARM_REACH_M) -> bool:
    """Is a target this far from base_link inside the arm envelope?"""
    return range_m <= reach_m


def lidar_stop_for(standoff_m: float = PRESS_STANDOFF_M,
                   *, lidar_forward_m: float = LIDAR_FORWARD_M) -> float:
    """Lidar range at which base_link sits ``standoff_m`` from the wall ahead.

    The lidar is FORWARD of base_link, so it reads a SHORTER range than the chassis centre is
    from the wall. Ignoring that offset is what put the base 1.21 m out while believing 0.90.
    """
    return max(MIN_LIDAR_RANGE_M, standoff_m - lidar_forward_m)


def shortfall(range_m: float, *, reach_m: float = ARM_REACH_M) -> float:
    """Metres the base must still close before the arm can touch this. 0 when already in reach."""
    return max(0.0, range_m - reach_m)


def check_before_reach(range_m: float, *, reach_m: float = ARM_REACH_M) -> tuple[bool, str]:
    """May the arm be commanded at a target this far away?

    Refusing is the whole point. Commanding a Cartesian goal outside the envelope does not produce
    a short reach -- the IK either fails or drives a joint into its stop, and the arm faults. On
    2026-08-29 that was ControllerError 21, and because the tool returned 0 the route logged the
    trial as complete.
    """
    if range_m != range_m or range_m <= 0:
        return False, f"target range is not a usable number ({range_m!r}); refusing to reach"
    if not in_reach(range_m, reach_m=reach_m):
        return False, (f"target is {range_m:.2f} m from base_link and the arm reaches "
                       f"{reach_m:.2f} m -- {shortfall(range_m, reach_m=reach_m):.2f} m short. "
                       f"Commanding this faults the arm (ControllerError 21, measured). Move the "
                       f"BASE closer; do not ask the arm for reach it does not have.")
    return True, f"in reach ({range_m:.2f} m of {reach_m:.2f} m)"
