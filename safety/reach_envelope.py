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
# FROM THE SIM, NOT INVENTED HERE. isaac_world.PRESS_STANDOFF_X = 0.50, "base-center to
# press-point x at the press pose", raised there from 0.425. The acceptance band and yaw tolerance
# below are isaac_world._press_pose_ok verbatim: dist <= d_goal + 0.18 and yaw_err <= 0.25. 0.68 m
# sits comfortably inside the 0.88 m arm; the hardware attempt that faulted had stopped at 1.23 m.
PRESS_STANDOFF_M = 0.50
PRESS_ACCEPT_SLACK_M = 0.18   # positioned if within standoff + this
PRESS_YAW_TOL_RAD = 0.25      # ~14.3 deg off the press axis is still reachable

# Approach budget and stall test, also isaac_world's. Latency here is a DATA-INTEGRITY concern in
# its words: "a doomed approach that eats the trial budget turns a would-be report_unreachable
# into a scored timeout".
APPROACH_BUDGET_S = 25.0
APPROACH_STALL_WINDOW_S = 4.0
APPROACH_STALL_D_M = 0.03
APPROACH_STALL_YAW_RAD = 0.03
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


def press_pose_ok(dist_m: float, yaw_err_rad: float,
                  *, standoff_m: float = PRESS_STANDOFF_M,
                  slack_m: float = PRESS_ACCEPT_SLACK_M,
                  yaw_tol_rad: float = PRESS_YAW_TOL_RAD) -> bool:
    """isaac_world._press_pose_ok, verbatim: the honest geometric test for "positioned".

    TARGET-RELATIVE, NOT LIDAR-RELATIVE, and the sim says why in its own comment: the self-hit
    filter cannot guard the sub-0.30 m close range, so the standoff is what keeps the chassis
    clear of the door. Servoing the final approach on the lidar instead is what left the hardware
    base 1.23 m from a plate a 0.88 m arm then faulted trying to reach.
    """
    return dist_m <= standoff_m + slack_m and abs(yaw_err_rad) <= yaw_tol_rad


def stalled(history, now: float,
            *, window_s: float = APPROACH_STALL_WINDOW_S,
            d_tol: float = APPROACH_STALL_D_M,
            yaw_tol: float = APPROACH_STALL_YAW_RAD) -> bool:
    """Has the approach stopped converging? ``history`` is [(t, |radial|, |yaw_err|), ...].

    Evidence, not a guess: over a full window neither error improved enough. The base is pressed
    against something, or the grounded point is not drivable-to, and driving longer cannot help.
    """
    if len(history) < 2:
        return False
    t0, d0, y0 = history[0]
    if now - t0 < window_s:
        return False
    _, d_now, y_now = history[-1]
    return (d0 - d_now) < d_tol and (y0 - y_now) < yaw_tol


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
