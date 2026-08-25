"""Pure lidar filtering logic, separated from ROS so it is headlessly testable.

Removes the sector where the A1M8 sees the ROBOT rather than the world. Getting the width right
matters in both directions: too narrow and the robot paints itself into the map as a moving wall
on every pose; too wide and scan matching is starved of the geometry it needs to fix a pose.

WHY 148 AND NOT 105
-------------------
The original 105 deg was a safe guess made before the mount pose was known. Two independent
sources now agree on where the self-hits actually stop:

  MEASURED (2026-08-25, 60 scans, robot stationary). Returns at robot range -- median 0.16-0.19 m
  -- appear ONLY beyond about |150| deg. From -145 deg round through 0 to +100 deg every sector
  that returns at all returns real world at 2-7 m.

  CAD (Ranger_mini_Xarm6_custom_box.stp, cross-validated against calib/handeye.json to 7 mm).
  The scan plane sits at z ~ 0.379 m, which is ABOVE the 0.345 m deck. So the chassis body does
  not occlude at all -- the lidar looks over it. What blocks is the superstructure behind the
  lidar: the arm riser, the EcoFlow battery, the mast, and the two power mounts at +-162 deg.
  All of it is clustered around 180 deg, none of it forward of |148| deg.

So 105 was discarding roughly 85 deg of live scan. On a sensor already returning on only ~23% of
its beams, that was not affordable.

148 keeps a 2 deg guard band inside the nearest measured self-hit (-150 deg) rather than sitting
on the boundary: the mount pose carries +-cm uncertainty and the robot flexes on its suspension.

NaN, NOT INF
------------
Rejected beams become NaN -- "no observation" -- never inf. inf means "observed, and empty out to
range_max", which would let the chassis clear real obstacles behind it from the costmap.
"""
import math

KEEP_HALF_ANGLE_DEG = 148.0


def filtered_ranges(ranges, angle_min, angle_increment, keep_half_angle_deg=KEEP_HALF_ANGLE_DEG):
    keep = math.radians(keep_half_angle_deg)
    out = list(ranges)
    for i in range(len(out)):
        a = math.atan2(math.sin(angle_min + i*angle_increment),
                       math.cos(angle_min + i*angle_increment))
        if abs(a) > keep:
            out[i] = float("nan")
    return out
