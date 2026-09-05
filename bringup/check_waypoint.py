#!/usr/bin/env python3
"""Check a recorded waypoint against the map, BEFORE you try to drive to it.

    python3 bringup/check_waypoint.py                 # every waypoint in the store
    python3 bringup/check_waypoint.py car_panel       # just this one
    python3 bringup/check_waypoint.py --standoff 1.4  # what the pose would show from further back

WHY THIS EXISTS. On 2026-09-01 `car_panel` was recorded with the robot's footprint 59 mm inside a
wall. Every check anyone ran said it was fine, because every check looked at the waypoint's CELL --
and the cell was free. The robot is not a point; it is a 0.72 x 0.50 m rectangle with 0.03 m of
padding, and Nav2's MPPI runs CostCritic with consider_footprint: true and collision_cost 1e6, so
no trajectory can ever terminate at a pose whose footprint overlaps a lethal cell. The leg simply
aborts, the planner looks broken, and hours go into Nav2 parameters that were never wrong.

Checking a point where the planner checks a rectangle is the whole bug. This checks the rectangle.

It also reports HOW MUCH WALL THE POSE PUTS IN FRAME. Nothing here needs to know how high the
button is -- the whole point of the system is that the VLM finds it. But the VLM can only ground
what is inside the image, and that is pure geometry: the D435's vertical FOV is 43.2 degrees (NOT
the D455's) and the lens sits at z = 1.471 m looking 5.4 degrees down, so the closer you park the
narrower the strip of wall you see. From 0.53 m the camera shows a 0.68 m strip; from 1.4 m it
shows about 1.5 m. A pose that shows a narrow strip is a pose where grounding depends on luck
about where the panel happens to sit, which is exactly the dependency the pipeline exists to
remove. safety/reach_envelope.SURVEY_STANDOFF_M = 1.40 is this number, and its comment says why:
"where to stop in order to LOOK is not where to stop in order to PRESS."

Everything here is offline. No ROS, no robot, no network -- run it while the robot is off.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent

# Footprint from nav2_bringup/nav2_params_os0_map.yaml (both costmaps agree, plus footprint_padding).
HALF_X, HALF_Y = 0.36 + 0.03, 0.25 + 0.03

# Camera: T_base_link_camera from calib/handeye.json, FOV from config/camera.yaml.
CAM_XYZ = (-0.326933889, -0.002611969, 1.471152384)
CAM_PITCH_DOWN_RAD = 0.090636624          # boresight below horizontal
CAM_VFOV_DEG = 43.2                       # measured_color_fov_deg[1] -- D435, not D455

# Arm: link_base sits 0.740 m above base_link on the axis (riser). ARM_REACH_M is the ENFORCED
# gate in safety/reach_envelope.py -- approach_target.py refuses anything beyond it.
ARM_Z = 0.740
ARM_REACH_M = 0.88


def load_map(name: str):
    y = (REPO / "maps" / f"{name}.yaml").read_text()
    res = float(re.search(r"resolution:\s*([-\d.]+)", y).group(1))
    ox, oy = (float(v) for v in re.search(r"origin:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)", y).groups())
    img_name = re.search(r"image:\s*(\S+)", y).group(1)
    from PIL import Image
    grid = np.array(Image.open(REPO / "maps" / img_name))
    return grid, res, ox, oy


def load_waypoints():
    """Regex, not yaml -- same reason stow_arm.py uses one: this must run under any interpreter."""
    txt = (REPO / "maps" / "waypoints.yaml").read_text()
    out, cur = {}, None
    for line in txt.splitlines():
        m = re.match(r"^([A-Za-z_][\w]*):\s*$", line)
        if m:
            cur = m.group(1); out[cur] = {}; continue
        if cur:
            m = re.match(r"^\s+(\w+):\s*(.+?)\s*$", line)
            if m:
                k, v = m.group(1), m.group(2)
                try:
                    out[cur][k] = float(v)
                except ValueError:
                    out[cur][k] = v.strip('"\'')
    return {k: v for k, v in out.items() if "x" in v and "y" in v}


def to_cell(x, y, res, ox, oy, H):
    return int((x - ox) / res), (H - 1) - int((y - oy) / res)


def footprint_hits(grid, res, ox, oy, x, y, yaw):
    """Lethal cells under the padded rectangle. Samples at half a cell so nothing slips between."""
    H, W = grid.shape
    c, s = math.cos(yaw), math.sin(yaw)
    hits, step = [], res / 2.0
    fwds = np.arange(-HALF_X, HALF_X + 1e-9, step)
    lats = np.arange(-HALF_Y, HALF_Y + 1e-9, step)
    for f in fwds:
        for l in lats:
            col, row = to_cell(x + f * c - l * s, y + f * s + l * c, res, ox, oy, H)
            if 0 <= col < W and 0 <= row < H and grid[row, col] == 0:
                hits.append((round(float(f), 3), round(float(l), 3)))
    return sorted(set(hits))


def clearance_m(grid, res, ox, oy, x, y, max_r=3.0):
    """Distance from the pose to the nearest lethal cell, by expanding search."""
    H, W = grid.shape
    col0, row0 = to_cell(x, y, res, ox, oy, H)
    best = float("inf")
    r = int(max_r / res)
    lo_r, hi_r = max(0, row0 - r), min(H, row0 + r + 1)
    lo_c, hi_c = max(0, col0 - r), min(W, col0 + r + 1)
    sub = grid[lo_r:hi_r, lo_c:hi_c]
    ys, xs = np.nonzero(sub == 0)
    if len(xs):
        dx = (xs + lo_c - col0) * res
        dy = (ys + lo_r - row0) * res
        best = float(np.min(np.hypot(dx, dy)))
    return best


def wall_ahead(grid, res, ox, oy, x, y, yaw, max_r=6.0):
    """Ray-cast along the heading: how far to the first lethal cell."""
    H, W = grid.shape
    c, s = math.cos(yaw), math.sin(yaw)
    d = 0.0
    while d < max_r:
        d += res / 2.0
        col, row = to_cell(x + d * c, y + d * s, res, ox, oy, H)
        if not (0 <= col < W and 0 <= row < H):
            return None
        if grid[row, col] == 0:
            return d
    return None


def visible_band(wall_from_base):
    """Height range on a wall `wall_from_base` metres ahead that falls inside the camera's FOV."""
    d = wall_from_base - CAM_XYZ[0]          # camera is aft of base_link, so this is LONGER
    half = math.radians(CAM_VFOV_DEG) / 2.0
    lo = CAM_XYZ[2] + d * math.tan(-CAM_PITCH_DOWN_RAD - half)
    hi = CAM_XYZ[2] + d * math.tan(-CAM_PITCH_DOWN_RAD + half)
    return lo, hi, d


def reach_band(wall_from_base):
    """Height range on that wall inside the arm's enforced 0.88 m sphere about link_base."""
    dx = wall_from_base
    if dx > ARM_REACH_M:
        return None
    dz = math.sqrt(ARM_REACH_M ** 2 - dx ** 2)
    return ARM_Z - dz, ARM_Z + dz


def image_row(height_m, wall_from_base, rows=720):
    """Where a target at that height lands vertically in the frame. rows/2 is the boresight."""
    d = wall_from_base - CAM_XYZ[0]
    ang = math.atan2(height_m - CAM_XYZ[2], d) + CAM_PITCH_DOWN_RAD
    half = math.radians(CAM_VFOV_DEG) / 2.0
    return rows / 2.0 - (ang / half) * (rows / 2.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("waypoint", nargs="*", help="names to check (default: all)")
    ap.add_argument("--standoff", type=float, default=None,
                    help="also report the wall strip visible from this distance (survey pose)")
    a = ap.parse_args()

    wps = load_waypoints()
    names = a.waypoint or sorted(wps)
    unknown = [n for n in names if n not in wps]
    if unknown:
        print(f"no such waypoint(s): {', '.join(unknown)}", file=sys.stderr)
        print(f"have: {', '.join(sorted(wps))}", file=sys.stderr)
        return 2

    bad = 0
    cache = {}

    for n in names:
        w = wps[n]
        mp = str(w.get("map_name", "floor1"))
        if mp not in cache:
            try:
                cache[mp] = load_map(mp)
            except Exception as e:
                print(f"{n}: cannot load map '{mp}': {e}", file=sys.stderr)
                bad += 1
                continue
        grid, res, ox, oy = cache[mp]
        x, y, yaw = float(w["x"]), float(w["y"]), float(w["yaw"])
        H, W = grid.shape
        col, row = to_cell(x, y, res, ox, oy, H)
        cellv = int(grid[row, col]) if (0 <= col < W and 0 <= row < H) else -1
        cellname = {0: "OCCUPIED", 254: "free", 205: "unknown"}.get(cellv, str(cellv))

        hits = footprint_hits(grid, res, ox, oy, x, y, yaw)
        clr = clearance_m(grid, res, ox, oy, x, y)
        wall = wall_ahead(grid, res, ox, oy, x, y, yaw)

        print(f"\n=== {n}   map={mp}   ({x:+.3f}, {y:+.3f})  yaw={math.degrees(yaw):+.1f} deg")
        print(f"  cell                {cellname}")
        print(f"  nearest obstacle    {clr:.3f} m from the pose")
        if hits:
            deep = max(
                max(abs(f) - HALF_X, 0) or 0 for f, l in hits
            )
            print(f"  FOOTPRINT           *** {len(hits)} LETHAL CELLS UNDER THE ROBOT ***")
            print(f"                      Nav2 cannot terminate a trajectory here (CostCritic")
            print(f"                      consider_footprint: true, collision_cost 1e6).")
            print(f"                      Back off along -heading and re-record.")
            bad += 1
        else:
            print(f"  footprint           clear")

        if wall is None:
            print(f"  wall ahead          nothing within 6 m along the heading")
            continue
        print(f"  wall ahead          {wall:.3f} m")

        lo, hi, dcam = visible_band(wall)
        print(f"  camera sees         {lo:.3f} - {hi:.3f} m  (wall is {dcam:.3f} m from the lens)")
        rb = reach_band(wall)
        if rb is None:
            print(f"  arm reaches         NOTHING -- wall is {wall:.3f} m, beyond the {ARM_REACH_M} m envelope")
        else:
            print(f"  arm reaches         {rb[0]:.3f} - {rb[1]:.3f} m")
            both_lo, both_hi = max(lo, rb[0]), min(hi, rb[1])
            if both_lo < both_hi:
                print(f"  SEE *and* REACH     {both_lo:.3f} - {both_hi:.3f} m")
            else:
                print(f"  SEE *and* REACH     EMPTY -- no height is both visible and reachable here")
                bad += 1

        strip = hi - lo
        print(f"  WALL STRIP IN VIEW  {strip:.2f} m tall")
        # DO NOT "FIX" A NARROW STRIP BY BACKING THE WAYPOINT OFF. Measured trade-off, wall
        # distance vs (strip seen / height that is both visible AND inside the 0.88 m arm envelope):
        #     0.53 m -> 0.69 m seen, 0.40 m usable        0.75 m -> 0.86 m seen, 0.27 m usable
        #     0.60 m -> 0.74 m seen, 0.38 m usable        0.85 m -> 0.94 m seen, 0.09 m usable
        # Backing off buys a little more wall in frame and destroys the reachable band, because the
        # arm sphere is centred 0.74 m up and shrinks fast as the wall recedes. The usable window is
        # widest at 0.45-0.60 m, which is where these poses already are. If grounding fails at press
        # range the answer is a TWO-STAGE approach -- ground far, then close in, which is what
        # bringup/face_target.py does -- not a waypoint parked further back.
        if strip < 0.80:
            print(f"                      narrow, but this is near the optimum: backing off trades")
            print(f"                      reach for view and the usable band shrinks faster than the")
            print(f"                      strip grows. Do not move the waypoint back to fix it.")
        if a.standoff:
            slo, shi, _ = visible_band(a.standoff)
            print(f"  from {a.standoff:.2f} m instead   {slo:.3f} - {shi:.3f} m  ({shi - slo:.2f} m tall)")

    print()
    if bad:
        print(f"{bad} waypoint(s) NOT usable as recorded. Re-record them.")
    else:
        print("all checked waypoints are drivable and their footprints are clear.")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
