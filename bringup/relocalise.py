#!/usr/bin/env python3
"""Find the robot on the loaded map and tell slam_toolbox where it is.

    python3 bringup/relocalise.py           # global search, publish /initialpose
    python3 bringup/relocalise.py --check   # score the current pose, change nothing

WHY. slam_toolbox in LOCALIZATION mode loses its lock when the robot rotates faster than the scan
rate can follow: /scan runs 4.6-6.4 Hz against the sensor's 10, so a brisk turn puts 5-7 degrees
between consecutive scans while the matcher's coarse_angle_resolution is 2 degrees. Beyond that it
cannot correlate them, and the pose silently walks off. Measured 2026-09-01: a robot 4.6 m from
where it believed it was, still publishing a confident TF.

There is no built-in recovery. AMCL would spread particles; slam_toolbox will not. So this does
what AMCL does once: score the live scan against the map over free cells x headings, take the best,
and publish it as /initialpose -- which localization mode accepts. MAPPING MODE IGNORES IT, which
is why RViz's 2D Pose Estimate appears to do nothing there.

A fit above ~80% is localized. 50% is lost. The number matters more than it looks: a waypoint
recorded at 51% is indistinguishable in the file from one recorded at 88%, and sends the arm at a
wall.
"""
import argparse, math, sys, time

# Search resolution. LATTICE_M is the spacing of the global candidate grid and also the span the
# first refine pass sweeps, so every pose is within half a step of a candidate and the refine
# closes the rest. 0.20 m over floor1 is ~12,700 candidates x 72 headings, about a second.
LATTICE_M = 0.20
YAW_STEP_DEG = 5
BEAMS = 120          # was 70; more beams separate a real match from a plausible one
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _ros_env import require_ros
require_ros()
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy)
import tf2_ros


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="score only; publish nothing")
    a = ap.parse_args()
    rclpy.init(); n = Node("utp_relocalise")
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
    q = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                   reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    d = {}
    n.create_subscription(OccupancyGrid, "/map", lambda m: d.__setitem__("map", m), q)
    n.create_subscription(LaserScan, "/scan", lambda m: d.__setitem__("scan", m), 10)
    pub = n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    t0 = time.time()
    while len(d) < 2 and time.time() - t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.2)
    if len(d) < 2:
        print("missing /map or /scan", file=sys.stderr); return 1
    # WAIT FOR TF, NOT FOR A FIXED NUMBER OF SPINS. 30 spins was ~3 s, and /map is latched while
    # /scan runs at 4-7 Hz, so both arrive almost immediately -- leaving the TF listener too little
    # time to receive map->odom AND odom->base_link. Measured 2026-09-05: 1 run in 4 printed
    # "no map->base_link yet" and emitted no fit line at all, which made
    # bringup/multifloor_route.sh's preflight abort the whole run with "could not score the
    # localization fit" on a robot that was localized the entire time. Poll for the transform the
    # caller actually needs, with a real budget.
    _tf_end = time.time() + 20
    while time.time() < _tf_end:
        rclpy.spin_once(n, timeout_sec=0.1)
        if buf.can_transform("map", "base_link", rclpy.time.Time()):
            break
    mp, sc = d["map"], d["scan"]; info = mp.info; res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    W, H = info.width, info.height
    grid = np.array(mp.data, dtype=np.int8).reshape(H, W)
    occ = grid > 50
    rs, angs = [], []
    ang = sc.angle_min
    for r in sc.ranges:
        aa = ang; ang += sc.angle_increment
        if r == r and sc.range_min < r < 15:
            rs.append(r); angs.append(aa)
    st = max(1, len(rs) // BEAMS)
    rs = np.array(rs[::st]); angs = np.array(angs[::st])

    def fit(px, py, yw):
        ii = ((px - ox) / res + rs * np.cos(angs + yw) / res).astype(np.int32)
        jj = ((py - oy) / res + rs * np.sin(angs + yw) / res).astype(np.int32)
        m = (ii >= 0) & (ii < W) & (jj >= 0) & (jj < H)
        return int(occ[jj[m], ii[m]].sum()) if m.any() else 0

    try:
        t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        cx, cy = t.transform.translation.x, t.transform.translation.y
        cw = 2 * math.atan2(t.transform.rotation.z, t.transform.rotation.w)
        print(f"  current ({cx:+.2f},{cy:+.2f},{math.degrees(cw):+.0f}deg) "
              f"fit {100*fit(cx,cy,cw)/len(rs):.1f}%")
    except Exception:
        cx = cy = cw = 0.0
        print("  no map->base_link yet")
    if a.check:
        return 0

    # GLOBAL SEARCH. Candidates come from a LATTICE over the free cells, not from `free[::step]`.
    #
    # The old line was `cand = free[:: max(1, len(free)//4000)]`. np.argwhere returns cells in
    # raster order, so striding it walks along rows: on floor1's 186,858 free cells that is every
    # 46th cell, leaving candidates ~2.3 m apart ALONG A ROW while the refine stage below only
    # searches +/-0.4 m around the winner. A true pose landing between two candidates cannot be
    # found at any score, and the tool reports "lost" for a robot sitting in plain view. Measured
    # 2026-09-06 on floor1: raster striding returned 40.5% and picked the arbitrary seed it was
    # given; the lattice below returned the real pose, and a later sweep of 831,312 poses could
    # not beat it -- which is what a converged global search is supposed to look like.
    #
    # Uniform in x AND y, so the worst-case distance from any pose to the nearest candidate is
    # bounded by the lattice step and the refine span can be sized against it honestly.
    step = max(1, int(round(LATTICE_M / res)))          # cells
    cj, ci = np.where(grid[::step, ::step] == 0)
    cj = (cj * step).astype(np.int32); ci = (ci * step).astype(np.int32)
    print(f"  searching {len(ci)} free cells on a {step*res:.2f} m lattice x "
          f"{len(range(0,360,YAW_STEP_DEG))} headings = {len(ci)*len(range(0,360,YAW_STEP_DEG)):,} poses")

    # Scored a heading at a time, all candidates at once. The per-candidate Python loop it
    # replaces is what made a dense search unaffordable and forced the sparse stride in the
    # first place.
    best = (fit(cx, cy, cw), cx, cy, cw)
    for yd in range(0, 360, YAW_STEP_DEG):
        yw = math.radians(yd)
        di = rs * np.cos(angs + yw) / res
        dj = rs * np.sin(angs + yw) / res
        ii = (ci[:, None] + di[None, :]).astype(np.int32)
        jj = (cj[:, None] + dj[None, :]).astype(np.int32)
        inb = (ii >= 0) & (ii < W) & (jj >= 0) & (jj < H)
        np.clip(ii, 0, W - 1, out=ii); np.clip(jj, 0, H - 1, out=jj)
        sc_ = (occ[jj, ii] & inb).sum(axis=1)
        k = int(sc_.argmax())
        if sc_[k] > best[0]:
            best = (int(sc_[k]), ox + (ci[k] + 0.5) * res, oy + (cj[k] + 0.5) * res, yw)
    s, bx, by, bw = best
    print(f"  coarse  ({bx:+.2f},{by:+.2f},{math.degrees(bw):+.0f}deg) fit {100*s/len(rs):.1f}%")
    for span, stp, yr in ((LATTICE_M, 0.05, range(-6, 7)), (0.08, 0.02, range(-3, 4))):
        b = (s, bx, by, bw)
        rr = [i * stp for i in range(-int(span / stp), int(span / stp) + 1)]
        for ddx in rr:
            for ddy in rr:
                for k in yr:
                    v = fit(bx + ddx, by + ddy, bw + math.radians(k))
                    if v > b[0]:
                        b = (v, bx + ddx, by + ddy, bw + math.radians(k))
        s, bx, by, bw = b
    print(f"  refined ({bx:+.2f},{by:+.2f},{math.degrees(bw):+.0f}deg) fit {100*s/len(rs):.1f}%")
    m = PoseWithCovarianceStamped()
    m.header.frame_id = "map"; m.header.stamp = n.get_clock().now().to_msg()
    m.pose.pose.position.x = float(bx); m.pose.pose.position.y = float(by)
    m.pose.pose.orientation.z = math.sin(bw / 2); m.pose.pose.orientation.w = math.cos(bw / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.1; m.pose.covariance[35] = 0.05
    for _ in range(4):
        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.2); time.sleep(0.25)
    print("  published /initialpose -- drive slowly for a few metres to let it settle")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
