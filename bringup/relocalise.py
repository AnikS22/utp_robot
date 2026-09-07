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

A high endpoint fit alone does not establish localization: repeated searches can agree on a
wrong location. Refine competing hypotheses and reject low or ambiguous scores before publishing.
The score-gap test is a heuristic, not proof that the pose is physically correct.
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
from localization_search import ScanMatcher, acceptance
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
    ap.add_argument("--min-range", type=float, default=0.0, metavar="M",
                    help="ignore returns closer than M metres. For the lift arrival, where ~90%% "
                         "of the scan is car wall that is not in the map. 0 = off (default).")
    ap.add_argument("--dry-run", action="store_true", help="search and validate without publishing")
    ap.add_argument("--expected-map", help="refuse unless slam_toolbox has this map loaded")
    ap.add_argument("--min-fit", type=float, default=55.0)
    ap.add_argument("--min-margin", type=float, default=5.0,
                    help="minimum percentage-point gap to a distinct competing location")
    a = ap.parse_args()
    if not (math.isfinite(a.min_range) and a.min_range >= 0 and
            0 <= a.min_fit <= 100 and 0 < a.min_margin <= 100):
        ap.error("invalid range, fit, or margin")
    rclpy.init(); n = Node("utp_relocalise")
    if a.expected_map:
        from rclpy.parameter_client import AsyncParameterClient
        client = AsyncParameterClient(n, "/slam_toolbox")
        if not client.wait_for_services(timeout_sec=5):
            print("rejected: cannot verify loaded map", file=sys.stderr); return 1
        future = client.get_parameters(["map_file_name", "mode"])
        rclpy.spin_until_future_complete(n, future, timeout_sec=5)
        result = future.result() if future.done() else None
        expected = Path(a.expected_map)
        if not expected.is_absolute():
            expected = Path(__file__).resolve().parents[1] / "maps" / expected
        if (result is None or len(result.values) != 2 or
                Path(result.values[0].string_value).resolve() != expected.resolve() or
                result.values[1].string_value != "localization"):
            print(f"rejected: localizer must be in localization mode on {expected}", file=sys.stderr)
            return 1
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
    # NEAR-FIELD MASK. Default is the scan's own range_min, i.e. no change.
    #
    # WHY IT EXISTS. Inside a lift car ~90% of the returns are the car itself, and the car is not
    # in the floor's map. This function scores endpoints landing on occupied cells with no penalty
    # for contradicting the map, so a scan of mostly-unmapped short rays scores best wherever the
    # map's walls are densest -- which on floor1 is an enclosed pocket 7.4 m from the lift. It is
    # not a search bug; it is the honest argmax of a question that should not have been asked.
    #
    # Dropping the near field leaves only the rays that reach out of the doorway, and those DO
    # correspond to mapped structure. Measured by replaying runs/20260907T012518Z and
    # runs/20260907T013926Z -- two physically separate arrivals -- through this search:
    #
    #     unmasked   (5.94,-5.85) 51-55%   the wrong pocket, 7.4 m out, every time
    #     >1.5 m     (2.59, 1.01) 74-79%   ten searches, 4 cm spread, 0.21 m from the lift
    #
    # The answer is flat from 1.3 m to 4.0 m and flips only below ~1.2 m, because the data has a
    # hole there: 765 returns under 1.5 m, TWO between 1.5 and 3.0, 86 beyond. The threshold sits
    # in the hole rather than on a slope. On a floor-2 lobby scan it removes 4 beams of 136 and
    # returns the identical pose -- a no-op where the scan is already a room.
    #
    # safety/floor_plan.py:138 called for exactly this and left it unwritten: "The honest fix is to
    # score only those beams (range beyond the car walls, ~1.5 m+)".
    #
    # NOT A DEFAULT. It throws away every close return, so it is right only where the near field is
    # known to be unmapped clutter -- the arrival, inside the car. Anywhere else those returns are
    # real evidence.
    mp, sc = d["map"], d["scan"]; info = mp.info; res = info.resolution
    _min_range = max(float(a.min_range), sc.range_min) if a.min_range else sc.range_min
    if a.min_range:
        _kept = sum(1 for r in sc.ranges if r == r and _min_range < r < min(15, sc.range_max))
        print(f"  near-field mask: dropping returns under {_min_range:.2f} m "
              f"-- {_kept} of {len(sc.ranges)} rays kept")
    if (sc.header.frame_id != "base_link" or mp.header.frame_id != "map" or
            abs(info.origin.orientation.x) > 1e-6 or
            abs(info.origin.orientation.y) > 1e-6 or abs(info.origin.orientation.z) > 1e-6):
        print("rejected: search requires base_link scan and unrotated map origin", file=sys.stderr)
        return 1
    stamp = sc.header.stamp.sec + sc.header.stamp.nanosec * 1e-9
    if not -0.2 <= time.time() - stamp <= 2.0:
        print("rejected: scan is stale or timestamp is in the future", file=sys.stderr); return 1
    ox, oy = info.origin.position.x, info.origin.position.y
    W, H = info.width, info.height
    grid = np.array(mp.data, dtype=np.int8).reshape(H, W)
    occ = grid > 50
    rs, angs = [], []
    ang = sc.angle_min
    for r in sc.ranges:
        aa = ang; ang += sc.angle_increment
        if r == r and _min_range < r < min(15, sc.range_max):
            rs.append(r); angs.append(aa)
    # BEFORE the subsample, deliberately. If the near field were dropped after this line the
    # thinning would already have spent most of its 120 slots on car wall.
    st = max(1, len(rs) // BEAMS)
    rs = np.array(rs[::st]); angs = np.array(angs[::st])

    try:
        matcher = ScanMatcher(grid, res, (ox, oy), rs, angs)
    except ValueError as exc:
        print(f"rejected: {exc}", file=sys.stderr); return 1
    fit = matcher.fit

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
            return 1
    if a.check:
        return 0

    try:
        hypotheses = matcher.search(initial=(cx, cy, cw))
    except ValueError as exc:
        print(f"rejected: {exc}", file=sys.stderr); return 1
    s, bx, by, bw = hypotheses[0]
    print(f"  refined ({bx:+.2f},{by:+.2f},{math.degrees(bw):+.0f}deg) fit {100*s/len(rs):.1f}%")
    for score, x, y, yaw in hypotheses[1:3]:
        print(f"  alternative ({x:+.2f},{y:+.2f},{math.degrees(yaw):+.0f}deg) fit {100*score/len(rs):.1f}%")
    accepted, reason = acceptance(hypotheses, len(rs), a.min_fit, a.min_margin)
    if not accepted:
        print(f"rejected: {reason}; no pose published", file=sys.stderr); return 2
    if a.dry_run:
        print(f"  dry run: {reason}; no pose published")
        return 0
    m = PoseWithCovarianceStamped()
    m.header.frame_id = "map"; m.header.stamp = n.get_clock().now().to_msg()
    m.pose.pose.position.x = float(bx); m.pose.pose.position.y = float(by)
    m.pose.pose.orientation.z = math.sin(bw / 2); m.pose.pose.orientation.w = math.cos(bw / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.1; m.pose.covariance[35] = 0.05
    for _ in range(4):
        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.2); time.sleep(0.25)
    print("  published /initialpose -- verify the physical pose before navigation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
