#!/usr/bin/env python3
"""Republish a BEST_EFFORT LaserScan as RELIABLE, and mask the robot out of its own scan.

    python3 bringup/scan_relay.py            # /scan_filtered (best effort) -> /scan (reliable)

WHY THE RELAY. pointcloud_to_laserscan publishes /scan_filtered with Reliability: BEST_EFFORT.
slam_toolbox subscribes to its scan topic with RELIABLE. Those are INCOMPATIBLE in DDS: a
best-effort publisher cannot satisfy a reliable subscriber, so not one message is delivered --
and nothing anywhere reports an error. slam_toolbox simply sits there having logged its stack
size, publishing no /map and no map->odom, looking exactly like a hung node.

Measured 2026-08-30: /scan_filtered at 9.2 Hz with a healthy odom->base_link TF, and
slam_toolbox silent from the moment it started.

This is the same silent-discard shape as the safety mux discarding commands and the deadman that
was never published: the system is working exactly as configured, and the configuration means
"deliver nothing".

WHY THE MASK. See the block above MASK_MIN_DEG. Short version: the OS0 sits on a mast behind the
stowed arm, both are inside the 0.20-1.20 m slice band, and Nav2 was marking the robot's own
geometry LETHAL around its own footprint.

IMPORTING THIS MODULE DOES NOT REQUIRE ROS. The masking logic is a module-level pure function
(mask_self_returns) above the ROS imports, and those imports are soft, so tests/test_scan_mask.py
can exercise the real code rather than pattern-match its source. Running it as a script still
requires ROS, and main() says so with the command that fixes it.
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path
from typing import NamedTuple, Sequence

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "bringup"))

# Overridable so a SECOND relay can carry the driver's native /ouster/scan to SLAM without
# disturbing the costmap chain. Measured 2026-09-04:
#     /ouster/scan     9.97 Hz   native LaserScan, a few kB
#     /ouster/points   2.66 Hz   3.1 MB per message (512 x 128 x 48 B), losing ~73% in DDS
#     /scan_filtered   4.39 Hz   what survives the cloud + pointcloud_to_laserscan
# The cloud is the bottleneck and it exists only to be flattened into a 2D scan. SLAM wants rate
# above all -- at 2 Hz the matcher cannot follow a turn and the pose slides, which is what put the
# robot 1.85 m from where Nav2 said it had arrived on 2026-09-03. The costmap wants the height
# band, because the native ring is a single elevation and would miss a low obstacle. So they get
# different inputs on purpose: SLAM the fast one, Nav2 the safe one.
IN_TOPIC = os.environ.get("UTP_SCAN_IN", "/scan_filtered")
OUT_TOPIC = os.environ.get("UTP_SCAN_OUT", "/scan")

# ---------------------------------------------------------------------------------------------
# SELF-OCCLUSION MASK -- the robot seeing its own arm, mast and chassis.
# ---------------------------------------------------------------------------------------------
# MEASURED 2026-09-01, 10 scans, robot stationary on open floor, nobody nearby. Minimum range by
# 15-degree sector, in base_link:
#
#     -120..-105   min 0.73   median 1.27
#     -105.. -90   min 0.79   median 1.18
#      -90..+60          3.0 - 8.8          <- forward hemisphere, completely clean
#      +90..+105   min 0.73   median 1.67
#     +105..+120   min 0.70   median 0.79
#     +120..+135   min 0.70   median 0.72   <- pinned across all ten scans
#     +135..+150   min 0.70   median 0.85
#
# A return that does not move between scans, only ever appears astern, and sits at a fixed radius
# is the ROBOT, not the room -- the same reasoning that identified the A1M8's dead arc in
# config/lidar.yaml. Here it is the stowed arm, the mast and the chassis rear, all of which are
# inside the 0.20-1.20 m height band by design.
#
# WHAT IT COST BEFORE IT WAS FOUND: Nav2's obstacle layer marked these as lethal cells wrapped
# around the footprint, so the planner believed the robot was standing inside an obstacle. It
# accepted goals and produced no motion, on open floor, with 4.10 m clear straight ahead.
#
# WHY A SECTOR MASK AND NOT A BIGGER range_min. Raising range_min blinds the robot in EVERY
# direction to fix a problem that is confined to the rear arc; at 0.95 m it would stop seeing a
# person standing 0.6 m off the bumper. This removes only the arc that is structure, and only
# within the radius that structure occupies -- beyond MASK_MAX_M the same bearings see the real
# room and are kept. config/ouster.yaml carries the same numbers with the same measurements, and
# tests/test_scan_mask.py pins the two together.
#
# RE-MEASURE THIS IF THE ARM MOVES OR THE MAST CHANGES. It is a fact about the current build,
# not about lidars. bringup/check_scan_geometry.py is the tool.
MASK_MIN_DEG = float(os.environ.get('UTP_MASK_MIN_DEG', 74.0))  # widened from 88 after re-measuring 2026-09-01: the +75..+90 bins are
                        # BIMODAL -- median 1.81-1.88 m (the real room) with a hard floor at
                        # 0.71-0.72 m and spread 1.1-1.3. Two populations at one bearing means
                        # structure in front of room, and the arm reaches further forward than
                        # 88 deg. Below 74 deg the floor climbs smoothly (1.91, 2.00, 2.08 ...)
                        # with spread under 0.25 -- that is room, and it is kept.
MASK_MAX_DEG = float(os.environ.get('UTP_MASK_MAX_DEG', 180.0))  # THE WHOLE REAR ARC, both sides (the test is on |bearing|).
                        #
                        # WAS 155, AND 155 WAS AN ARTEFACT OF THE BUG IT WAS MEANT TO FIX. The
                        # sweep that produced it was taken while pointcloud_to_laserscan's
                        # range_min was 0.50, so every return closer than 0.50 m had already been
                        # deleted before anything was measured. At 155-180 deg those sectors read
                        # min 1.02-1.10 m and looked clean, so the mask was stopped at 155.
                        #
                        # Lowering range_min to 0.30 -- done so a door first seen at 0.72 m could
                        # not vanish under a global cutoff -- immediately exposed, at the same
                        # bearings, on the same stationary robot (measured 2026-09-01 AFTER the
                        # change):
                        #     -180..-135   min 0.39
                        #     +135..+180   min 0.55
                        #     -135.. -90   min 1.00   <- already masked, i.e. the mask held
                        # 0.39 m astern is the chassis rear. It was always there; the cutoff was
                        # hiding it.
                        #
                        # THE GENERAL LESSON, and it is why 155 must never come back: A CUTOFF
                        # THAT HIDES A SELF-RETURN ALSO HIDES THE FACT THAT YOU NEEDED A MASK.
                        # Measuring the extent of a self-occlusion through a filter that is
                        # already deleting self-occlusions can only ever under-report it. Measure
                        # with range_min at its lowest, then mask.
MASK_MAX_M = float(os.environ.get('UTP_MASK_MAX_M', 0.90))  # structure measured at 0.39-0.85 m; this sits just above it.
                        #
                        # WAS 1.00, AND THAT COST A MAP. Measured in open floor, where nothing
                        # real lives inside a metre astern, so the choice looked free -- and I
                        # generalised from the one place it was free. In a TIGHT space it is not:
                        # measured inside the lift car, the side walls sit at min 1.00, median
                        # 1.06-1.15 m, i.e. ON the old boundary, and the rear sectors floored at
                        # exactly 1.00 -- the mask was deleting the very geometry the scan matcher
                        # needs to resolve rotation. slam_toolbox then had only the forward
                        # hemisphere, which is near-degenerate for heading, lost its lock on a
                        # fast turn, and wrote the failed match into the map.
                        #
                        # 0.90 keeps 10 cm of margin over the structure while returning the walls
                        # of any space wider than ~1.8 m. The honest fix is to mask the arm by its
                        # 3D geometry instead of a polar wedge -- the OS0 sees behind perfectly
                        # well and this throws away real returns to remove a known object. That is
                        # a change to the projection, not to this file, and it is the right next
                        # step if a tighter space than the lift is ever mapped.

# One line every LOG_PERIOD_S at most. An operator needs to see the mask working -- "0 bins
# masked" after the arm is re-stowed is the signal that the geometry moved -- but at 6-10 Hz a
# THE MASK IS A BAND, NOT A DISC -- and the lower edge exists for ONE consumer.
#
# UTP_MASK_MIN_M defaults to 0.0, so the mask is a disc out to MASK_MAX_M exactly as before and
# nothing about /scan changes. It is raised only by the SECOND relay instance that feeds Nav2.
#
# WHY THERE IS A SECOND RELAY. Measured 2026-09-05: the OS0 reports a ring of returns 0.85-1.20 m
# behind the robot at z 0.41-0.89 m, in an open lobby, with nothing physically there -- confirmed
# by the operator on foot, and confirmed as an artifact by raycasting the saved map (12 of 19
# bearings reported an obstacle metres inside the mapped wall, every one of them astern). The ring
# is fixed in base_link, so it travels with the robot and is re-marked every cycle; clearing the
# costmap removes it for under six seconds.
#
# What that costs: any Nav2 goal 0.85-1.65 m BEHIND the robot is unreachable, anywhere in the
# building, because the ring plus 0.30 m inflation sits across the path. That is exactly the lift
# entry and exit geometry, and it is why every ordering, waypoint and clear-timing variation tried
# on 2026-09-05 failed identically.
#
# WHY NOT JUST RAISE MASK_MAX_M ON /scan. Because slam_toolbox and Nav2 need opposite things from
# the same bearings, and one scalar cannot serve both:
#     slam_toolbox  needs returns past 1.15 m astern or it loses the lift car's side walls --
#                   MASK_MAX_M 1.00 deleted them once already and cost a map (docs/MORNING.md).
#     Nav2          must not see the ring at 0.85-1.20 m or it can never reverse out of anything.
# So /scan stays exactly as it is for SLAM, and a second relay publishes /scan_nav with the ring's
# band masked out for the costmap. Different consumers, different masks, one implementation.
#
# WHAT THIS GIVES UP, PLAINLY: /scan_nav masks the rear arc as a DISC out to UTP_MASK_MAX_M, so
# the costmap is blind astern to 1.30 m instead of the 0.90 m it was already blind to -- 0.40 m
# of new blindness directly behind the robot.
#
# IT HAS TO BE A DISC, NOT A BAND, AND THAT IS WORTH SPELLING OUT because the obvious improvement
# is wrong: bounding the mask BELOW (say 0.80-1.30 m) so close obstacles still register would let
# the robot's OWN chassis and mast back in -- that structure is measured at 0.39-0.85 m astern and
# removing it is the entire reason this mask exists. A lower bound re-creates the problem the file
# was written to solve. UTP_MASK_MIN_M exists and defaults to 0.0; leave it there unless the
# self-return geometry is re-measured and something is genuinely known to sit under it.
#
# This is a trade made with the artifact still unexplained. When the ring is understood at the
# sensor, DELETE the second relay -- do not widen this band and do not let it become permanent.
MASK_MIN_M = float(os.environ.get('UTP_MASK_MIN_M', 0.0))

# per-scan line is 10 lines a second of noise that hides everything else in the terminal.
LOG_PERIOD_S = 5.0

# Bearing -> bin index is fixed for a given (n, angle_min, angle_increment), and on this stack
# that triple never changes: 1031 bins, angle_min -pi, increment 0.0061 rad, 6-10 Hz forever.
# Computing math.degrees() and a modulo for all 1031 bins on every scan is ~10k trig calls a
# second to re-derive a constant. Cache the index list instead; the per-scan work is then one
# comparison per bin in the masked arc and nothing at all elsewhere.
_MASK_IDX_CACHE: dict[tuple, tuple[int, ...]] = {}


class MaskResult(NamedTuple):
    ranges: list[float]
    masked: int


def _masked_indices(n: int, angle_min: float, angle_increment: float) -> tuple[int, ...]:
    """Bin indices whose bearing falls in the masked arc. Cached per scan geometry."""
    key = (n, angle_min, angle_increment)
    hit = _MASK_IDX_CACHE.get(key)
    if hit is not None:
        return hit
    idx = []
    for i in range(n):
        # NORMALISE TO (-180, +180]. A 1031-bin scan with angle_min = -pi and increment 0.0061
        # spans 359.99 deg, so the last bins land at +179.99 -- but a scan that spans slightly
        # MORE than 360 deg (or one whose angle_min is 0 rather than -pi, which is what
        # pointcloud_to_laserscan emits if angle_min is left at its default) puts bearings at
        # +181 or +270, and an un-normalised abs() would then test |270| against 74..180 and keep
        # a self-return. Wrap first, compare second.
        deg = math.degrees(angle_min + i * angle_increment)
        deg = (deg + 180.0) % 360.0 - 180.0
        if MASK_MIN_DEG <= abs(deg) <= MASK_MAX_DEG:
            idx.append(i)
    if len(_MASK_IDX_CACHE) > 8:        # only ever one geometry in practice; bound it anyway
        _MASK_IDX_CACHE.clear()
    _MASK_IDX_CACHE[key] = hit = tuple(idx)
    return hit


def mask_self_returns(ranges: Sequence[float], angle_min: float,
                      angle_increment: float) -> MaskResult:
    """Replace self-returns with +inf. Returns (new_ranges, number_of_bins_masked).

    A bin is masked when its bearing is in +-MASK_MIN_DEG..MASK_MAX_DEG of astern AND its range
    is at most MASK_MAX_M. Both conditions, always: the same bearings see the real room further
    out and that is exactly what must survive.

    +inf, NOT deletion and NOT NaN.
      * Deleting entries would change the bin count and silently break the
        angle_min/angle_increment contract every subscriber relies on -- bin i would stop meaning
        angle_min + i*angle_increment and every consumer's geometry would quietly rotate.
      * +inf is p2l's own "no return" value here (session.sh passes use_inf:=true), so every
        consumer already handles it on the 84 empty bins in a typical scan.

    NaN and +-inf inputs pass through untouched -- they are already "no observation" and there is
    nothing to mask. Comparing NaN would silently answer False anyway; isfinite says so on
    purpose. The count returned is the count of bins this call actually changed.

    Pure and ROS-free by design, so tests/test_scan_mask.py can call it directly.
    """
    out = list(ranges)
    n = len(out)
    if n == 0 or not math.isfinite(angle_min) or not math.isfinite(angle_increment) \
            or angle_increment == 0.0:
        return MaskResult(out, 0)
    masked = 0
    inf = float("inf")
    for i in _masked_indices(n, angle_min, angle_increment):
        r = out[i]
        if math.isfinite(r) and MASK_MIN_M <= r <= MASK_MAX_M:
            out[i] = inf
            masked += 1
    return MaskResult(out, masked)


# --------------------------------------------------------------------------------------- ROS
# SOFT IMPORTS. Everything above this line is importable without a ROS environment, which is what
# lets the mask be tested for BEHAVIOUR instead of by grepping this file. main() calls
# require_ros() before touching any of it, so running the script with no ROS on the shell still
# prints the sourcing command rather than a traceback.
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                           QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data)
    from sensor_msgs.msg import LaserScan
except ImportError:                     # no ROS on this shell -- import-only use (tests)
    Node = object                       # type: ignore[assignment,misc]


class Relay(Node):
    def __init__(self) -> None:
        super().__init__("utp_scan_relay")
        out_qos = QoSProfile(depth=10,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, OUT_TOPIC, out_qos)
        self.create_subscription(LaserScan, IN_TOPIC, self._cb, qos_profile_sensor_data)
        self._masked_total = 0
        self._last_log = 0.0
        self.n = 0

    def _cb(self, m) -> None:
        m.ranges, masked = mask_self_returns(m.ranges, m.angle_min, m.angle_increment)
        self._masked_total += masked
        self.pub.publish(m)
        self.n += 1
        now = time.monotonic()
        if self.n == 1 or now - self._last_log >= LOG_PERIOD_S:
            self._last_log = now
            self.get_logger().info(
                f"{IN_TOPIC} -> {OUT_TOPIC}: {self.n} scans, mask {masked}/{len(m.ranges)} bins "
                f"this scan ({self._masked_total} total) at |bearing| "
                f"{MASK_MIN_DEG:.0f}-{MASK_MAX_DEG:.0f} deg within {MASK_MAX_M:.2f} m")


def main() -> int:
    from _ros_env import require_ros
    require_ros()
    rclpy.init()
    n = Relay()
    print(f"\n  {IN_TOPIC} (BEST_EFFORT) -> {OUT_TOPIC} (RELIABLE)"
          f"\n  self-return mask: |bearing| {MASK_MIN_DEG:.0f}-{MASK_MAX_DEG:.0f} deg "
          f"within {MASK_MAX_M:.2f} m -> +inf\n")
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
