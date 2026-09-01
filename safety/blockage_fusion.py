"""Fuse the CAMERA's blockage verdict with the LIDAR's. Pure logic, no ROS, no I/O, testable.

WHY THIS EXISTS. 2026-09-01, captures/trial_ours_001, hardware, real closed glass doors.

  * The camera frame (rgb.png) shows dark door frames at the left and right edges and, straight
    through the middle, an open covered walkway with pillars and trees. bringup/ask_blockage.py
    answers `blocked: False`, description "an open walkway with pillars". The VLM is NOT WRONG
    ABOUT THE PICTURE -- the picture genuinely shows an open walkway. Glass is transparent to a
    camera, so a correct perception of the image is a wrong perception of the world.
  * The lidar scan captured at the SAME INSTANT (scan.json, 1031 bins, 947 valid returns) has 85
    returns within +-20 deg of forward, the nearest at 0.72 m. Inside the drive corridor this
    module actually tests, 39 returns, nearest range 0.70 m. The lidar saw the door perfectly.

The system believed the camera and drove to within 0.72 m of a closed door.

THE RULE: BLOCKED IF EITHER SENSOR SAYS BLOCKED. A fail-closed OR -- not an AND, not a vote.

The justification is physics, not majority opinion. The two sensors do not fail on the same
things, they fail on OPPOSITE things:

  * The camera looks THROUGH glass and reports what is behind it (trial_ours_001, above).
  * The lidar looks through glass too, at the angles where the pane returns nothing to it, and it
    misses thin frames and low sills between beams. captures/trial_ours_002 is the same building,
    a different pose and lighting: the camera says `blocked: True, kind: door, "closed glass
    doors"` and is right, while the corridor rectangle holds ZERO returns and the nearest forward
    return is 1.40 m -- the lidar there sees nothing at all in front of the robot.

So each sensor is the only witness in exactly the case the other one gets wrong. Requiring
AGREEMENT would mean requiring BOTH to succeed on the case each is worst at, and the intersection
of "camera sees glass" and "lidar sees glass" is close to empty. An AND, or a two-of-two vote,
would have cleared BOTH captures above. One of them is a door at 0.72 m. Getting this wrong
drives the robot into a glass door, so the union is the only defensible combination: a single
positive from either sensor stops the base, and a clear verdict requires both to be silent.

The cost of the OR is a false stop when one sensor is wrong in the other direction, and that cost
is cheap and recoverable -- the robot stands still and the reasoner looks for a control. The cost
of the AND is not recoverable.

WHAT THIS DOES NOT DO. It does not decide what to do about the obstruction -- same line
bringup/ask_blockage.py draws. It reports what is there and which sensor saw it. And it never
invents `kind`: if the lidar fired and the camera did not classify anything, `kind` stays "".
Fabricating "door" would send the reasoner hunting for a control that may not exist, and the
trial would then record a REASONING failure that was really a PERCEPTION failure.
"""
from __future__ import annotations

import math

try:  # imported as `safety.blockage_fusion` (tests, bringup/ask_blockage.py)
    from safety.waypoint_drive import corridor_blocked
except ImportError:  # imported bare, with safety/ itself on sys.path
    from waypoint_drive import corridor_blocked  # type: ignore  # noqa: F401

# The `evidence` field. Which sensor(s) actually asserted the blockage -- NOT a confidence, and
# NOT redundant with `blocked`: "neither" can accompany blocked=True, and that combination means
# "stopped because nothing could see", which is a different thing to fix than a real door.
EV_CAMERA = "camera"
EV_LIDAR = "lidar"
EV_BOTH = "both"
EV_NEITHER = "neither"

# WHY look_ahead_m DEFAULTS TO 1.30 HERE AND 0.90 IN waypoint_drive.corridor_blocked.
# 0.90 m is the drive-tick veto: it asks "may I take the next step". This asks a different
# question -- "is the thing in front of me an obstruction the reasoner has to deal with" -- and it
# is asked at a standstill, after the robot has already stopped or is about to. The door that
# caused this module to exist was at 0.72 m, which 0.90 m would have caught with only 18 cm of
# margin; at Limits.v_max = 0.25 m/s that is under a second of travel, and the scan is a captured
# frame that may already be a tick or two old. 1.30 m puts a door detected at 0.72 m well inside
# the window with margin to spare, and the box is still far shorter than a corridor, so a wall at
# the far end of a room does not latch it.
DEFAULT_LOOK_AHEAD_M = 1.30
DEFAULT_HALF_WIDTH_M = 0.40   # matches corridor_blocked: the robot is 0.5 m wide at any range
DEFAULT_MIN_HITS = 3          # see corridor_blocked: one stray return is a speckle, not a thing


def _finite(v) -> bool:
    """True only for a real, finite number. Anything else -- None, a string, NaN, inf -- is not
    evidence and must never be compared against a threshold."""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return False
    return f == f and abs(f) != float("inf")


def _clean(ranges) -> list:
    """Ranges as plain floats, with everything unusable turned into NaN.

    NaN and inf are LEGAL in a LaserScan (REP 117: no return / out of range) and they survive the
    round trip to disk -- json.dump writes them as the NaN / Infinity literals and json.load reads
    them straight back. corridor_blocked already skips both. What it does NOT survive is a
    `null` or a string in the list, which would reach `abs(r)` and raise: this function is the
    place that turns "the scan is malformed" into "that bin has no return", because a malformed
    bin must not be able to crash the guard that stops the robot.
    """
    out = []
    for r in (ranges if ranges is not None else ()):
        try:
            out.append(float(r))
        except (TypeError, ValueError):
            out.append(float("nan"))
    return out


def _camera_verdict(camera) -> bool | None:
    """True / False / None-for-unknown from whatever `ask_blockage` handed us.

    Strict identity, exactly like ask_blockage.parse's `not (passable is True)`: only a real
    bool is a verdict. A missing key, None, "true", 1 -- anything else -- is UNKNOWN, not clear.
    Coercing a truthy string into a verdict is how a malformed reply becomes a confident answer.
    """
    if not isinstance(camera, dict):
        return None
    b = camera.get("blocked")
    if b is True:
        return True
    if b is False:
        return False
    return None


def _text(camera, key: str) -> str:
    if not isinstance(camera, dict):
        return ""
    v = camera.get(key)
    return "" if v is None else str(v).strip()


def _nearest_in_corridor(ranges, angle_min: float, angle_increment: float,
                         half_width_m: float, look_ahead_m: float):
    """Nearest RANGE among the returns inside the same rectangle corridor_blocked tests.

    THIS ONLY MEASURES. The decision is corridor_blocked's alone and is never derived from this
    number, so if the two ever drift apart about the box, the veto still wins and the worst that
    happens is a slightly wrong distance in a description.

    It is a RANGE, not a stopping distance. For a point off the axis the along-track x is
    smaller: in trial_ours_001 the nearest range in the box is 0.70 m but that return sits at
    -33 deg, x = 0.59 m. Use this to TELL somebody how close the thing is, never as a clearance
    budget. (The incident log quotes 0.72 m; that is the nearest return within +-20 deg of
    forward, a slightly narrower window than this 0.40 m-half-width box. Same door.)

    Returns None when the box is empty -- including when it holds fewer returns than min_hits,
    because a number here is a measurement, not a verdict.
    """
    best = None
    for i, r in enumerate(ranges):
        if r != r or abs(r) == float("inf"):
            continue
        a = angle_min + i*angle_increment
        x, y = r*math.cos(a), r*math.sin(a)
        if 0.0 < x <= look_ahead_m and abs(y) <= half_width_m:
            if best is None or r < best:
                best = r
    return None if best is None else float(best)


def clearance_ahead_m(ranges, angle_min: float, angle_increment: float,
                      *, half_width_m: float = DEFAULT_HALF_WIDTH_M,
                      look_ahead_m: float = DEFAULT_LOOK_AHEAD_M):
    """Smallest ALONG-TRACK distance to anything in the corridor, or None if the box is empty.

    THIS IS THE ONE TO REVERSE FROM. `_nearest_in_corridor` returns a RANGE, and its own docstring
    says not to use a range as a clearance budget -- yet the back-off distance was being computed
    from one. For an off-axis return the range overstates the clearance: in the real capture
    `captures/trial_ours_001/scan.json`, taken at closed glass doors, the nearest range in the box
    is 0.70 m but that return sits at -33 deg, so the along-track distance is 0.59 m. Backing off
    "to 1.40 m" from a 0.70 m range leaves the robot 11 cm closer than intended, in the one
    direction where the error is spent driving toward the thing you are backing away from.

    WHY THIS LIVES HERE AND NOT IN THE CALLER. There were three disagreeing implementations of
    "how far ahead is it" on 2026-09-01: a +-15 deg cone in approach_blockage, a +-20 deg cone in
    ros_world, and this rectangle. At 2 m a +-20 deg cone is 0.73 m half-width; at 0.5 m it is
    0.18 m -- so the same scan yielded different answers depending which module asked, and which
    one produced the number depended on whether an optional import had succeeded. A standoff is a
    physical quantity; it cannot have three values.

    The rectangle is the right shape for the question, for corridor_blocked's reason: the robot is
    0.5 m wide at every range, so a cone either clears a real obstacle far away or vetoes on a
    harmless wall up close.
    """
    best = None
    for i, r in enumerate(_clean(ranges)):
        if not _finite(r) or r <= 0.0:
            continue
        a = angle_min + i * angle_increment
        x, y = r * math.cos(a), r * math.sin(a)
        if 0.0 < x <= look_ahead_m and abs(y) <= half_width_m:
            best = x if best is None else min(best, x)
    return best


def fuse(camera: dict, ranges, angle_min: float, angle_increment: float,
         *, half_width_m: float = DEFAULT_HALF_WIDTH_M,
         look_ahead_m: float = DEFAULT_LOOK_AHEAD_M,
         min_hits: int = DEFAULT_MIN_HITS) -> dict:
    """Combine a VLM blockage verdict and a laser scan into one BlockageEvent. Fail-closed OR.

    camera           the dict bringup/ask_blockage.py returns: {blocked, kind, description}. May
                     be None, may be missing keys -- a VLM outage must not raise here.
    ranges           LaserScan.ranges, NaN and inf allowed and expected
    angle_min        LaserScan.angle_min, radians, in the scan's own frame
    angle_increment  LaserScan.angle_increment, radians
    half_width_m     corridor half width; the robot is 0.5 m wide at any range
    look_ahead_m     how far down the corridor to look -- see DEFAULT_LOOK_AHEAD_M above for why
                     this is 1.30 and not corridor_blocked's own 0.90
    min_hits         returns needed inside the box before it counts (stray-point rejection)

    Returns {blocked, kind, description, evidence, nearest_ahead_m}. `blocked` is the OR argued
    for in the module docstring; `evidence` says WHO saw it, one of "camera"/"lidar"/"both"/
    "neither"; `nearest_ahead_m` is a float or None and is informational only.

    THE BLIND CASE. When the camera gives no verdict AND the scan carries nothing usable, there
    is no evidence in either direction and this returns blocked=True with evidence="neither" --
    stopped because nothing could see, not because anything was seen. But when EITHER sensor is
    working and says clear, that is a real reading from a real sensor and it is honoured. Freezing
    the robot on every VLM hiccup would be a different policy (whether to run at all without a
    VLM), it belongs to preflight rather than to a per-tick fusion, and paying it here would also
    poison the OR: every dropped VLM call would become an "obstruction" for the reasoner to go
    hunting a control for. The caller can still see evidence="neither" and choose to stop.
    """
    cam_blocked = _camera_verdict(camera)
    kind = _text(camera, "kind")
    cam_desc = _text(camera, "description")

    rs = _clean(ranges)
    # "Usable" is stricter than "present": a scan of pure NaN, or one whose geometry fields are
    # NaN, cannot say clear OR blocked, and must not be mistaken for a clear reading. Without
    # this check corridor_blocked would dutifully return False on it -- every coordinate would be
    # NaN, every comparison False -- and a dead lidar would read exactly like an open corridor.
    lidar_usable = (bool(rs) and _finite(angle_min) and _finite(angle_increment)
                    and float(angle_increment) != 0.0
                    and any(r == r and abs(r) != float("inf") for r in rs))

    nearest = None
    lidar_blocked = False
    if lidar_usable:
        # THE RECTANGLE TEST IS NOT REIMPLEMENTED HERE. corridor_blocked is the one place in the
        # repo that decides whether the corridor is obstructed, so a fix to it fixes both the
        # drive-tick veto and this fusion. Only the look-ahead differs, and that is an argument.
        lidar_blocked = corridor_blocked(rs, float(angle_min), float(angle_increment),
                                         half_width_m=half_width_m,
                                         look_ahead_m=look_ahead_m,
                                         min_hits=min_hits)
        nearest = _nearest_in_corridor(rs, float(angle_min), float(angle_increment),
                                       half_width_m, look_ahead_m)

    cam_fired = cam_blocked is True
    if cam_fired and lidar_blocked:
        evidence = EV_BOTH
    elif cam_fired:
        evidence = EV_CAMERA
    elif lidar_blocked:
        evidence = EV_LIDAR
    else:
        evidence = EV_NEITHER

    blocked = cam_fired or lidar_blocked          # <- the whole point of the module
    if evidence == EV_NEITHER and cam_blocked is None and not lidar_usable:
        blocked = True                            # blind, see THE BLIND CASE above

    if evidence == EV_LIDAR:
        # The camera did not report this, so the camera's own words cannot be the description --
        # downstream that text is what a language model reasons over, and "an open walkway with
        # pillars" attached to blocked=True is worse than useless. Say what fired, say how far,
        # and keep the camera's words as the contradiction they are rather than dropping them:
        # the disagreement is itself the strongest hint that the obstruction is transparent.
        where = ("%.2f m ahead" % nearest) if nearest is not None else "just ahead"
        description = "something solid %s that the camera did not report" % where
        if cam_desc:
            description += '; the camera reported: "%s"' % cam_desc
    elif blocked and evidence == EV_NEITHER:
        description = ("no usable sensor evidence: the camera returned no verdict and the laser "
                       "scan had no usable returns -- stopped because nothing can confirm the "
                       "way is clear")
    else:
        # camera / both / clear: the camera's own description is already about the right thing,
        # and other code reasons over this string. Do not rewrite it; the distance is in
        # nearest_ahead_m for anyone who wants it.
        description = cam_desc

    return {"blocked": bool(blocked),
            "kind": kind,
            "description": description,
            "evidence": evidence,
            "nearest_ahead_m": nearest}
