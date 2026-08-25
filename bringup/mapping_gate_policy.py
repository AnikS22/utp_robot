"""Pure policy for admitting Ranger scans into a SLAM pose graph."""

DUAL_ACKERMANN = 0
PARALLEL_CRAB = 1
SPINNING = 2
STATE_TIMEOUT_S = 0.5


def may_map(motion_mode: int | None, state_age_s: float, unsafe_motion_latched: bool = False) -> bool:
    """Return true only for a fresh DualAckermann chassis-state sample."""
    return (not unsafe_motion_latched and motion_mode in (DUAL_ACKERMANN, PARALLEL_CRAB)
            and state_age_s <= STATE_TIMEOUT_S)


def must_latch(motion_mode: int | None, linear_x: float, linear_y: float,
               angular_z: float) -> bool:
    """Latch only measured spinning motion; crab has a dedicated driver odometry model."""
    return motion_mode == SPINNING and is_moving(linear_x, linear_y, angular_z)


def is_moving(linear_x: float, linear_y: float, angular_z: float) -> bool:
    """Ignore encoder jitter but catch meaningful translation or rotation."""
    return abs(linear_x) > 0.01 or abs(linear_y) > 0.01 or abs(angular_z) > 0.01
