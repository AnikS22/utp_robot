from bringup.mapping_gate_policy import is_moving, may_map, must_latch


def test_only_fresh_dual_ackermann_is_allowed():
    assert may_map(0, 0.1)
    assert may_map(1, 0.1)  # parallel/crab has explicit X/Y odometry in the Ranger driver
    assert not may_map(2, 0.1)  # spin
    assert not may_map(3, 0.1)  # park/side-slip, firmware dependent
    assert not may_map(None, 0.1)
    assert not may_map(0, 0.6)  # stale chassis state
    assert not may_map(0, 0.1, unsafe_motion_latched=True)


def test_motion_threshold_and_spin_latch_policy():
    assert is_moving(0.0, 0.02, 0.0)
    assert is_moving(0.0, 0.0, 0.02)
    assert not is_moving(0.005, -0.005, 0.005)
    assert must_latch(2, 0.0, 0.0, 0.02)
    assert not must_latch(1, 0.0, 0.2, 0.0)
