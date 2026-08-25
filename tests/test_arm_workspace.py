from bringup.arm_workspace import contains, effective_boundary


def config(margin=20):
    return {"margin_mm": margin, "bounds_mm": {
        "x_min": 100, "x_max": 600, "y_min": -300, "y_max": 300,
        "z_min": 50, "z_max": 700,
    }}


def test_sdk_boundary_order_and_inward_margin():
    assert effective_boundary(config()) == [580, 120, 280, -280, 680, 70]


def test_contains_inclusive_and_rejects_each_side():
    b = effective_boundary(config())
    assert contains(b, [120, -280, 70])
    assert contains(b, [580, 280, 680])
    assert not contains(b, [119, 0, 100])
    assert not contains(b, [581, 0, 100])
    assert not contains(b, [200, -281, 100])
    assert not contains(b, [200, 281, 100])
    assert not contains(b, [200, 0, 69])
    assert not contains(b, [200, 0, 681])


def test_missing_bound_fails_closed():
    c = config()
    c["bounds_mm"]["x_min"] = None
    try:
        effective_boundary(c)
    except ValueError as e:
        assert "x_min" in str(e)
    else:
        raise AssertionError("missing boundary accepted")


def test_margin_cannot_invert_envelope():
    c = config(margin=260)
    try:
        effective_boundary(c)
    except ValueError as e:
        assert "empty envelope" in str(e)
    else:
        raise AssertionError("inverted envelope accepted")
