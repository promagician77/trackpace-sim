import math

import pytest

from trackpace.track import Placement, Track, lane_radius

# World Athletics 400 m stagger (lane n vs lane 1), metres
STAGGER_400 = {2: 7.04, 3: 14.70, 4: 22.37, 5: 30.03, 6: 37.70, 7: 45.37, 8: 53.03}


def test_lane1_is_400m():
    t = Track(1)
    assert t.R == pytest.approx(36.80)
    assert t.length == pytest.approx(400.0, abs=0.002)
    assert t.bend == pytest.approx(115.61, abs=0.01)


@pytest.mark.parametrize("lane,stagger", STAGGER_400.items())
def test_staggers_match_world_athletics(lane, stagger):
    assert Track(lane).length - Track(1).length == pytest.approx(stagger, abs=0.01)


def test_path_is_continuous_and_unit_speed():
    t = Track(1)
    prev = t.sample(0.0)
    for i in range(1, 40001):
        s = i * 0.01
        p = t.sample(s)
        step = math.hypot(p.x - prev.x, p.y - prev.y)
        assert step == pytest.approx(0.01, abs=1e-6)
        assert math.hypot(p.tx, p.ty) == pytest.approx(1.0)
        prev = p


def test_tangent_matches_finite_difference():
    t = Track(3)
    for s in [1.0, 60.0, 115.0, 150.0, 250.0, 330.0, 399.0]:
        a, b = t.sample(s - 1e-4), t.sample(s + 1e-4)
        p = t.sample(s)
        assert (b.x - a.x) / 2e-4 == pytest.approx(p.tx, abs=1e-5)
        assert (b.y - a.y) / 2e-4 == pytest.approx(p.ty, abs=1e-5)


def test_counter_clockwise_and_finish_line():
    t = Track(1)
    p0 = t.sample(0.0)
    assert (p0.x, p0.y) == pytest.approx((t.L / 2, -t.R))   # end of home straight
    assert p0.tx == pytest.approx(1.0)
    # first bend turns left (counter-clockwise): heading rotates positively
    q = t.sample(t.bend / 2)
    assert (q.tx, q.ty) == pytest.approx((0.0, 1.0), abs=1e-9)
    assert t.sample(-10).segment == "home"                  # run-up lies on the home straight


@pytest.mark.parametrize("s", [0.5, 57.0, 115.0, 116.0, 180.0, 231.0, 300.0, 399.5])
@pytest.mark.parametrize("d", [-0.7, 0.0, 1.2])
def test_project_inverts_offset_point(s, d):
    t = Track(1)
    x, y, _ = t.offset_point(s, d)
    s2, d2 = t.project(x, y)
    assert s2 == pytest.approx(s, abs=1e-9)
    assert d2 == pytest.approx(d, abs=1e-9)


@pytest.mark.parametrize("bearing", [0.0, 37.0, 90.0, 215.0])
def test_placement_roundtrip_and_anchor(bearing):
    t = Track(1)
    pl = Placement.anchored_at(t, -46.0, 0.0, 3.0, -2.0, bearing)
    x, y, p = t.offset_point(-46.0, 0.0)
    assert pl.to_ned(x, y) == pytest.approx((3.0, -2.0))
    n, e = pl.to_ned(12.3, -4.5)
    assert pl.from_ned(n, e) == pytest.approx((12.3, -4.5))
    # nose points down the home straight = the bearing
    assert (pl.yaw_deg(p.tx, p.ty) - bearing + 180) % 360 - 180 == pytest.approx(0.0, abs=1e-9)


def test_running_direction_is_counter_clockwise_in_ned():
    # home straight running north: first bend must turn towards west (left)
    t = Track(1)
    pl = Placement(0.0, 0.0, 0.0)
    a = pl.to_ned(*t.offset_point(0.0, 0.0)[:2])
    b = pl.to_ned(*t.offset_point(t.bend, 0.0)[:2])
    assert b[1] < a[1]                                      # moved west


def test_gps_placement_scale():
    pl = Placement.from_gps(47.397742 + 0.001, 8.545594, 47.397742, 8.545594, 0.0)
    assert pl.north0 == pytest.approx(111.19, abs=0.01)
    assert pl.east0 == pytest.approx(0.0)


def test_lane_radius_bounds():
    assert lane_radius(2) == pytest.approx(37.92)
    with pytest.raises(ValueError):
        lane_radius(0)
