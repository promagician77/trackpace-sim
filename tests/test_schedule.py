import pytest

from trackpace.schedule import PacePlan

P = 400.0012


def integrate(plan, t0, t1, n=20000):
    # trapezoid of v(t) must match s(t1) - s(t0)
    h = (t1 - t0) / n
    acc = 0.5 * (plan.at(t0).v + plan.at(t1).v)
    for i in range(1, n):
        acc += plan.at(t0 + i * h).v
    return acc * h


def test_constant_pace_crosses_line_on_time():
    plan = PacePlan([59.0] * 5, P)
    for k, tc in enumerate(plan.crossings()):
        assert plan.at(tc).s == pytest.approx(k * P, abs=1e-9)
    assert plan.at(0.0).v == pytest.approx(P / 59.0)
    assert plan.max_speed == pytest.approx(6.7797, abs=1e-4)


def test_runup_starts_from_rest_on_home_straight():
    plan = PacePlan([59.0], P, runup_accel=0.5)
    g = plan.segs[0]
    assert g.v0 == 0.0 and g.v1 == pytest.approx(P / 59.0)
    assert g.s1 == pytest.approx(0.0, abs=1e-9)
    assert plan.runup_distance == pytest.approx((P / 59) ** 2 / 1.0)
    assert plan.runup_distance < 84.39


def test_fast_pace_lengthens_runup_acceleration_not_distance():
    plan = PacePlan([45.0], P, runup_accel=0.3, runup_max_m=80.0)
    assert plan.runup_distance == pytest.approx(80.0)


@pytest.mark.parametrize("laps", [[70, 66, 62, 59], [59, 62, 70], [60, 55, 60, 55], [75, 59]])
def test_pace_changes_keep_every_lap_exact(laps):
    plan = PacePlan([float(x) for x in laps], P, ramp_accel=0.4)
    cr = plan.crossings()
    for k, tc in enumerate(cr):
        assert plan.at(tc).s == pytest.approx(k * P, abs=1e-7)
    for k in range(len(laps)):
        assert integrate(plan, cr[k], cr[k + 1]) == pytest.approx(P, abs=1e-3)
    # speed is continuous everywhere
    ts = [plan.start + i * 0.01 for i in range(int((plan.end - plan.start) / 0.01))]
    for a, b in zip(ts, ts[1:]):
        assert abs(plan.at(b).v - plan.at(a).v) < 0.4 * 0.01 + 1.0 * 0.01 + 1e-9


def test_impossible_change_is_rejected():
    with pytest.raises(ValueError):
        PacePlan([80.0, 40.0], P, ramp_accel=0.05)


def test_stop_and_hold():
    plan = PacePlan([59.0], P, stop_decel=1.0)
    end = plan.at(plan.end + 5)
    assert end.v == 0.0
    assert end.s == pytest.approx(P + (P / 59) ** 2 / 2)
