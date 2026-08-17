import math

import pytest

from hnuter_iebc_cube_contact_experiment import ContactForceFilter, smoothstep01, wrap_pi


@pytest.mark.parametrize(
    ('u', 'position', 'slope'),
    [(-1.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.5, 0.5, 1.5),
     (1.0, 1.0, 0.0), (2.0, 1.0, 0.0)],
)
def test_smoothstep_is_bounded_and_stops_at_endpoints(u, position, slope):
    actual_position, actual_slope, _ = smoothstep01(u)
    assert actual_position == pytest.approx(position)
    assert actual_slope == pytest.approx(slope)


def test_wrap_pi_uses_shortest_signed_angle():
    assert wrap_pi(0.0) == pytest.approx(0.0)
    assert wrap_pi(2.0 * math.pi + 0.2) == pytest.approx(0.2)
    assert wrap_pi(-2.0 * math.pi - 0.2) == pytest.approx(-0.2)


def test_contact_filter_decays_after_sample_timeout():
    force_filter = ContactForceFilter(tau_s=0.08, timeout_s=0.15)
    force_filter.feed(4.0, received_s=10.0)

    assert force_filter.update(0.08, now_s=10.05) == pytest.approx(2.0)
    stale_value = force_filter.update(0.08, now_s=10.30)

    assert stale_value == pytest.approx(1.0)
    assert math.isfinite(stale_value)


def test_contact_filter_rejects_negative_force_magnitude():
    force_filter = ContactForceFilter(tau_s=0.0)
    force_filter.feed(-3.0, received_s=1.0)
    assert force_filter.update(0.01, now_s=1.0) == 0.0
