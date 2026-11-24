"""Per-camera uplink rate limiter and Pi clock-jump recovery."""

from jcheng_perception_core import uplink_allowed


def test_uplink_allowed_normal_gap_and_rate_limit():
    interval = 100_000_000
    allowed, last = uplink_allowed(1_000_000_000, 0, interval)
    assert allowed
    assert last == 1_000_000_000
    allowed, last = uplink_allowed(
        1_000_000_000 + 50_000_000, 1_000_000_000, interval)
    assert not allowed
    assert last == 1_000_000_000
    allowed, last = uplink_allowed(
        1_000_000_000 + 150_000_000, 1_000_000_000, interval)
    assert allowed
    assert last == 1_150_000_000


def test_uplink_allowed_recovers_from_backwards_clock_jump():
    interval = 100_000_000
    # The Pi clock jumped back; the old baseline is in the future. Do not skip
    # this camera forever.
    allowed, last = uplink_allowed(
        500_000_000, 3_000_000_000, interval)
    assert allowed
    assert last == 500_000_000


def test_uplink_allowed_treats_a_long_gap_as_a_fresh_start():
    interval = 100_000_000
    allowed, last = uplink_allowed(
        20_000_000_000, 1_000_000_000, interval)
    assert allowed
    assert last == 20_000_000_000
