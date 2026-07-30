"""Clock-domain classification tests."""

import pytest

from limo_ros_mcp.clocks import ClockJumpTracker, observation_timing


def test_headerless_message_uses_recorded_receive_time() -> None:
    timing = observation_timing(
        {},
        received_wall_time=100.0,
        received_monotonic=50.0,
        now_wall=100.2,
    )

    assert timing["clock_verified"] is True
    assert timing["freshness_basis"] == "RECEIVE_TIME"
    assert timing["age_sec"] == pytest.approx(0.2)


def test_future_ros_stamp_invalidates_clock() -> None:
    timing = observation_timing(
        {"header": {"stamp_sec": 101.0}},
        received_wall_time=100.0,
        received_monotonic=50.0,
        now_wall=100.0,
    )

    assert timing["clock_verified"] is False
    assert timing["clock_issue"] == "ros_stamp_in_future"
    assert timing["age_sec"] is None


def test_clock_tracker_detects_wall_jump_against_monotonic_time() -> None:
    tracker = ClockJumpTracker(tolerance_sec=0.5)

    assert tracker.observe(wall_time=100.0, monotonic_time=10.0) is False
    assert tracker.observe(wall_time=101.0, monotonic_time=11.0) is False
    assert tracker.observe(wall_time=90.0, monotonic_time=12.0) is True
