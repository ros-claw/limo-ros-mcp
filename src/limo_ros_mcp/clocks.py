"""Clock-domain handling for read-only ROS observations."""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import UTC, datetime
from typing import Any


@dataclass
class ClockJumpTracker:
    """Detect disagreement between wall and monotonic elapsed time across snapshots."""

    tolerance_sec: float = 1.0
    _last_wall: float | None = None
    _last_monotonic: float | None = None

    def observe(self, *, wall_time: float, monotonic_time: float) -> bool:
        jumped = False
        if self._last_wall is not None and self._last_monotonic is not None:
            wall_delta = wall_time - self._last_wall
            monotonic_delta = monotonic_time - self._last_monotonic
            jumped = wall_delta < 0.0 or abs(wall_delta - monotonic_delta) > self.tolerance_sec
        self._last_wall = wall_time
        self._last_monotonic = monotonic_time
        return jumped


def utc_iso(timestamp: float) -> str:
    return datetime.fromtimestamp(timestamp, tz=UTC).isoformat().replace("+00:00", "Z")


def finite_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def observation_timing(
    summary: dict[str, Any],
    *,
    received_wall_time: float | None,
    received_monotonic: float | None,
    now_wall: float,
    ros_time_active: bool = False,
    future_tolerance_sec: float = 0.25,
) -> dict[str, Any]:
    """Classify one observation without subtracting incompatible clock domains."""

    header_value = summary.get("header")
    header = header_value if isinstance(header_value, dict) else {}
    ros_stamp = finite_number(header.get("stamp_sec"))
    legacy_age = finite_number(header.get("age_sec"))
    receive_time = finite_number(received_wall_time)
    receive_monotonic = finite_number(received_monotonic)

    verified = receive_time is not None and receive_monotonic is not None
    issue: str | None = None
    if receive_time is None or receive_monotonic is None:
        issue = "missing_receive_timestamp"

    if ros_stamp is not None and ros_stamp > 0.0 and not ros_time_active:
        raw_age = now_wall - ros_stamp
        if raw_age < -future_tolerance_sec:
            verified = False
            issue = "ros_stamp_in_future"
            age_sec: float | None = None
        else:
            age_sec = max(0.0, raw_age)
        freshness_basis = "ROS_HEADER"
        clock_domain = "WALL"
    elif legacy_age is not None and legacy_age >= 0.0:
        age_sec = legacy_age
        freshness_basis = "SUMMARY_AGE"
        clock_domain = "ROS" if ros_time_active else "WALL"
    elif receive_time is not None:
        age_sec = max(0.0, now_wall - receive_time)
        freshness_basis = "RECEIVE_TIME"
        clock_domain = "ROS" if ros_time_active else "WALL"
    else:
        age_sec = None
        freshness_basis = "UNAVAILABLE"
        clock_domain = "UNVERIFIED"

    return {
        "ros_stamp_sec": ros_stamp,
        "received_wall_time": receive_time,
        "received_monotonic": receive_monotonic,
        "received_at": utc_iso(receive_time) if receive_time is not None else None,
        "age_sec": age_sec,
        "freshness_basis": freshness_basis,
        "clock_domain": clock_domain,
        "clock_verified": verified,
        "clock_issue": issue,
    }
