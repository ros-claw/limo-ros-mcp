"""Canonical readiness evidence integrity tests."""

import math

import pytest

from limo_ros_mcp.evidence import canonical_json, seal_snapshot, verify_snapshot


def test_snapshot_hash_is_order_independent_and_tamper_evident() -> None:
    first = seal_snapshot({"b": 2, "a": {"x": 1.25}})
    second = seal_snapshot({"a": {"x": 1.25}, "b": 2})

    assert first["snapshot_hash"] == second["snapshot_hash"]
    assert verify_snapshot(first) is True
    first["a"] = {"x": 1.5}
    assert verify_snapshot(first) is False


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_canonical_json_rejects_non_finite_numbers(value: float) -> None:
    with pytest.raises(ValueError, match="non-finite"):
        canonical_json({"unsafe": value})
