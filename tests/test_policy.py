"""Operator readiness policy validation tests."""

from pathlib import Path

import pytest
import yaml

from limo_ros_mcp.policy import load_readiness_policy


def test_default_policy_has_stable_identity_and_safe_thresholds() -> None:
    policy = load_readiness_policy()

    assert policy.policy_id == "limo-patrol-default"
    assert policy.policy_hash.startswith("sha256:")
    assert policy.snapshot_ttl_sec == 5.0
    assert policy.thresholds["front_clearance_m"] == 0.35
    assert policy.thresholds["laser_valid_ratio"] == 0.40
    assert policy.thresholds["status_fresh_sec"] == 3.0


def test_policy_cannot_expand_threshold_beyond_code_safety_bounds(tmp_path: Path) -> None:
    policy = load_readiness_policy()
    document = dict(policy.document)
    document["thresholds"] = {**policy.document["thresholds"], "front_clearance_m": 50.0}
    path = tmp_path / "unsafe.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")

    with pytest.raises(ValueError, match="front_clearance_m"):
        load_readiness_policy(path)
