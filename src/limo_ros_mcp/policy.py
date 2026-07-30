"""Operator-owned readiness policy loading and validation."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from limo_ros_mcp.evidence import sha256_document


@dataclass(frozen=True)
class ReadinessPolicy:
    policy_id: str
    snapshot_ttl_sec: float
    max_observation_skew_sec: float
    thresholds: dict[str, float]
    severities: dict[str, str]
    critical_observations: tuple[str, ...]
    document: dict[str, Any]
    policy_hash: str


_SAFE_BOUNDS: dict[str, tuple[float, float]] = {
    "status_fresh_sec": (0.1, 10.0),
    "odom_fresh_sec": (0.1, 10.0),
    "imu_fresh_sec": (0.1, 30.0),
    "laser_fresh_sec": (0.1, 10.0),
    "amcl_fresh_sec": (0.1, 30.0),
    "navigation_fresh_sec": (0.1, 30.0),
    "costmap_fresh_sec": (0.1, 30.0),
    "diagnostics_fresh_sec": (0.1, 30.0),
    "front_clearance_m": (0.1, 5.0),
    "laser_valid_ratio": (0.01, 1.0),
    "amcl_variance_x": (0.001, 10.0),
    "amcl_variance_y": (0.001, 10.0),
    "amcl_variance_yaw": (0.001, 10.0),
    "battery_return_voltage": (5.0, 30.0),
}


def _default_policy_path() -> Path:
    installed = Path(__file__).resolve().parent / "data" / "configs" / "limo_readiness_policy.yaml"
    if installed.exists():
        return installed
    return Path(__file__).resolve().parents[2] / "configs" / "limo_readiness_policy.yaml"


def _number(document: dict[str, Any], key: str) -> float:
    value = document.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"readiness policy threshold {key} must be numeric")
    result = float(value)
    lower, upper = _SAFE_BOUNDS[key]
    if not lower <= result <= upper:
        raise ValueError(f"readiness policy threshold {key} must be within [{lower}, {upper}]")
    return result


def load_readiness_policy(path: str | Path | None = None) -> ReadinessPolicy:
    """Load the operator policy; an environment override is operator deployment config."""

    configured = path or os.environ.get("ROSCLAW_LIMO_READINESS_POLICY")
    policy_path = Path(configured).expanduser() if configured else _default_policy_path()
    raw = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("readiness policy must be a YAML object")
    if raw.get("schema_version") != "limo.readiness-policy.v1":
        raise ValueError("unsupported readiness policy schema_version")

    thresholds_raw = raw.get("thresholds")
    if not isinstance(thresholds_raw, dict):
        raise ValueError("readiness policy thresholds must be an object")
    thresholds = {key: _number(thresholds_raw, key) for key in _SAFE_BOUNDS}

    severities_raw = raw.get("severities")
    if not isinstance(severities_raw, dict):
        raise ValueError("readiness policy severities must be an object")
    severities: dict[str, str] = {}
    for check_id, severity in severities_raw.items():
        if not isinstance(check_id, str) or severity not in {"BLOCK", "WARN"}:
            raise ValueError("readiness policy severities must be BLOCK or WARN")
        severities[check_id] = severity

    critical = raw.get("critical_observations")
    if not isinstance(critical, list) or not all(isinstance(item, str) for item in critical):
        raise ValueError("critical_observations must be a string list")

    ttl = raw.get("snapshot_ttl_sec")
    skew = raw.get("max_observation_skew_sec")
    if isinstance(ttl, bool) or not isinstance(ttl, (int, float)) or not 1.0 <= float(ttl) <= 10.0:
        raise ValueError("snapshot_ttl_sec must be within [1, 10]")
    if (
        isinstance(skew, bool)
        or not isinstance(skew, (int, float))
        or not 0.1 <= float(skew) <= 10.0
    ):
        raise ValueError("max_observation_skew_sec must be within [0.1, 10]")

    document = dict(raw)
    return ReadinessPolicy(
        policy_id=str(raw.get("policy_id", "limo-patrol-default")),
        snapshot_ttl_sec=float(ttl),
        max_observation_skew_sec=float(skew),
        thresholds=thresholds,
        severities=severities,
        critical_observations=tuple(critical),
        document=document,
        policy_hash=sha256_document(document),
    )
