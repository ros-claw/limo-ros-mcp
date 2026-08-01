"""Versioned public data contracts for LIMO MCP evidence."""

from __future__ import annotations

from typing import Any, TypedDict


class ReadinessCheckV1(TypedDict):
    check_id: str
    severity: str
    passed: bool
    observed: Any
    operator: str
    threshold: Any
    unit: str
    error_code: str
    evidence_refs: list[str]


class ObservationEvidenceV1(TypedDict):
    topic: str
    message_type: str
    ros_stamp_sec: float | None
    received_at: str | None
    age_sec: float | None
    freshness_basis: str
    clock_verified: bool
    clock_issue: str | None
    transport: Any
    transport_generation: Any
    summary_hash: str
    summary: dict[str, Any]
    timing: dict[str, Any]


class ReadinessEvidenceV1(TypedDict):
    schema_version: str
    snapshot_id: str
    robot_id: str
    body_id: str
    body_snapshot_hash: str | None
    created_at: str
    expires_at: str
    created_wall_time: float
    expires_wall_time: float
    created_monotonic: float
    clock: dict[str, Any]
    observation_window: dict[str, Any]
    transport_binding: dict[str, Any]
    observations: dict[str, ObservationEvidenceV1]
    observation_failures: list[str]
    checks: list[ReadinessCheckV1]
    state: str
    ready: bool
    blockers: list[str]
    warnings: list[str]
    policy_id: str
    policy_hash: str
    command_dispatched: bool
    usable_for_real_execution: bool
    snapshot_hash: str
