"""Pure, evidence-producing patrol readiness evaluation."""

from __future__ import annotations

import math
import time
import uuid
from collections import deque
from collections.abc import Mapping
from typing import Any, cast

from limo_ros_mcp.clocks import finite_number, observation_timing, utc_iso
from limo_ros_mcp.contract import LIMO_OBSERVATION_TOPICS
from limo_ros_mcp.errors import (
    LIMO_AMCL_COVARIANCE_HIGH,
    LIMO_AMCL_STALE,
    LIMO_BATTERY_BELOW_RETURN_THRESHOLD,
    LIMO_CLOCK_UNVERIFIED,
    LIMO_COSTMAP_STALE,
    LIMO_DIAGNOSTIC_ERROR,
    LIMO_MOTION_MODE_MISMATCH,
    LIMO_MOVE_BASE_UNAVAILABLE,
    LIMO_OBSERVATION_WINDOW_INCOHERENT,
    LIMO_ODOM_STALE,
    LIMO_SCAN_STALE,
    LIMO_SNAPSHOT_EXPIRED,
    LIMO_SNAPSHOT_HASH_MISMATCH,
    LIMO_STATUS_STALE,
    LIMO_TF_CHAIN_BROKEN,
)
from limo_ros_mcp.evidence import seal_snapshot, sha256_document, verify_snapshot
from limo_ros_mcp.policy import ReadinessPolicy, load_readiness_policy
from limo_ros_mcp.schemas import ReadinessEvidenceV1

_CHECK_ERROR_CODES = {
    "CLOCK_VERIFIED": LIMO_CLOCK_UNVERIFIED,
    "OBSERVATION_WINDOW_COHERENT": LIMO_OBSERVATION_WINDOW_INCOHERENT,
    "STATUS_AVAILABLE": "LIMO_STATUS_UNAVAILABLE",
    "STATUS_FRESH": LIMO_STATUS_STALE,
    "ERROR_CODE_ZERO": "LIMO_ERROR_CODE_NONZERO",
    "MOTION_MODE_KNOWN": LIMO_MOTION_MODE_MISMATCH,
    "BATTERY_ABOVE_RETURN_THRESHOLD": LIMO_BATTERY_BELOW_RETURN_THRESHOLD,
    "ODOM_AVAILABLE": "LIMO_ODOM_UNAVAILABLE",
    "ODOM_FRESH": LIMO_ODOM_STALE,
    "IMU_AVAILABLE": "LIMO_IMU_UNAVAILABLE",
    "IMU_FRESH": "LIMO_IMU_STALE",
    "LASER_AVAILABLE": "LIMO_SCAN_UNAVAILABLE",
    "LASER_FRESH": LIMO_SCAN_STALE,
    "LASER_VALID_RATIO": "LIMO_SCAN_INVALID_RATIO",
    "FRONT_CLEARANCE": "LIMO_FRONT_CLEARANCE_LOW",
    "AMCL_AVAILABLE": "LIMO_AMCL_UNAVAILABLE",
    "AMCL_FRESH": LIMO_AMCL_STALE,
    "AMCL_COVARIANCE": LIMO_AMCL_COVARIANCE_HIGH,
    "MOVE_BASE_AVAILABLE": LIMO_MOVE_BASE_UNAVAILABLE,
    "MAP_METADATA_VALID": "LIMO_MAP_METADATA_INVALID",
    "GLOBAL_COSTMAP_FRESH": LIMO_COSTMAP_STALE,
    "LOCAL_COSTMAP_FRESH": LIMO_COSTMAP_STALE,
    "DIAGNOSTICS_ERROR_ZERO": LIMO_DIAGNOSTIC_ERROR,
    "DIAGNOSTICS_WARN_POLICY": "LIMO_DIAGNOSTIC_WARN",
    "TF_MAP_ODOM_BASE": LIMO_TF_CHAIN_BROKEN,
    "BODY_SNAPSHOT_BOUND": "LIMO_BODY_SNAPSHOT_UNBOUND",
}


def _mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _severity(policy: ReadinessPolicy, check_id: str) -> str:
    return policy.severities.get(check_id, "BLOCK")


def _fresh(observations: Mapping[str, dict[str, Any]], name: str, limit: float) -> bool:
    observation = observations.get(name)
    if observation is None:
        return False
    age = finite_number(_mapping(observation.get("timing")).get("age_sec"))
    return age is not None and age <= limit


def _tf_chain_available(observations: Mapping[str, dict[str, Any]]) -> bool:
    graph: dict[str, set[str]] = {}
    for name in ("tf", "tf_static"):
        summary = _mapping(_mapping(observations.get(name)).get("summary"))
        transforms = summary.get("transforms")
        if not isinstance(transforms, list):
            continue
        for value in transforms:
            item = _mapping(value)
            parent = str(item.get("parent_frame", "")).lstrip("/")
            child = str(item.get("child_frame", "")).lstrip("/")
            if not parent or not child:
                continue
            graph.setdefault(parent, set()).add(child)
            graph.setdefault(child, set()).add(parent)
    if "map" not in graph:
        return False
    targets = {"base_link", "base_footprint"}
    queue: deque[str] = deque(["map"])
    visited = {"map"}
    while queue:
        current = queue.popleft()
        if current in targets:
            return True
        for child in graph.get(current, set()):
            if child not in visited:
                visited.add(child)
                queue.append(child)
    return False


def build_readiness_evidence(
    records: Mapping[str, Mapping[str, Any]],
    *,
    failures: Mapping[str, Any] | None = None,
    policy: ReadinessPolicy | None = None,
    robot_id: str = "limo",
    body_id: str = "limo",
    body_snapshot_hash: str = "",
    ros_time_active: bool = False,
    clock_jump_detected: bool = False,
    now_wall: float | None = None,
    now_monotonic: float | None = None,
) -> ReadinessEvidenceV1:
    """Build a hash-sealed ``ReadinessEvidenceV1`` from bounded observations."""

    selected_policy = policy or load_readiness_policy()
    wall = time.time() if now_wall is None else float(now_wall)
    monotonic = time.monotonic() if now_monotonic is None else float(now_monotonic)
    observations: dict[str, dict[str, Any]] = {}

    for name in sorted(records):
        record = records[name]
        summary_value = record.get("summary")
        if not isinstance(summary_value, dict):
            continue
        summary = dict(summary_value)
        timing = observation_timing(
            summary,
            received_wall_time=finite_number(record.get("received_wall_time")),
            received_monotonic=finite_number(record.get("received_monotonic")),
            now_wall=wall,
            ros_time_active=ros_time_active,
        )
        topic, message_type = LIMO_OBSERVATION_TOPICS.get(name, ("", ""))
        observations[name] = {
            "topic": topic,
            "message_type": message_type,
            "ros_stamp_sec": timing["ros_stamp_sec"],
            "received_at": timing["received_at"],
            "age_sec": timing["age_sec"],
            "freshness_basis": timing["freshness_basis"],
            "clock_verified": timing["clock_verified"],
            "clock_issue": timing["clock_issue"],
            "transport": record.get("transport"),
            "transport_generation": record.get("transport_generation"),
            "summary_hash": sha256_document(summary),
            "summary": summary,
            "timing": timing,
        }

    critical = selected_policy.critical_observations
    receive_times = [
        received_time
        for name in critical
        if (
            received_time := finite_number(
                _mapping(observations.get(name)).get("timing", {}).get("received_wall_time")
            )
        )
        is not None
    ]
    first_received = min(receive_times) if receive_times else None
    last_received = max(receive_times) if receive_times else None
    skew = (
        last_received - first_received
        if first_received is not None and last_received is not None
        else None
    )

    checks: list[dict[str, Any]] = []

    def add(
        check_id: str,
        passed: bool,
        observed: Any,
        operator: str,
        threshold: Any,
        unit: str = "",
        refs: tuple[str, ...] = (),
    ) -> None:
        checks.append(
            {
                "check_id": check_id,
                "severity": _severity(selected_policy, check_id),
                "passed": bool(passed),
                "observed": observed,
                "operator": operator,
                "threshold": threshold,
                "unit": unit,
                "error_code": _CHECK_ERROR_CODES[check_id],
                "evidence_refs": [f"observation://{name}" for name in refs],
            }
        )

    clock_ok = not clock_jump_detected and all(
        name in observations and bool(observations[name].get("clock_verified")) for name in critical
    )
    add("CLOCK_VERIFIED", clock_ok, clock_ok, "==", True, refs=tuple(critical))
    add(
        "OBSERVATION_WINDOW_COHERENT",
        skew is not None and skew <= selected_policy.max_observation_skew_sec,
        skew,
        "<=",
        selected_policy.max_observation_skew_sec,
        "sec",
        tuple(critical),
    )

    status = _mapping(_mapping(observations.get("status")).get("summary"))
    add("STATUS_AVAILABLE", bool(status), bool(status), "==", True, refs=("status",))
    add(
        "STATUS_FRESH",
        _fresh(observations, "status", selected_policy.thresholds["status_fresh_sec"]),
        _mapping(observations.get("status")).get("age_sec"),
        "<=",
        selected_policy.thresholds["status_fresh_sec"],
        "sec",
        ("status",),
    )
    add(
        "ERROR_CODE_ZERO",
        status.get("error_code") == 0,
        status.get("error_code"),
        "==",
        0,
        refs=("status",),
    )
    add(
        "MOTION_MODE_KNOWN",
        status.get("motion_mode") in {0, 1, 2},
        status.get("motion_mode"),
        "in",
        [0, 1, 2],
        refs=("status",),
    )
    battery = finite_number(status.get("battery_voltage"))
    add(
        "BATTERY_ABOVE_RETURN_THRESHOLD",
        battery is not None and battery >= selected_policy.thresholds["battery_return_voltage"],
        battery,
        ">=",
        selected_policy.thresholds["battery_return_voltage"],
        "V",
        ("status",),
    )

    odom = _mapping(_mapping(observations.get("odometry")).get("summary"))
    add("ODOM_AVAILABLE", bool(odom), bool(odom), "==", True, refs=("odometry",))
    add(
        "ODOM_FRESH",
        _fresh(observations, "odometry", selected_policy.thresholds["odom_fresh_sec"]),
        _mapping(observations.get("odometry")).get("age_sec"),
        "<=",
        selected_policy.thresholds["odom_fresh_sec"],
        "sec",
        ("odometry",),
    )

    imu = _mapping(_mapping(observations.get("imu")).get("summary"))
    add("IMU_AVAILABLE", bool(imu), bool(imu), "==", True, refs=("imu",))
    add(
        "IMU_FRESH",
        _fresh(observations, "imu", selected_policy.thresholds["imu_fresh_sec"]),
        _mapping(observations.get("imu")).get("age_sec"),
        "<=",
        selected_policy.thresholds["imu_fresh_sec"],
        "sec",
        ("imu",),
    )

    scan = _mapping(_mapping(observations.get("laser_scan")).get("summary"))
    add("LASER_AVAILABLE", bool(scan), bool(scan), "==", True, refs=("laser_scan",))
    add(
        "LASER_FRESH",
        _fresh(observations, "laser_scan", selected_policy.thresholds["laser_fresh_sec"]),
        _mapping(observations.get("laser_scan")).get("age_sec"),
        "<=",
        selected_policy.thresholds["laser_fresh_sec"],
        "sec",
        ("laser_scan",),
    )
    valid_count = finite_number(scan.get("usable_count", scan.get("valid_count")))
    sample_count = finite_number(scan.get("sample_count"))
    valid_ratio = (
        valid_count / sample_count
        if valid_count is not None and sample_count is not None and sample_count > 0.0
        else None
    )
    add(
        "LASER_VALID_RATIO",
        valid_ratio is not None and valid_ratio >= selected_policy.thresholds["laser_valid_ratio"],
        valid_ratio,
        ">=",
        selected_policy.thresholds["laser_valid_ratio"],
        "ratio",
        ("laser_scan",),
    )
    front_clearance = finite_number(_mapping(scan.get("sectors")).get("front_min_m"))
    add(
        "FRONT_CLEARANCE",
        front_clearance is not None
        and front_clearance >= selected_policy.thresholds["front_clearance_m"],
        front_clearance,
        ">=",
        selected_policy.thresholds["front_clearance_m"],
        "m",
        ("laser_scan",),
    )

    amcl = _mapping(_mapping(observations.get("localized_pose")).get("summary"))
    add("AMCL_AVAILABLE", bool(amcl), bool(amcl), "==", True, refs=("localized_pose",))
    add(
        "AMCL_FRESH",
        _fresh(observations, "localized_pose", selected_policy.thresholds["amcl_fresh_sec"]),
        _mapping(observations.get("localized_pose")).get("age_sec"),
        "<=",
        selected_policy.thresholds["amcl_fresh_sec"],
        "sec",
        ("localized_pose",),
    )
    variances = {
        "x": finite_number(amcl.get("position_variance_x")),
        "y": finite_number(amcl.get("position_variance_y")),
        "yaw": finite_number(amcl.get("yaw_variance")),
    }
    covariance_ok = all(value is not None for value in variances.values()) and bool(
        variances["x"] <= selected_policy.thresholds["amcl_variance_x"]  # type: ignore[operator]
        and variances["y"] <= selected_policy.thresholds["amcl_variance_y"]  # type: ignore[operator]
        and variances["yaw"] <= selected_policy.thresholds["amcl_variance_yaw"]  # type: ignore[operator]
    )
    add(
        "AMCL_COVARIANCE",
        covariance_ok,
        variances,
        "<=",
        {
            "x": selected_policy.thresholds["amcl_variance_x"],
            "y": selected_policy.thresholds["amcl_variance_y"],
            "yaw": selected_policy.thresholds["amcl_variance_yaw"],
        },
        "variance",
        ("localized_pose",),
    )

    navigation = _mapping(_mapping(observations.get("navigation_status")).get("summary"))
    navigation_ok = bool(navigation) and _fresh(
        observations, "navigation_status", selected_policy.thresholds["navigation_fresh_sec"]
    )
    add(
        "MOVE_BASE_AVAILABLE", navigation_ok, navigation_ok, "==", True, refs=("navigation_status",)
    )

    map_summary = _mapping(_mapping(observations.get("map_metadata")).get("summary"))
    resolution = finite_number(map_summary.get("resolution_m"))
    width = map_summary.get("width_cells")
    height = map_summary.get("height_cells")
    map_valid = (
        resolution is not None
        and resolution > 0.0
        and isinstance(width, int)
        and not isinstance(width, bool)
        and width > 0
        and isinstance(height, int)
        and not isinstance(height, bool)
        and height > 0
    )
    add(
        "MAP_METADATA_VALID",
        map_valid,
        {"resolution_m": resolution, "width_cells": width, "height_cells": height},
        "valid",
        True,
        refs=("map_metadata",),
    )

    for name, check_id in (
        ("global_costmap", "GLOBAL_COSTMAP_FRESH"),
        ("local_costmap", "LOCAL_COSTMAP_FRESH"),
    ):
        add(
            check_id,
            _fresh(observations, name, selected_policy.thresholds["costmap_fresh_sec"]),
            _mapping(observations.get(name)).get("age_sec"),
            "<=",
            selected_policy.thresholds["costmap_fresh_sec"],
            "sec",
            (name,),
        )

    diagnostics = _mapping(_mapping(observations.get("diagnostics")).get("summary"))
    counts = _mapping(diagnostics.get("counts"))
    diagnostics_fresh = _fresh(
        observations, "diagnostics", selected_policy.thresholds["diagnostics_fresh_sec"]
    )
    diagnostic_errors = int(counts.get("ERROR", 0) or 0) + int(counts.get("STALE", 0) or 0)
    add(
        "DIAGNOSTICS_ERROR_ZERO",
        bool(diagnostics) and diagnostics_fresh and diagnostic_errors == 0,
        diagnostic_errors,
        "==",
        0,
        "count",
        ("diagnostics",),
    )
    diagnostic_warnings = int(counts.get("WARN", 0) or 0)
    add(
        "DIAGNOSTICS_WARN_POLICY",
        diagnostic_warnings == 0,
        diagnostic_warnings,
        "==",
        0,
        "count",
        ("diagnostics",),
    )
    add(
        "TF_MAP_ODOM_BASE",
        _tf_chain_available(observations),
        _tf_chain_available(observations),
        "==",
        True,
        refs=("tf", "tf_static"),
    )
    add(
        "BODY_SNAPSHOT_BOUND",
        body_snapshot_hash.startswith("sha256:"),
        bool(body_snapshot_hash),
        "==",
        True,
    )

    checks.sort(key=lambda item: str(item["check_id"]))
    blockers = sorted(
        {
            str(check["error_code"])
            for check in checks
            if not check["passed"] and check["severity"] == "BLOCK"
        }
    )
    warnings = sorted(
        {
            str(check["error_code"])
            for check in checks
            if not check["passed"] and check["severity"] == "WARN"
        }
    )
    state = "BLOCKED" if blockers else ("DEGRADED" if warnings else "READY")
    failure_names = sorted(str(name) for name in (failures or {}))
    snapshot = {
        "schema_version": "limo.readiness.v1",
        "snapshot_id": f"limo-ready-{uuid.uuid4()}",
        "robot_id": robot_id,
        "body_id": body_id,
        "body_snapshot_hash": body_snapshot_hash or None,
        "created_at": utc_iso(wall),
        "expires_at": utc_iso(wall + selected_policy.snapshot_ttl_sec),
        "created_wall_time": wall,
        "expires_wall_time": wall + selected_policy.snapshot_ttl_sec,
        "created_monotonic": monotonic,
        "clock": {
            "wall_time_valid": math.isfinite(wall),
            "ros_time_active": ros_time_active,
            "clock_domain": "ROS" if ros_time_active else "WALL",
            "clock_verified": clock_ok,
            "clock_jump_detected": clock_jump_detected,
        },
        "observation_window": {
            "first_received_at": utc_iso(first_received) if first_received is not None else None,
            "last_received_at": utc_iso(last_received) if last_received is not None else None,
            "skew_ms": skew * 1000.0 if skew is not None else None,
        },
        "observations": observations,
        "observation_failures": failure_names,
        "checks": checks,
        "state": state,
        "ready": state == "READY",
        "blockers": blockers,
        "warnings": warnings,
        "policy_id": selected_policy.policy_id,
        "policy_hash": selected_policy.policy_hash,
        "command_dispatched": False,
        "usable_for_real_execution": False,
    }
    return cast(ReadinessEvidenceV1, seal_snapshot(snapshot))


def evaluate_summaries(
    summaries: Mapping[str, dict[str, Any]],
    *,
    policy: ReadinessPolicy | None = None,
    now_wall: float | None = None,
    now_monotonic: float | None = None,
) -> ReadinessEvidenceV1:
    """Compatibility entry point for unit callers that only have summaries."""

    wall = time.time() if now_wall is None else float(now_wall)
    monotonic = time.monotonic() if now_monotonic is None else float(now_monotonic)
    records = {
        name: {
            "summary": summary,
            "received_wall_time": wall,
            "received_monotonic": monotonic,
        }
        for name, summary in summaries.items()
    }
    return build_readiness_evidence(
        records,
        policy=policy,
        now_wall=wall,
        now_monotonic=monotonic,
    )


def validate_readiness_snapshot(
    snapshot: Mapping[str, Any], *, now_wall: float | None = None
) -> dict[str, Any]:
    """Check schema, integrity, and expiry without treating client evidence as authorization."""

    if snapshot.get("schema_version") != "limo.readiness.v1" or not verify_snapshot(snapshot):
        return {
            "ok": False,
            "error_code": LIMO_SNAPSHOT_HASH_MISMATCH,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }
    expires = finite_number(snapshot.get("expires_wall_time"))
    wall = time.time() if now_wall is None else float(now_wall)
    if expires is None or wall > expires:
        return {
            "ok": False,
            "error_code": LIMO_SNAPSHOT_EXPIRED,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }
    return {
        "ok": True,
        "snapshot_hash": snapshot["snapshot_hash"],
        "state": snapshot.get("state"),
        "expires_at": snapshot.get("expires_at"),
        "advisory_only": True,
        "command_dispatched": False,
        "usable_for_real_execution": False,
    }
