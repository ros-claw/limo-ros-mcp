"""ReadinessEvidenceV1 policy and fail-closed tests."""

from __future__ import annotations

from typing import Any

from limo_ros_mcp.evidence import verify_snapshot
from limo_ros_mcp.readiness import build_readiness_evidence, validate_readiness_snapshot


def _records(*, now: float = 100.0) -> dict[str, dict[str, Any]]:
    summaries: dict[str, dict[str, Any]] = {
        "status": {"error_code": 0, "motion_mode": 0, "battery_voltage": 12.0},
        "odometry": {"velocity": {"linear_x_mps": 0.0}},
        "imu": {"orientation": {"w": 1.0}},
        "laser_scan": {
            "sample_count": 10,
            "valid_count": 10,
            "sectors": {"front_min_m": 1.0},
        },
        "localized_pose": {
            "position_variance_x": 0.1,
            "position_variance_y": 0.1,
            "yaw_variance": 0.1,
        },
        "navigation_status": {"goal_count": 0},
        "map_metadata": {"resolution_m": 0.05, "width_cells": 100, "height_cells": 100},
        "global_costmap": {"resolution_m": 0.05, "width_cells": 20, "height_cells": 20},
        "local_costmap": {"resolution_m": 0.05, "width_cells": 20, "height_cells": 20},
        "diagnostics": {"counts": {"OK": 2, "WARN": 0, "ERROR": 0, "STALE": 0}},
        "tf": {
            "transforms": [
                {"parent_frame": "map", "child_frame": "odom"},
                {"parent_frame": "odom", "child_frame": "base_link"},
            ]
        },
        "tf_static": {"transforms": []},
    }
    return {
        name: {
            "summary": summary,
            "received_wall_time": now,
            "received_monotonic": 50.0,
        }
        for name, summary in summaries.items()
    }


def test_complete_evidence_is_ready_hash_sealed_and_stably_sorted() -> None:
    evidence = build_readiness_evidence(
        _records(),
        body_snapshot_hash="sha256:body",
        now_wall=100.0,
        now_monotonic=50.0,
    )

    assert evidence["schema_version"] == "limo.readiness.v1"
    assert evidence["state"] == "READY"
    assert evidence["ready"] is True
    assert verify_snapshot(evidence) is True
    assert [item["check_id"] for item in evidence["checks"]] == sorted(
        item["check_id"] for item in evidence["checks"]
    )


def test_untagged_rosclaw_body_digest_is_treated_as_bound() -> None:
    body_hash = "a" * 64
    evidence = build_readiness_evidence(
        _records(),
        body_snapshot_hash=body_hash,
        now_wall=100.0,
        now_monotonic=50.0,
    )

    check = next(item for item in evidence["checks"] if item["check_id"] == "BODY_SNAPSHOT_BOUND")
    assert check["passed"] is True
    assert evidence["body_snapshot_hash"] == body_hash


def test_stale_amcl_warns_but_missing_receive_time_fails_closed() -> None:
    records = _records()
    records["localized_pose"]["summary"]["header"] = {"age_sec": 30.0}
    records["laser_scan"].pop("received_monotonic")

    evidence = build_readiness_evidence(
        records,
        body_snapshot_hash="sha256:body",
        now_wall=100.0,
        now_monotonic=50.0,
    )

    assert evidence["state"] == "BLOCKED"
    assert "LIMO_AMCL_STALE" in evidence["warnings"]
    assert "LIMO_AMCL_STALE" not in evidence["blockers"]
    assert "LIMO_CLOCK_UNVERIFIED" in evidence["blockers"]


def test_snapshot_validation_rejects_expiry_and_tampering() -> None:
    evidence = build_readiness_evidence(
        _records(),
        body_snapshot_hash="sha256:body",
        now_wall=100.0,
        now_monotonic=50.0,
    )

    assert validate_readiness_snapshot(evidence, now_wall=104.0)["ok"] is True
    assert validate_readiness_snapshot(evidence, now_wall=106.0)["error_code"] == (
        "LIMO_SNAPSHOT_EXPIRED"
    )
    evidence["state"] = "BLOCKED"
    assert validate_readiness_snapshot(evidence, now_wall=104.0)["error_code"] == (
        "LIMO_SNAPSHOT_HASH_MISMATCH"
    )


def test_clock_jump_forces_snapshot_to_blocked() -> None:
    evidence = build_readiness_evidence(
        _records(),
        body_snapshot_hash="sha256:body",
        now_wall=100.0,
        now_monotonic=50.0,
        clock_jump_detected=True,
    )

    assert evidence["state"] == "BLOCKED"
    assert evidence["clock"]["clock_jump_detected"] is True
    assert "LIMO_CLOCK_UNVERIFIED" in evidence["blockers"]
