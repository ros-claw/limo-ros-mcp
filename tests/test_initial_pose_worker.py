"""Protocol tests for the daemon-owned, fixed-operation ROS 1 worker."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

WORKER = Path(__file__).parents[1] / "worker" / "limo_initial_pose_worker.py"


def _request() -> dict[str, object]:
    return {
        "protocol": "rosclaw.limo.worker.v1",
        "operation": "SET_INITIAL_POSE",
        "schema_version": "limo.initial-pose.v1",
        "action_id": "action-initial-pose-test",
        "body_id": "limo",
        "body_snapshot_hash": "sha256:test-body",
        "target_pose": {"frame_id": "map", "x": 0.75, "y": -1.25, "yaw": 0.35},
        "covariance_diagonal": [0.25, 0.25, 0.0, 0.0, 0.0, 0.0685],
        "subscriber_timeout_sec": 3.0,
        "verification_timeout_sec": 8.0,
    }


def _validate(payload: dict[str, object]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(WORKER)],
        input=json.dumps(payload),
        text=True,
        capture_output=True,
        env={**os.environ, "ROSCLAW_LIMO_VALIDATE_ONLY": "1"},
        timeout=2.0,
        check=False,
    )


def test_worker_accepts_only_bounded_initial_pose_contract() -> None:
    completed = _validate(_request())
    result = json.loads(completed.stdout)
    assert completed.returncode == 0
    assert result["ok"] is True
    assert result["validated_request"]["operation"] == "SET_INITIAL_POSE"


def test_worker_rejects_generic_ros_and_unknown_fields() -> None:
    payload = _request()
    payload["topic"] = "/cmd_vel"
    completed = _validate(payload)
    result = json.loads(completed.stdout)
    assert completed.returncode == 1
    assert result["ok"] is False
    assert "unknown request fields" in result["error"]


def test_worker_rejects_non_map_pose_and_non_finite_values() -> None:
    payload = _request()
    payload["target_pose"] = {"frame_id": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0}
    assert _validate(payload).returncode == 1
    payload = _request()
    payload["target_pose"] = {"frame_id": "map", "x": float("nan"), "y": 0.0, "yaw": 0.0}
    assert _validate(payload).returncode == 1
