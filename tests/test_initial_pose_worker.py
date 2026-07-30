"""Protocol tests for the daemon-owned, fixed-operation ROS 1 worker."""

from __future__ import annotations

import importlib.util
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

WORKER = Path(__file__).parents[1] / "worker" / "limo_initial_pose_worker.py"
SPEC = importlib.util.spec_from_file_location("limo_initial_pose_worker", WORKER)
assert SPEC is not None and SPEC.loader is not None
WORKER_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WORKER_MODULE)


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


def _pose_message(x: float, y: float, yaw: float) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            )
        ),
    )


def test_worker_waits_past_first_amcl_sample_until_converged() -> None:
    class FakeRospy:
        def __init__(self) -> None:
            self.messages = iter(
                [
                    _pose_message(5.8, 0.45, 0.28),
                    _pose_message(0.73, -1.24, 0.36),
                ]
            )
            self.calls = 0

        @staticmethod
        def is_shutdown() -> bool:
            return False

        def wait_for_message(self, _topic: str, _message_type: object, *, timeout: float):
            assert timeout > 0.0
            self.calls += 1
            return next(self.messages)

    rospy = FakeRospy()
    _message, observed = WORKER_MODULE._wait_for_converged_pose(
        rospy,
        object,
        {"frame_id": "map", "x": 0.75, "y": -1.25, "yaw": 0.35},
        2.0,
    )

    assert rospy.calls == 2
    assert observed["x"] == 0.73
    assert WORKER_MODULE._pose_converged(observed, {"x": 0.75, "y": -1.25, "yaw": 0.35})
