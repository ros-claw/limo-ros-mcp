"""Protocol tests for bounded tone and navigation daemon workers."""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

ROOT = Path(__file__).parents[1]


def _load(name: str) -> ModuleType:
    path = ROOT / "worker" / name
    spec = importlib.util.spec_from_file_location(name.removesuffix(".py"), path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TONE = _load("limo_tone_worker.py")
NAVIGATION = _load("limo_navigation_worker.py")


def _tone_request() -> dict[str, object]:
    return {
        "protocol": "rosclaw.limo.worker.v1",
        "operation": "PLAY_TONE",
        "schema_version": "limo.tone.v1",
        "action_id": "action-tone-test",
        "body_id": "limo",
        "body_snapshot_hash": "sha256:test-body",
        "frequency_hz": 660,
        "duration_sec": 0.6,
        "volume_percent": 18,
    }


def _navigation_request() -> dict[str, object]:
    return {
        "protocol": "rosclaw.limo.worker.v1",
        "operation": "NAVIGATE_TO_POSE",
        "schema_version": "limo.navigation.v2",
        "action_id": "action-navigation-test",
        "body_id": "limo",
        "body_snapshot_hash": "sha256:test-body",
        "target_pose": {"frame_id": "map", "x": 0.4, "y": 0.0, "yaw": 0.0},
        "goal_tolerance": {"xy_m": 0.15, "yaw_rad": 0.2},
        "server_timeout_sec": 3.0,
        "navigation_timeout_sec": 30.0,
        "verification_timeout_sec": 5.0,
    }


@pytest.mark.parametrize(
    ("module", "payload"),
    [(TONE, _tone_request()), (NAVIGATION, _navigation_request())],
)
def test_worker_contract_accepts_exact_bounded_request(
    module: ModuleType, payload: dict[str, object]
) -> None:
    normalized = module.validate_request(payload)

    assert normalized["action_id"] == payload["action_id"]
    assert normalized["body_snapshot_hash"] == payload["body_snapshot_hash"]


@pytest.mark.parametrize(
    ("module", "payload"),
    [(TONE, _tone_request()), (NAVIGATION, _navigation_request())],
)
def test_worker_contract_rejects_unknown_surface(
    module: ModuleType, payload: dict[str, object]
) -> None:
    payload["command"] = "arbitrary"

    with pytest.raises(module.RequestError, match="unknown request fields"):
        module.validate_request(payload)


@pytest.mark.parametrize(
    "override",
    [
        {"frequency_hz": 1000},
        {"duration_sec": 5.0},
        {"volume_percent": 80},
    ],
)
def test_tone_worker_rejects_unbounded_parameters(override: dict[str, object]) -> None:
    request = {**_tone_request(), **override}

    with pytest.raises(TONE.RequestError):
        TONE.validate_request(request)


def test_navigation_worker_rejects_non_map_and_unbounded_timeout() -> None:
    wrong_frame = _navigation_request()
    wrong_frame["target_pose"] = {"frame_id": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0}
    long_timeout = {**_navigation_request(), "navigation_timeout_sec": 121.0}

    with pytest.raises(NAVIGATION.RequestError, match="frame_id must be map"):
        NAVIGATION.validate_request(wrong_frame)
    with pytest.raises(NAVIGATION.RequestError, match="navigation_timeout_sec"):
        NAVIGATION.validate_request(long_timeout)


@pytest.mark.parametrize(
    ("worker", "payload"),
    [
        ("limo_tone_worker.py", _tone_request()),
        ("limo_navigation_worker.py", _navigation_request()),
    ],
)
def test_worker_validate_only_subprocess(worker: str, payload: dict[str, object]) -> None:
    completed = subprocess.run(
        [sys.executable, str(ROOT / "worker" / worker)],
        input=json.dumps(payload),
        text=True,
        capture_output=True,
        check=False,
        env={**os.environ, "ROSCLAW_LIMO_VALIDATE_ONLY": "1"},
    )

    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout)
    assert result["ok"] is True
