"""Protocol-surface and fail-closed tests for rosclaw-limo-mcp."""

from __future__ import annotations

import time
from typing import Any

import pytest

from limo_ros_mcp.evidence import seal_snapshot
from limo_ros_mcp.rosbridge import validate_rosbridge_endpoint
from limo_ros_mcp.server import (
    LimoMCPService,
    build_mcp_server,
)


@pytest.mark.asyncio
async def test_server_exposes_no_raw_ros_publish_tool() -> None:
    server = build_mcp_server(LimoMCPService(gateway=object()))
    tools = await server.list_tools()
    names = {tool.name for tool in tools}

    assert names == {
        "limo_get_base_state",
        "limo_get_contract",
        "limo_get_action_status",
        "limo_get_diagnostics",
        "limo_get_execution_receipt",
        "limo_get_laser_summary",
        "limo_get_localization_state",
        "limo_get_map_summary",
        "limo_get_navigation_state",
        "limo_get_patrol_readiness",
        "limo_get_runtime_status",
        "limo_get_topic_info",
        "limo_get_transform_state",
        "limo_list_observations",
        "limo_observe",
        "limo_probe_ros",
        "limo_request_navigation",
        "limo_sample_topic",
        "limo_emergency_stop",
        "limo_validate_navigation_goal",
        "limo_validate_velocity_command",
    }
    assert not any("publish" in name or "cmd_vel" in name for name in names)
    schemas = {tool.name: tool.inputSchema for tool in tools}
    navigation_properties = schemas["limo_request_navigation"]["properties"]
    assert "readiness_snapshot_hash" in schemas["limo_request_navigation"]["required"]
    assert "body_snapshot_hash" in schemas["limo_request_navigation"]["required"]
    assert {
        "localization_ready",
        "costmap_ready",
        "obstacle_check_enabled",
    }.isdisjoint(navigation_properties)


def test_rosbridge_endpoint_defaults_to_loopback_only(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ROSCLAW_LIMO_ROSBRIDGE_ALLOWLIST", raising=False)
    assert validate_rosbridge_endpoint("ws://127.0.0.1:9090") == "ws://127.0.0.1:9090"
    with pytest.raises(ValueError, match="not allowlisted"):
        validate_rosbridge_endpoint("ws://robot.example:9090")


def test_rosbridge_endpoint_honors_operator_allowlist(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ROSCLAW_LIMO_ROSBRIDGE_ALLOWLIST", "limo.local")
    assert validate_rosbridge_endpoint("wss://limo.local:9090") == "wss://limo.local:9090"


@pytest.mark.parametrize(
    "observation", ["color_image", "depth_points", "map", "global_costmap", "local_costmap"]
)
def test_large_binary_raw_observations_are_denied(observation: str) -> None:
    service = LimoMCPService(gateway=object())

    with pytest.raises(ValueError, match="include_raw is denied"):
        service.observe(observation, include_raw=True)
    with pytest.raises(ValueError, match="include_raw is denied"):
        service.sample(observation, include_raw=True)


def test_message_sampling_uses_ros_header_rate(monkeypatch: pytest.MonkeyPatch) -> None:
    class FakeTopicClient:
        def subscribe_many(
            self, _topic: str, _message_type: str, *, count: int
        ) -> list[dict[str, Any]]:
            assert count == 2
            return [
                {
                    "header": {"stamp": {"secs": 10, "nsecs": 0}},
                    "error_code": 0,
                    "motion_mode": 0,
                },
                {
                    "header": {"stamp": {"secs": 10, "nsecs": 200_000_000}},
                    "error_code": 0,
                    "motion_mode": 0,
                },
            ]

    monkeypatch.setattr(
        LimoMCPService,
        "_client",
        staticmethod(lambda *_args, **_kwargs: FakeTopicClient()),
    )

    result = LimoMCPService(gateway=object()).sample("status", count=2, transport="roscli")

    assert result["rate_source"] == "ros_header"
    assert result["estimated_rate_hz"] == pytest.approx(5.0)


def test_readiness_collection_reuses_preflighted_transport_and_aggregates_tf(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeSnapshotClient:
        transport_generation = "roscli-shared"

        def __init__(self) -> None:
            self.probe_count = 0

        def probe(self) -> dict[str, Any]:
            self.probe_count += 1
            return {
                "topics": ["/limo_status", "/odom", "/tf"],
                "types": ["limo_base/LimoStatus", "nav_msgs/Odometry", "tf2_msgs/TFMessage"],
                "nodes": [],
            }

        def subscribe_many(
            self, topic: str, _message_type: str, *, count: int
        ) -> list[dict[str, Any]]:
            if topic == "/limo_status":
                return [{"error_code": 0, "motion_mode": 0, "battery_voltage": 12.0}]
            if topic == "/odom":
                return [{"pose": {"pose": {}}, "twist": {"twist": {}}}]
            assert topic == "/tf"
            assert count == 5
            return [
                {
                    "transforms": [
                        {
                            "header": {"frame_id": "map" if index % 2 == 0 else "odom"},
                            "child_frame_id": "odom" if index % 2 == 0 else "base_link",
                        }
                    ]
                }
                for index in range(count)
            ]

    client = FakeSnapshotClient()
    monkeypatch.setattr(
        LimoMCPService,
        "_client",
        staticmethod(lambda *_args, **_kwargs: client),
    )

    result = LimoMCPService(gateway=object())._collect_summaries(
        ["status", "odometry", "tf"],
        endpoint="ws://127.0.0.1:9090",
        timeout_sec=2.0,
        transport="roscli",
    )

    assert result["ok"] is True
    assert result["available_count"] == 3
    assert result["transport_generation"] == "roscli-shared"
    assert client.probe_count == 1
    assert result["summaries"]["tf"]["transform_count"] == 2


class FakeGateway:
    def __init__(self) -> None:
        self.calls: list[dict[str, Any]] = []

    async def request_navigation(self, **kwargs: Any) -> dict[str, Any]:
        self.calls.append(kwargs)
        return {
            "ok": True,
            "state": "QUEUED",
            "action_id": kwargs.get("action_id") or "action-limo-test",
            "command_dispatched": False,
        }


class FakeGoalValidator:
    def validate(self, **kwargs: Any) -> dict[str, Any]:
        return {
            "ok": True,
            "decision": "ALLOW",
            "schema_version": "limo.navigation.v2",
            "normalized_goal": {
                "frame_id": kwargs["frame_id"],
                "x": float(kwargs["x"]),
                "y": float(kwargs["y"]),
                "yaw": float(kwargs["yaw"]),
            },
            "goal_tolerance": {"xy_m": 0.15, "yaw_rad": 0.2},
            "route_policy_id": "lab-default",
            "route_policy_hash": "sha256:test-route-policy",
            "map_id": "test-map",
            "map_image_hash": "sha256:test-map",
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }


def _service_with_readiness(
    gateway: FakeGateway,
    *,
    state: str = "READY",
    body_hash: str = "sha256:test-body-snapshot",
    expires_in: float = 60.0,
) -> tuple[LimoMCPService, str]:
    service = LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator())
    readiness = seal_snapshot(
        {
            "schema_version": "limo.readiness.v1",
            "state": state,
            "ready": state == "READY",
            "body_snapshot_hash": body_hash,
            "expires_wall_time": time.time() + expires_in,
            "expires_at": "test",
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }
    )
    service._remember_readiness(readiness)
    return service, str(readiness["snapshot_hash"])


@pytest.mark.asyncio
async def test_request_navigation_blocks_before_daemon_without_snapshot() -> None:
    gateway = FakeGateway()
    service = LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator())

    result = await service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        body_snapshot_hash="",
        readiness_snapshot_hash="sha256:unknown",
    )

    assert result["error_code"] == "BODY_SNAPSHOT_REQUIRED"
    assert result["command_dispatched"] is False
    assert gateway.calls == []


@pytest.mark.asyncio
async def test_request_navigation_submits_shadow_to_rosclawd_boundary() -> None:
    gateway = FakeGateway()
    service, readiness_hash = _service_with_readiness(gateway)

    result = await service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.25,
        frame_id="map",
        body_snapshot_hash="sha256:test-body-snapshot",
        readiness_snapshot_hash=readiness_hash,
        execution_mode="SHADOW",
        action_id="action-limo-shadow",
        wait_timeout_sec=0.0,
    )

    assert result["state"] == "QUEUED"
    assert len(gateway.calls) == 1
    call = gateway.calls[0]
    assert call["capability_id"] == "limo.navigate_to_pose"
    assert call["execution_mode"] == "SHADOW"
    assert call["body_id"] == "limo"
    assert call["arguments"]["target_pose"]["yaw"] == 0.25
    assert call["arguments"]["schema_version"] == "limo.navigation.v2"
    assert call["arguments"]["readiness_snapshot_hash"] == readiness_hash
    assert call["arguments"]["expected_effect"]["stop_required"] is True
    assert "preconditions" not in call["arguments"]
    assert result["navigation_contract"]["decision"] == "ALLOW"


@pytest.mark.asyncio
async def test_real_navigation_requires_daemon_owned_preflight() -> None:
    gateway = FakeGateway()
    service, readiness_hash = _service_with_readiness(gateway)

    result = await service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        body_snapshot_hash="sha256:test-body-snapshot",
        readiness_snapshot_hash=readiness_hash,
        execution_mode="REAL",
    )

    assert result["error_code"] == "LIMO_DAEMON_PREFLIGHT_REQUIRED"
    assert result["command_dispatched"] is False
    assert gateway.calls == []


@pytest.mark.asyncio
async def test_request_navigation_rejects_unknown_readiness_snapshot() -> None:
    gateway = FakeGateway()
    service = LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator())

    result = await service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        body_snapshot_hash="sha256:test-body-snapshot",
        readiness_snapshot_hash="sha256:not-generated-here",
    )

    assert result["error_code"] == "LIMO_READINESS_SNAPSHOT_UNKNOWN"
    assert gateway.calls == []


@pytest.mark.asyncio
async def test_request_navigation_rejects_body_binding_and_blocked_readiness() -> None:
    gateway = FakeGateway()
    service, readiness_hash = _service_with_readiness(gateway)
    mismatch = await service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        body_snapshot_hash="sha256:different-body",
        readiness_snapshot_hash=readiness_hash,
    )
    blocked_service, blocked_hash = _service_with_readiness(gateway, state="BLOCKED")
    blocked = await blocked_service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        body_snapshot_hash="sha256:test-body-snapshot",
        readiness_snapshot_hash=blocked_hash,
    )

    assert mismatch["error_code"] == "LIMO_BODY_SNAPSHOT_MISMATCH"
    assert blocked["error_code"] == "LIMO_READINESS_NOT_READY"
    assert gateway.calls == []


@pytest.mark.asyncio
async def test_request_navigation_rejects_expired_and_tampered_readiness() -> None:
    gateway = FakeGateway()
    expired_service, expired_hash = _service_with_readiness(gateway, expires_in=-1.0)
    expired = await expired_service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        body_snapshot_hash="sha256:test-body-snapshot",
        readiness_snapshot_hash=expired_hash,
    )

    tampered_service, tampered_hash = _service_with_readiness(gateway)
    tampered_service._readiness_snapshots[tampered_hash]["state"] = "BLOCKED"
    tampered = await tampered_service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        body_snapshot_hash="sha256:test-body-snapshot",
        readiness_snapshot_hash=tampered_hash,
    )

    assert expired["error_code"] == "LIMO_SNAPSHOT_EXPIRED"
    assert tampered["error_code"] == "LIMO_SNAPSHOT_HASH_MISMATCH"
    assert gateway.calls == []
