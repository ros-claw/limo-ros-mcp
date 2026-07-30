"""Protocol-surface and fail-closed tests for rosclaw-limo-mcp."""

from __future__ import annotations

from typing import Any

import pytest

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


@pytest.mark.asyncio
async def test_request_navigation_blocks_before_daemon_without_snapshot() -> None:
    gateway = FakeGateway()
    service = LimoMCPService(gateway=gateway)

    result = await service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        localization_ready=True,
        costmap_ready=True,
        obstacle_check_enabled=True,
        body_snapshot_hash="",
    )

    assert result["error_code"] == "BODY_SNAPSHOT_REQUIRED"
    assert result["command_dispatched"] is False
    assert gateway.calls == []


@pytest.mark.asyncio
async def test_request_navigation_submits_shadow_to_rosclawd_boundary() -> None:
    gateway = FakeGateway()
    service = LimoMCPService(gateway=gateway)

    result = await service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.25,
        frame_id="map",
        localization_ready=True,
        costmap_ready=True,
        obstacle_check_enabled=True,
        body_snapshot_hash="sha256:test-body-snapshot",
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
    assert result["deprecation_warnings"]


@pytest.mark.asyncio
async def test_legacy_readiness_booleans_can_never_authorize_real() -> None:
    gateway = FakeGateway()
    service = LimoMCPService(gateway=gateway)

    result = await service.request_navigation(
        x=1.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        localization_ready=True,
        costmap_ready=True,
        obstacle_check_enabled=True,
        body_snapshot_hash="sha256:test-body-snapshot",
        execution_mode="REAL",
    )

    assert result["error_code"] == "LIMO_LEGACY_READINESS_BOOLEAN_FORBIDDEN"
    assert result["command_dispatched"] is False
    assert gateway.calls == []
