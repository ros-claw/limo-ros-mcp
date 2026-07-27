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
        "limo_get_contract",
        "limo_probe_ros",
        "limo_observe",
        "limo_validate_navigation_goal",
        "limo_get_runtime_status",
        "limo_request_navigation",
        "limo_get_action_status",
        "limo_get_execution_receipt",
        "limo_emergency_stop",
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
