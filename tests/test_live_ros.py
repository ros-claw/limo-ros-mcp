"""Opt-in, read-only integration checks against the actual LIMO ROS 1 graph."""

from __future__ import annotations

import asyncio
import json
import os
import sys

import pytest
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

from limo_ros_mcp.server import LimoMCPService

pytestmark = [
    pytest.mark.live_ros,
    pytest.mark.skipif(
        os.environ.get("LIMO_LIVE_ROS") != "1",
        reason="set LIMO_LIVE_ROS=1 with an operator-started read-only LIMO ROS graph",
    ),
]


def test_live_limo_ros_graph_has_inspection_stack() -> None:
    result = LimoMCPService(gateway=object()).probe_ros(transport="roscli")

    assert result["ok"] is True
    assert result["missing_required_topics"] == []
    assert {"status", "odometry", "imu", "laser_scan"}.issubset(result["available_observations"])
    assert {"localized_pose", "navigation_status", "map_metadata"}.issubset(
        result["available_observations"]
    )
    assert result["command_dispatched"] is False


def test_live_status_message_decodes_driver_fields() -> None:
    result = LimoMCPService(gateway=object()).observe(
        "status",
        timeout_sec=10.0,
        transport="roscli",
        include_raw=True,
    )

    assert result["ok"] is True
    assert result["message_type"] == "limo_base/LimoStatus"
    assert result["summary"]["motion_mode"] in {0, 1, 2, 255}
    assert isinstance(result["summary"]["error_flags"], list)
    assert result["command_dispatched"] is False


def test_live_navigation_status_samples_at_message_layer() -> None:
    result = LimoMCPService(gateway=object()).sample(
        "navigation_status",
        count=2,
        timeout_sec=10.0,
        transport="roscli",
    )

    assert result["ok"] is True
    assert result["sample_count"] == 2
    assert all("latest" in summary for summary in result["summaries"])
    assert result["estimated_rate_hz"] is not None
    assert result["rate_source"] == "ros_header"
    assert result["command_dispatched"] is False


def test_live_laser_topic_has_expected_graph_endpoints() -> None:
    result = LimoMCPService(gateway=object()).topic_info(
        "laser_scan",
        timeout_sec=10.0,
        transport="roscli",
    )

    assert result["ok"] is True
    assert result["observed_type"] == "sensor_msgs/LaserScan"
    assert result["publishers"]
    assert result["subscribers"]
    assert result["command_dispatched"] is False


@pytest.mark.parametrize(
    "observation",
    [
        "color_image",
        "color_camera_info",
        "depth_image",
        "depth_camera_info",
        "depth_points",
    ],
)
def test_live_dabai_camera_streams_are_observable(observation: str) -> None:
    service = LimoMCPService(gateway=object())
    info = service.topic_info(observation, timeout_sec=10.0, transport="roscli")
    sample = service.observe(
        observation,
        timeout_sec=10.0,
        transport="roscli",
        include_raw=False,
    )

    assert info["ok"] is True
    assert info["publishers"]
    assert sample["ok"] is True
    assert sample["summary"]["height"] > 0
    assert sample["summary"]["width"] > 0
    assert sample["command_dispatched"] is False


@pytest.mark.parametrize("observation", ["infrared_image", "infrared_camera_info"])
def test_live_dabai_ir_endpoints_report_active_or_inactive_truthfully(observation: str) -> None:
    service = LimoMCPService(gateway=object())
    info = service.topic_info(observation, timeout_sec=10.0, transport="roscli")
    sample = service.observe(
        observation,
        timeout_sec=10.0,
        transport="roscli",
        include_raw=False,
    )

    assert info["ok"] is True
    assert info["publishers"]
    if sample["ok"]:
        assert sample["summary"]["height"] > 0
        assert sample["summary"]["width"] > 0
    else:
        assert sample["error_code"] == "LIMO_ROS_UNAVAILABLE"
        assert sample["errors"]
    assert sample["command_dispatched"] is False


def test_live_dabai_getter_services_report_device_and_ir_health() -> None:
    result = LimoMCPService(gateway=object()).dabai_device_state()

    assert result["ok"] is True
    assert result["device"]["serial"]
    assert "DaBai" in result["device"]["device_type"]
    assert result["device"]["version"]["firmware_version"]
    assert result["device"]["ir_temperature_c"] > 0
    assert result["command_dispatched"] is False


def test_live_dabai_aggregate_keeps_inactive_ir_optional() -> None:
    result = LimoMCPService(gateway=object()).camera_state(
        timeout_sec=10.0,
        transport="roscli",
    )

    assert result["ok"] is True
    assert result["core_ready"] is True
    assert result["core_failures"] == {}
    assert set(result["optional_inactive"]).issubset({"infrared_image", "infrared_camera_info"})
    assert result["command_dispatched"] is False


def test_live_limo_host_peripheral_inventory() -> None:
    result = LimoMCPService(gateway=object()).peripheral_inventory()

    assert result["ok"] is True
    by_id = {item["id"]: item for item in result["peripherals"]}
    assert by_id["voice_audio"]["status"] == "connected"
    assert by_id["rear_touchscreen"]["status"] == "connected"
    assert by_id["rear_display"]["status"] == "connected"
    assert by_id["front_oled"]["status"] == "declared_unbound"
    assert result["command_dispatched"] is False


def test_live_limo_audio_state_is_readable() -> None:
    result = LimoMCPService(gateway=object()).audio_state()

    assert result["ok"] is True
    assert result["playback_ready"] is True
    assert result["capture_ready"] is True
    assert result["speaker"]["available"] is True
    assert result["microphone"]["available"] is True
    assert result["command_dispatched"] is False


def test_live_limo_microphone_level_is_bounded_and_not_retained() -> None:
    result = LimoMCPService(gateway=object()).microphone_level(
        duration_sec=1,
        sample_rate_hz=16000,
    )

    assert result["ok"] is True
    assert result["sample_count"] >= 16000
    assert result["audio_retained"] is False
    assert result["audio_content_returned"] is False
    assert result["command_dispatched"] is False


def test_live_limo_display_and_platform_health() -> None:
    service = LimoMCPService(gateway=object())
    display = service.display_state()
    health = service.platform_health()

    assert display["ok"] is True
    assert display["rear_display_ready"] is True
    assert display["front_oled"]["interface_bound"] is False
    assert health["ok"] is True
    assert health["maximum_temperature_c"] is not None
    assert health["memory"]["available_bytes"] is not None
    assert display["command_dispatched"] is False
    assert health["command_dispatched"] is False


@pytest.mark.asyncio
async def test_live_mcp_patrol_readiness_protocol() -> None:
    params = StdioServerParameters(
        command=sys.executable,
        args=["-m", "limo_ros_mcp.server"],
        env=os.environ.copy(),
    )

    async with asyncio.timeout(60.0):
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                called = await session.call_tool(
                    "limo_get_patrol_readiness",
                    {"transport": "roscli", "timeout_sec": 10.0},
                )

    assert called.isError is False
    payload = json.loads(called.content[0].text)
    assert payload["available_count"] >= 7
    assert payload["readiness"]["state"] in {"READY", "DEGRADED", "BLOCKED"}
    assert payload["command_dispatched"] is False


@pytest.mark.asyncio
async def test_live_mcp_peripheral_protocol() -> None:
    params = StdioServerParameters(
        command=sys.executable,
        args=["-m", "limo_ros_mcp.server"],
        env=os.environ.copy(),
    )

    async with asyncio.timeout(120.0):
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                listed = await session.list_tools()
                names = {tool.name for tool in listed.tools}
                assert len(names) == 29
                calls = (
                    ("limo_list_peripherals", {}),
                    ("limo_get_audio_state", {}),
                    ("limo_measure_microphone", {"duration_sec": 1, "sample_rate_hz": 16000}),
                    ("limo_get_display_state", {}),
                    ("limo_get_platform_health", {}),
                    ("limo_get_dabai_device_state", {}),
                )
                payloads: dict[str, dict[str, object]] = {}
                for name, arguments in calls:
                    called = await session.call_tool(name, arguments)
                    assert called.isError is False
                    payloads[name] = json.loads(called.content[0].text)

    assert all(payload["ok"] is True for payload in payloads.values())
    assert payloads["limo_measure_microphone"]["audio_retained"] is False
    assert payloads["limo_measure_microphone"]["audio_content_returned"] is False
    assert all(payload["command_dispatched"] is False for payload in payloads.values())
