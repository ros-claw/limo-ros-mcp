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
