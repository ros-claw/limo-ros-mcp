"""Subprocess MCP protocol smoke test for the LIMO server."""

from __future__ import annotations

import asyncio
import json
import os
import sys

import pytest
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


@pytest.mark.asyncio
async def test_limo_mcp_stdio_initialize_list_and_call() -> None:
    env = os.environ.copy()
    env["ROSCLAW_PROJECT_ROOT"] = os.getcwd()
    params = StdioServerParameters(
        command=sys.executable,
        args=["-m", "limo_ros_mcp.server"],
        env=env,
    )

    async with asyncio.timeout(15.0):
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                listed = await session.list_tools()
                names = {tool.name for tool in listed.tools}
                assert len(names) == 30
                assert "limo_get_contract" in names
                assert "limo_list_peripherals" in names
                assert "limo_get_audio_state" in names
                assert "limo_measure_microphone" in names
                assert "limo_get_display_state" in names
                assert "limo_get_platform_health" in names
                assert "limo_get_camera_state" in names
                assert "limo_get_dabai_device_state" in names
                assert "limo_sample_topic" in names
                assert "limo_get_patrol_readiness" in names
                assert "limo_request_navigation" in names
                assert "limo_request_initial_pose" in names
                assert "limo_request_tone" in names
                assert not any("publish" in name or "cmd_vel" in name for name in names)

                called = await session.call_tool("limo_get_contract", {})
                assert called.isError is False
                payload = json.loads(called.content[0].text)
                assert payload["contract"]["robot_id"] == "limo"
                assert payload["command_dispatched"] is False

                observations = await session.call_tool(
                    "limo_list_observations", {"category": "navigation"}
                )
                assert observations.isError is False
                observation_payload = json.loads(observations.content[0].text)
                assert observation_payload["count"] == 6
