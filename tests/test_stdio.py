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
                assert len(names) == 35
                assert "limo_get_context" in names
                assert "limo_get_readiness" in names
                assert "limo_observe" in names
                assert "limo_get_runtime_status" in names
                assert "limo_get_camera_state" in names
                assert "limo_capture_camera_frame" in names
                assert "limo_get_robot_pose" in names
                assert "limo_get_audio_state" in names
                assert "limo_measure_microphone" in names
                assert "limo_request_navigation" in names
                assert "limo_request_initial_pose" in names
                assert "limo_request_tone" in names
                assert "limo_request_speech" in names
                assert not any("publish" in name or "cmd_vel" in name for name in names)

                called = await session.call_tool(
                    "limo_validate_navigation_goal",
                    {"x": 0.0, "y": 0.0, "yaw": 0.0},
                )
                assert called.isError is False
                payload = json.loads(called.content[0].text)
                assert payload["capability_id"] == "limo.navigate_to_pose"
                assert payload["command_dispatched"] is False


@pytest.mark.asyncio
async def test_limo_mcp_stdio_explicit_core_profile_remains_bounded() -> None:
    env = os.environ.copy()
    env["ROSCLAW_PROJECT_ROOT"] = os.getcwd()
    params = StdioServerParameters(
        command=sys.executable,
        args=["-m", "limo_ros_mcp.server", "--profile", "core"],
        env=env,
    )

    async with asyncio.timeout(15.0):
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                listed = await session.list_tools()
                names = {tool.name for tool in listed.tools}
                assert len(names) == 11
                assert "limo_get_context" in names
                assert "limo_get_readiness" in names
                assert "limo_request_navigation" in names
                assert "limo_get_camera_state" not in names
                assert "limo_get_audio_state" not in names
                assert "limo_get_runtime_status" not in names
                assert not any("publish" in name or "cmd_vel" in name for name in names)
