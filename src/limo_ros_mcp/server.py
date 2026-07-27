"""Safety-bounded MCP server for the AgileX LIMO ROS 1 interface."""

from __future__ import annotations

import argparse
import asyncio
from typing import Any

from mcp.server.fastmcp import FastMCP

from limo_ros_mcp.contract import (
    LIMO_OBSERVATION_TOPICS,
    get_limo_contract,
    validate_navigation_goal,
)
from limo_ros_mcp.rosbridge import RosbridgeReadOnlyClient, validate_rosbridge_endpoint
from limo_ros_mcp.rosclaw_gateway import RosclawGateway
from limo_ros_mcp.roscli import RosCliReadOnlyClient

REQUIRED_LIMO_TOPICS = {"/limo_status", "/odom", "/imu"}


def _control_error(operation: str, exc: Exception) -> dict[str, Any]:
    return {
        "ok": False,
        "operation": operation,
        "error_code": str(getattr(exc, "code", "ROSCLAWD_UNAVAILABLE")),
        "error": str(exc),
        "trust_level": "UNAVAILABLE",
        "command_dispatched": False,
        "usable_for_real_execution": False,
    }


class LimoMCPService:
    """Implementation behind the LIMO MCP tools, injectable for tests."""

    def __init__(self, gateway: RosclawGateway | Any | None = None) -> None:
        self._gateway = gateway or RosclawGateway()

    def get_contract(self) -> dict[str, Any]:
        return {
            "ok": True,
            "contract": get_limo_contract(),
            "trust_level": "STATIC_CONTRACT",
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

    def probe_ros(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        transport: str = "auto",
    ) -> dict[str, Any]:
        if transport not in {"auto", "rosbridge", "roscli"}:
            raise ValueError("transport must be auto, rosbridge, or roscli")
        errors: dict[str, str] = {}
        candidates = [transport] if transport != "auto" else ["rosbridge", "roscli"]
        for candidate in candidates:
            try:
                if candidate == "rosbridge":
                    endpoint = validate_rosbridge_endpoint(endpoint)
                    snapshot = RosbridgeReadOnlyClient(endpoint).probe()
                else:
                    snapshot = RosCliReadOnlyClient(dict(LIMO_OBSERVATION_TOPICS.values())).probe()
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                errors[candidate] = str(exc)
                continue
            topic_names = set(snapshot["topics"])
            optional = {topic for topic, _message_type in LIMO_OBSERVATION_TOPICS.values()}
            return {
                "ok": REQUIRED_LIMO_TOPICS.issubset(topic_names),
                "endpoint": endpoint,
                "transport": candidate,
                "ros_version": "ros1",
                "required_topics": sorted(REQUIRED_LIMO_TOPICS),
                "present_topics": sorted(optional & topic_names),
                "missing_required_topics": sorted(REQUIRED_LIMO_TOPICS - topic_names),
                "nodes": snapshot["nodes"],
                "trust_level": "LIVE_ROS_GRAPH_OBSERVATION",
                "command_dispatched": False,
                "usable_for_real_execution": False,
            }
        return {
            "ok": False,
            "endpoint": endpoint,
            "transport": transport,
            "error_code": "LIMO_ROS_UNAVAILABLE",
            "errors": errors,
            "trust_level": "UNAVAILABLE",
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

    def observe(
        self,
        observation: str,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        if observation not in LIMO_OBSERVATION_TOPICS:
            raise ValueError(f"observation must be one of {sorted(LIMO_OBSERVATION_TOPICS)}")
        if isinstance(timeout_sec, bool) or not 0.1 <= float(timeout_sec) <= 5.0:
            raise ValueError("timeout_sec must be within [0.1, 5.0]")
        if transport not in {"auto", "rosbridge", "roscli"}:
            raise ValueError("transport must be auto, rosbridge, or roscli")
        topic, message_type = LIMO_OBSERVATION_TOPICS[observation]
        errors: dict[str, str] = {}
        candidates = [transport] if transport != "auto" else ["rosbridge", "roscli"]
        for candidate in candidates:
            try:
                if candidate == "rosbridge":
                    endpoint = validate_rosbridge_endpoint(endpoint)
                    client: Any = RosbridgeReadOnlyClient(
                        endpoint,
                        timeout_sec=float(timeout_sec),
                    )
                else:
                    client = RosCliReadOnlyClient(
                        dict(LIMO_OBSERVATION_TOPICS.values()), timeout_sec=float(timeout_sec)
                    )
                message = client.subscribe_once(topic, message_type)
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                errors[candidate] = str(exc)
                continue
            return {
                "ok": True,
                "observation": observation,
                "topic": topic,
                "message_type": message_type,
                "message": message,
                "transport": candidate,
                "trust_level": "LIVE_ROS_TOPIC_OBSERVATION",
                "command_dispatched": False,
                "usable_for_real_execution": False,
            }
        return {
            "ok": False,
            "observation": observation,
            "topic": topic,
            "message_type": message_type,
            "message": None,
            "transport": transport,
            "error_code": "LIMO_ROS_UNAVAILABLE",
            "errors": errors,
            "trust_level": "UNAVAILABLE",
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

    def validate_goal(self, **kwargs: Any) -> dict[str, Any]:
        result = validate_navigation_goal(**kwargs)
        return {
            **result,
            "capability_id": "limo.navigate_to_pose",
            "trust_level": "STATIC_VALIDATION",
            "usable_for_real_execution": False,
        }

    async def runtime_status(self) -> dict[str, Any]:
        try:
            return await self._gateway.runtime_status()
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("runtime_status", exc)

    async def request_navigation(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        localization_ready: bool,
        costmap_ready: bool,
        obstacle_check_enabled: bool,
        body_snapshot_hash: str,
        execution_mode: str = "SHADOW",
        principal_id: str = "",
        approval_id: str | None = None,
        action_id: str | None = None,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        validation = validate_navigation_goal(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            localization_ready=localization_ready,
            costmap_ready=costmap_ready,
            obstacle_check_enabled=obstacle_check_enabled,
        )
        if not validation["ok"]:
            return {
                **validation,
                "error_code": "LIMO_NAVIGATION_PRECONDITION_FAILED",
                "command_dispatched": False,
                "usable_for_real_execution": False,
            }
        mode = str(execution_mode).upper()
        if mode not in {"SHADOW", "REAL"}:
            return {
                "ok": False,
                "decision": "BLOCK",
                "error_code": "INVALID_EXECUTION_MODE",
                "message": "Only SHADOW or REAL may be submitted to rosclawd.",
                "command_dispatched": False,
                "usable_for_real_execution": False,
            }
        if not str(body_snapshot_hash).strip():
            return {
                "ok": False,
                "decision": "BLOCK",
                "error_code": "BODY_SNAPSHOT_REQUIRED",
                "message": "SHADOW and REAL requests require an immutable LIMO body snapshot.",
                "command_dispatched": False,
                "usable_for_real_execution": False,
            }
        if isinstance(wait_timeout_sec, bool) or not 0.0 <= float(wait_timeout_sec) <= 30.0:
            raise ValueError("wait_timeout_sec must be within [0, 30]")

        goal = validation["normalized_goal"]
        try:
            return await self._gateway.request_navigation(
                capability_id="limo.navigate_to_pose",
                arguments={
                    "target_pose": goal,
                    "preconditions": {
                        "localization_ready": localization_ready,
                        "costmap_ready": costmap_ready,
                        "obstacle_check_enabled": obstacle_check_enabled,
                    },
                },
                execution_mode=mode,
                body_snapshot_hash=body_snapshot_hash,
                principal_id=principal_id,
                approval_id=approval_id,
                body_id="limo",
                action_id=action_id,
                required_evidence="TASK_VERIFIED",
                timeout_sec=30.0,
                wait_timeout_sec=float(wait_timeout_sec),
            )
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("request_navigation", exc)

    async def action_status(self, action_id: str) -> dict[str, Any]:
        try:
            return await self._gateway.action_status(action_id)
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("action_status", exc)

    async def execution_receipt(self, action_id: str) -> dict[str, Any]:
        try:
            return await self._gateway.execution_receipt(action_id)
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("execution_receipt", exc)

    async def emergency_stop(self, reason: str) -> dict[str, Any]:
        if not reason.strip():
            raise ValueError("reason is required")
        try:
            return await self._gateway.emergency_stop(reason)
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            result = _control_error("emergency_stop", exc)
            result["operator_action"] = "Use the certified physical E-stop immediately."
            return result


def build_mcp_server(service: LimoMCPService | None = None) -> FastMCP:
    """Build the standalone LIMO MCP server."""

    implementation = service or LimoMCPService()
    mcp = FastMCP(
        "rosclaw-limo",
        instructions=(
            "AgileX LIMO ROS 1 integration. Read-only telemetry may use rosbridge. "
            "Navigation is submitted only through rosclawd; direct /cmd_vel is never exposed."
        ),
    )

    @mcp.tool(description="Return the source-backed LIMO ROS interface and safety contract.")
    def limo_get_contract() -> dict[str, Any]:
        return implementation.get_contract()

    @mcp.tool(
        description="Read the ROS graph and verify required LIMO topics without moving the robot."
    )
    async def limo_probe_ros(
        endpoint: str = "ws://127.0.0.1:9090",
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(implementation.probe_ros, endpoint, transport)

    @mcp.tool(
        description="Read one allowlisted LIMO telemetry topic; no publish primitive is exposed."
    )
    async def limo_observe(
        observation: str,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.observe,
            observation,
            endpoint,
            timeout_sec,
            transport,
        )

    @mcp.tool(description="Validate a LIMO move_base goal without dispatching it.")
    def limo_validate_navigation_goal(
        x: float,
        y: float,
        yaw: float,
        frame_id: str = "map",
        localization_ready: bool = False,
        costmap_ready: bool = False,
        obstacle_check_enabled: bool = False,
    ) -> dict[str, Any]:
        return implementation.validate_goal(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            localization_ready=localization_ready,
            costmap_ready=costmap_ready,
            obstacle_check_enabled=obstacle_check_enabled,
        )

    @mcp.tool(description="Read rosclawd readiness; this never initializes a local executor.")
    async def limo_get_runtime_status() -> dict[str, Any]:
        return await implementation.runtime_status()

    @mcp.tool(
        description=(
            "Submit a validated LIMO move_base goal to rosclawd. Defaults to SHADOW; "
            "REAL remains authorization- and executor-gated by the daemon."
        )
    )
    async def limo_request_navigation(
        x: float,
        y: float,
        yaw: float,
        body_snapshot_hash: str,
        frame_id: str = "map",
        localization_ready: bool = False,
        costmap_ready: bool = False,
        obstacle_check_enabled: bool = False,
        execution_mode: str = "SHADOW",
        principal_id: str = "",
        approval_id: str | None = None,
        action_id: str | None = None,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        return await implementation.request_navigation(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            localization_ready=localization_ready,
            costmap_ready=costmap_ready,
            obstacle_check_enabled=obstacle_check_enabled,
            body_snapshot_hash=body_snapshot_hash,
            execution_mode=execution_mode,
            principal_id=principal_id,
            approval_id=approval_id,
            action_id=action_id,
            wait_timeout_sec=wait_timeout_sec,
        )

    @mcp.tool(description="Read an action state from rosclawd.")
    async def limo_get_action_status(action_id: str) -> dict[str, Any]:
        return await implementation.action_status(action_id)

    @mcp.tool(description="Read the canonical execution receipt from rosclawd.")
    async def limo_get_execution_receipt(action_id: str) -> dict[str, Any]:
        return await implementation.execution_receipt(action_id)

    @mcp.tool(
        description=(
            "Request emergency stop through rosclawd. This cannot prove physical motion stopped; "
            "use the certified hardware E-stop whenever motion may be active."
        )
    )
    async def limo_emergency_stop(reason: str) -> dict[str, Any]:
        return await implementation.emergency_stop(reason)

    return mcp


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ROSClaw AgileX LIMO MCP server")
    parser.add_argument("--transport", choices=["stdio", "http", "sse"], default="stdio")
    args = parser.parse_args(argv)
    transport = "streamable-http" if args.transport == "http" else args.transport
    build_mcp_server().run(transport=transport)  # type: ignore[arg-type]
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
