"""Inspection-focused MCP server for the AgileX LIMO ROS 1 interface."""

from __future__ import annotations

import argparse
import asyncio
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Any

from mcp.server.fastmcp import FastMCP

from limo_ros_mcp.contract import (
    LIMO_OBSERVATION_TOPICS,
    REQUIRED_LIMO_TOPICS,
    get_limo_contract,
    list_limo_observations,
    validate_navigation_goal,
    validate_velocity_command,
)
from limo_ros_mcp.messages import evaluate_patrol_readiness, summarize_observation
from limo_ros_mcp.rosbridge import RosbridgeReadOnlyClient, validate_rosbridge_endpoint
from limo_ros_mcp.rosclaw_gateway import RosclawGateway
from limo_ros_mcp.roscli import RosCliReadOnlyClient


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

    def list_observations(self, category: str = "all") -> dict[str, Any]:
        observations = list_limo_observations(category)
        return {
            "ok": True,
            "category": category,
            "count": len(observations),
            "observations": observations,
            "command_dispatched": False,
        }

    @staticmethod
    def _validate_transport(transport: str) -> None:
        if transport not in {"auto", "rosbridge", "roscli"}:
            raise ValueError("transport must be auto, rosbridge, or roscli")

    @staticmethod
    def _validate_timeout(timeout_sec: float) -> float:
        if isinstance(timeout_sec, bool) or not 0.1 <= float(timeout_sec) <= 10.0:
            raise ValueError("timeout_sec must be within [0.1, 10.0]")
        return float(timeout_sec)

    @staticmethod
    def _candidates(transport: str) -> list[str]:
        return [transport] if transport != "auto" else ["rosbridge", "roscli"]

    @staticmethod
    def _client(
        candidate: str,
        endpoint: str,
        timeout_sec: float,
    ) -> RosbridgeReadOnlyClient | RosCliReadOnlyClient:
        if candidate == "rosbridge":
            return RosbridgeReadOnlyClient(
                validate_rosbridge_endpoint(endpoint),
                timeout_sec=timeout_sec,
            )
        return RosCliReadOnlyClient(dict(LIMO_OBSERVATION_TOPICS.values()), timeout_sec=timeout_sec)

    def probe_ros(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        transport: str = "auto",
        timeout_sec: float = 5.0,
    ) -> dict[str, Any]:
        self._validate_transport(transport)
        timeout_sec = self._validate_timeout(timeout_sec)
        errors: dict[str, str] = {}
        for candidate in self._candidates(transport):
            try:
                client = self._client(candidate, endpoint, timeout_sec)
                snapshot = client.probe()
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                errors[candidate] = str(exc)
                continue
            topic_names = set(snapshot["topics"])
            optional = {topic for topic, _message_type in LIMO_OBSERVATION_TOPICS.values()}
            observed_types = dict(zip(snapshot["topics"], snapshot["types"], strict=False))
            available_observations = [
                name
                for name, (topic, expected_type) in LIMO_OBSERVATION_TOPICS.items()
                if topic in topic_names
                and observed_types.get(topic, expected_type) == expected_type
            ]
            type_mismatches = [
                {
                    "observation": name,
                    "topic": topic,
                    "expected_type": expected_type,
                    "observed_type": observed_types.get(topic),
                }
                for name, (topic, expected_type) in LIMO_OBSERVATION_TOPICS.items()
                if topic in topic_names
                and observed_types.get(topic)
                and observed_types.get(topic) != expected_type
            ]
            return {
                "ok": REQUIRED_LIMO_TOPICS.issubset(topic_names),
                "endpoint": endpoint,
                "transport": candidate,
                "ros_version": "ros1",
                "required_topics": sorted(REQUIRED_LIMO_TOPICS),
                "present_topics": sorted(optional & topic_names),
                "missing_required_topics": sorted(REQUIRED_LIMO_TOPICS - topic_names),
                "available_observations": available_observations,
                "type_mismatches": type_mismatches,
                "topic_count": len(topic_names),
                "topic_types": observed_types,
                "nodes": snapshot["nodes"],
                "node_count": len(snapshot["nodes"]),
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
        include_raw: bool = True,
    ) -> dict[str, Any]:
        if observation not in LIMO_OBSERVATION_TOPICS:
            raise ValueError(f"observation must be one of {sorted(LIMO_OBSERVATION_TOPICS)}")
        timeout_sec = self._validate_timeout(timeout_sec)
        self._validate_transport(transport)
        topic, message_type = LIMO_OBSERVATION_TOPICS[observation]
        errors: dict[str, str] = {}
        for candidate in self._candidates(transport):
            try:
                client = self._client(candidate, endpoint, timeout_sec)
                message = client.subscribe_once(topic, message_type)
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                errors[candidate] = str(exc)
                continue
            result = {
                "ok": True,
                "observation": observation,
                "topic": topic,
                "message_type": message_type,
                "summary": summarize_observation(observation, message),
                "transport": candidate,
                "trust_level": "LIVE_ROS_TOPIC_OBSERVATION",
                "command_dispatched": False,
                "usable_for_real_execution": False,
            }
            if include_raw:
                result["message"] = message
            return result
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

    def topic_info(
        self,
        observation: str,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        if observation not in LIMO_OBSERVATION_TOPICS:
            raise ValueError(f"observation must be one of {sorted(LIMO_OBSERVATION_TOPICS)}")
        timeout_sec = self._validate_timeout(timeout_sec)
        self._validate_transport(transport)
        topic, message_type = LIMO_OBSERVATION_TOPICS[observation]
        errors: dict[str, str] = {}
        for candidate in self._candidates(transport):
            try:
                client = self._client(candidate, endpoint, timeout_sec)
                info = client.topic_info(topic, message_type)
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                errors[candidate] = str(exc)
                continue
            return {
                "ok": True,
                "observation": observation,
                "transport": candidate,
                **info,
                "command_dispatched": False,
            }
        return {
            "ok": False,
            "observation": observation,
            "topic": topic,
            "message_type": message_type,
            "errors": errors,
            "error_code": "LIMO_ROS_UNAVAILABLE",
            "command_dispatched": False,
        }

    def sample(
        self,
        observation: str,
        count: int = 3,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
        transport: str = "auto",
        include_raw: bool = False,
    ) -> dict[str, Any]:
        if observation not in LIMO_OBSERVATION_TOPICS:
            raise ValueError(f"observation must be one of {sorted(LIMO_OBSERVATION_TOPICS)}")
        if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 10:
            raise ValueError("count must be an integer within [1, 10]")
        timeout_sec = self._validate_timeout(timeout_sec)
        self._validate_transport(transport)
        topic, message_type = LIMO_OBSERVATION_TOPICS[observation]
        errors: dict[str, str] = {}
        for candidate in self._candidates(transport):
            try:
                client = self._client(candidate, endpoint, timeout_sec)
                started = time.monotonic()
                messages = client.subscribe_many(topic, message_type, count=count)
                elapsed = time.monotonic() - started
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                errors[candidate] = str(exc)
                continue
            summaries = [summarize_observation(observation, message) for message in messages]
            header_stamps = [
                summary.get("header", {}).get("stamp_sec")
                for summary in summaries
                if isinstance(summary.get("header"), dict)
            ]
            numeric_stamps = [
                float(stamp)
                for stamp in header_stamps
                if isinstance(stamp, (int, float)) and not isinstance(stamp, bool)
            ]
            stamp_elapsed = (
                numeric_stamps[-1] - numeric_stamps[0] if len(numeric_stamps) > 1 else 0.0
            )
            if stamp_elapsed > 0:
                estimated_rate_hz: float | None = (len(numeric_stamps) - 1) / stamp_elapsed
                rate_source = "ros_header"
            elif len(messages) > 1 and elapsed > 0:
                estimated_rate_hz = (len(messages) - 1) / elapsed
                rate_source = "wall_clock"
            else:
                estimated_rate_hz = None
                rate_source = None
            result: dict[str, Any] = {
                "ok": True,
                "observation": observation,
                "topic": topic,
                "message_type": message_type,
                "transport": candidate,
                "sample_count": len(messages),
                "elapsed_sec": elapsed,
                "estimated_rate_hz": estimated_rate_hz,
                "rate_source": rate_source,
                "summaries": summaries,
                "command_dispatched": False,
            }
            if include_raw:
                result["messages"] = messages
            return result
        return {
            "ok": False,
            "observation": observation,
            "topic": topic,
            "message_type": message_type,
            "errors": errors,
            "error_code": "LIMO_ROS_UNAVAILABLE",
            "command_dispatched": False,
        }

    def _collect_summaries(
        self,
        observations: list[str],
        *,
        endpoint: str,
        timeout_sec: float,
        transport: str,
    ) -> dict[str, Any]:
        """Collect independent topics concurrently and preserve partial evidence."""

        timeout_sec = self._validate_timeout(timeout_sec)
        self._validate_transport(transport)
        summaries: dict[str, dict[str, Any]] = {}
        failures: dict[str, dict[str, Any]] = {}
        with ThreadPoolExecutor(max_workers=min(6, len(observations))) as executor:
            future_names = {
                executor.submit(
                    self.observe,
                    observation,
                    endpoint,
                    timeout_sec,
                    transport,
                    False,
                ): observation
                for observation in observations
            }
            for future in as_completed(future_names):
                observation = future_names[future]
                result = future.result()
                if result.get("ok") and isinstance(result.get("summary"), dict):
                    summaries[observation] = result["summary"]
                else:
                    failures[observation] = result
        return {
            "ok": not failures,
            "requested_observations": observations,
            "summaries": summaries,
            "failures": failures,
            "available_count": len(summaries),
            "missing_count": len(failures),
            "trust_level": "LIVE_ROS_TOPIC_OBSERVATION",
            "command_dispatched": False,
        }

    def base_state(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return self._collect_summaries(
            ["status", "odometry", "imu"],
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )

    def laser_summary(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return self.observe("laser_scan", endpoint, timeout_sec, transport, False)

    def localization_state(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return self._collect_summaries(
            ["localized_pose", "fused_pose"],
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )

    def navigation_state(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return self._collect_summaries(
            ["navigation_status", "current_goal", "global_plan", "local_plan"],
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )

    def map_summary(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
        include_grids: bool = False,
    ) -> dict[str, Any]:
        observations = ["map_metadata"]
        if include_grids:
            observations.extend(["map", "global_costmap", "local_costmap"])
        return self._collect_summaries(
            observations,
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )

    def diagnostics_state(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
        include_latest_log: bool = False,
    ) -> dict[str, Any]:
        observations = ["diagnostics"]
        if include_latest_log:
            observations.append("ros_logs")
        return self._collect_summaries(
            observations,
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )

    def transform_state(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return self._collect_summaries(
            ["tf", "tf_static"],
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )

    def patrol_readiness(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        snapshot = self._collect_summaries(
            [
                "status",
                "odometry",
                "imu",
                "laser_scan",
                "localized_pose",
                "navigation_status",
                "map_metadata",
                "diagnostics",
            ],
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )
        readiness = evaluate_patrol_readiness(snapshot["summaries"])
        return {**snapshot, "readiness": readiness, "ok": readiness["ready"]}

    def validate_goal(self, **kwargs: Any) -> dict[str, Any]:
        result = validate_navigation_goal(**kwargs)
        return {
            **result,
            "capability_id": "limo.navigate_to_pose",
            "trust_level": "STATIC_VALIDATION",
            "usable_for_real_execution": False,
        }

    def validate_velocity(self, **kwargs: Any) -> dict[str, Any]:
        result = validate_velocity_command(**kwargs)
        return {
            **result,
            "capability_id": "limo.base.velocity_command",
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
            "AgileX LIMO ROS 1 inspection integration. Discover, inspect, sample, and summarize "
            "the base, lidar, localization, navigation, map, diagnostics, TF, and camera topic "
            "contracts. Navigation is submitted through rosclawd."
        ),
    )

    @mcp.tool(description="Return the source-backed LIMO ROS interface and safety contract.")
    def limo_get_contract() -> dict[str, Any]:
        return implementation.get_contract()

    @mcp.tool(description="List named LIMO ROS observations, optionally filtered by category.")
    def limo_list_observations(category: str = "all") -> dict[str, Any]:
        return implementation.list_observations(category)

    @mcp.tool(
        description="Read the ROS graph and verify required LIMO topics without moving the robot."
    )
    async def limo_probe_ros(
        endpoint: str = "ws://127.0.0.1:9090",
        transport: str = "auto",
        timeout_sec: float = 5.0,
    ) -> dict[str, Any]:
        return await asyncio.to_thread(implementation.probe_ros, endpoint, transport, timeout_sec)

    @mcp.tool(
        description="Read one allowlisted LIMO telemetry topic; no publish primitive is exposed."
    )
    async def limo_observe(
        observation: str,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
        include_raw: bool = False,
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.observe,
            observation,
            endpoint,
            timeout_sec,
            transport,
            include_raw,
        )

    @mcp.tool(description="Inspect type, publishers, and subscribers for one named LIMO ROS topic.")
    async def limo_get_topic_info(
        observation: str,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.topic_info,
            observation,
            endpoint,
            timeout_sec,
            transport,
        )

    @mcp.tool(
        description=(
            "Sample 1-10 messages from a named LIMO ROS topic and estimate its rate. "
            "Large arrays are summarized unless include_raw is explicitly enabled."
        )
    )
    async def limo_sample_topic(
        observation: str,
        count: int = 3,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
        transport: str = "auto",
        include_raw: bool = False,
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.sample,
            observation,
            count,
            endpoint,
            timeout_sec,
            transport,
            include_raw,
        )

    @mcp.tool(description="Read and summarize LIMO status, odometry, and IMU concurrently.")
    async def limo_get_base_state(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(implementation.base_state, endpoint, timeout_sec, transport)

    @mcp.tool(
        description=(
            "Summarize lidar validity, percentiles, closest obstacle, and directional clearance."
        )
    )
    async def limo_get_laser_summary(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.laser_summary, endpoint, timeout_sec, transport
        )

    @mcp.tool(description="Read AMCL and fused pose summaries with localization covariance.")
    async def limo_get_localization_state(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.localization_state, endpoint, timeout_sec, transport
        )

    @mcp.tool(
        description="Read move_base goal status, current goal, and compact global/local plans."
    )
    async def limo_get_navigation_state(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.navigation_state, endpoint, timeout_sec, transport
        )

    @mcp.tool(
        description=("Read map metadata and optionally summarize map/global/local occupancy grids.")
    )
    async def limo_get_map_summary(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
        include_grids: bool = False,
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.map_summary,
            endpoint,
            timeout_sec,
            transport,
            include_grids,
        )

    @mcp.tool(description="Read ROS diagnostics and optionally the latest aggregated ROS log.")
    async def limo_get_diagnostics(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
        include_latest_log: bool = False,
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.diagnostics_state,
            endpoint,
            timeout_sec,
            transport,
            include_latest_log,
        )

    @mcp.tool(description="Read dynamic and static TF messages as compact parent-child edges.")
    async def limo_get_transform_state(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.transform_state, endpoint, timeout_sec, transport
        )

    @mcp.tool(
        description=(
            "Collect a patrol preflight snapshot across base, lidar, localization, navigation, "
            "map, and diagnostics topics; this does not dispatch motion."
        )
    )
    async def limo_get_patrol_readiness(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 2.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.patrol_readiness, endpoint, timeout_sec, transport
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

    @mcp.tool(
        description=(
            "Validate a bounded velocity experiment against LIMO motion-mode limits. "
            "Validation does not publish /cmd_vel."
        )
    )
    def limo_validate_velocity_command(
        linear_x: float,
        linear_y: float,
        angular_z: float,
        duration_sec: float,
        motion_mode: int,
    ) -> dict[str, Any]:
        return implementation.validate_velocity(
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=angular_z,
            duration_sec=duration_sec,
            motion_mode=motion_mode,
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
