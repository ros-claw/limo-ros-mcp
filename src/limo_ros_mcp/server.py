"""Inspection-focused MCP server for the AgileX LIMO ROS 1 interface."""

from __future__ import annotations

import argparse
import asyncio
import os
import time
import uuid
from collections import OrderedDict
from collections.abc import Mapping
from concurrent.futures import Future, ThreadPoolExecutor, as_completed
from datetime import UTC, datetime, timedelta
from threading import Lock
from typing import Any, Literal, cast

from mcp.server.fastmcp import Context, FastMCP
from mcp.types import ToolAnnotations

from limo_ros_mcp.action_specs import (
    initial_pose_arguments,
    initial_pose_display,
    navigation_arguments,
    navigation_display,
    speech_arguments,
    speech_display,
    tone_arguments,
    tone_display,
    validate_speech,
    validate_tone,
)
from limo_ros_mcp.cache import ObservationHub
from limo_ros_mcp.clocks import ClockJumpTracker
from limo_ros_mcp.contract import (
    LIMO_OBSERVATION_TOPICS,
    REQUIRED_LIMO_TOPICS,
    get_limo_contract,
    list_limo_observations,
    validate_velocity_command,
)
from limo_ros_mcp.messages import summarize_observation
from limo_ros_mcp.navigation import NavigationGoalValidator
from limo_ros_mcp.output import compact_readiness
from limo_ros_mcp.peripherals import PeripheralInspector
from limo_ros_mcp.profiles import select_tools
from limo_ros_mcp.readiness import build_readiness_evidence, validate_readiness_snapshot
from limo_ros_mcp.rosbridge import RosbridgeReadOnlyClient, validate_rosbridge_endpoint
from limo_ros_mcp.rosclaw_gateway import RosclawGateway
from limo_ros_mcp.roscli import RosCliReadOnlyClient
from limo_ros_mcp.runtime_info import interaction_plane_status, mcp_process_status

READ_ONLY_TOOL = ToolAnnotations(
    readOnlyHint=True,
    destructiveHint=False,
    idempotentHint=True,
    openWorldHint=False,
)
GUARDED_ACTION_TOOL = ToolAnnotations(
    readOnlyHint=False,
    destructiveHint=True,
    idempotentHint=False,
    openWorldHint=False,
)
EMERGENCY_TOOL = ToolAnnotations(
    readOnlyHint=False,
    destructiveHint=True,
    idempotentHint=True,
    openWorldHint=False,
)

OBSERVATION_MAX_AGE_SEC = {
    "status": 0.3,
    "odometry": 0.3,
    "imu": 0.3,
    "laser_scan": 0.5,
    "localized_pose": 1.0,
    "navigation_status": 0.5,
    "map_metadata": 60.0,
    "global_costmap": 0.5,
    "local_costmap": 0.5,
    "diagnostics": 2.0,
    "tf": 0.5,
    "tf_static": 60.0,
}

# ROS 1 CLI observations spawn one short-lived process per topic.  The LIMO
# Jetson cannot reliably service the complete readiness fan-out at once, even
# though the same topics are healthy when sampled in smaller groups.
ROSCLI_MAX_CONCURRENT_OBSERVATIONS = 8


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

    def __init__(
        self,
        gateway: RosclawGateway | Any | None = None,
        goal_validator: NavigationGoalValidator | Any | None = None,
        peripheral_inspector: PeripheralInspector | Any | None = None,
    ) -> None:
        self._gateway = gateway or RosclawGateway()
        try:
            from rosclaw.interaction import InteractionClient, InteractionCoordinator

            self._interaction: Any | None = InteractionCoordinator(InteractionClient(self._gateway))
        except ImportError:
            self._interaction = None
        self._goal_validator = goal_validator or NavigationGoalValidator()
        self._peripheral_inspector = peripheral_inspector or PeripheralInspector()
        self._clock_tracker = ClockJumpTracker()
        self._observation_hub = ObservationHub(
            lambda candidate, endpoint, timeout: self._client(candidate, endpoint, timeout)
        )
        self._readiness_snapshots: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._readiness_lock = Lock()
        self._readiness_flights: dict[tuple[str, str, str, str], Future[dict[str, Any]]] = {}
        self._readiness_flight_lock = Lock()

    def _remember_readiness(self, readiness: Mapping[str, Any]) -> None:
        snapshot_hash = readiness.get("snapshot_hash")
        if not isinstance(snapshot_hash, str):
            return
        with self._readiness_lock:
            self._readiness_snapshots[snapshot_hash] = dict(readiness)
            self._readiness_snapshots.move_to_end(snapshot_hash)
            while len(self._readiness_snapshots) > 32:
                self._readiness_snapshots.popitem(last=False)

    def _validate_readiness_reference(
        self, readiness_snapshot_hash: str, body_snapshot_hash: str
    ) -> dict[str, Any]:
        if not str(body_snapshot_hash).strip():
            return self._navigation_block(
                "BODY_SNAPSHOT_REQUIRED",
                "Navigation requests require an immutable LIMO body snapshot.",
            )
        if not str(readiness_snapshot_hash).strip():
            return self._navigation_block(
                "LIMO_READINESS_SNAPSHOT_REQUIRED",
                "A readiness snapshot generated by this MCP process is required.",
            )
        with self._readiness_lock:
            snapshot = self._readiness_snapshots.get(readiness_snapshot_hash)
        if snapshot is None:
            return self._navigation_block(
                "LIMO_READINESS_SNAPSHOT_UNKNOWN",
                "The readiness snapshot was not generated by this MCP process or was evicted.",
            )
        integrity = validate_readiness_snapshot(snapshot)
        if not integrity["ok"]:
            return self._navigation_block(
                str(integrity["error_code"]),
                "The referenced readiness snapshot is invalid or expired.",
            )
        if snapshot.get("snapshot_hash") != readiness_snapshot_hash:
            return self._navigation_block(
                "LIMO_SNAPSHOT_HASH_MISMATCH",
                "The readiness snapshot hash does not match the request.",
            )
        if snapshot.get("body_snapshot_hash") != body_snapshot_hash:
            return self._navigation_block(
                "LIMO_BODY_SNAPSHOT_MISMATCH",
                "The body snapshot does not match the referenced readiness snapshot.",
            )
        if snapshot.get("ready") is not True or snapshot.get("blockers"):
            return self._navigation_block(
                "LIMO_READINESS_NOT_READY",
                "The referenced readiness snapshot contains a blocking failure.",
            )
        binding = snapshot.get("transport_binding")
        if isinstance(binding, dict):
            endpoint = str(binding.get("endpoint") or "")
            transport = str(binding.get("transport") or "")
            expected_generation = binding.get("generation")
            current_generation = self._observation_hub.generation(transport, endpoint)
            if current_generation != expected_generation:
                return self._navigation_block(
                    "LIMO_TRANSPORT_GENERATION_CHANGED",
                    "The ROS observation transport reconnected after this readiness snapshot.",
                )
        return {"ok": True, "readiness": snapshot}

    @staticmethod
    def _navigation_block(error_code: str, message: str) -> dict[str, Any]:
        return {
            "ok": False,
            "decision": "BLOCK",
            "schema_version": "limo.navigation.v2",
            "error_code": error_code,
            "message": message,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

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
                client = self._observation_hub.client(candidate, endpoint, timeout_sec)
                snapshot = client.probe()
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                self._observation_hub.invalidate(candidate, endpoint)
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
        timeout_sec: float = 5.0,
        transport: str = "auto",
        include_raw: bool = True,
    ) -> dict[str, Any]:
        if observation not in LIMO_OBSERVATION_TOPICS:
            raise ValueError(f"observation must be one of {sorted(LIMO_OBSERVATION_TOPICS)}")
        if include_raw and observation in {
            "color_image",
            "depth_image",
            "depth_points",
            "infrared_image",
            "map",
            "global_costmap",
            "local_costmap",
        }:
            raise ValueError("include_raw is denied for binary and occupancy-grid observations")
        timeout_sec = self._validate_timeout(timeout_sec)
        self._validate_transport(transport)
        topic, message_type = LIMO_OBSERVATION_TOPICS[observation]
        errors: dict[str, str] = {}
        for candidate in self._candidates(transport):
            try:
                client = self._observation_hub.client(candidate, endpoint, timeout_sec)
                return self._observe_with_client(
                    observation,
                    client=client,
                    transport=candidate,
                    include_raw=include_raw,
                )
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                self._observation_hub.invalidate(candidate, endpoint)
                errors[candidate] = str(exc)
                continue
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

    @staticmethod
    def _observe_with_client(
        observation: str,
        *,
        client: RosbridgeReadOnlyClient | RosCliReadOnlyClient,
        transport: str,
        include_raw: bool = False,
        sample_count: int = 1,
    ) -> dict[str, Any]:
        """Collect one named observation using a preflighted shared client."""

        topic, message_type = LIMO_OBSERVATION_TOPICS[observation]
        messages = client.subscribe_many(topic, message_type, count=sample_count)
        received_wall_time = time.time()
        received_monotonic = time.monotonic()
        summaries = [summarize_observation(observation, message) for message in messages]
        if observation == "tf" and len(summaries) > 1:
            transforms: list[dict[str, Any]] = []
            seen: set[tuple[str, str]] = set()
            for summary in summaries:
                for value in summary.get("transforms", []):
                    if not isinstance(value, dict):
                        continue
                    edge = (str(value.get("parent_frame", "")), str(value.get("child_frame", "")))
                    if edge in seen:
                        continue
                    seen.add(edge)
                    transforms.append(value)
            summary = {"transform_count": len(transforms), "transforms": transforms}
        else:
            summary = summaries[-1]
        resolve_transform = getattr(client, "transform_available", None)
        if observation == "tf" and callable(resolve_transform):
            transforms = list(summary.get("transforms", []))
            seen = {
                (
                    str(value.get("parent_frame", "")).lstrip("/"),
                    str(value.get("child_frame", "")).lstrip("/"),
                )
                for value in transforms
                if isinstance(value, dict)
            }
            for parent, child in (("map", "odom"), ("odom", "base_link")):
                edge = (parent, child)
                if edge not in seen and resolve_transform(parent, child):
                    transforms.append(
                        {
                            "parent_frame": parent,
                            "child_frame": child,
                            "source": "tf_echo",
                        }
                    )
                    seen.add(edge)
            summary = {"transform_count": len(transforms), "transforms": transforms}
        result: dict[str, Any] = {
            "ok": True,
            "observation": observation,
            "topic": topic,
            "message_type": message_type,
            "summary": summary,
            "transport": transport,
            "transport_generation": getattr(client, "transport_generation", None),
            "received_wall_time": received_wall_time,
            "received_monotonic": received_monotonic,
            "trust_level": "LIVE_ROS_TOPIC_OBSERVATION",
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }
        if include_raw:
            result["message"] = messages[-1] if sample_count == 1 else messages
        return result

    @staticmethod
    def _supplement_tf_summary(
        summary: dict[str, Any], *, composed_chain_available: bool
    ) -> dict[str, Any]:
        """Prove the readiness-critical TF chain through one fixed tf_echo lookup."""

        transforms = [
            dict(value) for value in summary.get("transforms", []) if isinstance(value, dict)
        ]
        seen = {
            (
                str(value.get("parent_frame", "")).lstrip("/"),
                str(value.get("child_frame", "")).lstrip("/"),
            )
            for value in transforms
        }
        # A successful composed lookup is only possible when TF can resolve the
        # live map -> odom -> base_link chain.  One bounded process is more
        # reliable on the Jetson than racing two independent tf_echo nodes.
        edge = ("map", "base_link")
        if edge not in seen and composed_chain_available:
            transforms.append(
                {
                    "parent_frame": edge[0],
                    "child_frame": edge[1],
                    "source": "tf_echo_composed",
                }
            )
        return {"transform_count": len(transforms), "transforms": transforms}

    def topic_info(
        self,
        observation: str,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
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
                client = self._observation_hub.client(candidate, endpoint, timeout_sec)
                info = client.topic_info(topic, message_type)
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                self._observation_hub.invalidate(candidate, endpoint)
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
        if include_raw and observation in {
            "color_image",
            "depth_image",
            "depth_points",
            "infrared_image",
            "map",
            "global_costmap",
            "local_costmap",
        }:
            raise ValueError("include_raw is denied for binary and occupancy-grid observations")
        if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 10:
            raise ValueError("count must be an integer within [1, 10]")
        timeout_sec = self._validate_timeout(timeout_sec)
        self._validate_transport(transport)
        topic, message_type = LIMO_OBSERVATION_TOPICS[observation]
        errors: dict[str, str] = {}
        for candidate in self._candidates(transport):
            try:
                client = self._observation_hub.client(candidate, endpoint, timeout_sec)
                started = time.monotonic()
                messages = client.subscribe_many(topic, message_type, count=count)
                elapsed = time.monotonic() - started
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                self._observation_hub.invalidate(candidate, endpoint)
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
                "transport_generation": getattr(client, "transport_generation", None),
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
        transport_errors: dict[str, str] = {}
        cutoff_monotonic = time.monotonic()
        for candidate in self._candidates(transport):
            try:
                client = self._observation_hub.client(candidate, endpoint, timeout_sec)
                graph = client.probe()
            except Exception as exc:  # noqa: BLE001 - try the next read-only transport
                self._observation_hub.invalidate(candidate, endpoint)
                transport_errors[candidate] = str(exc)
                continue

            observed_types = dict(zip(graph["topics"], graph["types"], strict=False))
            summaries: dict[str, dict[str, Any]] = {}
            records: dict[str, dict[str, Any]] = {}
            failures: dict[str, dict[str, Any]] = {}
            available: list[str] = []
            cache_hits: list[str] = []
            transport_components = [
                {
                    "transport": candidate,
                    "generation": getattr(client, "transport_generation", None),
                }
            ]
            supplemental: RosCliReadOnlyClient | None = None
            tf_chain_resolved: bool | None = None
            for observation in observations:
                topic, expected_type = LIMO_OBSERVATION_TOPICS[observation]
                observed_type = observed_types.get(topic)
                if observed_type != expected_type:
                    failures[observation] = {
                        "ok": False,
                        "observation": observation,
                        "topic": topic,
                        "expected_type": expected_type,
                        "observed_type": observed_type,
                        "error_code": (
                            "LIMO_TOPIC_UNAVAILABLE"
                            if observed_type is None
                            else "LIMO_TOPIC_TYPE_MISMATCH"
                        ),
                        "command_dispatched": False,
                    }
                else:
                    cached = self._observation_hub.cached(
                        observation,
                        transport=candidate,
                        endpoint=endpoint,
                        max_age_sec=OBSERVATION_MAX_AGE_SEC.get(observation, 0.5),
                        cutoff_monotonic=cutoff_monotonic,
                    )
                    if cached is not None:
                        summaries[observation] = dict(cached["summary"])
                        records[observation] = cached
                        cache_hits.append(observation)
                    else:
                        available.append(observation)

            if candidate == "rosbridge" and any(
                observation in available
                for observation in ("global_costmap", "laser_scan", "tf")
            ):
                try:
                    supplemental = RosCliReadOnlyClient(
                        dict(LIMO_OBSERVATION_TOPICS.values()),
                        timeout_sec=timeout_sec,
                    )
                except Exception as exc:  # noqa: BLE001 - preserve rosbridge evidence
                    transport_errors["local_roscli_read_only"] = str(exc)
                else:
                    transport_components.append(
                        {
                            "transport": "local_roscli_read_only",
                            "generation": supplemental.transport_generation,
                            "purpose": "costmap_laser_and_tf_readiness_evidence",
                        }
                    )

            # A full 1984x1984 global costmap takes roughly 25 seconds to become
            # JSON over rosbridge on the LIMO Jetson.  Resolve its fixed --noarr
            # metadata and the composed TF chain concurrently, then immediately
            # collect all high-rate rosbridge evidence inside one coherent window.
            if candidate == "rosbridge" and supplemental is not None:
                with ThreadPoolExecutor(max_workers=3) as executor:
                    preflight_futures: dict[Any, str] = {}
                    if "global_costmap" in available:
                        available.remove("global_costmap")
                        preflight_futures[
                            executor.submit(
                                self._observe_with_client,
                                "global_costmap",
                                client=supplemental,
                                transport="local_roscli_read_only",
                                include_raw=False,
                            )
                        ] = "global_costmap"
                    if "laser_scan" in available:
                        available.remove("laser_scan")
                        preflight_futures[
                            executor.submit(
                                self._observe_with_client,
                                "laser_scan",
                                client=supplemental,
                                transport="local_roscli_read_only",
                                include_raw=False,
                            )
                        ] = "laser_scan"
                    if "tf" in available:
                        preflight_futures[
                            executor.submit(supplemental.transform_available, "map", "base_link")
                        ] = "tf_resolution"
                    for future in as_completed(preflight_futures):
                        purpose = preflight_futures[future]
                        try:
                            result = future.result()
                        except Exception as exc:  # noqa: BLE001 - preserve partial evidence
                            failures[purpose] = {
                                "ok": False,
                                "observation": purpose,
                                "error_code": "LIMO_OBSERVATION_FAILED",
                                "error": str(exc),
                                "command_dispatched": False,
                            }
                            continue
                        if purpose == "tf_resolution":
                            tf_chain_resolved = bool(result)
                        elif isinstance(result.get("summary"), dict):
                            observation = purpose
                            summaries[observation] = result["summary"]
                            records[observation] = {
                                "summary": result["summary"],
                                "received_wall_time": result.get("received_wall_time"),
                                "received_monotonic": result.get("received_monotonic"),
                                "transport": result.get("transport"),
                                "transport_generation": result.get("transport_generation"),
                            }

            if available:
                worker_limit = ROSCLI_MAX_CONCURRENT_OBSERVATIONS if candidate == "roscli" else 12
                with ThreadPoolExecutor(max_workers=min(worker_limit, len(available))) as executor:
                    future_names = {
                        executor.submit(
                            self._observe_with_client,
                            observation,
                            client=client,
                            transport=candidate,
                            include_raw=False,
                            sample_count=(
                                100
                                if observation == "tf" and candidate == "rosbridge"
                                else 5
                                if observation == "tf"
                                else 1
                            ),
                        ): observation
                        for observation in available
                    }
                    for future in as_completed(future_names):
                        observation = future_names[future]
                        try:
                            result = future.result()
                        except Exception as exc:  # noqa: BLE001 - preserve partial evidence
                            failures[observation] = {
                                "ok": False,
                                "observation": observation,
                                "error_code": "LIMO_OBSERVATION_FAILED",
                                "error": str(exc),
                                "command_dispatched": False,
                            }
                            continue
                        if observation == "tf" and tf_chain_resolved is not None:
                            result["summary"] = self._supplement_tf_summary(
                                result["summary"],
                                composed_chain_available=tf_chain_resolved,
                            )
                        if isinstance(result.get("summary"), dict):
                            summaries[observation] = result["summary"]
                            records[observation] = {
                                "summary": result["summary"],
                                "received_wall_time": result.get("received_wall_time"),
                                "received_monotonic": result.get("received_monotonic"),
                                "transport": result.get("transport"),
                                "transport_generation": result.get("transport_generation"),
                            }
                            self._observation_hub.remember(
                                observation,
                                transport=candidate,
                                endpoint=endpoint,
                                record=records[observation],
                            )

            return {
                "ok": not failures,
                "requested_observations": observations,
                "summaries": summaries,
                "observation_records": records,
                "failures": failures,
                "available_count": len(summaries),
                "missing_count": len(failures),
                "transport": candidate,
                "transport_generation": getattr(client, "transport_generation", None),
                "transport_components": transport_components,
                "endpoint": endpoint,
                "snapshot_cutoff_monotonic": cutoff_monotonic,
                "cache_hits": sorted(cache_hits),
                "cache": self._observation_hub.stats(),
                "trust_level": "LIVE_ROS_TOPIC_OBSERVATION",
                "command_dispatched": False,
            }
        return {
            "ok": False,
            "requested_observations": observations,
            "summaries": {},
            "observation_records": {},
            "failures": {},
            "available_count": 0,
            "missing_count": len(observations),
            "transport": transport,
            "transport_errors": transport_errors,
            "trust_level": "UNAVAILABLE",
            "command_dispatched": False,
        }

    def base_state(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return self.observe("laser_scan", endpoint, timeout_sec, transport, False)

    def localization_state(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return self._collect_summaries(
            ["tf", "tf_static"],
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )

    def camera_state(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        core = [
            "color_image",
            "color_camera_info",
            "depth_image",
            "depth_camera_info",
        ]
        optional = ["depth_points", "infrared_image", "infrared_camera_info"]
        result = self._collect_summaries(
            [*core, *optional],
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )
        failures = result.get("failures", {})
        summaries = result.get("summaries", {})
        core_failures = {
            name: (
                failures.get(name, {"error_code": "LIMO_CAMERA_STREAM_UNAVAILABLE"})
                if isinstance(failures, dict)
                else {"error_code": "LIMO_CAMERA_STREAM_UNAVAILABLE"}
            )
            for name in core
            if not isinstance(summaries, dict) or name not in summaries
        }
        optional_inactive = [
            name for name in optional if not isinstance(summaries, dict) or name not in summaries
        ]
        return {
            **result,
            "ok": not core_failures,
            "core_ready": not core_failures,
            "core_failures": core_failures,
            "optional_inactive": optional_inactive,
        }

    def peripheral_inventory(self) -> dict[str, Any]:
        return self._peripheral_inspector.inventory()

    def audio_state(self) -> dict[str, Any]:
        return self._peripheral_inspector.audio_state()

    def microphone_level(self, *, duration_sec: int, sample_rate_hz: int) -> dict[str, Any]:
        return self._peripheral_inspector.microphone_level(
            duration_sec=duration_sec,
            sample_rate_hz=sample_rate_hz,
        )

    def display_state(self) -> dict[str, Any]:
        return self._peripheral_inspector.display_state()

    def dabai_device_state(self) -> dict[str, Any]:
        return self._peripheral_inspector.dabai_device_state()

    def platform_health(self) -> dict[str, Any]:
        return self._peripheral_inspector.platform_health()

    def patrol_readiness(
        self,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 10.0,
        transport: str = "auto",
        body_id: str = "limo",
        body_snapshot_hash: str = "",
    ) -> dict[str, Any]:
        key = (endpoint, transport, body_id, body_snapshot_hash)
        with self._readiness_flight_lock:
            flight = self._readiness_flights.get(key)
            if flight is None:
                flight = Future()
                self._readiness_flights[key] = flight
                leader = True
            else:
                leader = False
        if not leader:
            return flight.result(timeout=max(15.0, float(timeout_sec) + 5.0))
        try:
            result = self._build_patrol_readiness(
                endpoint=endpoint,
                timeout_sec=timeout_sec,
                transport=transport,
                body_id=body_id,
                body_snapshot_hash=body_snapshot_hash,
            )
            flight.set_result(result)
            return result
        except BaseException as exc:
            flight.set_exception(exc)
            raise
        finally:
            with self._readiness_flight_lock:
                self._readiness_flights.pop(key, None)

    def _build_patrol_readiness(
        self,
        *,
        endpoint: str,
        timeout_sec: float,
        transport: str,
        body_id: str,
        body_snapshot_hash: str,
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
                "global_costmap",
                "local_costmap",
                "diagnostics",
                "tf",
            ],
            endpoint=endpoint,
            timeout_sec=timeout_sec,
            transport=transport,
        )
        receive_wall_times = [
            float(record["received_wall_time"])
            for record in snapshot["observation_records"].values()
            if isinstance(record.get("received_wall_time"), (int, float))
            and not isinstance(record.get("received_wall_time"), bool)
        ]
        receive_monotonic_times = [
            float(record["received_monotonic"])
            for record in snapshot["observation_records"].values()
            if isinstance(record.get("received_monotonic"), (int, float))
            and not isinstance(record.get("received_monotonic"), bool)
        ]
        snapshot_wall = max(receive_wall_times, default=time.time())
        snapshot_monotonic = max(receive_monotonic_times, default=time.monotonic())
        clock_jump_detected = self._clock_tracker.observe(
            wall_time=snapshot_wall, monotonic_time=snapshot_monotonic
        )
        readiness = build_readiness_evidence(
            snapshot["observation_records"],
            failures=snapshot["failures"],
            body_id=body_id,
            body_snapshot_hash=body_snapshot_hash,
            now_wall=snapshot_wall,
            now_monotonic=snapshot_monotonic,
            clock_jump_detected=clock_jump_detected,
            transport_binding={
                "endpoint": snapshot.get("endpoint", endpoint),
                "transport": snapshot.get("transport", transport),
                "generation": snapshot.get("transport_generation"),
                "components": snapshot.get("transport_components", []),
            },
            observation_cutoff_monotonic=snapshot.get("snapshot_cutoff_monotonic"),
        )
        self._remember_readiness(readiness)
        return {**snapshot, "readiness": readiness, "ok": readiness["ready"]}

    def validate_goal(self, **kwargs: Any) -> dict[str, Any]:
        result = self._goal_validator.validate(**kwargs)
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
            runtime = await self._gateway.runtime_status()
            return {
                **runtime,
                "mcp_process": mcp_process_status(),
                "interaction_plane": interaction_plane_status(runtime),
            }
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return {
                **_control_error("runtime_status", exc),
                "mcp_process": mcp_process_status(),
                "interaction_plane": interaction_plane_status({}),
            }

    async def request_navigation(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        body_snapshot_hash: str,
        readiness_snapshot_hash: str,
        route_policy_id: str = "lab-default",
        goal_tolerance_xy_m: float | None = None,
        goal_tolerance_yaw_rad: float | None = None,
        execution_mode: str = "SHADOW",
        action_id: str | None = None,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        mode = str(execution_mode).upper()
        if mode not in {"SHADOW", "REAL"}:
            return self._navigation_block(
                "INVALID_EXECUTION_MODE", "Only SHADOW or REAL may be submitted to rosclawd."
            )
        if isinstance(wait_timeout_sec, bool) or not 0.0 <= float(wait_timeout_sec) <= 30.0:
            raise ValueError("wait_timeout_sec must be within [0, 30]")

        readiness_reference = self._validate_readiness_reference(
            readiness_snapshot_hash, body_snapshot_hash
        )
        if not readiness_reference["ok"]:
            return readiness_reference
        validation = self._goal_validator.validate(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            route_policy_id=route_policy_id,
            goal_tolerance_xy_m=goal_tolerance_xy_m,
            goal_tolerance_yaw_rad=goal_tolerance_yaw_rad,
        )
        if not validation["ok"]:
            return validation
        if mode == "REAL":
            return self._navigation_block(
                "MCP_OPERATOR_CONFIRMATION_REQUIRED",
                "REAL navigation must use the MCP in-context confirmation flow.",
            )
        goal = validation["normalized_goal"]
        try:
            response = await self._gateway.request_navigation(
                capability_id="limo.navigate_to_pose",
                arguments=navigation_arguments(goal, validation, readiness_snapshot_hash),
                execution_mode=mode,
                body_snapshot_hash=body_snapshot_hash,
                body_id="limo",
                action_id=action_id,
                required_evidence="TASK_VERIFIED",
                timeout_sec=30.0,
                wait_timeout_sec=float(wait_timeout_sec),
            )
            return {**response, "navigation_contract": validation}
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("request_navigation", exc)

    def prepare_navigation_confirmation(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        body_snapshot_hash: str,
        readiness_snapshot_hash: str,
        route_policy_id: str,
        goal_tolerance_xy_m: float | None,
        goal_tolerance_yaw_rad: float | None,
        action_id: str,
        deadline_at: str,
    ) -> dict[str, Any]:
        """Prepare one readiness-bound REAL navigation proposal for MCP elicitation."""

        readiness_reference = self._validate_readiness_reference(
            readiness_snapshot_hash, body_snapshot_hash
        )
        if not readiness_reference["ok"]:
            return readiness_reference
        validation = self._goal_validator.validate(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            route_policy_id=route_policy_id,
            goal_tolerance_xy_m=goal_tolerance_xy_m,
            goal_tolerance_yaw_rad=goal_tolerance_yaw_rad,
        )
        if not validation["ok"]:
            return validation
        goal = validation["normalized_goal"]
        arguments = navigation_arguments(goal, validation, readiness_snapshot_hash)
        try:
            prepared = self._gateway.prepare_operator_action(
                capability_id="limo.navigate_to_pose",
                arguments=arguments,
                body_snapshot_hash=body_snapshot_hash,
                body_id="limo",
                action_id=action_id,
                deadline_at=deadline_at,
                required_evidence="TASK_VERIFIED",
                timeout_sec=120.0,
                display=navigation_display(goal),
            )
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("prepare_navigation_confirmation", exc)
        return {
            "ok": True,
            "decision": "CONFIRMATION_REQUIRED",
            "approval_request": dict(prepared.approval_request),
            "prepared_action": prepared,
            "navigation_contract": validation,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

    async def request_tone(
        self,
        *,
        frequency_hz: int,
        duration_sec: float,
        volume_percent: int,
        body_snapshot_hash: str,
        execution_mode: str,
        action_id: str | None,
        wait_timeout_sec: float,
    ) -> dict[str, Any]:
        mode = str(execution_mode).upper()
        if mode not in {"SHADOW", "REAL"}:
            return self._navigation_block(
                "INVALID_EXECUTION_MODE", "Only SHADOW or REAL may be submitted to rosclawd."
            )
        if isinstance(wait_timeout_sec, bool) or not 0.0 <= float(wait_timeout_sec) <= 30.0:
            raise ValueError("wait_timeout_sec must be within [0, 30]")
        validation = validate_tone(
            frequency_hz=frequency_hz,
            duration_sec=duration_sec,
            volume_percent=volume_percent,
            body_snapshot_hash=body_snapshot_hash,
        )
        if not validation["ok"]:
            return validation
        if mode == "REAL":
            return self._navigation_block(
                "MCP_OPERATOR_CONFIRMATION_REQUIRED",
                "REAL speaker playback must use the MCP in-context confirmation flow.",
            )
        try:
            response = await self._gateway.request_tone(
                capability_id="limo.play_tone",
                arguments=tone_arguments(validation["normalized_tone"]),
                execution_mode=mode,
                body_snapshot_hash=body_snapshot_hash,
                body_id="limo",
                action_id=action_id,
                required_evidence="TASK_VERIFIED",
                timeout_sec=10.0,
                wait_timeout_sec=float(wait_timeout_sec),
            )
            return {**response, "tone_contract": validation}
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("request_tone", exc)

    def prepare_tone_confirmation(
        self,
        *,
        frequency_hz: int,
        duration_sec: float,
        volume_percent: int,
        body_snapshot_hash: str,
        action_id: str,
        deadline_at: str,
    ) -> dict[str, Any]:
        validation = validate_tone(
            frequency_hz=frequency_hz,
            duration_sec=duration_sec,
            volume_percent=volume_percent,
            body_snapshot_hash=body_snapshot_hash,
        )
        if not validation["ok"]:
            return validation
        tone = validation["normalized_tone"]
        try:
            prepared = self._gateway.prepare_operator_action(
                capability_id="limo.play_tone",
                arguments=tone_arguments(tone),
                body_snapshot_hash=body_snapshot_hash,
                body_id="limo",
                action_id=action_id,
                deadline_at=deadline_at,
                required_evidence="TASK_VERIFIED",
                timeout_sec=10.0,
                display=tone_display(tone),
            )
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("prepare_tone_confirmation", exc)
        return {
            "ok": True,
            "decision": "CONFIRMATION_REQUIRED",
            "approval_request": dict(prepared.approval_request),
            "prepared_action": prepared,
            "tone_contract": validation,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

    async def request_speech(
        self,
        *,
        text: str,
        language: str,
        volume_percent: int,
        rate_wpm: int,
        body_snapshot_hash: str,
        execution_mode: str,
        action_id: str | None,
        wait_timeout_sec: float,
    ) -> dict[str, Any]:
        mode = str(execution_mode).upper()
        if mode not in {"SHADOW", "REAL"}:
            return self._navigation_block(
                "INVALID_EXECUTION_MODE", "Only SHADOW or REAL may be submitted to rosclawd."
            )
        if isinstance(wait_timeout_sec, bool) or not 0.0 <= float(wait_timeout_sec) <= 30.0:
            raise ValueError("wait_timeout_sec must be within [0, 30]")
        validation = validate_speech(
            text=text,
            language=language,
            volume_percent=volume_percent,
            rate_wpm=rate_wpm,
            body_snapshot_hash=body_snapshot_hash,
        )
        if not validation["ok"]:
            return validation
        if mode == "REAL":
            return self._navigation_block(
                "MCP_OPERATOR_CONFIRMATION_REQUIRED",
                "REAL speech playback must use the MCP in-context confirmation flow.",
            )
        try:
            response = await self._gateway.request_speech(
                capability_id="limo.speak_text",
                arguments=speech_arguments(validation["normalized_speech"]),
                execution_mode=mode,
                body_snapshot_hash=body_snapshot_hash,
                body_id="limo",
                action_id=action_id,
                required_evidence="TASK_VERIFIED",
                timeout_sec=15.0,
                wait_timeout_sec=float(wait_timeout_sec),
            )
            return {**response, "speech_contract": validation}
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("request_speech", exc)

    def prepare_speech_confirmation(
        self,
        *,
        text: str,
        language: str,
        volume_percent: int,
        rate_wpm: int,
        body_snapshot_hash: str,
        action_id: str,
        deadline_at: str,
    ) -> dict[str, Any]:
        validation = validate_speech(
            text=text,
            language=language,
            volume_percent=volume_percent,
            rate_wpm=rate_wpm,
            body_snapshot_hash=body_snapshot_hash,
        )
        if not validation["ok"]:
            return validation
        speech = validation["normalized_speech"]
        try:
            prepared = self._gateway.prepare_operator_action(
                capability_id="limo.speak_text",
                arguments=speech_arguments(speech),
                body_snapshot_hash=body_snapshot_hash,
                body_id="limo",
                action_id=action_id,
                deadline_at=deadline_at,
                required_evidence="TASK_VERIFIED",
                timeout_sec=15.0,
                display=speech_display(speech),
            )
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("prepare_speech_confirmation", exc)
        return {
            "ok": True,
            "decision": "CONFIRMATION_REQUIRED",
            "approval_request": dict(prepared.approval_request),
            "prepared_action": prepared,
            "speech_contract": validation,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

    async def request_initial_pose(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        body_snapshot_hash: str,
        route_policy_id: str = "lab-default",
        execution_mode: str = "SHADOW",
        action_id: str | None = None,
        deadline_at: str | None = None,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        mode = str(execution_mode).upper()
        if mode not in {"SHADOW", "REAL"}:
            return self._navigation_block(
                "INVALID_EXECUTION_MODE", "Only SHADOW or REAL may be submitted to rosclawd."
            )
        if not str(body_snapshot_hash).strip():
            return self._navigation_block(
                "BODY_SNAPSHOT_REQUIRED",
                "Initial-pose requests require an immutable LIMO body snapshot.",
            )
        if isinstance(wait_timeout_sec, bool) or not 0.0 <= float(wait_timeout_sec) <= 30.0:
            raise ValueError("wait_timeout_sec must be within [0, 30]")
        validation = self._goal_validator.validate(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            route_policy_id=route_policy_id,
        )
        if not validation["ok"]:
            return validation
        if mode == "REAL":
            return self._navigation_block(
                "MCP_OPERATOR_CONFIRMATION_REQUIRED",
                "REAL localization initialization must use the MCP in-context confirmation flow.",
            )
        pose = validation["normalized_goal"]
        try:
            response = await self._gateway.request_initial_pose(
                capability_id="limo.set_initial_pose",
                arguments=initial_pose_arguments(pose, validation),
                execution_mode=mode,
                body_snapshot_hash=body_snapshot_hash,
                body_id="limo",
                action_id=action_id,
                deadline_at=deadline_at,
                required_evidence="TASK_VERIFIED",
                timeout_sec=15.0,
                wait_timeout_sec=float(wait_timeout_sec),
            )
            return {**response, "initial_pose_contract": validation}
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("request_initial_pose", exc)

    def prepare_initial_pose_confirmation(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        frame_id: str,
        body_snapshot_hash: str,
        route_policy_id: str = "lab-default",
        action_id: str,
        deadline_at: str,
    ) -> dict[str, Any]:
        """Prepare a validated one-shot REAL initial-pose proposal for MCP elicitation."""

        if not str(body_snapshot_hash).strip():
            return self._navigation_block(
                "BODY_SNAPSHOT_REQUIRED",
                "Initial-pose requests require an immutable LIMO body snapshot.",
            )
        validation = self._goal_validator.validate(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            route_policy_id=route_policy_id,
        )
        if not validation["ok"]:
            return validation
        pose = validation["normalized_goal"]
        arguments = initial_pose_arguments(pose, validation)
        try:
            prepared = self._gateway.prepare_operator_action(
                capability_id="limo.set_initial_pose",
                arguments=arguments,
                body_snapshot_hash=body_snapshot_hash,
                body_id="limo",
                action_id=action_id,
                deadline_at=deadline_at,
                required_evidence="TASK_VERIFIED",
                timeout_sec=15.0,
                display=initial_pose_display(pose),
            )
        except Exception as exc:  # noqa: BLE001 - MCP boundary returns structured errors
            return _control_error("prepare_initial_pose_confirmation", exc)
        return {
            "ok": True,
            "decision": "CONFIRMATION_REQUIRED",
            "approval_request": dict(prepared.approval_request),
            "prepared_action": prepared,
            "initial_pose_contract": validation,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

    async def complete_real_action(
        self,
        *,
        ctx: Any,
        proposal: dict[str, Any],
        contract_key: str,
        prepared_action: Any,
        approval_request: Mapping[str, Any],
        wait_timeout_sec: float,
    ) -> dict[str, Any]:
        """Delegate all generic interaction behavior to the ROSClaw SDK."""

        if self._interaction is None:
            return _control_error(
                "operator_confirmation",
                RuntimeError(
                    "ROSClaw Interaction SDK is unavailable; install the sibling rosclaw checkout"
                ),
            )
        result = await self._interaction.complete_prepared(
            ctx,
            prepared_action=prepared_action,
            approval_request=approval_request,
            wait_timeout_sec=wait_timeout_sec,
        )
        return {**result, contract_key: proposal.get(contract_key)}

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


def build_mcp_server(
    service: LimoMCPService | None = None,
    *,
    profile: str = "core",
    compat_tools: bool = False,
) -> FastMCP:
    """Build the standalone LIMO MCP server."""

    implementation = service or LimoMCPService()
    mcp = FastMCP(
        "rosclaw-limo",
        instructions=(
            "AgileX LIMO ROS 1 inspection integration. Discover, inspect, sample, and summarize "
            "the base, lidar, localization, navigation, map, diagnostics, TF, camera, audio, "
            "display, and Jetson host contracts. State-changing requests are submitted through "
            "rosclawd."
        ),
    )

    async def complete_real_confirmation(
        ctx: Context[Any, Any, Any],
        proposal: dict[str, Any],
        *,
        contract_key: str,
        wait_timeout_sec: float,
    ) -> dict[str, Any]:
        prepared_action = proposal.pop("prepared_action", None)
        approval_request = proposal.get("approval_request")
        if proposal.get("ok") is not True or prepared_action is None:
            return proposal
        if not isinstance(approval_request, dict):
            return _control_error(
                "prepare_operator_action",
                RuntimeError("ROSClaw returned no operator confirmation request"),
            )
        return await implementation.complete_real_action(
            ctx=ctx,
            proposal=proposal,
            prepared_action=prepared_action,
            approval_request=approval_request,
            wait_timeout_sec=wait_timeout_sec,
            contract_key=contract_key,
        )

    @mcp.tool(
        description="Return the source-backed LIMO ROS interface and safety contract.",
        annotations=READ_ONLY_TOOL,
    )
    def limo_get_contract() -> dict[str, Any]:
        return implementation.get_contract()

    @mcp.tool(
        description=(
            "Return a bounded LIMO operator context with daemon status, readiness, map policy, "
            "and current blockers."
        ),
        annotations=READ_ONLY_TOOL,
    )
    async def limo_get_context(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
        transport: str = "auto",
        body_snapshot_hash: str = "",
    ) -> dict[str, Any]:
        readiness_task = asyncio.to_thread(
            implementation.patrol_readiness,
            endpoint,
            timeout_sec,
            transport,
            "limo",
            body_snapshot_hash,
        )
        readiness_result, runtime_result = await asyncio.gather(
            readiness_task,
            implementation.runtime_status(),
        )
        readiness = readiness_result.get("readiness", {})
        observations = readiness.get("observations", {}) if isinstance(readiness, dict) else {}
        map_observation = (
            observations.get("map_metadata", {}) if isinstance(observations, dict) else {}
        )
        return {
            "ok": bool(readiness_result.get("ok")),
            "schema_version": "limo.context.v1",
            "body": {"body_id": "limo", "snapshot_hash": body_snapshot_hash or None},
            "runtime": {
                key: runtime_result.get(key)
                for key in (
                    "running",
                    "supervision_state",
                    "southbound_owner",
                    "driver_ready",
                    "generation",
                    "mcp_process",
                    "interaction_plane",
                )
                if key in runtime_result
            },
            "readiness": {
                key: readiness.get(key)
                for key in (
                    "state",
                    "ready",
                    "blockers",
                    "warnings",
                    "snapshot_hash",
                    "expires_at",
                    "transport_binding",
                )
                if key in readiness
            },
            "map": {
                "summary_hash": map_observation.get("summary_hash"),
                "policy_id": readiness.get("policy_id") if isinstance(readiness, dict) else None,
                "policy_hash": readiness.get("policy_hash")
                if isinstance(readiness, dict)
                else None,
            },
            "risk_blockers": readiness.get("blockers", []) if isinstance(readiness, dict) else [],
            "recent_receipt_id": None,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }

    @mcp.tool(
        description="List named LIMO ROS observations, optionally filtered by category.",
        annotations=READ_ONLY_TOOL,
    )
    def limo_list_observations(category: str = "all") -> dict[str, Any]:
        return implementation.list_observations(category)

    @mcp.tool(
        description="Read the ROS graph and verify required LIMO topics without moving the robot.",
        annotations=READ_ONLY_TOOL,
    )
    async def limo_probe_ros(
        endpoint: str = "ws://127.0.0.1:9090",
        transport: str = "auto",
        timeout_sec: float = 5.0,
    ) -> dict[str, Any]:
        return await asyncio.to_thread(implementation.probe_ros, endpoint, transport, timeout_sec)

    @mcp.tool(
        description="Read one allowlisted LIMO telemetry topic; no publish primitive is exposed.",
        annotations=READ_ONLY_TOOL,
        meta={"physicalExecution": False},
    )
    async def limo_observe(
        observation: str,
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.laser_summary, endpoint, timeout_sec, transport
        )

    @mcp.tool(description="Read AMCL and fused pose summaries with localization covariance.")
    async def limo_get_localization_state(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
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
        timeout_sec: float = 5.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.transform_state, endpoint, timeout_sec, transport
        )

    @mcp.tool(
        description=(
            "Read all Dabai color, depth, IR, point-cloud, and calibration streams as compact "
            "summaries."
        )
    )
    async def limo_get_camera_state(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 5.0,
        transport: str = "auto",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.camera_state, endpoint, timeout_sec, transport
        )

    @mcp.tool(
        description=(
            "Inventory documented LIMO peripherals against live USB, framebuffer, input, and "
            "ROS evidence without opening serial or motor interfaces."
        )
    )
    async def limo_list_peripherals() -> dict[str, Any]:
        return await asyncio.to_thread(implementation.peripheral_inventory)

    @mcp.tool(
        description=(
            "Read the LIMO USB sound-card playback, capture, speaker gain, microphone gain, and "
            "amplifier-interface state without changing volume."
        )
    )
    async def limo_get_audio_state() -> dict[str, Any]:
        return await asyncio.to_thread(implementation.audio_state)

    @mcp.tool(
        description=(
            "Measure bounded microphone RMS and peak levels in memory; no audio samples are "
            "retained or returned."
        )
    )
    async def limo_measure_microphone(
        duration_sec: int = 1,
        sample_rate_hz: int = 16000,
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.microphone_level,
            duration_sec=duration_sec,
            sample_rate_hz=sample_rate_hz,
        )

    @mcp.tool(
        description=(
            "Read the LIMO rear framebuffer and touchscreen state and report whether the "
            "documented front OLED has a bound software interface."
        )
    )
    async def limo_get_display_state() -> dict[str, Any]:
        return await asyncio.to_thread(implementation.display_state)

    @mcp.tool(
        description=(
            "Read Dabai device identity, firmware/SDK version, IR temperature, and LDP state "
            "through a fixed allowlist of getter-only ROS services."
        )
    )
    async def limo_get_dabai_device_state() -> dict[str, Any]:
        return await asyncio.to_thread(implementation.dabai_device_state)

    @mcp.tool(
        description=(
            "Read Jetson thermal zones, memory, swap, disk, load average, and uptime for patrol "
            "preflight."
        )
    )
    async def limo_get_platform_health() -> dict[str, Any]:
        return await asyncio.to_thread(implementation.platform_health)

    @mcp.tool(
        description=(
            "Collect a patrol preflight snapshot across base, lidar, localization, navigation, "
            "map, and diagnostics topics; this does not dispatch motion."
        )
    )
    async def limo_get_patrol_readiness(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 10.0,
        transport: str = "auto",
        body_id: str = "limo",
        body_snapshot_hash: str = "",
    ) -> dict[str, Any]:
        return await asyncio.to_thread(
            implementation.patrol_readiness,
            endpoint,
            timeout_sec,
            transport,
            body_id,
            body_snapshot_hash,
        )

    @mcp.tool(
        description=(
            "Return one coherent, hash-sealed patrol-readiness snapshot from the bounded "
            "ObservationHub cache."
        ),
        annotations=READ_ONLY_TOOL,
    )
    async def limo_get_readiness(
        endpoint: str = "ws://127.0.0.1:9090",
        timeout_sec: float = 10.0,
        transport: str = "auto",
        body_id: str = "limo",
        body_snapshot_hash: str = "",
        detail: str = "summary",
    ) -> dict[str, Any]:
        result = await asyncio.to_thread(
            implementation.patrol_readiness,
            endpoint,
            timeout_sec,
            transport,
            body_id,
            body_snapshot_hash,
        )
        return compact_readiness(result, detail=detail)

    @mcp.tool(
        description="Validate a LIMO move_base goal without dispatching it.",
        annotations=READ_ONLY_TOOL,
        meta={"physicalExecution": False},
    )
    def limo_validate_navigation_goal(
        x: float,
        y: float,
        yaw: float,
        frame_id: str = "map",
        route_policy_id: str = "lab-default",
        goal_tolerance_xy_m: float | None = None,
        goal_tolerance_yaw_rad: float | None = None,
    ) -> dict[str, Any]:
        return implementation.validate_goal(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            route_policy_id=route_policy_id,
            goal_tolerance_xy_m=goal_tolerance_xy_m,
            goal_tolerance_yaw_rad=goal_tolerance_yaw_rad,
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
            "Submit a validated LIMO move_base goal to rosclawd. REAL runs daemon-owned "
            "preflight and uses an in-context single-action operator confirmation."
        ),
        annotations=GUARDED_ACTION_TOOL,
        meta={"physicalExecution": True, "operatorConfirmation": "internal"},
    )
    async def limo_request_navigation(
        ctx: Context[Any, Any, Any],
        x: float,
        y: float,
        yaw: float,
        body_snapshot_hash: str,
        readiness_snapshot_hash: str,
        frame_id: str = "map",
        route_policy_id: str = "lab-default",
        goal_tolerance_xy_m: float | None = None,
        goal_tolerance_yaw_rad: float | None = None,
        execution_mode: str = "SHADOW",
        action_id: str | None = None,
        deadline_at: str | None = None,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        if str(execution_mode).upper() == "REAL":
            exact_action_id = action_id or f"action_limo_navigation_{uuid.uuid4().hex}"
            exact_deadline = deadline_at or (
                datetime.now(UTC) + timedelta(seconds=150)
            ).isoformat().replace("+00:00", "Z")
            proposal = implementation.prepare_navigation_confirmation(
                x=x,
                y=y,
                yaw=yaw,
                frame_id=frame_id,
                body_snapshot_hash=body_snapshot_hash,
                readiness_snapshot_hash=readiness_snapshot_hash,
                route_policy_id=route_policy_id,
                goal_tolerance_xy_m=goal_tolerance_xy_m,
                goal_tolerance_yaw_rad=goal_tolerance_yaw_rad,
                action_id=exact_action_id,
                deadline_at=exact_deadline,
            )
            return await complete_real_confirmation(
                ctx,
                proposal,
                contract_key="navigation_contract",
                wait_timeout_sec=wait_timeout_sec,
            )
        return await implementation.request_navigation(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            body_snapshot_hash=body_snapshot_hash,
            readiness_snapshot_hash=readiness_snapshot_hash,
            route_policy_id=route_policy_id,
            goal_tolerance_xy_m=goal_tolerance_xy_m,
            goal_tolerance_yaw_rad=goal_tolerance_yaw_rad,
            execution_mode=execution_mode,
            action_id=action_id,
            wait_timeout_sec=wait_timeout_sec,
        )

    @mcp.tool(
        description=(
            "Play one bounded synthesized tone through the LIMO USB speaker via rosclawd. "
            "REAL uses an in-context single-action confirmation and restores mixer state."
        ),
        annotations=GUARDED_ACTION_TOOL,
        meta={"physicalExecution": True, "operatorConfirmation": "internal"},
    )
    async def limo_request_tone(
        ctx: Context[Any, Any, Any],
        body_snapshot_hash: str,
        frequency_hz: int = 660,
        duration_sec: float = 0.6,
        volume_percent: int = 18,
        execution_mode: str = "SHADOW",
        action_id: str | None = None,
        deadline_at: str | None = None,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        if str(execution_mode).upper() == "REAL":
            exact_action_id = action_id or f"action_limo_tone_{uuid.uuid4().hex}"
            exact_deadline = deadline_at or (
                datetime.now(UTC) + timedelta(seconds=120)
            ).isoformat().replace("+00:00", "Z")
            proposal = implementation.prepare_tone_confirmation(
                frequency_hz=frequency_hz,
                duration_sec=duration_sec,
                volume_percent=volume_percent,
                body_snapshot_hash=body_snapshot_hash,
                action_id=exact_action_id,
                deadline_at=exact_deadline,
            )
            return await complete_real_confirmation(
                ctx,
                proposal,
                contract_key="tone_contract",
                wait_timeout_sec=wait_timeout_sec,
            )
        return await implementation.request_tone(
            frequency_hz=frequency_hz,
            duration_sec=duration_sec,
            volume_percent=volume_percent,
            body_snapshot_hash=body_snapshot_hash,
            execution_mode=execution_mode,
            action_id=action_id,
            wait_timeout_sec=wait_timeout_sec,
        )

    @mcp.tool(
        description=(
            "Speak one bounded Chinese or English message through the LIMO USB speaker via "
            "rosclawd. REAL uses in-context confirmation, microphone loopback, and mixer restore."
        ),
        annotations=GUARDED_ACTION_TOOL,
        meta={"physicalExecution": True, "operatorConfirmation": "internal"},
    )
    async def limo_request_speech(
        ctx: Context[Any, Any, Any],
        text: str,
        body_snapshot_hash: str,
        language: Literal["cmn", "en"] = "cmn",
        volume_percent: int = 18,
        rate_wpm: int = 160,
        execution_mode: str = "SHADOW",
        action_id: str | None = None,
        deadline_at: str | None = None,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        if str(execution_mode).upper() == "REAL":
            exact_action_id = action_id or f"action_limo_speech_{uuid.uuid4().hex}"
            exact_deadline = deadline_at or (
                datetime.now(UTC) + timedelta(seconds=120)
            ).isoformat().replace("+00:00", "Z")
            proposal = implementation.prepare_speech_confirmation(
                text=text,
                language=language,
                volume_percent=volume_percent,
                rate_wpm=rate_wpm,
                body_snapshot_hash=body_snapshot_hash,
                action_id=exact_action_id,
                deadline_at=exact_deadline,
            )
            return await complete_real_confirmation(
                ctx,
                proposal,
                contract_key="speech_contract",
                wait_timeout_sec=wait_timeout_sec,
            )
        return await implementation.request_speech(
            text=text,
            language=language,
            volume_percent=volume_percent,
            rate_wpm=rate_wpm,
            body_snapshot_hash=body_snapshot_hash,
            execution_mode=execution_mode,
            action_id=action_id,
            wait_timeout_sec=wait_timeout_sec,
        )

    @mcp.tool(
        description=(
            "Submit a map-validated AMCL initial-pose estimate to rosclawd. The daemon-owned "
            "LIMO executor is the only component allowed to publish /initialpose. REAL mode "
            "uses an in-context operator confirmation; permit identifiers are never inputs."
        ),
        annotations=GUARDED_ACTION_TOOL,
        meta={"physicalExecution": True, "operatorConfirmation": "internal"},
    )
    async def limo_request_initial_pose(
        ctx: Context[Any, Any, Any],
        x: float,
        y: float,
        yaw: float,
        body_snapshot_hash: str,
        frame_id: str = "map",
        route_policy_id: str = "lab-default",
        execution_mode: str = "SHADOW",
        action_id: str | None = None,
        deadline_at: str | None = None,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        if str(execution_mode).upper() == "REAL":
            exact_action_id = action_id or f"action_limo_initial_pose_{uuid.uuid4().hex}"
            exact_deadline = deadline_at or (
                datetime.now(UTC) + timedelta(seconds=120)
            ).isoformat().replace("+00:00", "Z")
            proposal = implementation.prepare_initial_pose_confirmation(
                x=x,
                y=y,
                yaw=yaw,
                frame_id=frame_id,
                body_snapshot_hash=body_snapshot_hash,
                route_policy_id=route_policy_id,
                action_id=exact_action_id,
                deadline_at=exact_deadline,
            )
            return await complete_real_confirmation(
                ctx,
                proposal,
                contract_key="initial_pose_contract",
                wait_timeout_sec=wait_timeout_sec,
            )
        return await implementation.request_initial_pose(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            body_snapshot_hash=body_snapshot_hash,
            route_policy_id=route_policy_id,
            execution_mode=execution_mode,
            action_id=action_id,
            deadline_at=deadline_at,
            wait_timeout_sec=wait_timeout_sec,
        )

    @mcp.tool(description="Read an action state from rosclawd.", annotations=READ_ONLY_TOOL)
    async def limo_get_action_status(action_id: str) -> dict[str, Any]:
        return await implementation.action_status(action_id)

    @mcp.tool(
        description="Read the canonical execution receipt from rosclawd.",
        annotations=READ_ONLY_TOOL,
    )
    async def limo_get_execution_receipt(action_id: str) -> dict[str, Any]:
        return await implementation.execution_receipt(action_id)

    @mcp.tool(
        description=(
            "Request emergency stop through rosclawd. This cannot prove physical motion stopped; "
            "use the certified hardware E-stop whenever motion may be active."
        ),
        annotations=EMERGENCY_TOOL,
        meta={"physicalExecution": True, "operatorConfirmation": "emergency"},
    )
    async def limo_emergency_stop(reason: str) -> dict[str, Any]:
        return await implementation.emergency_stop(reason)

    available = {tool.name for tool in mcp._tool_manager.list_tools()}
    selected = select_tools(profile, available, compat_tools=compat_tools)
    for tool_name in available - selected:
        mcp._tool_manager.remove_tool(tool_name)
    return mcp


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ROSClaw AgileX LIMO MCP server")
    parser.add_argument("--transport", choices=["stdio", "http", "sse"], default="stdio")
    parser.add_argument(
        "--profile",
        choices=["core", "inspection", "full"],
        default=os.environ.get("ROSCLAW_LIMO_MCP_PROFILE", "core"),
    )
    parser.add_argument(
        "--compat-tools",
        action="store_true",
        default=os.environ.get("ROSCLAW_LIMO_MCP_COMPAT_TOOLS", "").lower()
        in {"1", "true", "yes", "on"},
    )
    args = parser.parse_args(argv)
    transport = cast(
        Literal["stdio", "sse", "streamable-http"],
        "streamable-http" if args.transport == "http" else args.transport,
    )
    build_mcp_server(profile=args.profile, compat_tools=args.compat_tools).run(transport=transport)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
