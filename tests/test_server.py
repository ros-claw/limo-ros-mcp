"""Protocol-surface and fail-closed tests for rosclaw-limo-mcp."""

from __future__ import annotations

import json
import time
import tomllib
from pathlib import Path
from threading import Barrier, Lock
from types import SimpleNamespace
from typing import Any

import pytest

from limo_ros_mcp.contract import LIMO_OBSERVATION_TOPICS
from limo_ros_mcp.evidence import seal_snapshot
from limo_ros_mcp.rosbridge import validate_rosbridge_endpoint
from limo_ros_mcp.runtime_info import interaction_plane_status, mcp_process_status
from limo_ros_mcp.server import (
    LimoMCPService,
    build_mcp_server,
)


def test_mcp_process_status_exposes_bounded_restart_evidence() -> None:
    result = mcp_process_status()

    assert result["schema_version"] == "limo.mcp-process.v1"
    assert result["server_name"] == "rosclaw-limo"
    assert result["package_version"] == "0.9.0"
    assert isinstance(result["distribution_version"], str)
    assert isinstance(result["installation_metadata_matches_source"], bool)
    assert isinstance(result["pid"], int)
    assert str(result["instance_id"]).startswith("limo-mcp-")
    assert str(result["startup_source_fingerprint"]).startswith("sha256:")
    assert str(result["current_source_fingerprint"]).startswith("sha256:")
    assert result["source_changed_since_start"] is False
    assert result["restart_required"] is False


def test_interaction_plane_reports_stale_and_current_daemons() -> None:
    stale = interaction_plane_status({"running": True})
    current = interaction_plane_status(
        {"running": True, "operator_proposals": {"pending": 0, "total": 0}}
    )

    assert stale["operator_broker_available"] is False
    assert stale["real_confirmation_ready"] is False
    assert stale["daemon_restart_required"] is True
    assert current["operator_broker_available"] is True
    assert current["real_confirmation_ready"] is True
    assert current["daemon_restart_required"] is False


@pytest.mark.asyncio
async def test_runtime_status_includes_mcp_and_interaction_provenance() -> None:
    class RuntimeGateway:
        async def runtime_status(self) -> dict[str, Any]:
            return {
                "running": True,
                "operator_proposals": {"pending": 0, "total": 0},
            }

    result = await LimoMCPService(gateway=RuntimeGateway()).runtime_status()

    assert result["mcp_process"]["server_name"] == "rosclaw-limo"
    assert result["interaction_plane"]["real_confirmation_ready"] is True


def test_package_and_manifest_versions_match() -> None:
    root = Path(__file__).parents[1]
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    project = tomllib.loads((root / "pyproject.toml").read_text(encoding="utf-8"))

    assert manifest["version"] == project["project"]["version"]
    assert manifest["mcp_tool_count"] == 33


def test_dabai_camera_contract_uses_live_astra_topics() -> None:
    assert LIMO_OBSERVATION_TOPICS["color_image"] == (
        "/camera/color/image_raw",
        "sensor_msgs/Image",
    )
    assert LIMO_OBSERVATION_TOPICS["depth_image"] == (
        "/camera/depth/image_raw",
        "sensor_msgs/Image",
    )
    assert LIMO_OBSERVATION_TOPICS["depth_points"] == (
        "/camera/depth/points",
        "sensor_msgs/PointCloud2",
    )
    assert LIMO_OBSERVATION_TOPICS["infrared_image"] == (
        "/camera/ir/image_raw",
        "sensor_msgs/Image",
    )
    assert LIMO_OBSERVATION_TOPICS["infrared_camera_info"] == (
        "/camera/ir/camera_info",
        "sensor_msgs/CameraInfo",
    )


@pytest.mark.asyncio
async def test_server_exposes_no_raw_ros_publish_tool() -> None:
    server = build_mcp_server(LimoMCPService(gateway=object()), profile="full")
    tools = await server.list_tools()
    names = {tool.name for tool in tools}

    assert names == {
        "limo_get_base_state",
        "limo_get_camera_state",
        "limo_get_contract",
        "limo_get_context",
        "limo_get_audio_state",
        "limo_get_action_status",
        "limo_get_dabai_device_state",
        "limo_get_diagnostics",
        "limo_get_display_state",
        "limo_get_execution_receipt",
        "limo_get_laser_summary",
        "limo_get_localization_state",
        "limo_get_map_summary",
        "limo_get_navigation_state",
        "limo_get_patrol_readiness",
        "limo_get_readiness",
        "limo_get_platform_health",
        "limo_get_runtime_status",
        "limo_get_topic_info",
        "limo_get_transform_state",
        "limo_list_peripherals",
        "limo_list_observations",
        "limo_measure_microphone",
        "limo_observe",
        "limo_probe_ros",
        "limo_request_navigation",
        "limo_request_initial_pose",
        "limo_request_tone",
        "limo_request_speech",
        "limo_sample_topic",
        "limo_emergency_stop",
        "limo_validate_navigation_goal",
        "limo_validate_velocity_command",
    }
    assert not any("publish" in name or "cmd_vel" in name for name in names)
    schemas = {tool.name: tool.inputSchema for tool in tools}
    for name in {
        "limo_probe_ros",
        "limo_observe",
        "limo_get_topic_info",
        "limo_sample_topic",
        "limo_get_base_state",
        "limo_get_camera_state",
        "limo_get_laser_summary",
        "limo_get_localization_state",
        "limo_get_navigation_state",
        "limo_get_map_summary",
        "limo_get_diagnostics",
        "limo_get_transform_state",
    }:
        assert schemas[name]["properties"]["timeout_sec"]["default"] == 5.0
    assert schemas["limo_get_patrol_readiness"]["properties"]["timeout_sec"]["default"] == 10.0
    assert schemas["limo_get_readiness"]["properties"]["timeout_sec"]["default"] == 10.0
    navigation_properties = schemas["limo_request_navigation"]["properties"]
    assert navigation_properties["wait_timeout_sec"]["default"] == 2.0
    assert "readiness_snapshot_hash" in schemas["limo_request_navigation"]["required"]
    assert "body_snapshot_hash" in schemas["limo_request_navigation"]["required"]
    assert {
        "localization_ready",
        "costmap_ready",
        "obstacle_check_enabled",
    }.isdisjoint(navigation_properties)


@pytest.mark.asyncio
async def test_core_and_inspection_profiles_bound_tool_discovery() -> None:
    core = await build_mcp_server(LimoMCPService(gateway=object())).list_tools()
    inspection = await build_mcp_server(
        LimoMCPService(gateway=object()), profile="inspection"
    ).list_tools()
    compat = await build_mcp_server(
        LimoMCPService(gateway=object()), compat_tools=True
    ).list_tools()

    core_names = {tool.name for tool in core}
    inspection_names = {tool.name for tool in inspection}
    compat_names = {tool.name for tool in compat}
    assert len(core_names) == 11
    assert "limo_get_context" in core_names
    assert "limo_get_readiness" in core_names
    assert "limo_get_patrol_readiness" not in core_names
    assert "limo_get_audio_state" in inspection_names
    assert "limo_request_navigation" not in inspection_names
    assert "limo_get_patrol_readiness" in compat_names

    annotations = {tool.name: tool.annotations for tool in core}
    assert annotations["limo_get_context"].readOnlyHint is True
    assert annotations["limo_request_navigation"].destructiveHint is True


def test_rosbridge_endpoint_defaults_to_loopback_only(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("ROSCLAW_LIMO_ROSBRIDGE_ALLOWLIST", raising=False)
    assert validate_rosbridge_endpoint("ws://127.0.0.1:9090") == "ws://127.0.0.1:9090"
    with pytest.raises(ValueError, match="not allowlisted"):
        validate_rosbridge_endpoint("ws://robot.example:9090")


def test_rosbridge_endpoint_honors_operator_allowlist(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ROSCLAW_LIMO_ROSBRIDGE_ALLOWLIST", "limo.local")
    assert validate_rosbridge_endpoint("wss://limo.local:9090") == "wss://limo.local:9090"


@pytest.mark.parametrize(
    "observation",
    [
        "color_image",
        "infrared_image",
        "depth_points",
        "map",
        "global_costmap",
        "local_costmap",
    ],
)
def test_large_binary_raw_observations_are_denied(observation: str) -> None:
    service = LimoMCPService(gateway=object())

    with pytest.raises(ValueError, match="include_raw is denied"):
        service.observe(observation, include_raw=True)
    with pytest.raises(ValueError, match="include_raw is denied"):
        service.sample(observation, include_raw=True)


def test_camera_state_keeps_inactive_ir_optional(monkeypatch: pytest.MonkeyPatch) -> None:
    service = LimoMCPService(gateway=object())
    core = {
        name: {"width": 640, "height": 480}
        for name in (
            "color_image",
            "color_camera_info",
            "depth_image",
            "depth_camera_info",
        )
    }
    monkeypatch.setattr(
        service,
        "_collect_summaries",
        lambda *_args, **_kwargs: {
            "ok": False,
            "summaries": core,
            "failures": {
                "depth_points": {"error_code": "LIMO_OBSERVATION_FAILED"},
                "infrared_image": {"error_code": "LIMO_OBSERVATION_FAILED"},
                "infrared_camera_info": {"error_code": "LIMO_OBSERVATION_FAILED"},
            },
        },
    )

    result = service.camera_state()

    assert result["ok"] is True
    assert result["core_ready"] is True
    assert result["core_failures"] == {}
    assert result["optional_inactive"] == [
        "depth_points",
        "infrared_image",
        "infrared_camera_info",
    ]


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


def test_rosbridge_readiness_bounds_tf_sampling_window(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeRosbridgeClient:
        transport_generation = "rosbridge-shared"

        def probe(self) -> dict[str, Any]:
            return {
                "topics": ["/tf"],
                "types": ["tf2_msgs/TFMessage"],
                "nodes": [],
            }

        def subscribe_many(
            self, _topic: str, _message_type: str, *, count: int
        ) -> list[dict[str, Any]]:
            assert count == 5
            return [
                {
                    "transforms": [
                        {
                            "header": {"frame_id": "map" if index == 0 else "odom"},
                            "child_frame_id": "odom" if index == 0 else "base_link",
                        }
                    ]
                }
                for index in range(count)
            ]

    client = FakeRosbridgeClient()
    monkeypatch.setattr(
        LimoMCPService,
        "_client",
        staticmethod(lambda *_args, **_kwargs: client),
    )

    result = LimoMCPService(gateway=object())._collect_summaries(
        ["tf"],
        endpoint="ws://127.0.0.1:9090",
        timeout_sec=2.0,
        transport="rosbridge",
    )

    assert result["ok"] is True
    assert result["summaries"]["tf"]["transform_count"] == 2


def test_rosbridge_readiness_uses_noarr_cli_summary_for_global_costmap(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeRosbridgeClient:
        transport_generation = "rosbridge-primary"

        def probe(self) -> dict[str, Any]:
            return {
                "topics": ["/move_base/global_costmap/costmap"],
                "types": ["nav_msgs/OccupancyGrid"],
                "nodes": [],
            }

    class FakeRoscliClient:
        transport_generation = "roscli-summary"

        def subscribe_many(
            self, _topic: str, _message_type: str, *, count: int
        ) -> list[dict[str, Any]]:
            assert count == 1
            return [
                {
                    "header": {"stamp": {"secs": 10, "nsecs": 0}, "frame_id": "map"},
                    "info": {"resolution": 0.05, "width": 1984, "height": 1984},
                    "data": "<array type: int8, length: 3936256>",
                }
            ]

    rosbridge = FakeRosbridgeClient()
    roscli = FakeRoscliClient()
    monkeypatch.setattr(
        LimoMCPService,
        "_client",
        staticmethod(
            lambda candidate, *_args, **_kwargs: rosbridge if candidate == "rosbridge" else roscli
        ),
    )
    monkeypatch.setattr(
        "limo_ros_mcp.server.RosCliReadOnlyClient", lambda *_args, **_kwargs: roscli
    )

    result = LimoMCPService(gateway=object())._collect_summaries(
        ["global_costmap"],
        endpoint="ws://127.0.0.1:9090",
        timeout_sec=5.0,
        transport="rosbridge",
    )

    assert result["ok"] is True
    assert result["observation_records"]["global_costmap"]["transport"] == (
        "local_roscli_read_only"
    )
    assert result["transport_components"] == [
        {"transport": "rosbridge", "generation": "rosbridge-primary"},
        {
            "transport": "local_roscli_read_only",
            "generation": "roscli-summary",
            "purpose": "costmap_laser_and_tf_readiness_evidence",
        },
    ]


def test_rosbridge_readiness_supplements_critical_tf_edges(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeRosbridgeClient:
        transport_generation = "rosbridge-primary"

        def probe(self) -> dict[str, Any]:
            return {"topics": ["/tf"], "types": ["tf2_msgs/TFMessage"], "nodes": []}

        def subscribe_many(
            self, _topic: str, _message_type: str, *, count: int
        ) -> list[dict[str, Any]]:
            return [
                {
                    "transforms": [
                        {
                            "header": {"frame_id": "/base_link"},
                            "child_frame_id": "/imu_link",
                        }
                    ]
                }
                for _ in range(count)
            ]

    class FakeRoscliClient:
        transport_generation = "roscli-tf"

        def transform_available(self, parent: str, child: str) -> bool:
            return (parent, child) == ("map", "base_link")

    rosbridge = FakeRosbridgeClient()
    roscli = FakeRoscliClient()
    monkeypatch.setattr(
        LimoMCPService,
        "_client",
        staticmethod(lambda *_args, **_kwargs: rosbridge),
    )
    monkeypatch.setattr(
        "limo_ros_mcp.server.RosCliReadOnlyClient", lambda *_args, **_kwargs: roscli
    )

    result = LimoMCPService(gateway=object())._collect_summaries(
        ["tf"],
        endpoint="ws://127.0.0.1:9090",
        timeout_sec=5.0,
        transport="rosbridge",
    )

    edges = {
        (item["parent_frame"].lstrip("/"), item["child_frame"].lstrip("/"))
        for item in result["summaries"]["tf"]["transforms"]
    }
    assert ("map", "base_link") in edges


def test_rosbridge_readiness_collects_laser_through_bounded_roscli(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeRosbridgeClient:
        transport_generation = "rosbridge-primary"

        def probe(self) -> dict[str, Any]:
            return {"topics": ["/scan"], "types": ["sensor_msgs/LaserScan"], "nodes": []}

        def subscribe_many(self, *_args: Any, **_kwargs: Any) -> list[dict[str, Any]]:
            raise AssertionError("readiness must not transfer LaserScan through rosbridge")

    class FakeRoscliClient:
        transport_generation = "roscli-laser"

        def subscribe_many(
            self, _topic: str, _message_type: str, *, count: int
        ) -> list[dict[str, Any]]:
            assert count == 1
            return [
                {
                    "header": {"stamp": {"secs": 10, "nsecs": 0}, "frame_id": "laser_link"},
                    "angle_min": -1.0,
                    "angle_increment": 1.0,
                    "range_min": 0.1,
                    "range_max": 12.0,
                    "ranges": [1.0, 2.0, 3.0],
                }
            ]

    rosbridge = FakeRosbridgeClient()
    roscli = FakeRoscliClient()
    monkeypatch.setattr(
        LimoMCPService,
        "_client",
        staticmethod(lambda *_args, **_kwargs: rosbridge),
    )
    monkeypatch.setattr(
        "limo_ros_mcp.server.RosCliReadOnlyClient", lambda *_args, **_kwargs: roscli
    )

    result = LimoMCPService(gateway=object())._collect_summaries(
        ["laser_scan"],
        endpoint="ws://127.0.0.1:9090",
        timeout_sec=5.0,
        transport="rosbridge",
    )

    assert result["ok"] is True
    assert result["summaries"]["laser_scan"]["sample_count"] == 3
    assert result["observation_records"]["laser_scan"]["transport"] == (
        "local_roscli_read_only"
    )


def test_rosbridge_readiness_uses_parallel_slow_workers_then_one_batch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class FakeRosbridgeClient:
        transport_generation = "rosbridge-primary"

        def __init__(self) -> None:
            self.batch_calls = 0

        def probe(self) -> dict[str, Any]:
            return {
                "topics": ["/move_base/global_costmap/costmap", "/scan", "/limo_status"],
                "types": [
                    "nav_msgs/OccupancyGrid",
                    "sensor_msgs/LaserScan",
                    "limo_base/LimoStatus",
                ],
                "nodes": [],
            }

        def subscribe_batch(self, topic_types: dict[str, str]) -> dict[str, dict[str, Any]]:
            self.batch_calls += 1
            assert topic_types == {"/limo_status": "limo_base/LimoStatus"}
            return {
                "/limo_status": {
                    "message": {"battery_voltage": 12.0, "error_code": 0, "motion_mode": 1},
                    "received_wall_time": time.time(),
                    "received_monotonic": time.monotonic(),
                }
            }

    class FakeRoscliClient:
        transport_generation = "roscli-parallel"

        def __init__(self) -> None:
            self.slow_workers = Barrier(2, timeout=1.0)

        def subscribe_many(
            self, topic: str, _message_type: str, *, count: int
        ) -> list[dict[str, Any]]:
            assert count == 1
            self.slow_workers.wait()
            if topic == "/scan":
                return [
                    {
                        "angle_min": -1.0,
                        "angle_increment": 1.0,
                        "range_min": 0.1,
                        "range_max": 12.0,
                        "ranges": [1.0, 2.0, 3.0],
                    }
                ]
            return [{"info": {"resolution": 0.05, "width": 20, "height": 20}}]

    rosbridge = FakeRosbridgeClient()
    roscli = FakeRoscliClient()
    monkeypatch.setattr(
        LimoMCPService,
        "_client",
        staticmethod(lambda *_args, **_kwargs: rosbridge),
    )
    monkeypatch.setattr(
        "limo_ros_mcp.server.RosCliReadOnlyClient", lambda *_args, **_kwargs: roscli
    )

    result = LimoMCPService(gateway=object())._collect_summaries(
        ["global_costmap", "laser_scan", "status"],
        endpoint="ws://127.0.0.1:9090",
        timeout_sec=5.0,
        transport="rosbridge",
    )

    assert result["ok"] is True
    assert set(result["summaries"]) == {"global_costmap", "laser_scan", "status"}
    assert rosbridge.batch_calls == 1


def test_roscli_readiness_collection_limits_process_fanout(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class CountingClient:
        transport_generation = "roscli-bounded"

        def __init__(self) -> None:
            self.active = 0
            self.peak = 0
            self.lock = Lock()

        def probe(self) -> dict[str, Any]:
            observations = [
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
            ]
            topics = [LIMO_OBSERVATION_TOPICS[name][0] for name in observations]
            types = [LIMO_OBSERVATION_TOPICS[name][1] for name in observations]
            return {"topics": topics, "types": types, "nodes": []}

        def subscribe_many(
            self, topic: str, _message_type: str, *, count: int
        ) -> list[dict[str, Any]]:
            del topic, count
            with self.lock:
                self.active += 1
                self.peak = max(self.peak, self.active)
            try:
                time.sleep(0.02)
                return [{}]
            finally:
                with self.lock:
                    self.active -= 1

    client = CountingClient()
    monkeypatch.setattr(
        LimoMCPService,
        "_client",
        staticmethod(lambda *_args, **_kwargs: client),
    )

    result = LimoMCPService(gateway=object())._collect_summaries(
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
        ],
        endpoint="ws://127.0.0.1:9090",
        timeout_sec=2.0,
        transport="roscli",
    )

    assert result["available_count"] == 10
    # Thread scheduling need not saturate the executor on every host; the
    # invariant is bounded fanout with observable concurrency, not peak=8.
    assert 1 < client.peak <= 8


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

    async def request_initial_pose(self, **kwargs: Any) -> dict[str, Any]:
        return await self.request_navigation(**kwargs)

    async def request_tone(self, **kwargs: Any) -> dict[str, Any]:
        return await self.request_navigation(**kwargs)

    async def request_speech(self, **kwargs: Any) -> dict[str, Any]:
        return await self.request_navigation(**kwargs)

    def prepare_operator_action(self, **kwargs: Any) -> Any:
        self.calls.append({"operation": "prepare", **kwargs})
        return SimpleNamespace(
            approval_request={
                "schema_version": "rosclaw.operator_confirmation.v1",
                "action_id": kwargs["action_id"],
                "action_intent_hash": "sha256:exact-action",
                "body_id": "limo",
                "capability_id": kwargs["capability_id"],
                "execution_mode": "REAL",
                "deadline_at": kwargs["deadline_at"],
                "display": kwargs["display"],
            }
        )

    async def confirm_operator_action(self, prepared: Any, **kwargs: Any) -> dict[str, Any]:
        self.calls.append({"operation": "confirm", "prepared": prepared, **kwargs})
        return {
            "ok": True,
            "state": "QUEUED",
            "action_id": prepared.approval_request["action_id"],
            "operator_confirmation": {
                "accepted": True,
                "permit_injected": True,
                "permit_exposed": False,
            },
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
            "ready": state != "BLOCKED",
            "blockers": ["test-blocker"] if state == "BLOCKED" else [],
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
async def test_request_initial_pose_submits_exact_shadow_contract() -> None:
    gateway = FakeGateway()
    service = LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator())
    result = await service.request_initial_pose(
        x=0.75,
        y=-1.25,
        yaw=0.35,
        frame_id="map",
        body_snapshot_hash="sha256:test-body-snapshot",
        execution_mode="SHADOW",
        action_id="action-limo-initial-pose",
        deadline_at="2030-01-02T03:04:05Z",
        wait_timeout_sec=0.0,
    )
    assert result["state"] == "QUEUED"
    call = gateway.calls[0]
    assert call["capability_id"] == "limo.set_initial_pose"
    assert call["execution_mode"] == "SHADOW"
    assert call["deadline_at"] == "2030-01-02T03:04:05Z"
    assert call["arguments"]["schema_version"] == "limo.initial-pose.v1"
    assert call["arguments"]["target_pose"]["frame_id"] == "map"
    assert call["arguments"]["covariance_diagonal"] == [
        0.25,
        0.25,
        0.0,
        0.0,
        0.0,
        0.0685,
    ]
    assert call["arguments"]["expected_effect"]["map_to_odom_required"] is True


@pytest.mark.asyncio
async def test_direct_real_initial_pose_requires_contextual_interaction() -> None:
    gateway = FakeGateway()
    service = LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator())
    result = await service.request_initial_pose(
        x=0.0,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        body_snapshot_hash="sha256:test-body-snapshot",
        execution_mode="REAL",
    )
    assert result["error_code"] == "MCP_OPERATOR_CONFIRMATION_REQUIRED"
    assert gateway.calls == []


class FakeElicitationContext:
    request_id = "mcp-request-1"

    def __init__(self, *, accepted: bool) -> None:
        self.accepted = accepted
        self.messages: list[str] = []
        self.schemas: list[dict[str, Any]] = []
        self.client_params = SimpleNamespace(
            capabilities=SimpleNamespace(elicitation=SimpleNamespace(form=SimpleNamespace()))
        )
        self.request_context = SimpleNamespace(session=self)

    async def elicit_form(
        self,
        *,
        message: str,
        related_request_id: str,
        **kwargs: Any,
    ) -> Any:
        self.messages.append(message)
        self.schemas.append(kwargs["requestedSchema"])
        assert related_request_id == self.request_id
        return SimpleNamespace(
            action="accept" if self.accepted else "decline",
            content={} if self.accepted else None,
        )


class FakeUnsupportedElicitationContext:
    request_id = "mcp-request-no-elicitation"
    request_context = SimpleNamespace(
        session=SimpleNamespace(
            client_params=SimpleNamespace(capabilities=SimpleNamespace(elicitation=None))
        )
    )

    async def elicit(self, *, message: str, schema: Any) -> Any:
        raise AssertionError("elicit must not be called for an unsupported client")


@pytest.mark.asyncio
async def test_real_initial_pose_uses_elicitation_and_hides_permit_input() -> None:
    gateway = FakeGateway()
    server = build_mcp_server(LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator()))
    tools = {tool.name: tool for tool in await server.list_tools()}
    schema = tools["limo_request_initial_pose"].inputSchema
    assert "approval_id" not in schema["properties"]
    assert "principal_id" not in schema["properties"]

    context = FakeElicitationContext(accepted=True)
    result = await server._tool_manager.call_tool(
        "limo_request_initial_pose",
        {
            "x": 0.75,
            "y": -1.25,
            "yaw": 0.35,
            "body_snapshot_hash": "sha256:test-body-snapshot",
            "execution_mode": "REAL",
            "action_id": "action-interactive-initial-pose",
            "deadline_at": "2030-01-02T03:04:05Z",
            "wait_timeout_sec": 0.0,
        },
        context=context,
        convert_result=False,
    )

    assert result["state"] == "QUEUED"
    assert result["interaction"]["decision"] == "CONFIRMED"
    assert "permit" not in str(result).lower()
    assert "action_intent_hash" not in str(result)
    assert context.messages and "target_pose" in context.messages[0]
    assert context.schemas == [{"type": "object", "properties": {}}]
    assert [call["operation"] for call in gateway.calls] == ["prepare", "confirm"]


@pytest.mark.asyncio
async def test_declined_initial_pose_confirmation_never_submits() -> None:
    gateway = FakeGateway()
    server = build_mcp_server(LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator()))
    result = await server._tool_manager.call_tool(
        "limo_request_initial_pose",
        {
            "x": 0.75,
            "y": -1.25,
            "yaw": 0.35,
            "body_snapshot_hash": "sha256:test-body-snapshot",
            "execution_mode": "REAL",
            "action_id": "action-declined-initial-pose",
            "deadline_at": "2030-01-02T03:04:05Z",
        },
        context=FakeElicitationContext(accepted=False),
        convert_result=False,
    )

    assert result["error_code"] == "OPERATOR_CONFIRMATION_DECLINED"
    assert result["command_dispatched"] is False
    assert [call["operation"] for call in gateway.calls] == ["prepare"]


@pytest.mark.asyncio
async def test_real_initial_pose_fails_fast_without_form_elicitation() -> None:
    gateway = FakeGateway()
    server = build_mcp_server(LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator()))
    result = await server._tool_manager.call_tool(
        "limo_request_initial_pose",
        {
            "x": 0.75,
            "y": -1.25,
            "yaw": 0.35,
            "body_snapshot_hash": "sha256:test-body-snapshot",
            "execution_mode": "REAL",
            "action_id": "action-no-elicitation-initial-pose",
            "deadline_at": "2030-01-02T03:04:05Z",
        },
        context=FakeUnsupportedElicitationContext(),
        convert_result=False,
    )

    assert result["error_code"] == "APPROVAL_CHANNEL_UNAVAILABLE"
    assert result["command_dispatched"] is False
    assert "approval_request" not in result
    assert [call["operation"] for call in gateway.calls] == ["prepare"]


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
async def test_request_navigation_accepts_degraded_snapshot_without_blockers() -> None:
    gateway = FakeGateway()
    service, readiness_hash = _service_with_readiness(gateway, state="DEGRADED")

    result = await service.request_navigation(
        x=0.3,
        y=0.0,
        yaw=0.0,
        frame_id="map",
        body_snapshot_hash="sha256:test-body-snapshot",
        readiness_snapshot_hash=readiness_hash,
        execution_mode="SHADOW",
        action_id="action-limo-degraded-shadow",
        wait_timeout_sec=0.0,
    )

    assert result["state"] == "QUEUED"
    assert gateway.calls[0]["arguments"]["readiness_snapshot_hash"] == readiness_hash


@pytest.mark.asyncio
async def test_real_navigation_requires_in_context_operator_confirmation() -> None:
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

    assert result["error_code"] == "MCP_OPERATOR_CONFIRMATION_REQUIRED"
    assert result["command_dispatched"] is False
    assert gateway.calls == []


@pytest.mark.asyncio
async def test_real_navigation_uses_elicitation_and_hides_permit_input() -> None:
    gateway = FakeGateway()
    service, readiness_hash = _service_with_readiness(gateway)
    server = build_mcp_server(service)
    tools = {tool.name: tool for tool in await server.list_tools()}
    schema = tools["limo_request_navigation"].inputSchema
    assert "approval_id" not in schema["properties"]
    assert "principal_id" not in schema["properties"]

    context = FakeElicitationContext(accepted=True)
    result = await server._tool_manager.call_tool(
        "limo_request_navigation",
        {
            "x": 1.0,
            "y": 0.0,
            "yaw": 0.0,
            "body_snapshot_hash": "sha256:test-body-snapshot",
            "readiness_snapshot_hash": readiness_hash,
            "execution_mode": "REAL",
            "action_id": "action-interactive-navigation",
            "deadline_at": "2030-01-02T03:04:05Z",
            "wait_timeout_sec": 0.0,
        },
        context=context,
        convert_result=False,
    )

    assert result["state"] == "QUEUED"
    assert result["interaction"]["decision"] == "CONFIRMED"
    assert "permit" not in str(result).lower()
    assert context.messages and "mobile base will move" in context.messages[0]
    assert [call["operation"] for call in gateway.calls] == ["prepare", "confirm"]
    assert gateway.calls[0]["capability_id"] == "limo.navigate_to_pose"


@pytest.mark.asyncio
async def test_tone_shadow_and_real_confirmation_contracts() -> None:
    gateway = FakeGateway()
    service = LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator())
    shadow = await service.request_tone(
        frequency_hz=660,
        duration_sec=0.6,
        volume_percent=18,
        body_snapshot_hash="sha256:test-body-snapshot",
        execution_mode="SHADOW",
        action_id="action-tone-shadow",
        wait_timeout_sec=0.0,
    )
    assert shadow["state"] == "QUEUED"
    assert gateway.calls[0]["capability_id"] == "limo.play_tone"
    assert gateway.calls[0]["arguments"]["expected_effect"]["mixer_restore_required"] is True

    gateway.calls.clear()
    server = build_mcp_server(service)
    tools = {tool.name: tool for tool in await server.list_tools()}
    schema = tools["limo_request_tone"].inputSchema
    assert "approval_id" not in schema["properties"]
    assert "principal_id" not in schema["properties"]
    result = await server._tool_manager.call_tool(
        "limo_request_tone",
        {
            "body_snapshot_hash": "sha256:test-body-snapshot",
            "frequency_hz": 660,
            "duration_sec": 0.6,
            "volume_percent": 18,
            "execution_mode": "REAL",
            "action_id": "action-tone-real",
            "deadline_at": "2030-01-02T03:04:05Z",
            "wait_timeout_sec": 0.0,
        },
        context=FakeElicitationContext(accepted=True),
        convert_result=False,
    )

    assert result["state"] == "QUEUED"
    assert result["interaction"]["decision"] == "CONFIRMED"
    assert "permit" not in str(result).lower()
    assert [call["operation"] for call in gateway.calls] == ["prepare", "confirm"]
    assert gateway.calls[0]["capability_id"] == "limo.play_tone"


@pytest.mark.asyncio
async def test_speech_shadow_and_real_confirmation_contracts() -> None:
    gateway = FakeGateway()
    service = LimoMCPService(gateway=gateway, goal_validator=FakeGoalValidator())
    shadow = await service.request_speech(
        text="你好，我是 LIMO 巡检机器人。",
        language="cmn",
        volume_percent=18,
        rate_wpm=160,
        body_snapshot_hash="sha256:test-body-snapshot",
        execution_mode="SHADOW",
        action_id="action-speech-shadow",
        wait_timeout_sec=0.0,
    )
    assert shadow["state"] == "QUEUED"
    assert gateway.calls[0]["capability_id"] == "limo.speak_text"
    assert gateway.calls[0]["arguments"]["expected_effect"] == {
        "kind": "speaker_speech",
        "playback_required": True,
        "mixer_restore_required": True,
        "microphone_loopback_required": True,
        "content_recognition_required": False,
    }

    gateway.calls.clear()
    server = build_mcp_server(service)
    tools = {tool.name: tool for tool in await server.list_tools()}
    schema = tools["limo_request_speech"].inputSchema
    assert "approval_id" not in schema["properties"]
    assert schema["properties"]["language"]["default"] == "cmn"
    result = await server._tool_manager.call_tool(
        "limo_request_speech",
        {
            "text": "你好，我是 LIMO 巡检机器人。",
            "body_snapshot_hash": "sha256:test-body-snapshot",
            "language": "cmn",
            "volume_percent": 18,
            "rate_wpm": 160,
            "execution_mode": "REAL",
            "action_id": "action-speech-real",
            "deadline_at": "2030-01-02T03:04:05Z",
            "wait_timeout_sec": 0.0,
        },
        context=FakeElicitationContext(accepted=True),
        convert_result=False,
    )

    assert result["state"] == "QUEUED"
    assert result["interaction"]["decision"] == "CONFIRMED"
    assert [call["operation"] for call in gateway.calls] == ["prepare", "confirm"]
    assert gateway.calls[0]["capability_id"] == "limo.speak_text"


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
