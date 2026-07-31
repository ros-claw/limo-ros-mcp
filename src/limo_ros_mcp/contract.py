"""Static, source-backed ROS contract for the AgileX LIMO inspection stack."""

from __future__ import annotations

import copy
import math
from typing import Any

LIMO_OBSERVATION_SPECS: dict[str, dict[str, Any]] = {
    "status": {
        "topic": "/limo_status",
        "message_type": "limo_base/LimoStatus",
        "category": "base",
        "required": True,
    },
    "odometry": {
        "topic": "/odom",
        "message_type": "nav_msgs/Odometry",
        "category": "base",
        "required": True,
    },
    "imu": {
        "topic": "/imu",
        "message_type": "sensor_msgs/Imu",
        "category": "base",
        "required": True,
    },
    "laser_scan": {
        "topic": "/scan",
        "message_type": "sensor_msgs/LaserScan",
        "category": "perception",
        "required": False,
        "patrol_required": True,
    },
    "localized_pose": {
        "topic": "/amcl_pose",
        "message_type": "geometry_msgs/PoseWithCovarianceStamped",
        "category": "localization",
        "required": False,
        "patrol_required": True,
    },
    "fused_pose": {
        "topic": "/robot_pose_ekf/odom_combined",
        "message_type": "geometry_msgs/PoseWithCovarianceStamped",
        "category": "localization",
        "required": False,
    },
    "navigation_status": {
        "topic": "/move_base/status",
        "message_type": "actionlib_msgs/GoalStatusArray",
        "category": "navigation",
        "required": False,
        "patrol_required": True,
    },
    "navigation_feedback": {
        "topic": "/move_base/feedback",
        "message_type": "move_base_msgs/MoveBaseActionFeedback",
        "category": "navigation",
        "required": False,
    },
    "navigation_result": {
        "topic": "/move_base/result",
        "message_type": "move_base_msgs/MoveBaseActionResult",
        "category": "navigation",
        "required": False,
    },
    "current_goal": {
        "topic": "/move_base/current_goal",
        "message_type": "geometry_msgs/PoseStamped",
        "category": "navigation",
        "required": False,
    },
    "global_plan": {
        "topic": "/move_base/GlobalPlanner/plan",
        "message_type": "nav_msgs/Path",
        "category": "navigation",
        "required": False,
    },
    "local_plan": {
        "topic": "/move_base/TrajectoryPlannerROS/local_plan",
        "message_type": "nav_msgs/Path",
        "category": "navigation",
        "required": False,
    },
    "map_metadata": {
        "topic": "/map_metadata",
        "message_type": "nav_msgs/MapMetaData",
        "category": "mapping",
        "required": False,
        "patrol_required": True,
    },
    "map": {
        "topic": "/map",
        "message_type": "nav_msgs/OccupancyGrid",
        "category": "mapping",
        "required": False,
    },
    "global_costmap": {
        "topic": "/move_base/global_costmap/costmap",
        "message_type": "nav_msgs/OccupancyGrid",
        "category": "mapping",
        "required": False,
    },
    "local_costmap": {
        "topic": "/move_base/local_costmap/costmap",
        "message_type": "nav_msgs/OccupancyGrid",
        "category": "mapping",
        "required": False,
    },
    "diagnostics": {
        "topic": "/diagnostics",
        "message_type": "diagnostic_msgs/DiagnosticArray",
        "category": "diagnostics",
        "required": False,
    },
    "tf": {
        "topic": "/tf",
        "message_type": "tf2_msgs/TFMessage",
        "category": "transforms",
        "required": False,
    },
    "tf_static": {
        "topic": "/tf_static",
        "message_type": "tf2_msgs/TFMessage",
        "category": "transforms",
        "required": False,
    },
    "ros_logs": {
        "topic": "/rosout_agg",
        "message_type": "rosgraph_msgs/Log",
        "category": "diagnostics",
        "required": False,
    },
    "color_image": {
        "topic": "/camera/color/image_raw",
        "message_type": "sensor_msgs/Image",
        "category": "camera",
        "required": False,
    },
    "color_camera_info": {
        "topic": "/camera/color/camera_info",
        "message_type": "sensor_msgs/CameraInfo",
        "category": "camera",
        "required": False,
    },
    "depth_image": {
        "topic": "/camera/depth/image_raw",
        "message_type": "sensor_msgs/Image",
        "category": "camera",
        "required": False,
    },
    "depth_camera_info": {
        "topic": "/camera/depth/camera_info",
        "message_type": "sensor_msgs/CameraInfo",
        "category": "camera",
        "required": False,
    },
    "depth_points": {
        "topic": "/camera/depth/points",
        "message_type": "sensor_msgs/PointCloud2",
        "category": "camera",
        "required": False,
    },
    "infrared_image": {
        "topic": "/camera/ir/image_raw",
        "message_type": "sensor_msgs/Image",
        "category": "camera",
        "required": False,
    },
    "infrared_camera_info": {
        "topic": "/camera/ir/camera_info",
        "message_type": "sensor_msgs/CameraInfo",
        "category": "camera",
        "required": False,
    },
}

LIMO_OBSERVATION_TOPICS: dict[str, tuple[str, str]] = {
    name: (str(spec["topic"]), str(spec["message_type"]))
    for name, spec in LIMO_OBSERVATION_SPECS.items()
}

REQUIRED_LIMO_TOPICS = {
    str(spec["topic"]) for spec in LIMO_OBSERVATION_SPECS.values() if spec.get("required")
}

LIMO_ERROR_FLAGS: dict[int, str] = {
    0x0001: "battery_critically_low",
    0x0002: "battery_low",
    0x0004: "remote_control_disconnected",
    0x0008: "motor_driver_1_error",
    0x0010: "motor_driver_2_error",
    0x0020: "motor_driver_3_error",
    0x0040: "motor_driver_4_error",
    0x0100: "drive_status_error",
}

MOTION_MODE_NAMES = {0: "four_wheel_differential", 1: "ackermann", 2: "mecanum", 255: "unknown"}

LIMO_INTERFACE_CONTRACT: dict[str, Any] = {
    "schema_version": "limo_ros_mcp.interface.v3",
    "robot_id": "limo",
    "vendor": "AgileX Robotics",
    "ros": {
        "version": "ros1",
        "transports": ["rosbridge", "local_roscli_read_only"],
        "default_transport": "auto",
        "default_endpoint": "ws://127.0.0.1:9090",
    },
    "source": {
        "limo_ros": {
            "url": "https://github.com/agilexrobotics/limo_ros",
            "commit": "4c78efc674cfc154012fe851fbda89c50be5b983",
        },
        "limo_doc": {
            "url": "https://github.com/agilexrobotics/limo-doc",
            "commit": "d78a730163f50e1a5e5631ffd651444e0ac6abc5",
        },
    },
    "geometry": {
        "wheelbase_m": 0.2,
        "track_m": 0.172,
        "wheel_radius_m": 0.045,
    },
    "motion_modes": {
        "four_wheel_differential": 0,
        "ackermann": 1,
        "mecanum": 2,
        "unknown": 255,
    },
    "interfaces": [
        *[
            {
                "capability_id": f"limo.observe.{name}",
                "kind": "observation",
                "ros_kind": "topic",
                "ros_name": spec["topic"],
                "ros_type": spec["message_type"],
                "category": spec["category"],
                "read_only": True,
                "required": bool(spec.get("required", False)),
                "patrol_required": bool(spec.get("patrol_required", False)),
            }
            for name, spec in LIMO_OBSERVATION_SPECS.items()
        ],
        {
            "capability_id": "limo.observe.dabai_device_state",
            "kind": "observation",
            "ros_kind": "service_group",
            "ros_names": [
                "/camera/get_device_info",
                "/camera/get_device_type",
                "/camera/get_serial",
                "/camera/get_version",
                "/camera/get_ir_temperature",
                "/camera/get_ldp_status",
            ],
            "read_only": True,
            "required": False,
            "patrol_required": False,
        },
        *[
            {
                "capability_id": capability_id,
                "kind": "observation",
                "host_kind": host_kind,
                "read_only": True,
                "required": False,
                "patrol_required": patrol_required,
            }
            for capability_id, host_kind, patrol_required in (
                ("limo.observe.peripheral_inventory", "linux_sysfs_and_ros_graph", False),
                ("limo.observe.audio_state", "alsa_mixer", False),
                ("limo.observe.microphone_level", "alsa_pcm_bounded_no_retention", False),
                ("limo.observe.display_state", "linux_framebuffer_and_input", False),
                ("limo.observe.platform_health", "linux_sysfs_and_procfs", True),
            )
        ],
        {
            "capability_id": "limo.navigate_to_pose",
            "kind": "actuation",
            "ros_kind": "action",
            "ros_name": "/move_base",
            "ros_type": "move_base_msgs/MoveBaseAction",
            "read_only": False,
            "execution_boundary": "rosclawd.request_action",
        },
        {
            "capability_id": "limo.base.velocity_command",
            "kind": "actuation",
            "ros_kind": "topic",
            "ros_name": "/cmd_vel",
            "ros_type": "geometry_msgs/Twist",
            "read_only": False,
            "enabled": False,
            "reason": "Direct Agent publishing is forbidden; use guarded high-level navigation.",
        },
        {
            "capability_id": "limo.set_initial_pose",
            "kind": "actuation",
            "ros_kind": "topic",
            "ros_name": "/initialpose",
            "ros_type": "geometry_msgs/PoseWithCovarianceStamped",
            "read_only": False,
            "execution_boundary": "rosclawd.request_action",
            "worker_protocol": "rosclaw.limo.worker.v1",
        },
    ],
    "safety": {
        "direct_cmd_vel_exposed": False,
        "max_linear_velocity_mps": 0.2,
        "max_lateral_velocity_mps": 0.1,
        "max_angular_velocity_radps": 0.5,
        "max_velocity_command_duration_sec": 1.0,
        "readiness_evidence_schema": "limo.readiness.v1",
        "readiness_policy": "configs/limo_readiness_policy.yaml",
        "navigation_contract_schema": "limo.navigation.v2",
        "navigation_policy": "configs/patrol_lab.example.yaml",
        "readiness_reference_same_process_required": True,
        "body_snapshot_binding_required": True,
        "legacy_booleans_usable_for_real": False,
        "real_navigation_enabled": False,
        "real_navigation_blocker": (
            "Daemon-owned trusted preflight and a verified executor are not implemented."
        ),
        "real_initial_pose_enabled": True,
        "initial_pose_contract_schema": "limo.initial-pose.v1",
    },
    "evidence_boundary": {
        "static_contract_is_hardware_evidence": False,
        "ros_graph_probe_is_motion_evidence": False,
        "topic_observation_is_action_success": False,
        "real_success_requires_execution_receipt": True,
    },
}


def get_limo_contract() -> dict[str, Any]:
    """Return a caller-owned copy of the canonical LIMO interface contract."""

    return copy.deepcopy(LIMO_INTERFACE_CONTRACT)


def list_limo_observations(category: str = "all") -> list[dict[str, Any]]:
    """Return the named topic catalogue used by MCP observation tools."""

    categories = {str(spec["category"]) for spec in LIMO_OBSERVATION_SPECS.values()}
    if category != "all" and category not in categories:
        raise ValueError(f"category must be one of {sorted(categories | {'all'})}")
    return [
        {"observation": name, **copy.deepcopy(spec)}
        for name, spec in LIMO_OBSERVATION_SPECS.items()
        if category == "all" or spec["category"] == category
    ]


def decode_limo_error_code(error_code: Any) -> list[str]:
    """Decode driver error bits using the pinned limo_driver.cpp implementation."""

    if isinstance(error_code, bool) or not isinstance(error_code, int):
        raise ValueError("error_code must be an integer")
    if not 0 <= error_code <= 0xFFFF:
        raise ValueError("error_code must be within uint16 range")
    return [name for bit, name in LIMO_ERROR_FLAGS.items() if error_code & bit]


def _finite_number(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def validate_velocity_command(
    *,
    linear_x: Any,
    linear_y: Any,
    angular_z: Any,
    duration_sec: Any,
    motion_mode: int,
) -> dict[str, Any]:
    """Validate the disabled low-level velocity interface for diagnostics only."""

    try:
        vx = _finite_number("linear_x", linear_x)
        vy = _finite_number("linear_y", linear_y)
        wz = _finite_number("angular_z", angular_z)
        duration = _finite_number("duration_sec", duration_sec)
    except ValueError as exc:
        return {"ok": False, "decision": "BLOCK", "violations": [str(exc)]}

    violations: list[str] = []
    if motion_mode not in {0, 1, 2}:
        violations.append("motion_mode must be known (0 differential, 1 Ackermann, 2 mecanum)")
    if abs(vx) > 0.2:
        violations.append("linear_x exceeds 0.2 m/s")
    if abs(wz) > 0.5:
        violations.append("angular_z exceeds 0.5 rad/s")
    lateral_limit = 0.1 if motion_mode == 2 else 0.0
    if abs(vy) > lateral_limit:
        violations.append(f"linear_y exceeds {lateral_limit} m/s for motion mode {motion_mode}")
    if not 0.0 < duration <= 1.0:
        violations.append("duration_sec must be within (0, 1.0]")

    return {
        "ok": not violations,
        "decision": "ALLOW" if not violations else "BLOCK",
        "violations": violations,
        "command_dispatched": False,
        "interface_enabled": False,
        "note": "Validation never authorizes direct /cmd_vel publishing.",
    }
