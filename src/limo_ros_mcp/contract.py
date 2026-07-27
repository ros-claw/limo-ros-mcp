"""Static, source-backed safety contract for the AgileX LIMO ROS 1 driver."""

from __future__ import annotations

import copy
import math
from typing import Any

LIMO_OBSERVATION_TOPICS: dict[str, tuple[str, str]] = {
    "status": ("/limo_status", "limo_base/LimoStatus"),
    "odometry": ("/odom", "nav_msgs/Odometry"),
    "imu": ("/imu", "sensor_msgs/Imu"),
    "laser_scan": ("/scan", "sensor_msgs/LaserScan"),
}

LIMO_INTERFACE_CONTRACT: dict[str, Any] = {
    "schema_version": "limo_ros_mcp.interface.v1",
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
        {
            "capability_id": "limo.observe.status",
            "kind": "observation",
            "ros_kind": "topic",
            "ros_name": "/limo_status",
            "ros_type": "limo_base/LimoStatus",
            "read_only": True,
        },
        {
            "capability_id": "limo.observe.odom",
            "kind": "observation",
            "ros_kind": "topic",
            "ros_name": "/odom",
            "ros_type": "nav_msgs/Odometry",
            "read_only": True,
        },
        {
            "capability_id": "limo.observe.imu",
            "kind": "observation",
            "ros_kind": "topic",
            "ros_name": "/imu",
            "ros_type": "sensor_msgs/Imu",
            "read_only": True,
        },
        {
            "capability_id": "limo.observe.scan",
            "kind": "observation",
            "ros_kind": "topic",
            "ros_name": "/scan",
            "ros_type": "sensor_msgs/LaserScan",
            "read_only": True,
            "required": False,
        },
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
    ],
    "safety": {
        "direct_cmd_vel_exposed": False,
        "max_linear_velocity_mps": 0.2,
        "max_lateral_velocity_mps": 0.1,
        "max_angular_velocity_radps": 0.5,
        "max_velocity_command_duration_sec": 1.0,
        "navigation_preconditions": [
            "localization_ready",
            "costmap_ready",
            "obstacle_check_enabled",
            "limo_status_fresh",
            "error_code == 0",
            "motion_mode_known",
        ],
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


def _finite_number(name: str, value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def validate_navigation_goal(
    *,
    x: Any,
    y: Any,
    yaw: Any,
    frame_id: str,
    localization_ready: bool,
    costmap_ready: bool,
    obstacle_check_enabled: bool,
) -> dict[str, Any]:
    """Validate a high-level ROS 1 move_base goal without dispatching it."""

    violations: list[str] = []
    try:
        x_value = _finite_number("x", x)
        y_value = _finite_number("y", y)
        yaw_value = _finite_number("yaw", yaw)
    except ValueError as exc:
        return {"ok": False, "decision": "BLOCK", "violations": [str(exc)]}

    if frame_id not in {"map", "odom"}:
        violations.append("frame_id must be 'map' or 'odom'")
    if abs(x_value) > 100.0 or abs(y_value) > 100.0:
        violations.append("goal exceeds the conservative 100 m coordinate envelope")
    if not -math.pi <= yaw_value <= math.pi:
        violations.append("yaw must be within [-pi, pi]")
    if not localization_ready:
        violations.append("localization_ready is required")
    if not costmap_ready:
        violations.append("costmap_ready is required")
    if not obstacle_check_enabled:
        violations.append("obstacle_check_enabled is required")

    if violations:
        return {"ok": False, "decision": "BLOCK", "violations": violations}
    return {
        "ok": True,
        "decision": "ALLOW",
        "violations": [],
        "normalized_goal": {
            "frame_id": frame_id,
            "x": x_value,
            "y": y_value,
            "yaw": yaw_value,
        },
        "command_dispatched": False,
    }


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
