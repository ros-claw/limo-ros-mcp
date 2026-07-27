"""ROS message summaries used by the LIMO inspection MCP tools."""

from __future__ import annotations

import math
import time
from typing import Any

from limo_ros_mcp.contract import MOTION_MODE_NAMES, decode_limo_error_code

GOAL_STATUS_NAMES = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}

DIAGNOSTIC_LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def _mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _number(value: Any, default: float = 0.0) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return default
    result = float(value)
    return result if math.isfinite(result) else default


def ros_stamp_seconds(message: dict[str, Any]) -> float | None:
    """Extract a ROS 1 header stamp as Unix/ROS seconds when available."""

    stamp = _mapping(_mapping(message.get("header")).get("stamp"))
    secs = stamp.get("secs")
    nsecs = stamp.get("nsecs", 0)
    if isinstance(secs, bool) or not isinstance(secs, (int, float)):
        return None
    if isinstance(nsecs, bool) or not isinstance(nsecs, (int, float)):
        nsecs = 0
    return float(secs) + float(nsecs) / 1_000_000_000.0


def message_age_seconds(message: dict[str, Any], *, now: float | None = None) -> float | None:
    stamp = ros_stamp_seconds(message)
    if stamp is None or stamp <= 0:
        return None
    return max(0.0, (time.time() if now is None else now) - stamp)


def _quaternion_yaw(orientation: dict[str, Any]) -> float:
    x = _number(orientation.get("x"))
    y = _number(orientation.get("y"))
    z = _number(orientation.get("z"))
    w = _number(orientation.get("w"), 1.0)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose_summary(pose_value: Any) -> dict[str, Any]:
    pose = _mapping(pose_value)
    position = _mapping(pose.get("position"))
    orientation = _mapping(pose.get("orientation"))
    return {
        "position": {
            "x": _number(position.get("x")),
            "y": _number(position.get("y")),
            "z": _number(position.get("z")),
        },
        "orientation": {
            "x": _number(orientation.get("x")),
            "y": _number(orientation.get("y")),
            "z": _number(orientation.get("z")),
            "w": _number(orientation.get("w"), 1.0),
            "yaw_rad": _quaternion_yaw(orientation),
        },
    }


def _header_summary(message: dict[str, Any]) -> dict[str, Any]:
    header = _mapping(message.get("header"))
    return {
        "seq": header.get("seq"),
        "stamp_sec": ros_stamp_seconds(message),
        "age_sec": message_age_seconds(message),
        "frame_id": str(header.get("frame_id", "")),
    }


def summarize_status(message: dict[str, Any]) -> dict[str, Any]:
    error_code_value = message.get("error_code", 0)
    error_code = error_code_value if isinstance(error_code_value, int) else 0
    motion_mode_value = message.get("motion_mode", 255)
    motion_mode = motion_mode_value if isinstance(motion_mode_value, int) else 255
    return {
        "header": _header_summary(message),
        "vehicle_state": message.get("vehicle_state"),
        "control_mode": message.get("control_mode"),
        "battery_voltage": _number(message.get("battery_voltage")),
        "error_code": error_code,
        "error_flags": decode_limo_error_code(error_code),
        "motion_mode": motion_mode,
        "motion_mode_name": MOTION_MODE_NAMES.get(motion_mode, "unrecognized"),
        "healthy": error_code == 0 and motion_mode in {0, 1, 2},
    }


def summarize_odometry(message: dict[str, Any]) -> dict[str, Any]:
    pose_container = _mapping(message.get("pose"))
    twist_container = _mapping(message.get("twist"))
    twist = _mapping(twist_container.get("twist"))
    linear = _mapping(twist.get("linear"))
    angular = _mapping(twist.get("angular"))
    return {
        "header": _header_summary(message),
        "child_frame_id": str(message.get("child_frame_id", "")),
        "pose": _pose_summary(_mapping(pose_container.get("pose"))),
        "velocity": {
            "linear_x_mps": _number(linear.get("x")),
            "linear_y_mps": _number(linear.get("y")),
            "linear_z_mps": _number(linear.get("z")),
            "angular_z_radps": _number(angular.get("z")),
        },
        "pose_covariance": _list(pose_container.get("covariance")),
        "twist_covariance": _list(twist_container.get("covariance")),
    }


def summarize_imu(message: dict[str, Any]) -> dict[str, Any]:
    orientation = _mapping(message.get("orientation"))
    angular = _mapping(message.get("angular_velocity"))
    acceleration = _mapping(message.get("linear_acceleration"))
    return {
        "header": _header_summary(message),
        "orientation": {
            "x": _number(orientation.get("x")),
            "y": _number(orientation.get("y")),
            "z": _number(orientation.get("z")),
            "w": _number(orientation.get("w"), 1.0),
            "yaw_rad": _quaternion_yaw(orientation),
        },
        "angular_velocity_radps": {axis: _number(angular.get(axis)) for axis in ("x", "y", "z")},
        "linear_acceleration_mps2": {
            axis: _number(acceleration.get(axis)) for axis in ("x", "y", "z")
        },
        "orientation_covariance": _list(message.get("orientation_covariance")),
    }


def _quantile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _sector_min(samples: list[tuple[float, float]], start: float, end: float) -> float | None:
    values = [distance for angle, distance in samples if start <= angle <= end]
    return min(values) if values else None


def summarize_laser_scan(message: dict[str, Any]) -> dict[str, Any]:
    angle_min = _number(message.get("angle_min"))
    angle_increment = _number(message.get("angle_increment"))
    range_min = _number(message.get("range_min"))
    range_max = _number(message.get("range_max"), math.inf)
    raw_ranges = _list(message.get("ranges"))
    samples: list[tuple[float, float]] = []
    for index, raw in enumerate(raw_ranges):
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            continue
        distance = float(raw)
        if math.isfinite(distance) and distance >= range_min and distance <= range_max:
            angle = angle_min + index * angle_increment
            samples.append((angle, distance))
    distances = [distance for _angle, distance in samples]
    closest = min(samples, key=lambda item: item[1]) if samples else None
    return {
        "header": _header_summary(message),
        "sample_count": len(raw_ranges),
        "valid_count": len(samples),
        "invalid_count": len(raw_ranges) - len(samples),
        "range_min_m": range_min,
        "range_max_m": None if math.isinf(range_max) else range_max,
        "minimum_m": min(distances) if distances else None,
        "p10_m": _quantile(distances, 0.10),
        "median_m": _quantile(distances, 0.50),
        "p90_m": _quantile(distances, 0.90),
        "closest_angle_rad": closest[0] if closest else None,
        "sectors": {
            "front_min_m": _sector_min(samples, -math.pi / 6, math.pi / 6),
            "left_min_m": _sector_min(samples, math.pi / 6, 2 * math.pi / 3),
            "right_min_m": _sector_min(samples, -2 * math.pi / 3, -math.pi / 6),
            "rear_min_m": min(
                [distance for angle, distance in samples if abs(angle) >= 5 * math.pi / 6],
                default=None,
            ),
        },
    }


def summarize_localized_pose(message: dict[str, Any]) -> dict[str, Any]:
    pose_container = _mapping(message.get("pose"))
    covariance = _list(pose_container.get("covariance"))
    return {
        "header": _header_summary(message),
        "pose": _pose_summary(pose_container.get("pose")),
        "covariance": covariance,
        "position_variance_x": covariance[0] if len(covariance) > 0 else None,
        "position_variance_y": covariance[7] if len(covariance) > 7 else None,
        "yaw_variance": covariance[35] if len(covariance) > 35 else None,
    }


def summarize_goal_status(message: dict[str, Any]) -> dict[str, Any]:
    statuses: list[dict[str, Any]] = []
    for raw in _list(message.get("status_list")):
        item = _mapping(raw)
        goal_id = _mapping(item.get("goal_id"))
        status_value = item.get("status", 9)
        status = status_value if isinstance(status_value, int) else 9
        statuses.append(
            {
                "goal_id": str(goal_id.get("id", "")),
                "status": status,
                "status_name": GOAL_STATUS_NAMES.get(status, "UNKNOWN"),
                "text": str(item.get("text", "")),
            }
        )
    return {
        "header": _header_summary(message),
        "goal_count": len(statuses),
        "goals": statuses,
        "latest": statuses[-1] if statuses else None,
    }


def summarize_path(message: dict[str, Any]) -> dict[str, Any]:
    poses = _list(message.get("poses"))
    points: list[tuple[float, float]] = []
    for raw in poses:
        pose_stamped = _mapping(raw)
        pose = _mapping(pose_stamped.get("pose"))
        position = _mapping(pose.get("position"))
        points.append((_number(position.get("x")), _number(position.get("y"))))
    length = sum(
        math.hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in zip(points, points[1:], strict=False)
    )
    return {
        "header": _header_summary(message),
        "pose_count": len(points),
        "length_m": length,
        "start": {"x": points[0][0], "y": points[0][1]} if points else None,
        "end": {"x": points[-1][0], "y": points[-1][1]} if points else None,
    }


def summarize_map(message: dict[str, Any]) -> dict[str, Any]:
    info = _mapping(message.get("info", message))
    width_value = info.get("width", 0)
    height_value = info.get("height", 0)
    width = width_value if isinstance(width_value, int) else 0
    height = height_value if isinstance(height_value, int) else 0
    resolution = _number(info.get("resolution"))
    data = _list(message.get("data"))
    known = [value for value in data if isinstance(value, int) and value >= 0]
    occupied = [value for value in known if value >= 50]
    return {
        "header": _header_summary(message),
        "resolution_m": resolution,
        "width_cells": width,
        "height_cells": height,
        "width_m": width * resolution,
        "height_m": height * resolution,
        "origin": _pose_summary(_mapping(info.get("origin"))),
        "cell_count": len(data),
        "known_cell_count": len(known),
        "occupied_cell_count": len(occupied),
        "occupancy_fraction": len(occupied) / len(known) if known else None,
    }


def summarize_diagnostics(message: dict[str, Any]) -> dict[str, Any]:
    statuses: list[dict[str, Any]] = []
    counts = dict.fromkeys(DIAGNOSTIC_LEVEL_NAMES.values(), 0)
    for raw in _list(message.get("status")):
        item = _mapping(raw)
        level_value = item.get("level", 3)
        level = level_value if isinstance(level_value, int) else 3
        level_name = DIAGNOSTIC_LEVEL_NAMES.get(level, "UNKNOWN")
        counts[level_name] = counts.get(level_name, 0) + 1
        statuses.append(
            {
                "level": level,
                "level_name": level_name,
                "name": str(item.get("name", "")),
                "hardware_id": str(item.get("hardware_id", "")),
                "message": str(item.get("message", "")),
            }
        )
    return {
        "header": _header_summary(message),
        "counts": counts,
        "status": statuses,
        "healthy": all(counts.get(level, 0) == 0 for level in ("WARN", "ERROR", "STALE")),
    }


def summarize_tf(message: dict[str, Any]) -> dict[str, Any]:
    transforms: list[dict[str, Any]] = []
    for raw in _list(message.get("transforms")):
        item = _mapping(raw)
        header = _mapping(item.get("header"))
        transforms.append(
            {
                "parent_frame": str(header.get("frame_id", "")),
                "child_frame": str(item.get("child_frame_id", "")),
            }
        )
    return {"transform_count": len(transforms), "transforms": transforms}


def summarize_binary_sensor(message: dict[str, Any]) -> dict[str, Any]:
    data = message.get("data")
    data_length = len(data) if isinstance(data, (str, bytes, list)) else 0
    fields = [str(_mapping(item).get("name", "")) for item in _list(message.get("fields"))]
    return {
        "header": _header_summary(message),
        "height": message.get("height"),
        "width": message.get("width"),
        "encoding": message.get("encoding"),
        "step": message.get("step"),
        "point_step": message.get("point_step"),
        "row_step": message.get("row_step"),
        "is_dense": message.get("is_dense"),
        "fields": fields,
        "data_length": data_length,
    }


def summarize_ros_log(message: dict[str, Any]) -> dict[str, Any]:
    return {
        "header": _header_summary(message),
        "level": message.get("level"),
        "name": str(message.get("name", "")),
        "message": str(message.get("msg", "")),
        "file": str(message.get("file", "")),
        "function": str(message.get("function", "")),
        "line": message.get("line"),
    }


def summarize_observation(observation: str, message: dict[str, Any]) -> dict[str, Any]:
    """Convert a raw ROS message into a compact, inspection-oriented representation."""

    if observation == "status":
        return summarize_status(message)
    if observation == "odometry":
        return summarize_odometry(message)
    if observation == "imu":
        return summarize_imu(message)
    if observation == "laser_scan":
        return summarize_laser_scan(message)
    if observation in {"localized_pose", "fused_pose"}:
        return summarize_localized_pose(message)
    if observation == "navigation_status":
        return summarize_goal_status(message)
    if observation in {"global_plan", "local_plan"}:
        return summarize_path(message)
    if observation in {"map_metadata", "map", "global_costmap", "local_costmap"}:
        return summarize_map(message)
    if observation == "diagnostics":
        return summarize_diagnostics(message)
    if observation in {"tf", "tf_static"}:
        return summarize_tf(message)
    if observation in {"color_image", "depth_image", "depth_points"}:
        return summarize_binary_sensor(message)
    if observation == "ros_logs":
        return summarize_ros_log(message)
    if observation == "current_goal":
        return {"header": _header_summary(message), "pose": _pose_summary(message.get("pose"))}
    return {
        "header": _header_summary(message),
        "fields": sorted(str(key) for key in message),
    }


def evaluate_patrol_readiness(summaries: dict[str, dict[str, Any]]) -> dict[str, Any]:
    """Evaluate message-level readiness without authorizing or dispatching motion."""

    blockers: list[str] = []
    warnings: list[str] = []
    status = summaries.get("status")
    if status is None:
        blockers.append("limo_status_unavailable")
    else:
        if status.get("error_code") != 0:
            blockers.append("limo_error_code_nonzero")
        if status.get("motion_mode") not in {0, 1, 2}:
            blockers.append("motion_mode_unknown")
    for required in ("laser_scan", "localized_pose", "navigation_status", "map_metadata"):
        if required not in summaries:
            blockers.append(f"{required}_unavailable")
    scan = summaries.get("laser_scan")
    if scan is not None:
        front_min = _mapping(scan.get("sectors")).get("front_min_m")
        if isinstance(front_min, (int, float)) and not isinstance(front_min, bool):
            if float(front_min) < 0.35:
                blockers.append("front_clearance_below_0_35m")
        elif scan.get("valid_count", 0) == 0:
            blockers.append("laser_scan_has_no_valid_ranges")
    pose = summaries.get("localized_pose")
    if pose is not None:
        x_variance = pose.get("position_variance_x")
        y_variance = pose.get("position_variance_y")
        yaw_variance = pose.get("yaw_variance")
        if any(
            isinstance(value, (int, float)) and not isinstance(value, bool) and float(value) > limit
            for value, limit in ((x_variance, 1.0), (y_variance, 1.0), (yaw_variance, 0.5))
        ):
            warnings.append("localization_covariance_high")
    for useful in ("odometry", "imu", "diagnostics"):
        if useful not in summaries:
            warnings.append(f"{useful}_unavailable")
    freshness_limits = {
        "status": 2.0,
        "odometry": 2.0,
        "imu": 2.0,
        "laser_scan": 2.0,
        "localized_pose": 5.0,
        "navigation_status": 5.0,
    }
    blocker_if_stale = {"status", "laser_scan", "localized_pose"}
    for observation, limit in freshness_limits.items():
        summary = summaries.get(observation)
        if summary is None:
            continue
        age = _mapping(summary.get("header")).get("age_sec")
        if isinstance(age, (int, float)) and not isinstance(age, bool) and float(age) > limit:
            issue = f"{observation}_stale"
            if observation in blocker_if_stale:
                blockers.append(issue)
            else:
                warnings.append(issue)
    diagnostics = summaries.get("diagnostics")
    if diagnostics is not None and not diagnostics.get("healthy", True):
        warnings.append("diagnostics_non_nominal")
    state = "BLOCKED" if blockers else ("DEGRADED" if warnings else "READY")
    return {
        "ready": not blockers,
        "state": state,
        "blockers": sorted(set(blockers)),
        "warnings": sorted(set(warnings)),
        "evaluated_observations": sorted(summaries),
        "command_dispatched": False,
    }
