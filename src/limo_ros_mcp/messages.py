"""ROS message summaries used by the LIMO inspection MCP tools."""

from __future__ import annotations

import base64
import binascii
import contextlib
import math
import re
import time
from typing import Any

from limo_ros_mcp.contract import MOTION_MODE_NAMES, decode_limo_error_code
from limo_ros_mcp.readiness import evaluate_summaries
from limo_ros_mcp.schemas import ReadinessEvidenceV1

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
ARRAY_PLACEHOLDER_RE = re.compile(r"^<array type: [^,>]+, length: (\d+)>$")


def _mapping(value: Any) -> dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


def _numeric_list(value: Any) -> tuple[list[float | None], bool]:
    raw = _list(value)
    numbers = [_number(item) for item in raw]
    return numbers, bool(raw) and all(item is not None for item in numbers)


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def ros_stamp_seconds(message: dict[str, Any]) -> float | None:
    """Extract a ROS 1 header stamp as Unix/ROS seconds when available."""

    stamp = _mapping(_mapping(message.get("header")).get("stamp"))
    secs = stamp.get("secs")
    nsecs = stamp.get("nsecs", 0)
    if isinstance(secs, bool) or not isinstance(secs, (int, float)) or not math.isfinite(secs):
        return None
    if isinstance(nsecs, bool) or not isinstance(nsecs, (int, float)) or not math.isfinite(nsecs):
        return None
    return float(secs) + float(nsecs) / 1_000_000_000.0


def message_age_seconds(message: dict[str, Any], *, now: float | None = None) -> float | None:
    stamp = ros_stamp_seconds(message)
    if stamp is None or stamp <= 0:
        return None
    return max(0.0, (time.time() if now is None else now) - stamp)


def _quaternion_yaw(orientation: dict[str, Any]) -> float | None:
    x = _number(orientation.get("x"))
    y = _number(orientation.get("y"))
    z = _number(orientation.get("z"))
    w = _number(orientation.get("w"))
    if any(value is None for value in (x, y, z, w)):
        return None
    assert x is not None and y is not None and z is not None and w is not None
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
            "w": _number(orientation.get("w")),
            "yaw_rad": _quaternion_yaw(orientation),
        },
        "valid": all(
            _number(value) is not None
            for value in (
                position.get("x"),
                position.get("y"),
                position.get("z"),
                orientation.get("x"),
                orientation.get("y"),
                orientation.get("z"),
                orientation.get("w"),
            )
        ),
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
    error_code_value = message.get("error_code")
    error_code = (
        error_code_value
        if isinstance(error_code_value, int) and not isinstance(error_code_value, bool)
        else None
    )
    motion_mode_value = message.get("motion_mode")
    motion_mode = (
        motion_mode_value
        if isinstance(motion_mode_value, int) and not isinstance(motion_mode_value, bool)
        else None
    )
    battery_voltage = _number(message.get("battery_voltage"))
    return {
        "header": _header_summary(message),
        "vehicle_state": message.get("vehicle_state"),
        "control_mode": message.get("control_mode"),
        "battery_voltage": battery_voltage,
        "battery_voltage_valid": battery_voltage is not None,
        "error_code": error_code,
        "error_code_valid": error_code is not None and 0 <= error_code <= 0xFFFF,
        "error_flags": (
            decode_limo_error_code(error_code)
            if error_code is not None and 0 <= error_code <= 0xFFFF
            else []
        ),
        "motion_mode": motion_mode,
        "motion_mode_name": (
            MOTION_MODE_NAMES.get(motion_mode, "unrecognized")
            if motion_mode is not None
            else "invalid"
        ),
        "motion_mode_valid": motion_mode in {0, 1, 2},
        "healthy": error_code == 0 and motion_mode in {0, 1, 2},
    }


def summarize_odometry(message: dict[str, Any]) -> dict[str, Any]:
    pose_container = _mapping(message.get("pose"))
    twist_container = _mapping(message.get("twist"))
    twist = _mapping(twist_container.get("twist"))
    linear = _mapping(twist.get("linear"))
    angular = _mapping(twist.get("angular"))
    pose_covariance, pose_covariance_valid = _numeric_list(pose_container.get("covariance"))
    twist_covariance, twist_covariance_valid = _numeric_list(twist_container.get("covariance"))
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
        "pose_covariance": pose_covariance,
        "pose_covariance_valid": pose_covariance_valid,
        "twist_covariance": twist_covariance,
        "twist_covariance_valid": twist_covariance_valid,
    }


def summarize_imu(message: dict[str, Any]) -> dict[str, Any]:
    orientation = _mapping(message.get("orientation"))
    angular = _mapping(message.get("angular_velocity"))
    acceleration = _mapping(message.get("linear_acceleration"))
    orientation_covariance, orientation_covariance_valid = _numeric_list(
        message.get("orientation_covariance")
    )
    return {
        "header": _header_summary(message),
        "orientation": {
            "x": _number(orientation.get("x")),
            "y": _number(orientation.get("y")),
            "z": _number(orientation.get("z")),
            "w": _number(orientation.get("w")),
            "yaw_rad": _quaternion_yaw(orientation),
        },
        "angular_velocity_radps": {axis: _number(angular.get(axis)) for axis in ("x", "y", "z")},
        "linear_acceleration_mps2": {
            axis: _number(acceleration.get(axis)) for axis in ("x", "y", "z")
        },
        "orientation_covariance": orientation_covariance,
        "orientation_covariance_valid": orientation_covariance_valid,
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
    range_max = _number(message.get("range_max"))
    scan_parameters_valid = all(
        value is not None for value in (angle_min, angle_increment, range_min, range_max)
    )
    raw_ranges = _list(message.get("ranges"))
    samples: list[tuple[float, float]] = []
    finite_valid_count = 0
    no_return_count = 0
    for index, raw in enumerate(raw_ranges):
        if (
            isinstance(raw, str)
            and raw in {"inf", "+inf", "Infinity", "+Infinity"}
            and scan_parameters_valid
            and range_max is not None
            and angle_min is not None
            and angle_increment is not None
        ):
            angle = angle_min + index * angle_increment
            samples.append((angle, range_max))
            no_return_count += 1
            continue
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            continue
        distance = float(raw)
        if (
            scan_parameters_valid
            and math.isfinite(distance)
            and range_min is not None
            and range_max is not None
            and angle_min is not None
            and angle_increment is not None
            and distance >= range_min
            and distance <= range_max
        ):
            angle = angle_min + index * angle_increment
            samples.append((angle, distance))
            finite_valid_count += 1
        elif (
            scan_parameters_valid
            and math.isinf(distance)
            and distance > 0.0
            and range_max is not None
            and angle_min is not None
            and angle_increment is not None
        ):
            # ROS lidars commonly encode "no obstacle within range" as +Inf. It is
            # usable coverage for clearance, but remains distinct from a finite hit.
            angle = angle_min + index * angle_increment
            samples.append((angle, range_max))
            no_return_count += 1
    distances = [distance for _angle, distance in samples]
    closest = min(samples, key=lambda item: item[1]) if samples else None
    return {
        "header": _header_summary(message),
        "sample_count": len(raw_ranges),
        "valid_count": finite_valid_count,
        "no_return_count": no_return_count,
        "usable_count": len(samples),
        "usable_ratio": len(samples) / len(raw_ranges) if raw_ranges else None,
        "invalid_count": len(raw_ranges) - len(samples),
        "parameters_valid": scan_parameters_valid,
        "range_min_m": range_min,
        "range_max_m": range_max,
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
    covariance, covariance_valid = _numeric_list(pose_container.get("covariance"))
    return {
        "header": _header_summary(message),
        "pose": _pose_summary(pose_container.get("pose")),
        "covariance": covariance,
        "covariance_valid": covariance_valid and len(covariance) == 36,
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
        x = _number(position.get("x"))
        y = _number(position.get("y"))
        if x is not None and y is not None:
            points.append((x, y))
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
        "width_m": width * resolution if resolution is not None else None,
        "height_m": height * resolution if resolution is not None else None,
        "metadata_valid": resolution is not None and resolution > 0.0 and width > 0 and height > 0,
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
    data_placeholder = ARRAY_PLACEHOLDER_RE.fullmatch(data) if isinstance(data, str) else None
    decoded_base64_length: int | None = None
    if isinstance(data, str) and data_placeholder is None:
        with contextlib.suppress(binascii.Error, ValueError):
            decoded_base64_length = len(base64.b64decode(data, validate=True))
    data_length = (
        int(data_placeholder.group(1))
        if data_placeholder is not None
        else decoded_base64_length
        if decoded_base64_length is not None
        else len(data)
        if isinstance(data, (str, bytes, list))
        else 0
    )
    fields_value = message.get("fields")
    fields_placeholder = (
        ARRAY_PLACEHOLDER_RE.fullmatch(fields_value) if isinstance(fields_value, str) else None
    )
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
        "field_count": (
            int(fields_placeholder.group(1)) if fields_placeholder is not None else len(fields)
        ),
        "data_length": data_length,
        "data_truncated": data_placeholder is not None,
        "data_transport_encoding": "base64" if decoded_base64_length is not None else None,
    }


def summarize_camera_info(message: dict[str, Any]) -> dict[str, Any]:
    """Summarize intrinsic calibration without returning large matrices verbatim."""

    distortion = _list(message.get("D"))
    intrinsic = _list(message.get("K"))
    rectification = _list(message.get("R"))
    projection = _list(message.get("P"))
    return {
        "header": _header_summary(message),
        "height": message.get("height"),
        "width": message.get("width"),
        "distortion_model": str(message.get("distortion_model", "")),
        "distortion_coefficient_count": len(distortion),
        "intrinsic_matrix_valid": len(intrinsic) == 9,
        "rectification_matrix_valid": len(rectification) == 9,
        "projection_matrix_valid": len(projection) == 12,
        "binning_x": message.get("binning_x"),
        "binning_y": message.get("binning_y"),
        "roi": _mapping(message.get("roi")),
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
    if observation in {"color_image", "depth_image", "depth_points", "infrared_image"}:
        return summarize_binary_sensor(message)
    if observation in {"color_camera_info", "depth_camera_info", "infrared_camera_info"}:
        return summarize_camera_info(message)
    if observation == "ros_logs":
        return summarize_ros_log(message)
    if observation == "current_goal":
        return {"header": _header_summary(message), "pose": _pose_summary(message.get("pose"))}
    return {
        "header": _header_summary(message),
        "fields": sorted(str(key) for key in message),
    }


def evaluate_patrol_readiness(summaries: dict[str, dict[str, Any]]) -> ReadinessEvidenceV1:
    """Compatibility wrapper returning hash-sealed ReadinessEvidenceV1."""

    return evaluate_summaries(summaries)
