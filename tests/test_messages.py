"""ROS message-level summary and patrol-readiness tests."""

from __future__ import annotations

import math

from limo_ros_mcp.messages import (
    evaluate_patrol_readiness,
    summarize_binary_sensor,
    summarize_camera_info,
    summarize_diagnostics,
    summarize_goal_status,
    summarize_laser_scan,
    summarize_map,
    summarize_odometry,
    summarize_path,
    summarize_status,
)


def test_binary_sensor_summary_decodes_roscli_noarr_lengths() -> None:
    summary = summarize_binary_sensor(
        {
            "height": 400,
            "width": 640,
            "point_step": 16,
            "row_step": 10240,
            "fields": "<array type: sensor_msgs/PointField, length: 3>",
            "data": "<array type: uint8, length: 4096000>",
        }
    )

    assert summary["field_count"] == 3
    assert summary["fields"] == []
    assert summary["data_length"] == 4_096_000
    assert summary["data_truncated"] is True


def test_binary_sensor_summary_decodes_rosbridge_base64_length() -> None:
    summary = summarize_binary_sensor(
        {
            "height": 1,
            "width": 4,
            "encoding": "mono8",
            "step": 4,
            "data": "AQIDBA==",
        }
    )

    assert summary["data_length"] == 4
    assert summary["data_truncated"] is False
    assert summary["data_transport_encoding"] == "base64"


def test_camera_info_summary_reports_calibration_shape() -> None:
    summary = summarize_camera_info(
        {
            "header": {"frame_id": "camera_color_optical_frame"},
            "height": 480,
            "width": 640,
            "distortion_model": "plumb_bob",
            "D": [0.0] * 5,
            "K": [0.0] * 9,
            "R": [0.0] * 9,
            "P": [0.0] * 12,
            "binning_x": 0,
            "binning_y": 0,
            "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0},
        }
    )

    assert summary["width"] == 640
    assert summary["height"] == 480
    assert summary["distortion_coefficient_count"] == 5
    assert summary["intrinsic_matrix_valid"] is True
    assert summary["projection_matrix_valid"] is True


def test_status_decodes_motion_mode_and_driver_error_bits() -> None:
    summary = summarize_status(
        {
            "vehicle_state": 0,
            "control_mode": 1,
            "battery_voltage": 10.4,
            "error_code": 0x0002 | 0x0008,
            "motion_mode": 0,
        }
    )

    assert summary["motion_mode_name"] == "four_wheel_differential"
    assert summary["control_mode_name"] == "can"
    assert summary["control_mode_valid"] is True
    assert summary["error_flags"] == ["battery_low", "motor_driver_1_error"]
    assert summary["healthy"] is False


def test_non_finite_status_values_are_not_silently_coerced_to_zero() -> None:
    summary = summarize_status(
        {
            "battery_voltage": float("nan"),
            "control_mode": False,
            "error_code": "0",
            "motion_mode": False,
        }
    )

    assert summary["battery_voltage"] is None
    assert summary["battery_voltage_valid"] is False
    assert summary["error_code"] is None
    assert summary["error_code_valid"] is False
    assert summary["control_mode"] is None
    assert summary["control_mode_name"] == "invalid"
    assert summary["control_mode_valid"] is False
    assert summary["motion_mode"] is None
    assert summary["motion_mode_valid"] is False
    assert summary["healthy"] is False


def test_status_decodes_uart_and_rc_control_modes() -> None:
    assert summarize_status({"control_mode": 2})["control_mode_name"] == "uart"
    assert summarize_status({"control_mode": 3})["control_mode_name"] == "rc"


def test_odometry_extracts_pose_yaw_and_velocity() -> None:
    summary = summarize_odometry(
        {
            "header": {"frame_id": "odom"},
            "child_frame_id": "base_link",
            "pose": {
                "pose": {
                    "position": {"x": 1.5, "y": -0.5, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "covariance": [0.0] * 36,
            },
            "twist": {
                "twist": {
                    "linear": {"x": 0.12, "y": 0.0, "z": 0.0},
                    "angular": {"z": -0.2},
                },
                "covariance": [0.0] * 36,
            },
        }
    )

    assert summary["pose"]["position"]["x"] == 1.5
    assert summary["pose"]["orientation"]["yaw_rad"] == 0.0
    assert summary["velocity"]["linear_x_mps"] == 0.12
    assert summary["velocity"]["angular_z_radps"] == -0.2


def test_laser_scan_compacts_ranges_into_clearance_sectors() -> None:
    summary = summarize_laser_scan(
        {
            "angle_min": -math.pi / 2,
            "angle_increment": math.pi / 4,
            "range_min": 0.1,
            "range_max": 10.0,
            "ranges": [1.0, 2.0, 0.5, "inf", 4.0],
        }
    )

    assert summary["sample_count"] == 5
    assert summary["valid_count"] == 4
    assert summary["no_return_count"] == 1
    assert summary["usable_count"] == 5
    assert summary["usable_ratio"] == 1.0
    assert summary["minimum_m"] == 0.5
    assert summary["sectors"]["front_min_m"] == 0.5


def test_navigation_status_and_path_are_model_friendly() -> None:
    status = summarize_goal_status(
        {"status_list": [{"goal_id": {"id": "patrol-1"}, "status": 3, "text": "Goal reached."}]}
    )
    path = summarize_path(
        {
            "poses": [
                {"pose": {"position": {"x": 0.0, "y": 0.0}}},
                {"pose": {"position": {"x": 3.0, "y": 4.0}}},
            ]
        }
    )

    assert status["latest"]["status_name"] == "SUCCEEDED"
    assert path["pose_count"] == 2
    assert path["length_m"] == 5.0


def test_map_and_diagnostics_summaries_do_not_return_large_payloads() -> None:
    map_summary = summarize_map(
        {
            "info": {"resolution": 0.5, "width": 2, "height": 2, "origin": {}},
            "data": [-1, 0, 50, 100],
        }
    )
    diagnostics = summarize_diagnostics(
        {
            "status": [
                {"level": 0, "name": "lidar", "hardware_id": "ydlidar", "message": "OK"},
                {"level": 1, "name": "battery", "hardware_id": "limo", "message": "low"},
            ]
        }
    )

    assert map_summary["width_m"] == 1.0
    assert map_summary["occupied_cell_count"] == 2
    assert diagnostics["counts"] == {"OK": 1, "WARN": 1, "ERROR": 0, "STALE": 0}
    assert diagnostics["healthy"] is False


def test_patrol_readiness_reports_ready_and_obstacle_blocked_states() -> None:
    summaries = {
        "status": {"error_code": 0, "motion_mode": 0},
        "odometry": {},
        "imu": {},
        "laser_scan": {"valid_count": 10, "sectors": {"front_min_m": 1.0}},
        "localized_pose": {
            "position_variance_x": 0.1,
            "position_variance_y": 0.1,
            "yaw_variance": 0.1,
        },
        "navigation_status": {},
        "map_metadata": {},
        "diagnostics": {"healthy": True},
    }

    ready = evaluate_patrol_readiness(summaries)
    summaries["laser_scan"] = {"valid_count": 10, "sectors": {"front_min_m": 0.2}}
    blocked = evaluate_patrol_readiness(summaries)

    assert ready["state"] == "BLOCKED"
    assert "LIMO_BATTERY_BELOW_RETURN_THRESHOLD" in ready["blockers"]
    assert blocked["state"] == "BLOCKED"
    assert "LIMO_FRONT_CLEARANCE_LOW" in blocked["blockers"]


def test_patrol_readiness_warns_on_stale_localization_and_diagnostics() -> None:
    summaries = {
        "status": {"error_code": 0, "motion_mode": 0, "header": {"age_sec": 0.1}},
        "odometry": {"header": {"age_sec": 0.1}},
        "imu": {"header": {"age_sec": 0.1}},
        "laser_scan": {
            "valid_count": 10,
            "sectors": {"front_min_m": 1.0},
            "header": {"age_sec": 0.1},
        },
        "localized_pose": {
            "position_variance_x": 0.1,
            "position_variance_y": 0.1,
            "yaw_variance": 0.1,
            "header": {"age_sec": 30.0},
        },
        "navigation_status": {"header": {"age_sec": 0.1}},
        "map_metadata": {},
        "diagnostics": {"healthy": False},
    }

    result = evaluate_patrol_readiness(summaries)

    assert result["state"] == "BLOCKED"
    assert "LIMO_AMCL_STALE" in result["warnings"]
    assert "LIMO_AMCL_STALE" not in result["blockers"]
