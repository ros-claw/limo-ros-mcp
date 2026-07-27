"""ROS message-level summary and patrol-readiness tests."""

from __future__ import annotations

import math

from limo_ros_mcp.messages import (
    evaluate_patrol_readiness,
    summarize_diagnostics,
    summarize_goal_status,
    summarize_laser_scan,
    summarize_map,
    summarize_odometry,
    summarize_path,
    summarize_status,
)


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
    assert summary["error_flags"] == ["battery_low", "motor_driver_1_error"]
    assert summary["healthy"] is False


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
            "ranges": [1.0, 2.0, 0.5, float("inf"), 4.0],
        }
    )

    assert summary["sample_count"] == 5
    assert summary["valid_count"] == 4
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

    assert ready["state"] == "READY"
    assert ready["ready"] is True
    assert blocked["state"] == "BLOCKED"
    assert "front_clearance_below_0_35m" in blocked["blockers"]


def test_patrol_readiness_blocks_stale_localization_and_degrades_on_diagnostics() -> None:
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
    assert "localized_pose_stale" in result["blockers"]
    assert "diagnostics_non_nominal" in result["warnings"]
