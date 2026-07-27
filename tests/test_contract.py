"""LIMO source contract and ROS embodiment-card tests."""

from __future__ import annotations

from limo_ros_mcp.contract import (
    get_limo_contract,
    validate_navigation_goal,
    validate_velocity_command,
)


def test_contract_exposes_only_source_backed_interfaces() -> None:
    contract = get_limo_contract()
    interfaces = {item["ros_name"]: item for item in contract["interfaces"]}

    assert set(interfaces) == {
        "/limo_status",
        "/odom",
        "/imu",
        "/scan",
        "/move_base",
        "/cmd_vel",
    }
    assert interfaces["/limo_status"]["ros_type"] == "limo_base/LimoStatus"
    assert interfaces["/cmd_vel"]["enabled"] is False
    assert contract["geometry"] == {
        "wheelbase_m": 0.2,
        "track_m": 0.172,
        "wheel_radius_m": 0.045,
    }


def test_navigation_goal_fails_closed_on_missing_preconditions() -> None:
    result = validate_navigation_goal(
        x=1.0,
        y=2.0,
        yaw=0.0,
        frame_id="map",
        localization_ready=False,
        costmap_ready=True,
        obstacle_check_enabled=True,
    )

    assert result["decision"] == "BLOCK"
    assert "localization_ready is required" in result["violations"]


def test_navigation_goal_allows_validated_plan_without_dispatch() -> None:
    result = validate_navigation_goal(
        x=1.0,
        y=-2.0,
        yaw=0.5,
        frame_id="map",
        localization_ready=True,
        costmap_ready=True,
        obstacle_check_enabled=True,
    )

    assert result["decision"] == "ALLOW"
    assert result["command_dispatched"] is False


def test_velocity_validation_is_diagnostic_and_mode_aware() -> None:
    valid = validate_velocity_command(
        linear_x=0.1,
        linear_y=0.05,
        angular_z=0.2,
        duration_sec=0.5,
        motion_mode=2,
    )
    invalid = validate_velocity_command(
        linear_x=0.1,
        linear_y=0.05,
        angular_z=0.2,
        duration_sec=0.5,
        motion_mode=0,
    )

    assert valid["decision"] == "ALLOW"
    assert valid["interface_enabled"] is False
    assert invalid["decision"] == "BLOCK"


def test_contract_records_immutable_upstream_revisions() -> None:
    sources = get_limo_contract()["source"]

    assert len(sources["limo_ros"]["commit"]) == 40
    assert len(sources["limo_doc"]["commit"]) == 40
