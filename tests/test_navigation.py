"""Operator map and Navigation Contract v2 tests."""

from __future__ import annotations

import hashlib
from pathlib import Path

import yaml

from limo_ros_mcp.navigation import NavigationGoalValidator, load_patrol_map_policy


def _digest(data: bytes) -> str:
    return f"sha256:{hashlib.sha256(data).hexdigest()}"


def _policy_file(tmp_path: Path, *, no_go: list[list[list[float]]] | None = None) -> Path:
    map_yaml = b"image: map.pgm\nresolution: 1.0\norigin: [0, 0, 0]\n"
    pixels = bytes([254] * 49)
    image = b"P5\n7 7\n255\n" + pixels
    map_yaml_path = tmp_path / "map.yaml"
    image_path = tmp_path / "map.pgm"
    map_yaml_path.write_bytes(map_yaml)
    image_path.write_bytes(image)
    policy = {
        "schema_version": "limo.patrol-map.v1",
        "route_policy_id": "test-route",
        "map_id": "test-map",
        "frame_id": "map",
        "map_yaml_uri": str(map_yaml_path),
        "map_yaml_sha256": _digest(map_yaml),
        "map_image_uri": str(image_path),
        "map_image_sha256": _digest(image),
        "map": {
            "resolution_m": 1.0,
            "width_cells": 7,
            "height_cells": 7,
            "origin": [0.0, 0.0, 0.0],
            "occupied_threshold": 0.65,
            "free_threshold": 0.196,
            "allow_unknown": False,
        },
        "geofence": {"polygon": [[0.0, 0.0], [7.0, 0.0], [7.0, 7.0], [0.0, 7.0]]},
        "no_go_zones": no_go or [],
        "goal_tolerance": {
            "default_xy_m": 0.15,
            "default_yaw_rad": 0.2,
            "min_xy_m": 0.05,
            "max_xy_m": 0.5,
            "min_yaw_rad": 0.05,
            "max_yaw_rad": 0.75,
        },
        "minimum_obstacle_clearance_m": 0.05,
        "expected_motion_mode": 0,
    }
    policy_path = tmp_path / "patrol.yaml"
    policy_path.write_text(yaml.safe_dump(policy), encoding="utf-8")
    return policy_path


def test_policy_loads_and_free_goal_is_allowed(tmp_path: Path) -> None:
    validator = NavigationGoalValidator(load_patrol_map_policy(_policy_file(tmp_path)))

    result = validator.validate(x=3.5, y=3.5, yaw=0.2, route_policy_id="test-route")

    assert result["decision"] == "ALLOW"
    assert result["schema_version"] == "limo.navigation.v2"
    assert result["map_id"] == "test-map"
    assert result["command_dispatched"] is False


def test_goal_validation_rejects_frame_policy_bounds_and_tolerance(tmp_path: Path) -> None:
    validator = NavigationGoalValidator(load_patrol_map_policy(_policy_file(tmp_path)))

    assert validator.validate(x=3.5, y=3.5, yaw=0.0, frame_id="odom")["error_code"] == (
        "LIMO_GOAL_FRAME_INVALID"
    )
    assert validator.validate(x=3.5, y=3.5, yaw=0.0)["error_code"] == ("LIMO_ROUTE_POLICY_MISMATCH")
    assert (
        validator.validate(x=9.0, y=3.5, yaw=0.0, route_policy_id="test-route")["error_code"]
        == "LIMO_GOAL_OUTSIDE_MAP"
    )
    assert (
        validator.validate(
            x=3.5,
            y=3.5,
            yaw=0.0,
            route_policy_id="test-route",
            goal_tolerance_xy_m=0.9,
        )["error_code"]
        == "LIMO_GOAL_TOLERANCE_INVALID"
    )


def test_goal_validation_rejects_no_go_and_occupied_cells(tmp_path: Path) -> None:
    no_go = [[[3.0, 3.0], [4.0, 3.0], [4.0, 4.0], [3.0, 4.0]]]
    validator = NavigationGoalValidator(load_patrol_map_policy(_policy_file(tmp_path, no_go=no_go)))
    assert (
        validator.validate(x=3.5, y=3.5, yaw=0.0, route_policy_id="test-route")["error_code"]
        == "LIMO_GOAL_OUTSIDE_GEOFENCE"
    )

    policy_path = _policy_file(tmp_path)
    image_path = tmp_path / "map.pgm"
    image = bytearray(image_path.read_bytes())
    header_size = len(b"P5\n7 7\n255\n")
    image[header_size + (7 - 1 - 3) * 7 + 3] = 0
    image_path.write_bytes(bytes(image))
    raw = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
    raw["map_image_sha256"] = _digest(bytes(image))
    policy_path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    occupied = NavigationGoalValidator(load_patrol_map_policy(policy_path))
    assert (
        occupied.validate(x=3.5, y=3.5, yaw=0.0, route_policy_id="test-route")["error_code"]
        == "LIMO_GOAL_OCCUPIED"
    )


def test_map_hash_mismatch_fails_closed(tmp_path: Path) -> None:
    policy_path = _policy_file(tmp_path)
    raw = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
    raw["map_image_sha256"] = "sha256:" + "0" * 64
    policy_path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    result = NavigationGoalValidator(load_patrol_map_policy(policy_path)).validate(
        x=3.5, y=3.5, yaw=0.0, route_policy_id="test-route"
    )

    assert result["error_code"] == "LIMO_MAP_ARTIFACT_INVALID"


def test_pgm_pixel_whitespace_is_not_consumed_as_header(tmp_path: Path) -> None:
    policy_path = _policy_file(tmp_path)
    image_path = tmp_path / "map.pgm"
    image = b"P5\n7 7\n255\n" + bytes([10]) + bytes([254] * 48)
    image_path.write_bytes(image)
    raw = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
    raw["map_image_sha256"] = _digest(image)
    policy_path.write_text(yaml.safe_dump(raw), encoding="utf-8")

    result = NavigationGoalValidator(load_patrol_map_policy(policy_path)).validate(
        x=3.5, y=3.5, yaw=0.0, route_policy_id="test-route"
    )

    assert result["decision"] == "ALLOW"
