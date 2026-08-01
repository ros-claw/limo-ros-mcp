"""Navigation action-intent construction."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


def navigation_arguments(
    goal: Mapping[str, Any],
    validation: Mapping[str, Any],
    readiness_snapshot_hash: str,
) -> dict[str, Any]:
    return {
        "schema_version": "limo.navigation.v2",
        "target_pose": dict(goal),
        "readiness_snapshot_hash": readiness_snapshot_hash,
        "route_policy_id": validation["route_policy_id"],
        "route_policy_hash": validation["route_policy_hash"],
        "map_id": validation["map_id"],
        "map_image_hash": validation["map_image_hash"],
        "goal_tolerance": validation["goal_tolerance"],
        "expected_effect": {
            "kind": "navigate_to_pose",
            "final_frame": "map",
            "stop_required": True,
        },
    }


def navigation_display(goal: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": "rosclaw.action-display.v2",
        "title": "Move LIMO to a patrol waypoint",
        "summary": "Run daemon-owned preflight and send one bounded move_base goal.",
        "body": {"robot_id": "limo", "target_pose": dict(goal)},
        "risk_tier": "HIGH",
        "physical_effects": ["The mobile base will move and then stop at the waypoint."],
        "constraints": [
            "The readiness and body snapshots must remain valid.",
            "The goal must remain inside the configured patrol boundary.",
        ],
        "verification": ["Verify move_base completion and final stopped state."],
        "abort": ["Cancel before dispatch or use the certified hardware E-stop."],
    }
