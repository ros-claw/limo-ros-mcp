"""Localization initialization action-intent construction."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


def initial_pose_arguments(
    pose: Mapping[str, Any], validation: Mapping[str, Any]
) -> dict[str, Any]:
    return {
        "schema_version": "limo.initial-pose.v1",
        "target_pose": dict(pose),
        "route_policy_id": validation["route_policy_id"],
        "route_policy_hash": validation["route_policy_hash"],
        "map_id": validation["map_id"],
        "map_image_hash": validation["map_image_hash"],
        "covariance_diagonal": [0.25, 0.25, 0.0, 0.0, 0.0, 0.0685],
        "expected_effect": {
            "kind": "localization_initialized",
            "final_frame": "map",
            "map_to_odom_required": True,
        },
    }


def initial_pose_display(pose: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": "rosclaw.action-display.v2",
        "title": "Initialize LIMO localization",
        "summary": "Publish one bounded /initialpose estimate through rosclawd.",
        "body": {"robot_id": "limo", "target_pose": dict(pose)},
        "risk_tier": "MEDIUM",
        "physical_effects": ["Localization changes; no drive command is requested."],
        "constraints": ["The pose must remain inside the configured map boundary."],
        "verification": ["Verify AMCL and map-to-odom convergence."],
        "abort": ["Re-submit the prior validated pose if localization diverges."],
    }
