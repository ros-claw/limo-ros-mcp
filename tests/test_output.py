"""Bounded MCP output projections."""

from __future__ import annotations

import json

from limo_ros_mcp.output import compact_readiness


def test_default_readiness_projection_is_bounded_and_keeps_snapshot_reference() -> None:
    result = {
        "ok": True,
        "transport": "roscli",
        "available_count": 12,
        "failures": {},
        "readiness": {
            "schema_version": "limo.readiness.v1",
            "snapshot_hash": "sha256:ready",
            "state": "READY",
            "ready": True,
            "blockers": [],
            "warnings": [],
            "observations": {
                f"topic-{index}": {
                    "age_sec": 0.1,
                    "clock_verified": True,
                    "summary_hash": f"sha256:{index}",
                    "summary": {"large": "x" * 20_000},
                }
                for index in range(12)
            },
            "checks": [{"large": "x" * 20_000}],
        },
    }
    compact = compact_readiness(result)
    assert compact["readiness"]["snapshot_hash"] == "sha256:ready"
    assert "summary" not in compact["readiness"]["freshness"]["topic-0"]
    assert len(json.dumps(compact).encode()) < 10_000
