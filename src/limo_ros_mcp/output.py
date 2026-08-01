"""Bounded public output projections for context-constrained MCP clients."""

from __future__ import annotations

from typing import Any


def compact_readiness(result: dict[str, Any], *, detail: str = "summary") -> dict[str, Any]:
    """Keep default readiness output small while retaining the sealed snapshot reference."""

    if detail == "full":
        return result
    if detail != "summary":
        raise ValueError("detail must be summary or full")
    readiness = result.get("readiness")
    if not isinstance(readiness, dict):
        return result
    observations = readiness.get("observations")
    freshness: dict[str, Any] = {}
    if isinstance(observations, dict):
        for name, value in observations.items():
            if isinstance(value, dict):
                freshness[str(name)] = {
                    key: value.get(key) for key in ("age_sec", "clock_verified", "summary_hash")
                }
    compact = {
        key: readiness.get(key)
        for key in (
            "schema_version",
            "snapshot_id",
            "snapshot_hash",
            "body_id",
            "body_snapshot_hash",
            "created_at",
            "expires_at",
            "state",
            "ready",
            "blockers",
            "warnings",
            "policy_id",
            "policy_hash",
            "transport_binding",
            "observation_window",
        )
        if key in readiness
    }
    compact["freshness"] = freshness
    return {
        key: value
        for key, value in {
            "ok": result.get("ok"),
            "transport": result.get("transport"),
            "transport_generation": result.get("transport_generation"),
            "available_count": result.get("available_count"),
            "missing_count": result.get("missing_count"),
            "cache_hits": result.get("cache_hits", []),
            "failure_names": sorted((result.get("failures") or {}).keys()),
            "readiness": compact,
            "command_dispatched": False,
            "usable_for_real_execution": False,
        }.items()
        if value is not None
    }
