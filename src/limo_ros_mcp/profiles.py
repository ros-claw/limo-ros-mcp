"""Bounded MCP tool profiles for clients with different discovery capabilities."""

from __future__ import annotations

CORE_TOOLS = {
    "limo_get_context",
    "limo_observe",
    "limo_get_readiness",
    "limo_validate_navigation_goal",
    "limo_request_navigation",
    "limo_request_initial_pose",
    "limo_request_tone",
    "limo_get_action_status",
    "limo_get_execution_receipt",
    "limo_emergency_stop",
}

CONTROL_TOOLS = {
    "limo_request_navigation",
    "limo_request_initial_pose",
    "limo_request_tone",
    "limo_emergency_stop",
}

COMPAT_TOOLS = {"limo_get_patrol_readiness"}


def select_tools(
    profile: str,
    available: set[str],
    *,
    compat_tools: bool = False,
) -> set[str]:
    """Select a stable tool set after all implementations are registered."""

    normalized = str(profile).lower()
    if normalized == "core":
        selected = set(CORE_TOOLS)
    elif normalized == "inspection":
        selected = available - CONTROL_TOOLS - COMPAT_TOOLS
    elif normalized == "full":
        selected = set(available)
    else:
        raise ValueError("profile must be core, inspection, or full")
    if compat_tools:
        selected |= COMPAT_TOOLS
    return selected & available
