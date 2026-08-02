"""Runtime provenance for detecting stale long-lived MCP processes."""

from __future__ import annotations

import hashlib
import json
import os
import uuid
from datetime import UTC, datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Any

_PACKAGE_ROOT = Path(__file__).resolve().parent


def _source_fingerprint(package_root: Path = _PACKAGE_ROOT) -> str | None:
    """Hash executable package sources without relying on Git being installed."""

    digest = hashlib.sha256()
    try:
        sources = sorted(package_root.rglob("*.py"))
        if not sources:
            return None
        for path in sources:
            digest.update(path.relative_to(package_root).as_posix().encode("utf-8"))
            digest.update(b"\0")
            digest.update(path.read_bytes())
            digest.update(b"\0")
    except OSError:
        return None
    return f"sha256:{digest.hexdigest()}"


def _declared_package_version(package_root: Path = _PACKAGE_ROOT) -> str:
    candidates = (
        package_root / "data" / "manifest.json",
        package_root.parents[1] / "manifest.json",
    )
    for candidate in candidates:
        try:
            payload = json.loads(candidate.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        declared = payload.get("version")
        if isinstance(declared, str) and declared:
            return declared
    return "0+unknown"


try:
    _DISTRIBUTION_VERSION = version("limo-ros-mcp")
except PackageNotFoundError:  # pragma: no cover - editable/dev fallback
    _DISTRIBUTION_VERSION = "0+unknown"

_PACKAGE_VERSION = _declared_package_version()
_SERVER_INSTANCE_ID = f"limo-mcp-{uuid.uuid4().hex}"
_SERVER_STARTED_AT = datetime.now(UTC).isoformat()
_STARTUP_SOURCE_FINGERPRINT = _source_fingerprint()


def mcp_process_status() -> dict[str, Any]:
    """Return bounded process identity and whether on-disk code changed after startup."""

    current_fingerprint = _source_fingerprint()
    source_changed = bool(
        _STARTUP_SOURCE_FINGERPRINT
        and current_fingerprint
        and current_fingerprint != _STARTUP_SOURCE_FINGERPRINT
    )
    return {
        "schema_version": "limo.mcp-process.v1",
        "server_name": "rosclaw-limo",
        "package_version": _PACKAGE_VERSION,
        "distribution_version": _DISTRIBUTION_VERSION,
        "installation_metadata_matches_source": (
            _DISTRIBUTION_VERSION == "0+unknown" or _DISTRIBUTION_VERSION == _PACKAGE_VERSION
        ),
        "pid": os.getpid(),
        "instance_id": _SERVER_INSTANCE_ID,
        "started_at": _SERVER_STARTED_AT,
        "startup_source_fingerprint": _STARTUP_SOURCE_FINGERPRINT,
        "current_source_fingerprint": current_fingerprint,
        "source_changed_since_start": source_changed,
        "restart_required": source_changed,
    }


def interaction_plane_status(runtime: dict[str, Any]) -> dict[str, Any]:
    """Describe whether the connected daemon exposes the Operator Broker RPCs."""

    broker = runtime.get("operator_proposals")
    broker_available = isinstance(broker, dict)
    daemon_running = runtime.get("running") is True
    restart_required = daemon_running and not broker_available
    if broker_available:
        reason = "The connected daemon exposes the Operator Broker control plane."
    elif daemon_running:
        reason = (
            "The running daemon predates the Operator Broker; restart rosclawd from the "
            "current ROSClaw checkout before using the refactored REAL confirmation flow."
        )
    else:
        reason = "rosclawd is not running."
    return {
        "schema_version": "limo.interaction-plane.v1",
        "operator_broker_available": broker_available,
        "real_confirmation_ready": daemon_running and broker_available,
        "daemon_restart_required": restart_required,
        "reason": reason,
        "operator_proposals": broker if broker_available else None,
    }
