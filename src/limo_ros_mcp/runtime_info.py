"""Runtime provenance for detecting stale long-lived MCP processes."""

from __future__ import annotations

import hashlib
import json
import os
import socket
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


def _operatord_socket_ready(path: Path | None = None) -> bool:
    """Return whether operatord is accepting connections, not merely leaving a socket file."""

    socket_path = (
        path
        or Path(os.environ.get("ROSCLAW_HOME", str(Path.home() / ".rosclaw")))
        / "run"
        / "operatord.sock"
    )
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(0.2)
    try:
        client.connect(str(socket_path))
    except OSError:
        return False
    finally:
        client.close()
    return True


def interaction_plane_status(
    runtime: dict[str, Any], *, operator_socket_ready: bool | None = None
) -> dict[str, Any]:
    """Describe whether both rosclawd and live operatord confirmation RPCs are available."""

    broker = runtime.get("operator_proposals")
    broker_available = isinstance(broker, dict)
    daemon_running = runtime.get("running") is True
    operatord_available = (
        _operatord_socket_ready() if operator_socket_ready is None else operator_socket_ready
    )
    restart_required = daemon_running and not broker_available
    operatord_start_required = daemon_running and broker_available and not operatord_available
    if broker_available and operatord_available:
        reason = "rosclawd and the live operatord confirmation channel are available."
    elif broker_available and daemon_running:
        reason = (
            "rosclawd exposes Operator Broker RPCs, but operatord is not accepting "
            "connections; start operatord before requesting a REAL action."
        )
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
        "operatord_available": operatord_available,
        "real_confirmation_ready": daemon_running and broker_available and operatord_available,
        "daemon_restart_required": restart_required,
        "operatord_start_required": operatord_start_required,
        "reason": reason,
        "operator_proposals": broker if broker_available else None,
    }
