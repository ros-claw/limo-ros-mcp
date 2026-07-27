"""Minimal read-only ROS 1 rosbridge client used by the LIMO MCP server."""

from __future__ import annotations

import json
import os
import socket
import ssl
import time
import uuid
from contextlib import closing
from typing import Any
from urllib.parse import urlparse

import websocket


def allowed_rosbridge_hosts() -> set[str]:
    """Return loopback plus hosts explicitly approved by the operator."""

    configured = os.environ.get("ROSCLAW_LIMO_ROSBRIDGE_ALLOWLIST", "")
    return {"127.0.0.1", "localhost", "::1"} | {
        item.strip().lower() for item in configured.split(",") if item.strip()
    }


def validate_rosbridge_endpoint(endpoint: str) -> str:
    """Allow only websocket endpoints on operator-approved hosts."""

    parsed = urlparse(endpoint)
    host = (parsed.hostname or "").lower()
    if parsed.scheme not in {"ws", "wss"}:
        raise ValueError("rosbridge endpoint must use ws:// or wss://")
    if not host or host not in allowed_rosbridge_hosts():
        raise ValueError(
            "rosbridge host is not allowlisted; set ROSCLAW_LIMO_ROSBRIDGE_ALLOWLIST "
            "from the operator environment"
        )
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError("rosbridge endpoint credentials, query, and fragment are not allowed")
    return endpoint


class RosbridgeReadOnlyClient:
    """A deliberately small rosbridge client with no publish or advertise API."""

    def __init__(
        self,
        endpoint: str,
        *,
        timeout_sec: float = 2.0,
        max_message_bytes: int = 2_000_000,
    ) -> None:
        self.endpoint = validate_rosbridge_endpoint(endpoint)
        self.timeout_sec = float(timeout_sec)
        self.max_message_bytes = int(max_message_bytes)

    def _connect(self) -> Any:
        # Approved robot endpoints are expected to be directly reachable.  Explicitly
        # bypass ambient HTTP proxy variables so loopback/LAN probes cannot be sent to
        # an unrelated corporate or development proxy.
        parsed = urlparse(self.endpoint)
        host = parsed.hostname or ""
        port = parsed.port or (443 if parsed.scheme == "wss" else 80)
        direct_socket = socket.create_connection((host, port), timeout=self.timeout_sec)
        try:
            if parsed.scheme == "wss":
                direct_socket = ssl.create_default_context().wrap_socket(
                    direct_socket,
                    server_hostname=host,
                )
            return websocket.create_connection(
                self.endpoint,
                timeout=self.timeout_sec,
                socket=direct_socket,
            )
        except Exception:
            direct_socket.close()
            raise

    def _receive_until(self, connection: Any, predicate: Any) -> dict[str, Any]:
        deadline = time.monotonic() + self.timeout_sec
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("rosbridge response timed out")
            connection.settimeout(remaining)
            raw = connection.recv()
            if not isinstance(raw, str):
                raise RuntimeError("rosbridge returned a non-text websocket frame")
            if len(raw.encode("utf-8")) > self.max_message_bytes:
                raise RuntimeError("rosbridge message exceeds the configured size limit")
            message = json.loads(raw)
            if isinstance(message, dict) and predicate(message):
                return message

    def call_service(self, service: str, args: dict[str, Any] | None = None) -> dict[str, Any]:
        request_id = f"limo-mcp-{uuid.uuid4()}"
        request = {
            "op": "call_service",
            "service": service,
            "args": args or {},
            "id": request_id,
        }
        with closing(self._connect()) as connection:
            connection.send(json.dumps(request, separators=(",", ":")))
            response = self._receive_until(
                connection,
                lambda item: item.get("op") == "service_response" and item.get("id") == request_id,
            )
        if response.get("result") is False:
            raise RuntimeError(str(response.get("values") or "rosbridge service call failed"))
        values = response.get("values", {})
        return values if isinstance(values, dict) else {"value": values}

    def probe(self) -> dict[str, Any]:
        topics = self.call_service("/rosapi/topics")
        try:
            nodes = self.call_service("/rosapi/nodes")
        except Exception:  # noqa: BLE001 - nodes are optional diagnostic evidence
            nodes = {"nodes": []}
        return {
            "topics": list(topics.get("topics") or []),
            "types": list(topics.get("types") or []),
            "nodes": list(nodes.get("nodes") or []),
        }

    def subscribe_once(self, topic: str, message_type: str) -> dict[str, Any]:
        subscription_id = f"limo-mcp-{uuid.uuid4()}"
        subscribe = {
            "op": "subscribe",
            "id": subscription_id,
            "topic": topic,
            "type": message_type,
            "queue_length": 1,
            "throttle_rate": 0,
        }
        unsubscribe = {"op": "unsubscribe", "id": subscription_id, "topic": topic}
        with closing(self._connect()) as connection:
            connection.send(json.dumps(subscribe, separators=(",", ":")))
            try:
                response = self._receive_until(
                    connection,
                    lambda item: item.get("op") == "publish" and item.get("topic") == topic,
                )
            finally:
                connection.send(json.dumps(unsubscribe, separators=(",", ":")))
        message = response.get("msg")
        if not isinstance(message, dict):
            raise RuntimeError("rosbridge topic response did not contain an object message")
        return message
