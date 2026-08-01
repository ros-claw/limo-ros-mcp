"""Minimal read-only ROS 1 rosbridge client used by the LIMO MCP server."""

from __future__ import annotations

import json
import os
import socket
import ssl
import struct
import time
import uuid
import zlib
from base64 import b64decode
from contextlib import closing
from typing import Any
from urllib.parse import urlparse

import websocket

_PNG_COMPRESSED_TOPICS = frozenset({"/map", "/move_base/global_costmap/costmap"})


def _paeth_predictor(left: int, above: int, upper_left: int) -> int:
    estimate = left + above - upper_left
    left_distance = abs(estimate - left)
    above_distance = abs(estimate - above)
    upper_left_distance = abs(estimate - upper_left)
    if left_distance <= above_distance and left_distance <= upper_left_distance:
        return left
    if above_distance <= upper_left_distance:
        return above
    return upper_left


def _decode_rosbridge_png(data: str, *, max_decoded_bytes: int) -> dict[str, Any]:
    """Decode rosbridge's RGB-PNG wrapped JSON with a bounded stdlib decoder."""

    encoded = b64decode(data, validate=True)
    if not encoded.startswith(b"\x89PNG\r\n\x1a\n"):
        raise RuntimeError("rosbridge PNG response has an invalid signature")

    offset = 8
    width = height = bit_depth = color_type = interlace = None
    compressed_parts: list[bytes] = []
    while offset + 12 <= len(encoded):
        length = struct.unpack(">I", encoded[offset : offset + 4])[0]
        chunk_type = encoded[offset + 4 : offset + 8]
        end = offset + 12 + length
        if end > len(encoded):
            raise RuntimeError("rosbridge PNG response is truncated")
        chunk = encoded[offset + 8 : offset + 8 + length]
        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, _compression, _filter, interlace = struct.unpack(
                ">IIBBBBB", chunk
            )
        elif chunk_type == b"IDAT":
            compressed_parts.append(chunk)
        elif chunk_type == b"IEND":
            break
        offset = end

    if (
        not isinstance(width, int)
        or not isinstance(height, int)
        or width <= 0
        or height <= 0
        or bit_depth != 8
        or color_type != 2
        or interlace != 0
    ):
        raise RuntimeError("rosbridge PNG response is not bounded 8-bit RGB data")
    row_bytes = width * 3
    expected_bytes = height * (row_bytes + 1)
    pixel_bytes = height * row_bytes
    if pixel_bytes > max_decoded_bytes:
        raise RuntimeError("rosbridge decompressed message exceeds the configured size limit")

    decompressor = zlib.decompressobj()
    filtered = decompressor.decompress(b"".join(compressed_parts), expected_bytes + 1)
    if len(filtered) != expected_bytes or not decompressor.eof:
        raise RuntimeError("rosbridge PNG response has an invalid decompressed size")

    decoded = bytearray()
    previous = bytearray(row_bytes)
    cursor = 0
    for _row in range(height):
        filter_type = filtered[cursor]
        scanline = filtered[cursor + 1 : cursor + 1 + row_bytes]
        cursor += row_bytes + 1
        reconstructed = bytearray(row_bytes)
        for index, value in enumerate(scanline):
            left = reconstructed[index - 3] if index >= 3 else 0
            above = previous[index]
            upper_left = previous[index - 3] if index >= 3 else 0
            if filter_type == 0:
                predictor = 0
            elif filter_type == 1:
                predictor = left
            elif filter_type == 2:
                predictor = above
            elif filter_type == 3:
                predictor = (left + above) // 2
            elif filter_type == 4:
                predictor = _paeth_predictor(left, above, upper_left)
            else:
                raise RuntimeError("rosbridge PNG response uses an unsupported filter")
            reconstructed[index] = (value + predictor) & 0xFF
        decoded.extend(reconstructed)
        previous = reconstructed

    try:
        message = json.loads(bytes(decoded).rstrip(b"\n").decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeError("rosbridge PNG response does not contain valid JSON") from exc
    if not isinstance(message, dict):
        raise RuntimeError("rosbridge PNG response did not contain an object message")
    return message


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
        max_decompressed_message_bytes: int = 32_000_000,
    ) -> None:
        self.endpoint = validate_rosbridge_endpoint(endpoint)
        self.timeout_sec = float(timeout_sec)
        self.max_message_bytes = int(max_message_bytes)
        self.max_decompressed_message_bytes = int(max_decompressed_message_bytes)
        self.transport_generation = f"rosbridge-{uuid.uuid4()}"

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
            if isinstance(message, dict) and message.get("op") == "png":
                message = _decode_rosbridge_png(
                    str(message.get("data", "")),
                    max_decoded_bytes=self.max_decompressed_message_bytes,
                )
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

    def topic_info(self, topic: str, expected_type: str) -> dict[str, Any]:
        """Read rosapi type and endpoint metadata for one contract topic."""

        topic_type = self.call_service("/rosapi/topic_type", {"topic": topic})
        observed_type = str(topic_type.get("type", ""))
        if observed_type and observed_type != expected_type:
            raise RuntimeError(
                f"ROS topic type mismatch for {topic}: expected {expected_type}, got {observed_type}"
            )
        publishers = self.call_service("/rosapi/publishers", {"topic": topic})
        subscribers = self.call_service("/rosapi/subscribers", {"topic": topic})
        return {
            "topic": topic,
            "expected_type": expected_type,
            "observed_type": observed_type,
            "publishers": list(publishers.get("publishers") or []),
            "subscribers": list(subscribers.get("subscribers") or []),
        }

    def subscribe_many(
        self,
        topic: str,
        message_type: str,
        *,
        count: int,
    ) -> list[dict[str, Any]]:
        """Collect a bounded number of messages from one contract topic."""

        subscription_id = f"limo-mcp-{uuid.uuid4()}"
        subscribe = {
            "op": "subscribe",
            "id": subscription_id,
            "topic": topic,
            "type": message_type,
            "queue_length": 1,
            "throttle_rate": 0,
        }
        if topic in _PNG_COMPRESSED_TOPICS:
            subscribe["compression"] = "png"
        unsubscribe = {"op": "unsubscribe", "id": subscription_id, "topic": topic}
        messages: list[dict[str, Any]] = []
        with closing(self._connect()) as connection:
            connection.send(json.dumps(subscribe, separators=(",", ":")))
            try:
                for _index in range(count):
                    response = self._receive_until(
                        connection,
                        lambda item: item.get("op") == "publish" and item.get("topic") == topic,
                    )
                    message = response.get("msg")
                    if not isinstance(message, dict):
                        raise RuntimeError(
                            "rosbridge topic response did not contain an object message"
                        )
                    messages.append(message)
            finally:
                connection.send(json.dumps(unsubscribe, separators=(",", ":")))
        return messages

    def subscribe_once(self, topic: str, message_type: str) -> dict[str, Any]:
        return self.subscribe_many(topic, message_type, count=1)[0]
