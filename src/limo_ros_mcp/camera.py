"""Bounded conversion of allowlisted ROS camera frames to MCP-safe PNG images."""

from __future__ import annotations

import base64
import hashlib
import math
import struct
import zlib
from dataclasses import dataclass
from typing import Any

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
MAX_SOURCE_PIXELS = 1920 * 1080
MAX_SOURCE_BYTES = 8_000_000
MIN_OUTPUT_DIMENSION = 160
MAX_OUTPUT_DIMENSION = 1280


@dataclass(frozen=True)
class EncodedCameraFrame:
    png: bytes
    metadata: dict[str, Any]


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)


def _png(width: int, height: int, rows: list[bytes], *, color_type: int) -> bytes:
    filtered = b"".join(b"\x00" + row for row in rows)
    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    return (
        PNG_SIGNATURE
        + _png_chunk(b"IHDR", header)
        + _png_chunk(b"IDAT", zlib.compress(filtered, level=6))
        + _png_chunk(b"IEND", b"")
    )


def _decode_data(value: Any) -> bytes:
    if not isinstance(value, str) or not value:
        raise ValueError("camera frame data must be a non-empty rosbridge base64 string")
    try:
        decoded = base64.b64decode(value, validate=True)
    except (ValueError, TypeError) as exc:
        raise ValueError("camera frame data is not valid base64") from exc
    if len(decoded) > MAX_SOURCE_BYTES:
        raise ValueError("camera frame exceeds the bounded source byte limit")
    return decoded


def encode_ros_image_png(
    message: dict[str, Any],
    *,
    max_dimension: int = 640,
) -> EncodedCameraFrame:
    """Convert one ROS ``sensor_msgs/Image`` JSON message to a bounded PNG.

    Only common 8-bit color/mono encodings are accepted. The conversion is
    deterministic, uses no native codec, and downsamples with nearest-neighbor
    selection when the source exceeds ``max_dimension``.
    """

    if isinstance(max_dimension, bool) or not isinstance(max_dimension, int):
        raise ValueError("max_dimension must be an integer")
    if not MIN_OUTPUT_DIMENSION <= max_dimension <= MAX_OUTPUT_DIMENSION:
        raise ValueError(
            f"max_dimension must be within [{MIN_OUTPUT_DIMENSION}, {MAX_OUTPUT_DIMENSION}]"
        )

    width = message.get("width")
    height = message.get("height")
    step = message.get("step")
    if (
        isinstance(width, bool)
        or not isinstance(width, int)
        or isinstance(height, bool)
        or not isinstance(height, int)
        or width <= 0
        or height <= 0
        or width * height > MAX_SOURCE_PIXELS
    ):
        raise ValueError("camera frame dimensions are invalid or exceed the pixel limit")

    encoding = str(message.get("encoding", "")).lower()
    layouts: dict[str, tuple[int, str]] = {
        "rgb8": (3, "rgb"),
        "bgr8": (3, "bgr"),
        "rgba8": (4, "rgba"),
        "bgra8": (4, "bgra"),
        "mono8": (1, "mono"),
        "8uc1": (1, "mono"),
    }
    layout = layouts.get(encoding)
    if layout is None:
        raise ValueError("camera frame encoding must be rgb8, bgr8, rgba8, bgra8, mono8, or 8UC1")
    bytes_per_pixel, channel_order = layout
    minimum_step = width * bytes_per_pixel
    if isinstance(step, bool) or not isinstance(step, int) or step < minimum_step:
        raise ValueError("camera frame step is smaller than the encoded row width")

    data = _decode_data(message.get("data"))
    required_bytes = step * height
    if required_bytes > MAX_SOURCE_BYTES or len(data) < required_bytes:
        raise ValueError("camera frame payload is truncated or exceeds the byte limit")

    scale = max(1, math.ceil(max(width, height) / max_dimension))
    output_width = (width + scale - 1) // scale
    output_height = (height + scale - 1) // scale
    rows: list[bytes] = []
    for output_y in range(output_height):
        source_y = min(output_y * scale, height - 1)
        source_row = data[source_y * step : source_y * step + minimum_step]
        row = bytearray()
        for output_x in range(output_width):
            source_x = min(output_x * scale, width - 1)
            offset = source_x * bytes_per_pixel
            pixel = source_row[offset : offset + bytes_per_pixel]
            if channel_order == "rgb":
                row.extend(pixel)
            elif channel_order == "bgr":
                row.extend((pixel[2], pixel[1], pixel[0]))
            elif channel_order == "rgba":
                row.extend(pixel[:3])
            elif channel_order == "bgra":
                row.extend((pixel[2], pixel[1], pixel[0]))
            else:
                row.append(pixel[0])
        rows.append(bytes(row))

    png = _png(
        output_width,
        output_height,
        rows,
        color_type=0 if channel_order == "mono" else 2,
    )
    raw_header = message.get("header")
    header: dict[str, Any] = raw_header if isinstance(raw_header, dict) else {}
    return EncodedCameraFrame(
        png=png,
        metadata={
            "schema_version": "limo.camera_frame.v1",
            "source_width": width,
            "source_height": height,
            "output_width": output_width,
            "output_height": output_height,
            "source_encoding": encoding,
            "downsample_factor": scale,
            "source_byte_count": required_bytes,
            "png_byte_count": len(png),
            "png_sha256": f"sha256:{hashlib.sha256(png).hexdigest()}",
            "frame_id": str(header.get("frame_id", "")),
            "sequence": header.get("seq"),
        },
    )
