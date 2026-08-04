"""Bounded camera-frame conversion tests."""

from __future__ import annotations

import base64
import struct
import zlib

import pytest

from limo_ros_mcp.camera import PNG_SIGNATURE, encode_ros_image_png


def _image(*, width: int, height: int, encoding: str, step: int, data: bytes) -> dict:
    return {
        "header": {"seq": 7, "frame_id": "camera_color_optical_frame"},
        "width": width,
        "height": height,
        "encoding": encoding,
        "step": step,
        "data": base64.b64encode(data).decode(),
    }


def _decoded_scanlines(png: bytes) -> tuple[tuple[int, int, int], bytes]:
    assert png.startswith(PNG_SIGNATURE)
    offset = len(PNG_SIGNATURE)
    width = height = color_type = 0
    compressed = bytearray()
    while offset < len(png):
        length = struct.unpack(">I", png[offset : offset + 4])[0]
        kind = png[offset + 4 : offset + 8]
        payload = png[offset + 8 : offset + 8 + length]
        if kind == b"IHDR":
            width, height, _depth, color_type, *_rest = struct.unpack(">IIBBBBB", payload)
        elif kind == b"IDAT":
            compressed.extend(payload)
        elif kind == b"IEND":
            break
        offset += 12 + length
    return (width, height, color_type), zlib.decompress(bytes(compressed))


def test_bgr_frame_is_converted_to_rgb_png() -> None:
    encoded = encode_ros_image_png(
        _image(width=2, height=1, encoding="bgr8", step=6, data=bytes([3, 2, 1, 6, 5, 4]))
    )

    header, scanlines = _decoded_scanlines(encoded.png)
    assert header == (2, 1, 2)
    assert scanlines == bytes([0, 1, 2, 3, 4, 5, 6])
    assert encoded.metadata["png_sha256"].startswith("sha256:")
    assert encoded.metadata["frame_id"] == "camera_color_optical_frame"


def test_frame_is_downsampled_to_bounded_dimension() -> None:
    source = bytes([20, 30, 40] * 320)
    encoded = encode_ros_image_png(
        _image(width=320, height=1, encoding="rgb8", step=960, data=source),
        max_dimension=160,
    )
    assert encoded.metadata["output_width"] == 160
    assert encoded.metadata["output_height"] == 1
    assert encoded.metadata["downsample_factor"] == 2


@pytest.mark.parametrize(
    ("encoding", "step", "data", "match"),
    [
        ("32FC1", 8, bytes(8), "encoding"),
        ("rgb8", 5, bytes(6), "step"),
        ("rgb8", 6, bytes(5), "truncated"),
    ],
)
def test_invalid_frame_is_rejected(encoding: str, step: int, data: bytes, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        encode_ros_image_png(_image(width=2, height=1, encoding=encoding, step=step, data=data))


def test_dimension_bound_is_validated() -> None:
    with pytest.raises(ValueError, match="max_dimension"):
        encode_ros_image_png(
            _image(width=1, height=1, encoding="mono8", step=1, data=b"\x00"),
            max_dimension=32,
        )
