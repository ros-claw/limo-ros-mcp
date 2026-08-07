#!/usr/bin/env python2
# mypy: ignore-errors
"""Capture one allowlisted ROS image as a bounded PNG.

This helper is intentionally Python 2 compatible because ROS Melodic's rospy and
cv_bridge bindings are installed for Python 2 on the LIMO image.  It has no ROS
publisher and accepts no arbitrary topic, command, or output path.
"""

# This subprocess must remain valid Python 2 for ROS Melodic.  Its narrow JSON
# protocol and behavior are covered by the modern-Python parent tests instead.

import base64
import hashlib
import json
import sys

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

PROTOCOL = "rosclaw.limo.camera-capture-worker.v1"
STREAMS = {
    "color": ("/camera/color/image_raw", "bgr8"),
    "infrared": ("/camera/ir/image_raw", "mono8"),
}
MAX_SOURCE_PIXELS = 1920 * 1080
MAX_PNG_BYTES = 4000000


def _response(ok, **values):
    payload = {"ok": bool(ok), "protocol": PROTOCOL}
    payload.update(values)
    return payload


def main():
    try:
        request = json.load(sys.stdin)
        if not isinstance(request, dict) or request.get("protocol") != PROTOCOL:
            raise ValueError("invalid camera worker protocol")
        stream = request.get("stream")
        if stream not in STREAMS:
            raise ValueError("stream must be color or infrared")
        timeout_sec = float(request.get("timeout_sec", 5.0))
        if timeout_sec < 0.1 or timeout_sec > 10.0:
            raise ValueError("timeout_sec must be within [0.1, 10.0]")
        max_dimension = request.get("max_dimension", 640)
        if isinstance(max_dimension, bool) or not isinstance(max_dimension, int):
            raise ValueError("max_dimension must be an integer")
        if max_dimension < 160 or max_dimension > 1280:
            raise ValueError("max_dimension must be within [160, 1280]")

        topic, desired_encoding = STREAMS[stream]
        rospy.init_node("rosclaw_limo_camera_capture_worker", anonymous=True)
        message = rospy.wait_for_message(topic, Image, timeout=timeout_sec)
        if message.width <= 0 or message.height <= 0:
            raise ValueError("camera frame dimensions are invalid")
        if message.width * message.height > MAX_SOURCE_PIXELS:
            raise ValueError("camera frame exceeds the pixel limit")
        frame = CvBridge().imgmsg_to_cv2(message, desired_encoding=desired_encoding)
        scale = max(
            1, int((max(message.width, message.height) + max_dimension - 1) / max_dimension)
        )
        if scale > 1:
            output_width = int((message.width + scale - 1) / scale)
            output_height = int((message.height + scale - 1) / scale)
            frame = cv2.resize(
                frame,
                (output_width, output_height),
                interpolation=cv2.INTER_NEAREST,
            )
        else:
            output_width = int(message.width)
            output_height = int(message.height)
        encoded_ok, encoded = cv2.imencode(".png", frame, [cv2.IMWRITE_PNG_COMPRESSION, 6])
        if not encoded_ok:
            raise RuntimeError("OpenCV failed to encode the camera frame")
        png = encoded.tostring()
        if not png.startswith(b"\x89PNG\r\n\x1a\n") or len(png) > MAX_PNG_BYTES:
            raise RuntimeError("encoded camera PNG is invalid or exceeds the byte limit")
        response = _response(
            True,
            stream=stream,
            topic=topic,
            source_width=int(message.width),
            source_height=int(message.height),
            output_width=output_width,
            output_height=output_height,
            source_encoding=str(message.encoding),
            downsample_factor=scale,
            png_byte_count=len(png),
            png_sha256="sha256:" + hashlib.sha256(png).hexdigest(),
            frame_id=str(message.header.frame_id),
            sequence=int(message.header.seq),
            stamp_sec=float(message.header.stamp.to_sec()),
            png_base64=base64.b64encode(png).decode("ascii"),
        )
    except Exception as exc:
        response = _response(False, error=str(exc))
    print(json.dumps(response, separators=(",", ":"), sort_keys=True))
    return 0 if response.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
