#!/usr/bin/env python2
"""Bounded read-only ROS 1 sampler for freshness-critical LIMO topics.

The Python 3 MCP process cannot import the ROS Melodic Python 2 message stack.
This helper accepts one exact JSON request and returns one compact message. It
has no publish, service-call, parameter-write, or arbitrary-topic surface.
"""

import json
import math
import sys
import time

PROTOCOL = "rosclaw.limo.readiness-worker.v1"
MAX_REQUEST_BYTES = 4096
MAX_RESPONSE_BYTES = 262144


class RequestError(Exception):
    pass


def _finite_timeout(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RequestError("timeout_sec must be numeric")
    result = float(value)
    if math.isnan(result) or math.isinf(result) or not 0.1 <= result <= 10.0:
        raise RequestError("timeout_sec must be within [0.1, 10]")
    return result


def validate_request(raw):
    if not isinstance(raw, dict):
        raise RequestError("request must be an object")
    if set(raw) != set(["protocol", "observation", "timeout_sec"]):
        raise RequestError("request fields must be exactly protocol, observation, timeout_sec")
    if raw.get("protocol") != PROTOCOL:
        raise RequestError("unsupported worker protocol")
    observation = raw.get("observation")
    if observation not in ("global_costmap", "laser_scan"):
        raise RequestError("unsupported observation")
    return {
        "protocol": PROTOCOL,
        "observation": observation,
        "timeout_sec": _finite_timeout(raw.get("timeout_sec")),
    }


def _header(message):
    return {
        "seq": int(message.header.seq),
        "stamp": {
            "secs": int(message.header.stamp.secs),
            "nsecs": int(message.header.stamp.nsecs),
        },
        "frame_id": message.header.frame_id,
    }


def _json_range(value):
    result = float(value)
    if math.isnan(result):
        return "nan"
    if math.isinf(result):
        return "inf" if result > 0.0 else "-inf"
    return result


def sample(request):
    import rospy
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan

    rospy.init_node("rosclaw_limo_readiness_worker", anonymous=True, disable_signals=True)
    if request["observation"] == "global_costmap":
        message = rospy.wait_for_message(
            "/move_base/global_costmap/costmap",
            OccupancyGrid,
            timeout=request["timeout_sec"],
        )
        compact = {
            "header": _header(message),
            "info": {
                "resolution": float(message.info.resolution),
                "width": int(message.info.width),
                "height": int(message.info.height),
                "origin": {
                    "position": {
                        "x": float(message.info.origin.position.x),
                        "y": float(message.info.origin.position.y),
                        "z": float(message.info.origin.position.z),
                    },
                    "orientation": {
                        "x": float(message.info.origin.orientation.x),
                        "y": float(message.info.origin.orientation.y),
                        "z": float(message.info.origin.orientation.z),
                        "w": float(message.info.origin.orientation.w),
                    },
                },
            },
            "data": "<array type: int8, length: %d>" % len(message.data),
        }
    else:
        message = rospy.wait_for_message("/scan", LaserScan, timeout=request["timeout_sec"])
        compact = {
            "header": _header(message),
            "angle_min": float(message.angle_min),
            "angle_max": float(message.angle_max),
            "angle_increment": float(message.angle_increment),
            "time_increment": float(message.time_increment),
            "scan_time": float(message.scan_time),
            "range_min": float(message.range_min),
            "range_max": float(message.range_max),
            "ranges": [_json_range(value) for value in message.ranges],
        }
    return {
        "ok": True,
        "protocol": PROTOCOL,
        "observation": request["observation"],
        "received_wall_time": time.time(),
        "message": compact,
    }


def main():
    try:
        payload = sys.stdin.read(MAX_REQUEST_BYTES + 1)
        if len(payload) > MAX_REQUEST_BYTES:
            raise RequestError("request exceeds size limit")
        request = validate_request(json.loads(payload))
        result = sample(request)
    except Exception as exc:
        result = {
            "ok": False,
            "protocol": PROTOCOL,
            "error": "%s: %s" % (exc.__class__.__name__, exc),
        }
    encoded = json.dumps(result, separators=(",", ":"), allow_nan=False)
    if len(encoded) > MAX_RESPONSE_BYTES:
        encoded = json.dumps(
            {"ok": False, "protocol": PROTOCOL, "error": "response exceeds size limit"},
            separators=(",", ":"),
        )
    sys.stdout.write(encoded + "\n")
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
