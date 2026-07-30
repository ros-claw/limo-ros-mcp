#!/usr/bin/env python2
"""Bounded daemon-owned ROS 1 worker for LIMO localization initialization.

The worker accepts exactly one JSON request on stdin and writes exactly one
JSON result on stdout.  It intentionally implements no generic ROS publish
surface; the only supported operation is SET_INITIAL_POSE on /initialpose.
"""

import json
import math
import os
import sys
import time

PROTOCOL = "rosclaw.limo.worker.v1"
SCHEMA = "limo.initial-pose.v1"
MAX_REQUEST_BYTES = 65536
try:
    STRING_TYPES = (basestring,)  # type: ignore[name-defined]  # noqa: F821
except NameError:
    STRING_TYPES = (str,)


class RequestError(Exception):
    pass


def _finite(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RequestError("%s must be a number" % name)
    result = float(value)
    if math.isnan(result) or math.isinf(result):
        raise RequestError("%s must be finite" % name)
    return result


def validate_request(raw):
    if not isinstance(raw, dict):
        raise RequestError("request must be an object")
    allowed = set(
        [
            "protocol",
            "operation",
            "schema_version",
            "action_id",
            "body_id",
            "body_snapshot_hash",
            "target_pose",
            "covariance_diagonal",
            "subscriber_timeout_sec",
            "verification_timeout_sec",
        ]
    )
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise RequestError("unknown request fields: %s" % ", ".join(unknown))
    if raw.get("protocol") != PROTOCOL:
        raise RequestError("unsupported worker protocol")
    if raw.get("operation") != "SET_INITIAL_POSE":
        raise RequestError("unsupported operation")
    if raw.get("schema_version") != SCHEMA:
        raise RequestError("unsupported initial-pose schema")
    action_id = raw.get("action_id")
    body_hash = raw.get("body_snapshot_hash")
    if not isinstance(action_id, STRING_TYPES) or not action_id:
        raise RequestError("action_id is required")
    if not isinstance(body_hash, STRING_TYPES) or not body_hash:
        raise RequestError("body_snapshot_hash is required")
    pose = raw.get("target_pose")
    if not isinstance(pose, dict) or set(pose) != set(["frame_id", "x", "y", "yaw"]):
        raise RequestError("target_pose must contain exactly frame_id, x, y, yaw")
    if pose.get("frame_id") != "map":
        raise RequestError("target_pose frame_id must be map")
    normalized_pose = {
        "frame_id": "map",
        "x": _finite("x", pose.get("x")),
        "y": _finite("y", pose.get("y")),
        "yaw": _finite("yaw", pose.get("yaw")),
    }
    if not -math.pi <= normalized_pose["yaw"] <= math.pi:
        raise RequestError("yaw must be within [-pi, pi]")
    covariance = raw.get("covariance_diagonal")
    if not isinstance(covariance, list) or len(covariance) != 6:
        raise RequestError("covariance_diagonal must contain six values")
    normalized_covariance = [_finite("covariance", value) for value in covariance]
    if any(value < 0.0 or value > 100.0 for value in normalized_covariance):
        raise RequestError("covariance values must be within [0, 100]")
    subscriber_timeout = _finite("subscriber_timeout_sec", raw.get("subscriber_timeout_sec", 3.0))
    verification_timeout = _finite(
        "verification_timeout_sec", raw.get("verification_timeout_sec", 8.0)
    )
    if not 0.1 <= subscriber_timeout <= 10.0:
        raise RequestError("subscriber_timeout_sec must be within [0.1, 10]")
    if not 0.5 <= verification_timeout <= 30.0:
        raise RequestError("verification_timeout_sec must be within [0.5, 30]")
    return {
        "protocol": PROTOCOL,
        "operation": "SET_INITIAL_POSE",
        "schema_version": SCHEMA,
        "action_id": action_id,
        "body_id": raw.get("body_id"),
        "body_snapshot_hash": body_hash,
        "target_pose": normalized_pose,
        "covariance_diagonal": normalized_covariance,
        "subscriber_timeout_sec": subscriber_timeout,
        "verification_timeout_sec": verification_timeout,
    }


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _run_ros(request):
    import rospy
    import tf
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rospy.init_node("rosclaw_limo_initial_pose_worker", anonymous=True, disable_signals=True)
    publisher = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
    deadline = time.time() + request["subscriber_timeout_sec"]
    while (
        publisher.get_num_connections() < 1 and time.time() < deadline and not rospy.is_shutdown()
    ):
        time.sleep(0.05)
    if publisher.get_num_connections() < 1:
        raise RequestError("/initialpose has no subscriber")

    pose = request["target_pose"]
    message = PoseWithCovarianceStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "map"
    message.pose.pose.position.x = pose["x"]
    message.pose.pose.position.y = pose["y"]
    message.pose.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
    message.pose.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
    for index, value in enumerate(request["covariance_diagonal"]):
        message.pose.covariance[index * 6 + index] = value

    dispatched_wall = time.time()
    publisher.publish(message)
    observed = rospy.wait_for_message(
        "/amcl_pose", PoseWithCovarianceStamped, timeout=request["verification_timeout_sec"]
    )
    listener = tf.TransformListener()
    remaining = max(0.1, request["verification_timeout_sec"] - (time.time() - dispatched_wall))
    listener.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(remaining))
    translation, rotation = listener.lookupTransform("map", "odom", rospy.Time(0))
    observed_pose = {
        "frame_id": observed.header.frame_id,
        "x": observed.pose.pose.position.x,
        "y": observed.pose.pose.position.y,
        "yaw": _yaw_from_quaternion(observed.pose.pose.orientation),
    }
    return {
        "protocol": PROTOCOL,
        "ok": True,
        "accepted": True,
        "action_id": request["action_id"],
        "operation": request["operation"],
        "topic": "/initialpose",
        "subscriber_count": publisher.get_num_connections(),
        "dispatched_wall_time": dispatched_wall,
        "completed_wall_time": time.time(),
        "observed_amcl_pose": observed_pose,
        "observed_covariance": list(observed.pose.covariance),
        "map_to_odom": {
            "translation": list(translation),
            "rotation": list(rotation),
        },
    }


def main():
    raw_bytes = sys.stdin.read(MAX_REQUEST_BYTES + 1)
    if len(raw_bytes) > MAX_REQUEST_BYTES:
        raise RequestError("request exceeds byte limit")
    request = validate_request(json.loads(raw_bytes))
    if os.environ.get("ROSCLAW_LIMO_VALIDATE_ONLY") == "1":
        return {"protocol": PROTOCOL, "ok": True, "validated_request": request}
    return _run_ros(request)


if __name__ == "__main__":
    try:
        result = main()
        print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    except Exception as exc:
        print(
            json.dumps(
                {
                    "protocol": PROTOCOL,
                    "ok": False,
                    "error_code": "LIMO_WORKER_FAILED",
                    "error": str(exc),
                },
                sort_keys=True,
                separators=(",", ":"),
            )
        )
        sys.exit(1)
