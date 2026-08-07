#!/usr/bin/env python2
"""Look up the fixed map-to-base transform without exposing arbitrary TF access."""

import json
import math
import sys
import time

PROTOCOL = "rosclaw.limo.robot-pose-worker.v1"


def main():
    try:
        request = json.load(sys.stdin)
        if not isinstance(request, dict) or set(request) != {"protocol", "timeout_sec"}:
            raise ValueError("request fields must be exactly protocol and timeout_sec")
        if request.get("protocol") != PROTOCOL:
            raise ValueError("invalid robot pose worker protocol")
        timeout_sec = float(request.get("timeout_sec"))
        if math.isnan(timeout_sec) or math.isinf(timeout_sec) or not 0.1 <= timeout_sec <= 5.0:
            raise ValueError("timeout_sec must be within [0.1, 5.0]")

        import rospy
        import tf

        rospy.init_node("rosclaw_limo_robot_pose_worker", anonymous=True, disable_signals=True)
        listener = tf.TransformListener()
        listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(timeout_sec))
        translation, quaternion = listener.lookupTransform("map", "base_link", rospy.Time(0))
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        response = {
            "ok": True,
            "protocol": PROTOCOL,
            "parent_frame": "map",
            "child_frame": "base_link",
            "translation": {
                "x": float(translation[0]),
                "y": float(translation[1]),
                "z": float(translation[2]),
            },
            "rotation": {
                "x": float(quaternion[0]),
                "y": float(quaternion[1]),
                "z": float(quaternion[2]),
                "w": float(quaternion[3]),
                "yaw_rad": float(yaw),
            },
            "received_wall_time": time.time(),
        }
    except Exception as exc:
        response = {"ok": False, "protocol": PROTOCOL, "error": str(exc)}
    sys.stdout.write(json.dumps(response, separators=(",", ":"), sort_keys=True) + "\n")
    return 0 if response.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
