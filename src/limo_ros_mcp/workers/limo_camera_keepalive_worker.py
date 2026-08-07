#!/usr/bin/env python2
"""Keep the fixed Dabai color stream subscribed for one parent mission process."""

import json
import sys
import threading

PROTOCOL = "rosclaw.limo.camera-keepalive-worker.v1"


def main():
    import rospy
    from sensor_msgs.msg import Image

    ready = threading.Event()
    state = {"sequence": None}

    def observe(message):
        state["sequence"] = int(message.header.seq)
        ready.set()

    rospy.init_node("rosclaw_limo_camera_keepalive", anonymous=True, disable_signals=True)
    subscriber = rospy.Subscriber("/camera/color/image_raw", Image, observe, queue_size=1)
    if not ready.wait(12.0):
        sys.stdout.write(
            json.dumps({"ok": False, "protocol": PROTOCOL, "error": "camera stream timeout"}) + "\n"
        )
        sys.stdout.flush()
        subscriber.unregister()
        return 1
    sys.stdout.write(
        json.dumps({"ok": True, "protocol": PROTOCOL, "sequence": state["sequence"]}) + "\n"
    )
    sys.stdout.flush()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
