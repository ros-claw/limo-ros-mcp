#!/usr/bin/env python2
"""Bounded daemon-owned ROS 1 worker for one LIMO move_base goal.

The worker accepts one exact JSON request on stdin and writes one JSON result
on stdout.  It has no generic topic-publish or arbitrary action surface.
"""

import json
import math
import os
import signal
import sys
import time

PROTOCOL = "rosclaw.limo.worker.v1"
SCHEMA = "limo.navigation.v2"
MAX_REQUEST_BYTES = 65536
MIN_OBSERVED_TRANSLATION_M = 0.02
MIN_OBSERVED_ROTATION_RAD = 0.03
MOTION_COMMAND_LINEAR_THRESHOLD_MPS = 0.03
MOTION_COMMAND_ANGULAR_THRESHOLD_RADPS = 0.08
MOTION_RESPONSE_GRACE_SEC = 3.0
RESULT_POLL_SEC = 0.1
_CANCEL_REQUESTED = False
try:
    STRING_TYPES = (basestring,)  # type: ignore[name-defined]  # noqa: F821
except NameError:
    STRING_TYPES = (str,)


class RequestError(Exception):
    pass


def _request_cancel(_signum=None, _frame=None):
    global _CANCEL_REQUESTED
    _CANCEL_REQUESTED = True


class MotionWatchdog:
    """Detect a sustained motion command with no odometry response."""

    def __init__(self, odom_pose_before, grace_sec=MOTION_RESPONSE_GRACE_SEC):
        self.odom_pose_before = dict(odom_pose_before)
        self.grace_sec = float(grace_sec)
        self.first_nonzero_command_at = None
        self.motion_observed = False

    def observe_command(self, message, observed_at=None):
        observed_at = time.time() if observed_at is None else float(observed_at)
        linear = math.hypot(message.linear.x, message.linear.y)
        angular = abs(message.angular.z)
        if (
            linear > MOTION_COMMAND_LINEAR_THRESHOLD_MPS
            or angular > MOTION_COMMAND_ANGULAR_THRESHOLD_RADPS
        ) and self.first_nonzero_command_at is None:
            self.first_nonzero_command_at = observed_at

    def observe_odometry(self, message):
        current = _pose_from_odometry(message)
        translation, rotation = _pose_delta(self.odom_pose_before, current)
        linear = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        angular = abs(message.twist.twist.angular.z)
        if (
            translation >= MIN_OBSERVED_TRANSLATION_M
            or rotation >= MIN_OBSERVED_ROTATION_RAD
            or linear > MOTION_COMMAND_LINEAR_THRESHOLD_MPS
            or angular > MOTION_COMMAND_ANGULAR_THRESHOLD_RADPS
        ):
            self.motion_observed = True

    def commanded_without_motion(self, observed_at=None):
        observed_at = time.time() if observed_at is None else float(observed_at)
        return (
            self.first_nonzero_command_at is not None
            and not self.motion_observed
            and observed_at - self.first_nonzero_command_at >= self.grace_sec
        )


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
            "goal_tolerance",
            "server_timeout_sec",
            "navigation_timeout_sec",
            "verification_timeout_sec",
        ]
    )
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise RequestError("unknown request fields: %s" % ", ".join(unknown))
    if raw.get("protocol") != PROTOCOL:
        raise RequestError("unsupported worker protocol")
    if raw.get("operation") != "NAVIGATE_TO_POSE":
        raise RequestError("unsupported operation")
    if raw.get("schema_version") != SCHEMA:
        raise RequestError("unsupported navigation schema")
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
    tolerance = raw.get("goal_tolerance")
    if not isinstance(tolerance, dict) or set(tolerance) != set(["xy_m", "yaw_rad"]):
        raise RequestError("goal_tolerance must contain exactly xy_m and yaw_rad")
    normalized_tolerance = {
        "xy_m": _finite("goal_tolerance.xy_m", tolerance.get("xy_m")),
        "yaw_rad": _finite("goal_tolerance.yaw_rad", tolerance.get("yaw_rad")),
    }
    if not 0.05 <= normalized_tolerance["xy_m"] <= 0.5:
        raise RequestError("goal_tolerance.xy_m must be within [0.05, 0.5]")
    if not 0.05 <= normalized_tolerance["yaw_rad"] <= 0.8:
        raise RequestError("goal_tolerance.yaw_rad must be within [0.05, 0.8]")
    timeouts = {}
    for name, lower, upper in (
        ("server_timeout_sec", 0.5, 10.0),
        ("navigation_timeout_sec", 5.0, 120.0),
        ("verification_timeout_sec", 0.5, 10.0),
    ):
        value = _finite(name, raw.get(name))
        if not lower <= value <= upper:
            raise RequestError("%s must be within [%s, %s]" % (name, lower, upper))
        timeouts[name] = value
    return {
        "protocol": PROTOCOL,
        "operation": "NAVIGATE_TO_POSE",
        "schema_version": SCHEMA,
        "action_id": action_id,
        "body_id": raw.get("body_id"),
        "body_snapshot_hash": body_hash,
        "target_pose": normalized_pose,
        "goal_tolerance": normalized_tolerance,
        "server_timeout_sec": timeouts["server_timeout_sec"],
        "navigation_timeout_sec": timeouts["navigation_timeout_sec"],
        "verification_timeout_sec": timeouts["verification_timeout_sec"],
    }


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _pose_from_amcl(message):
    return {
        "frame_id": message.header.frame_id,
        "x": message.pose.pose.position.x,
        "y": message.pose.pose.position.y,
        "yaw": _yaw_from_quaternion(message.pose.pose.orientation),
    }


def _pose_from_transform(translation, rotation):
    return {
        "frame_id": "map",
        "x": translation[0],
        "y": translation[1],
        "yaw": _yaw_from_quaternion(
            type(
                "Quaternion",
                (object,),
                {"x": rotation[0], "y": rotation[1], "z": rotation[2], "w": rotation[3]},
            )()
        ),
    }


def _pose_from_odometry(message):
    return {
        "frame_id": message.header.frame_id,
        "x": message.pose.pose.position.x,
        "y": message.pose.pose.position.y,
        "yaw": _yaw_from_quaternion(message.pose.pose.orientation),
    }


def _pose_error(observed, target):
    position_error = math.hypot(observed["x"] - target["x"], observed["y"] - target["y"])
    yaw_delta = observed["yaw"] - target["yaw"]
    yaw_error = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
    return position_error, yaw_error


def _pose_delta(before, after):
    translation = math.hypot(after["x"] - before["x"], after["y"] - before["y"])
    yaw_delta = after["yaw"] - before["yaw"]
    rotation = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
    return translation, rotation


def _active_goal_tolerance(rospy, requested):
    xy = rospy.get_param("/move_base/TrajectoryPlannerROS/xy_goal_tolerance", requested["xy_m"])
    yaw = rospy.get_param(
        "/move_base/TrajectoryPlannerROS/yaw_goal_tolerance", requested["yaw_rad"]
    )
    try:
        xy = float(xy)
        yaw = float(yaw)
    except (TypeError, ValueError):
        raise RequestError("active TrajectoryPlannerROS goal tolerance is malformed")
    if (
        math.isnan(xy)
        or math.isinf(xy)
        or math.isnan(yaw)
        or math.isinf(yaw)
        or xy <= 0.0
        or yaw <= 0.0
    ):
        raise RequestError("active TrajectoryPlannerROS goal tolerance is invalid")
    return {"xy_m": xy, "yaw_rad": yaw}


def _require_compatible_goal_tolerance(requested, active):
    """Reject contracts the active planner is not configured to satisfy."""

    if active["xy_m"] > requested["xy_m"] or active["yaw_rad"] > requested["yaw_rad"]:
        raise RequestError(
            "active TrajectoryPlannerROS goal tolerance exceeds approved verification tolerance"
        )


def _wait_for_stopped_odometry(rospy, odometry_type, timeout_sec):
    deadline = time.time() + timeout_sec
    last = None
    while not rospy.is_shutdown() and time.time() < deadline:
        remaining = max(0.05, deadline - time.time())
        last = rospy.wait_for_message("/odom", odometry_type, timeout=remaining)
        linear = math.hypot(last.twist.twist.linear.x, last.twist.twist.linear.y)
        angular = abs(last.twist.twist.angular.z)
        if linear <= 0.03 and angular <= 0.08:
            return last, linear, angular
    raise RequestError("base did not report a stopped odometry state")


def _wait_for_navigation_result(rospy, server, timeout_sec, watchdog):
    """Poll the action client so process signals and motion faults remain observable."""

    deadline = time.time() + timeout_sec
    while not rospy.is_shutdown() and time.time() < deadline:
        if _CANCEL_REQUESTED:
            return "interrupted"
        if watchdog.commanded_without_motion():
            return "no_motion"
        remaining = max(0.0, deadline - time.time())
        if server.wait_for_result(rospy.Duration(min(RESULT_POLL_SEC, remaining))):
            return "finished"
    return "interrupted" if _CANCEL_REQUESTED or rospy.is_shutdown() else "timeout"


def _cancel_goal_and_confirm_stop(rospy, server, odometry_type, timeout_sec):
    """Cancel the active goal before allowing the one-shot worker to exit."""

    server.cancel_goal()
    cancel_deadline = time.time() + min(1.0, timeout_sec)
    while not rospy.is_shutdown() and time.time() < cancel_deadline:
        remaining = max(0.0, cancel_deadline - time.time())
        if server.wait_for_result(rospy.Duration(min(RESULT_POLL_SEC, remaining))):
            break
    return _wait_for_stopped_odometry(rospy, odometry_type, timeout_sec)


def _amcl_is_post_dispatch(message, received_wall_time, dispatched_wall_time, dispatched_ros_sec):
    if message is None or received_wall_time is None or received_wall_time < dispatched_wall_time:
        return False
    try:
        stamp_sec = float(message.header.stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return False
    return stamp_sec >= dispatched_ros_sec


def _wait_for_post_dispatch_amcl(
    rospy, state, dispatched_wall_time, dispatched_ros_sec, timeout_sec
):
    deadline = time.time() + timeout_sec
    while not rospy.is_shutdown() and time.time() < deadline:
        message = state.get("message")
        received_wall_time = state.get("received_wall_time")
        if _amcl_is_post_dispatch(
            message,
            received_wall_time,
            dispatched_wall_time,
            dispatched_ros_sec,
        ):
            return message
        rospy.sleep(0.02)
    return None


def _run_ros(request):
    global _CANCEL_REQUESTED

    import actionlib
    import rospy
    import tf
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from limo_base.msg import LimoStatus
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    _CANCEL_REQUESTED = False
    previous_sigterm = signal.signal(signal.SIGTERM, _request_cancel)
    previous_sigint = signal.signal(signal.SIGINT, _request_cancel)
    rospy.init_node("rosclaw_limo_navigation_worker", anonymous=True, disable_signals=True)
    server = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
    amcl_subscriber = None
    command_subscriber = None
    odom_subscriber = None
    goal_dispatched = False
    goal_terminal = False
    try:
        if not server.wait_for_server(rospy.Duration(request["server_timeout_sec"])):
            raise RequestError("/move_base action server unavailable")

        amcl_state = {"message": None, "received_wall_time": None}

        def capture_amcl(message):
            amcl_state["message"] = message
            amcl_state["received_wall_time"] = time.time()

        amcl_subscriber = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, capture_amcl, queue_size=1
        )
        preflight_started = time.time()
        scan = rospy.wait_for_message("/scan", LaserScan, timeout=2.0)
        odom_before = rospy.wait_for_message("/odom", Odometry, timeout=2.0)
        status = rospy.wait_for_message("/limo_status", LimoStatus, timeout=2.0)
        error_code = int(getattr(status, "error_code", -1))
        if error_code != 0:
            raise RequestError("LIMO chassis reports non-zero error_code")
        if not scan.ranges:
            raise RequestError("lidar scan contains no ranges")

        listener = tf.TransformListener()
        listener.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(2.0))
        listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(2.0))
        map_to_odom = listener.lookupTransform("map", "odom", rospy.Time(0))
        map_to_base_before = listener.lookupTransform("map", "base_link", rospy.Time(0))

        pose = request["target_pose"]
        map_pose_before = _pose_from_transform(map_to_base_before[0], map_to_base_before[1])
        odom_pose_before = _pose_from_odometry(odom_before)
        initial_position_error, initial_yaw_error = _pose_error(map_pose_before, pose)
        active_tolerance = _active_goal_tolerance(rospy, request["goal_tolerance"])
        requested_tolerance = request["goal_tolerance"]
        _require_compatible_goal_tolerance(requested_tolerance, active_tolerance)
        goal_already_satisfied = (
            initial_position_error <= active_tolerance["xy_m"]
            and initial_yaw_error <= active_tolerance["yaw_rad"]
        )
        requested_goal_satisfied = (
            initial_position_error <= requested_tolerance["xy_m"]
            and initial_yaw_error <= requested_tolerance["yaw_rad"]
        )
        if goal_already_satisfied and not requested_goal_satisfied:
            raise RequestError(
                "goal is inside active planner tolerance but outside approved verification tolerance"
            )
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = pose["x"]
        goal.target_pose.pose.position.y = pose["y"]
        goal.target_pose.pose.orientation.z = math.sin(pose["yaw"] / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(pose["yaw"] / 2.0)
        watchdog = MotionWatchdog(odom_pose_before)
        command_subscriber = rospy.Subscriber(
            "/cmd_vel", Twist, watchdog.observe_command, queue_size=1
        )
        odom_subscriber = rospy.Subscriber(
            "/odom", Odometry, watchdog.observe_odometry, queue_size=1
        )
        amcl_observed_before_dispatch = amcl_state.get("message") is not None
        dispatched_ros_sec = rospy.Time.now().to_sec()
        dispatched_wall = time.time()
        server.send_goal(goal)
        goal_dispatched = True
        outcome = _wait_for_navigation_result(
            rospy, server, request["navigation_timeout_sec"], watchdog
        )
        if outcome != "finished":
            _cancel_goal_and_confirm_stop(
                rospy, server, Odometry, request["verification_timeout_sec"]
            )
            goal_terminal = True
            if outcome == "no_motion":
                raise RequestError(
                    "LIMO motion command remained non-zero for %.1f sec without odometry response"
                    % MOTION_RESPONSE_GRACE_SEC
                )
            if outcome == "interrupted":
                raise RequestError("navigation worker interrupted; move_base goal cancelled")
            raise RequestError("move_base timed out and the goal was cancelled")
        goal_terminal = True
        terminal_state = int(server.get_state())
        terminal_text = str(server.get_goal_status_text())
        if terminal_state != 3:
            _cancel_goal_and_confirm_stop(
                rospy, server, Odometry, request["verification_timeout_sec"]
            )
            raise RequestError("move_base finished without SUCCEEDED: %s" % terminal_text)

        odom_after, stopped_linear, stopped_angular = _wait_for_stopped_odometry(
            rospy, Odometry, request["verification_timeout_sec"]
        )
        amcl_after = _wait_for_post_dispatch_amcl(
            rospy,
            amcl_state,
            dispatched_wall,
            dispatched_ros_sec,
            min(0.5, request["verification_timeout_sec"]),
        )
        map_to_base_after = listener.lookupTransform("map", "base_link", rospy.Time(0))
        if amcl_after is not None:
            observed = _pose_from_amcl(amcl_after)
            observed_source = "amcl_pose"
        else:
            observed = _pose_from_transform(map_to_base_after[0], map_to_base_after[1])
            observed_source = "tf_map_to_base"
        position_error, yaw_error = _pose_error(observed, pose)
        map_translation, map_rotation = _pose_delta(
            map_pose_before, _pose_from_transform(map_to_base_after[0], map_to_base_after[1])
        )
        odom_translation, odom_rotation = _pose_delta(
            odom_pose_before, _pose_from_odometry(odom_after)
        )
        motion_observed = (
            odom_translation >= MIN_OBSERVED_TRANSLATION_M
            or odom_rotation >= MIN_OBSERVED_ROTATION_RAD
        )
        movement_expected = not goal_already_satisfied
        tolerance = request["goal_tolerance"]
        if observed["frame_id"] != "map":
            raise RequestError("post-navigation final pose is not in map frame")
        if position_error > tolerance["xy_m"] or yaw_error > tolerance["yaw_rad"]:
            raise RequestError("post-navigation final pose is outside requested tolerance")
        return {
            "protocol": PROTOCOL,
            "ok": True,
            "accepted": True,
            "action_id": request["action_id"],
            "operation": request["operation"],
            "action_server": "/move_base",
            "terminal_state": terminal_state,
            "terminal_text": terminal_text,
            "preflight_started_wall_time": preflight_started,
            "dispatched_wall_time": dispatched_wall,
            "completed_wall_time": time.time(),
            "preflight": {
                "amcl_pose_observed_before_dispatch": amcl_observed_before_dispatch,
                "odom_frame_id": odom_before.header.frame_id,
                "lidar_range_count": len(scan.ranges),
                "chassis_error_code": error_code,
                "map_to_odom": {
                    "translation": list(map_to_odom[0]),
                    "rotation": list(map_to_odom[1]),
                },
                "map_to_base": {
                    "translation": list(map_to_base_before[0]),
                    "rotation": list(map_to_base_before[1]),
                },
                "initial_goal_error": {
                    "position_m": initial_position_error,
                    "yaw_rad": initial_yaw_error,
                },
                "active_goal_tolerance": active_tolerance,
                "goal_already_satisfied": goal_already_satisfied,
                "requested_goal_satisfied": requested_goal_satisfied,
            },
            "observed_final_pose": observed,
            "observed_final_pose_source": observed_source,
            "amcl_pose_observed_after_dispatch": amcl_after is not None,
            "position_error_m": position_error,
            "yaw_error_rad": yaw_error,
            "stopped_odometry": {
                "linear_speed_mps": stopped_linear,
                "angular_speed_radps": stopped_angular,
            },
            "map_to_base_after": {
                "translation": list(map_to_base_after[0]),
                "rotation": list(map_to_base_after[1]),
            },
            "motion_evidence": {
                "movement_expected": movement_expected,
                "motion_observed": motion_observed,
                "odom_translation_m": odom_translation,
                "odom_rotation_rad": odom_rotation,
                "map_translation_m": map_translation,
                "map_rotation_rad": map_rotation,
                "translation_threshold_m": MIN_OBSERVED_TRANSLATION_M,
                "rotation_threshold_rad": MIN_OBSERVED_ROTATION_RAD,
                "physical_motion_independently_observed": False,
            },
        }
    finally:
        if goal_dispatched and not goal_terminal:
            try:  # noqa: SIM105 - contextlib.suppress is unavailable on Python 2.
                _cancel_goal_and_confirm_stop(
                    rospy, server, Odometry, request["verification_timeout_sec"]
                )
            except Exception:
                pass
        for subscriber in (odom_subscriber, command_subscriber, amcl_subscriber):
            if subscriber is not None:
                subscriber.unregister()
        signal.signal(signal.SIGTERM, previous_sigterm)
        signal.signal(signal.SIGINT, previous_sigint)


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
                    "error_code": "LIMO_NAVIGATION_WORKER_FAILED",
                    "error": str(exc),
                },
                sort_keys=True,
                separators=(",", ":"),
            )
        )
        sys.exit(1)
