"""Tests for the fixed-command local ROS 1 observation backend."""

from __future__ import annotations

import base64
import hashlib
import io
import json
import subprocess
from typing import Any

import pytest

from limo_ros_mcp.roscli import RosCliReadOnlyClient, build_ros1_cli_environment


def test_roscli_observation_uses_only_type_and_echo(monkeypatch: Any) -> None:
    calls: list[list[str]] = []
    environments: list[dict[str, str]] = []

    def run(command: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
        calls.append(command)
        environments.append(kwargs["env"])
        output = "sensor_msgs/Imu\n" if command[1] == "type" else "orientation:\n  w: 1.0\n---\n"
        return subprocess.CompletedProcess(command, 0, stdout=output, stderr="")

    monkeypatch.setattr(
        "limo_ros_mcp.roscli.Path.is_dir",
        lambda path: str(path).startswith("/opt/ros/melodic"),
    )
    monkeypatch.setattr("shutil.which", lambda name: f"/opt/ros/melodic/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    client = RosCliReadOnlyClient({"/imu": "sensor_msgs/Imu"})

    message = client.subscribe_once("/imu", "sensor_msgs/Imu")

    assert client.timeout_sec == 5.0
    assert message["orientation"]["w"] == 1.0
    assert [command[1:3] for command in calls] == [["type", "/imu"], ["echo", "-n"]]
    assert not any("pub" in command or "publish" in command for command in calls)
    assert all(environment["ROS_PYTHON_VERSION"] == "2" for environment in environments)
    assert all(
        "/opt/ros/melodic/lib/python2.7/dist-packages" in environment["PYTHONPATH"]
        for environment in environments
    )


def test_roscli_builds_melodic_environment_without_inherited_ros_vars(
    monkeypatch: Any,
) -> None:
    for name in (
        "PYTHONPATH",
        "ROS_PACKAGE_PATH",
        "ROS_MASTER_URI",
        "ROS_DISTRO",
        "ROS_VERSION",
        "ROS_PYTHON_VERSION",
        "ROS_ROOT",
        "ROS_ETC_DIR",
    ):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(
        "limo_ros_mcp.roscli.Path.is_dir",
        lambda path: str(path).startswith("/opt/ros/melodic"),
    )

    environment = build_ros1_cli_environment()

    assert environment["ROS_MASTER_URI"] == "http://localhost:11311"
    assert environment["ROS_DISTRO"] == "melodic"
    assert environment["ROS_PYTHON_VERSION"] == "2"
    assert "/opt/ros/melodic/lib/python2.7/dist-packages" in environment["PYTHONPATH"]


def test_transform_available_accepts_success_after_initial_failure(monkeypatch: Any) -> None:
    class Process:
        pid = 123

        def __init__(self) -> None:
            self.stdout = io.StringIO(
                "Failure at 0.000\n"
                "At time 12.3\n"
                "- Translation: [0.0, 0.0, 0.0]\n"
                "- Rotation: in Quaternion [0.0, 0.0, 0.0, 1.0]\n"
            )
            self.returncode: int | None = None

        def poll(self) -> int | None:
            return self.returncode

        def wait(self, *, timeout: float) -> int:
            del timeout
            self.returncode = -15
            return self.returncode

    monkeypatch.setattr("shutil.which", lambda name: f"/opt/ros/melodic/bin/{name}")
    monkeypatch.setattr("subprocess.Popen", lambda *_args, **_kwargs: Process())
    monkeypatch.setattr("select.select", lambda values, *_args: (values, [], []))
    monkeypatch.setattr("os.killpg", lambda *_args: None)
    client = RosCliReadOnlyClient({})

    assert client.transform_available("map", "odom") is True


def test_roscli_samples_multiple_yaml_documents(monkeypatch: Any) -> None:
    def run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[str]:
        output = (
            "nav_msgs/Odometry\n"
            if command[1] == "type"
            else "header:\n  seq: 1\n---\nheader:\n  seq: 2\n---\n"
        )
        return subprocess.CompletedProcess(command, 0, stdout=output, stderr="")

    monkeypatch.setattr("shutil.which", lambda name: f"/opt/ros/melodic/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    client = RosCliReadOnlyClient({"/odom": "nav_msgs/Odometry"})

    messages = client.subscribe_many("/odom", "nav_msgs/Odometry", count=2)

    assert [message["header"]["seq"] for message in messages] == [1, 2]


def test_roscli_probe_reads_all_published_types_in_one_graph_command(monkeypatch: Any) -> None:
    calls: list[list[str]] = []

    def run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[str]:
        calls.append(command)
        if command[0].endswith("rostopic"):
            output = (
                "Published topics:\n"
                " * /odom [nav_msgs/Odometry] 1 publisher\n"
                " * /scan [sensor_msgs/LaserScan] 1 publisher\n\n"
                "Subscribed topics:\n * /cmd_vel [geometry_msgs/Twist] 1 subscriber\n"
            )
        else:
            output = "/move_base\n"
        return subprocess.CompletedProcess(command, 0, stdout=output, stderr="")

    monkeypatch.setattr("shutil.which", lambda name: f"/opt/ros/melodic/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    client = RosCliReadOnlyClient({"/odom": "nav_msgs/Odometry", "/scan": "sensor_msgs/LaserScan"})

    graph = client.probe()

    assert graph["topics"] == ["/odom", "/scan"]
    assert graph["types"] == ["nav_msgs/Odometry", "sensor_msgs/LaserScan"]
    assert graph["nodes"] == ["/move_base"]
    assert [command[1:] for command in calls] == [["list", "-v"], ["list"]]


def test_roscli_topic_info_parses_publishers_and_subscribers(monkeypatch: Any) -> None:
    def run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[str]:
        output = (
            "sensor_msgs/LaserScan\n"
            if command[1] == "type"
            else (
                "Type: sensor_msgs/LaserScan\n\nPublishers:\n"
                " * /ydlidar_node (http://robot:1/)\n\nSubscribers:\n"
                " * /move_base (http://robot:2/)\n"
            )
        )
        return subprocess.CompletedProcess(command, 0, stdout=output, stderr="")

    monkeypatch.setattr("shutil.which", lambda name: f"/opt/ros/melodic/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    client = RosCliReadOnlyClient({"/scan": "sensor_msgs/LaserScan"})

    info = client.topic_info("/scan", "sensor_msgs/LaserScan")

    assert info["publishers"] == ["/ydlidar_node (http://robot:1/)"]
    assert info["subscribers"] == ["/move_base (http://robot:2/)"]


def test_each_roscli_client_has_a_distinct_transport_generation(monkeypatch: Any) -> None:
    monkeypatch.setattr("shutil.which", lambda name: f"/opt/ros/melodic/bin/{name}")

    first = RosCliReadOnlyClient({"/scan": "sensor_msgs/LaserScan"})
    second = RosCliReadOnlyClient({"/scan": "sensor_msgs/LaserScan"})

    assert first.transport_generation.startswith("roscli-")
    assert first.transport_generation != second.transport_generation


def test_roscli_resolves_only_fixed_readiness_transforms(monkeypatch: Any) -> None:
    calls: list[list[str]] = []

    class Process:
        pid = 123

        def __init__(self) -> None:
            self.stdout = io.StringIO("At time 1.0\n- Translation: [0, 0, 0]\n")
            self.returncode: int | None = None

        def poll(self) -> int | None:
            return self.returncode

        def wait(self, *, timeout: float) -> int:
            del timeout
            self.returncode = -15
            return self.returncode

    def popen(command: list[str], **_kwargs: Any) -> Process:
        calls.append(command)
        return Process()

    monkeypatch.setattr("shutil.which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr("subprocess.Popen", popen)
    monkeypatch.setattr("select.select", lambda values, *_args: (values, [], []))
    monkeypatch.setattr("os.killpg", lambda *_args: None)
    client = RosCliReadOnlyClient({"/tf": "tf2_msgs/TFMessage"})

    assert client.transform_available("map", "odom") is True
    assert calls[0][-4:] == ["tf", "tf_echo", "map", "odom"]
    try:
        client.transform_available("map", "camera_link")
    except ValueError as exc:
        assert "immutable readiness allowlist" in str(exc)
    else:
        raise AssertionError("unexpected transform was not rejected")


def test_freshness_critical_topics_use_bounded_readiness_worker(monkeypatch: Any) -> None:
    calls: list[tuple[list[str], dict[str, Any]]] = []

    def run(command: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
        calls.append((command, kwargs))
        if command[0].endswith("rostopic"):
            return subprocess.CompletedProcess(
                command, 0, stdout="sensor_msgs/LaserScan\n", stderr=""
            )
        assert '"observation":"laser_scan"' in kwargs["input"]
        output = (
            '{"ok":true,"protocol":"rosclaw.limo.readiness-worker.v1",'
            '"observation":"laser_scan","message":{"ranges":[1.0]}}\n'
        )
        return subprocess.CompletedProcess(command, 0, stdout=output, stderr="")

    monkeypatch.setattr("shutil.which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    monkeypatch.setattr("limo_ros_mcp.roscli.Path.is_file", lambda _path: True)
    client = RosCliReadOnlyClient({"/scan": "sensor_msgs/LaserScan"})

    messages = client.subscribe_many("/scan", "sensor_msgs/LaserScan", count=1)

    assert messages == [{"ranges": [1.0]}]
    worker_command, worker_kwargs = calls[-1]
    assert worker_command[0] == "/usr/bin/python2"
    assert worker_command[1].endswith("worker/limo_readiness_worker.py")
    assert worker_kwargs["timeout"] == 7.0


def test_readiness_worker_rejects_mismatched_response(monkeypatch: Any) -> None:
    def run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[str]:
        if command[0].endswith("rostopic"):
            return subprocess.CompletedProcess(
                command, 0, stdout="nav_msgs/OccupancyGrid\n", stderr=""
            )
        output = (
            '{"ok":true,"protocol":"rosclaw.limo.readiness-worker.v1",'
            '"observation":"laser_scan","message":{}}\n'
        )
        return subprocess.CompletedProcess(command, 0, stdout=output, stderr="")

    monkeypatch.setattr("shutil.which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    monkeypatch.setattr("limo_ros_mcp.roscli.Path.is_file", lambda _path: True)
    client = RosCliReadOnlyClient({"/move_base/global_costmap/costmap": "nav_msgs/OccupancyGrid"})

    with pytest.raises(RuntimeError, match="ROS readiness worker failed"):
        client.subscribe_many(
            "/move_base/global_costmap/costmap", "nav_msgs/OccupancyGrid", count=1
        )


def test_roscli_camera_capture_uses_fixed_read_only_worker(monkeypatch: Any) -> None:
    calls: list[tuple[list[str], dict[str, Any]]] = []
    png = b"\x89PNG\r\n\x1a\nfixture"

    def run(command: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
        calls.append((command, kwargs))
        request = json.loads(kwargs["input"])
        assert request == {
            "protocol": "rosclaw.limo.camera-capture-worker.v1",
            "stream": "color",
            "timeout_sec": 5.0,
            "max_dimension": 640,
        }
        response = {
            "ok": True,
            "protocol": "rosclaw.limo.camera-capture-worker.v1",
            "stream": "color",
            "topic": "/camera/color/image_raw",
            "source_width": 640,
            "source_height": 480,
            "output_width": 640,
            "output_height": 480,
            "png_sha256": f"sha256:{hashlib.sha256(png).hexdigest()}",
            "png_base64": base64.b64encode(png).decode(),
        }
        return subprocess.CompletedProcess(command, 0, stdout=json.dumps(response), stderr="")

    monkeypatch.setattr("shutil.which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    monkeypatch.setattr("limo_ros_mcp.roscli.Path.is_file", lambda _path: True)
    client = RosCliReadOnlyClient({})

    metadata, observed_png = client.capture_camera_frame(stream="color", max_dimension=640)

    assert observed_png == png
    assert metadata["topic"] == "/camera/color/image_raw"
    assert "png_base64" not in metadata
    command, kwargs = calls[0]
    assert command[0] == "/usr/bin/python2"
    assert command[1].endswith("workers/limo_camera_capture_worker.py")
    assert kwargs["timeout"] == 8.0


def test_roscli_robot_pose_uses_fixed_map_base_worker(monkeypatch: Any) -> None:
    calls: list[tuple[list[str], dict[str, Any]]] = []

    def run(command: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
        calls.append((command, kwargs))
        assert json.loads(kwargs["input"]) == {
            "protocol": "rosclaw.limo.robot-pose-worker.v1",
            "timeout_sec": 5.0,
        }
        response = {
            "ok": True,
            "protocol": "rosclaw.limo.robot-pose-worker.v1",
            "parent_frame": "map",
            "child_frame": "base_link",
            "translation": {"x": 0.1, "y": -0.2, "z": 0.0},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0, "yaw_rad": 0.0},
            "received_wall_time": 123.0,
        }
        return subprocess.CompletedProcess(command, 0, stdout=json.dumps(response), stderr="")

    monkeypatch.setattr("shutil.which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    monkeypatch.setattr("limo_ros_mcp.roscli.Path.is_file", lambda _path: True)
    client = RosCliReadOnlyClient({})

    pose = client.robot_pose()

    assert pose["parent_frame"] == "map"
    assert pose["child_frame"] == "base_link"
    assert pose["translation"]["x"] == 0.1
    assert calls[0][0][1].endswith("workers/limo_robot_pose_worker.py")
