"""Tests for the fixed-command local ROS 1 observation backend."""

from __future__ import annotations

import subprocess
from typing import Any

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
    def run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[str]:
        output = (
            "Failure at 0.000\n"
            "At time 12.3\n"
            "- Translation: [0.0, 0.0, 0.0]\n"
            "- Rotation: in Quaternion [0.0, 0.0, 0.0, 1.0]\n"
        )
        return subprocess.CompletedProcess(command, 124, stdout=output, stderr="")

    monkeypatch.setattr("shutil.which", lambda name: f"/opt/ros/melodic/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
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

    def run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[str]:
        calls.append(command)
        return subprocess.CompletedProcess(
            command,
            124,
            stdout="At time 1.0\n- Translation: [0, 0, 0]\n",
            stderr="",
        )

    monkeypatch.setattr("shutil.which", lambda name: f"/usr/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    client = RosCliReadOnlyClient({"/tf": "tf2_msgs/TFMessage"})

    assert client.transform_available("map", "odom") is True
    assert calls[0][2] == "5"
    assert calls[0][-4:] == ["tf", "tf_echo", "map", "odom"]
    try:
        client.transform_available("map", "camera_link")
    except ValueError as exc:
        assert "immutable readiness allowlist" in str(exc)
    else:
        raise AssertionError("unexpected transform was not rejected")
