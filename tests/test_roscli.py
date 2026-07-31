"""Tests for the fixed-command local ROS 1 observation backend."""

from __future__ import annotations

import subprocess
from typing import Any

from limo_ros_mcp.roscli import RosCliReadOnlyClient


def test_roscli_observation_uses_only_type_and_echo(monkeypatch: Any) -> None:
    calls: list[list[str]] = []

    def run(command: list[str], **_kwargs: Any) -> subprocess.CompletedProcess[str]:
        calls.append(command)
        output = "sensor_msgs/Imu\n" if command[1] == "type" else "orientation:\n  w: 1.0\n---\n"
        return subprocess.CompletedProcess(command, 0, stdout=output, stderr="")

    monkeypatch.setattr("shutil.which", lambda name: f"/opt/ros/melodic/bin/{name}")
    monkeypatch.setattr("subprocess.run", run)
    client = RosCliReadOnlyClient({"/imu": "sensor_msgs/Imu"})

    message = client.subscribe_once("/imu", "sensor_msgs/Imu")

    assert client.timeout_sec == 5.0
    assert message["orientation"]["w"] == 1.0
    assert [command[1:3] for command in calls] == [["type", "/imu"], ["echo", "-n"]]
    assert not any("pub" in command or "publish" in command for command in calls)


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
    assert calls[0][-4:] == ["tf", "tf_echo", "map", "odom"]
    try:
        client.transform_available("map", "camera_link")
    except ValueError as exc:
        assert "immutable readiness allowlist" in str(exc)
    else:
        raise AssertionError("unexpected transform was not rejected")
