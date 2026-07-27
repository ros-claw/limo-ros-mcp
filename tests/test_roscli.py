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
