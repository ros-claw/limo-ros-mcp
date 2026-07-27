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
