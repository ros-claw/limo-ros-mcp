"""Read-only local ROS 1 CLI backend for systems without rosbridge."""

from __future__ import annotations

import shutil
import subprocess
import time
from typing import Any

import yaml


class RosCliReadOnlyClient:
    """Use fixed read-only ROS commands; no arbitrary command surface is exposed."""

    def __init__(
        self,
        topic_types: dict[str, str],
        *,
        timeout_sec: float = 2.0,
        max_output_bytes: int = 2_000_000,
    ) -> None:
        self.topic_types = dict(topic_types)
        self.timeout_sec = float(timeout_sec)
        self.max_output_bytes = int(max_output_bytes)
        self.last_sample_elapsed_sec = 0.0
        rostopic = shutil.which("rostopic")
        rosnode = shutil.which("rosnode")
        if not rostopic or not rosnode:
            raise RuntimeError("ROS 1 rostopic/rosnode commands are unavailable")
        self.rostopic: str = rostopic
        self.rosnode: str = rosnode

    def _run(self, command: list[str]) -> str:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=self.timeout_sec,
        )
        output = result.stdout
        if len(output.encode("utf-8")) > self.max_output_bytes:
            raise RuntimeError("ROS CLI output exceeds the configured size limit")
        if result.returncode != 0:
            error = result.stderr.strip() or output.strip() or "ROS CLI command failed"
            raise RuntimeError(error)
        return output

    def probe(self) -> dict[str, Any]:
        topics = [line.strip() for line in self._run([self.rostopic, "list"]).splitlines()]
        nodes = [line.strip() for line in self._run([self.rosnode, "list"]).splitlines()]
        observed_types: list[str] = []
        for topic in topics:
            expected = self.topic_types.get(topic)
            if expected is None:
                observed_types.append("")
                continue
            observed_types.append(self._run([self.rostopic, "type", topic]).strip())
        return {"topics": topics, "types": observed_types, "nodes": nodes}

    def topic_info(self, topic: str, expected_type: str) -> dict[str, Any]:
        if self.topic_types.get(topic) != expected_type:
            raise ValueError("topic is not present in the immutable LIMO observation allowlist")
        observed_type = self._run([self.rostopic, "type", topic]).strip()
        if observed_type != expected_type:
            raise RuntimeError(
                f"ROS topic type mismatch for {topic}: expected {expected_type}, got {observed_type}"
            )
        output = self._run([self.rostopic, "info", topic])
        publishers: list[str] = []
        subscribers: list[str] = []
        target = publishers
        for line in output.splitlines():
            stripped = line.strip()
            if stripped == "Publishers:":
                target = publishers
            elif stripped == "Subscribers:":
                target = subscribers
            elif stripped.startswith("*"):
                target.append(stripped[1:].strip())
        return {
            "topic": topic,
            "expected_type": expected_type,
            "observed_type": observed_type,
            "publishers": publishers,
            "subscribers": subscribers,
        }

    def subscribe_many(
        self,
        topic: str,
        message_type: str,
        *,
        count: int,
    ) -> list[dict[str, Any]]:
        if self.topic_types.get(topic) != message_type:
            raise ValueError("topic is not present in the immutable LIMO observation allowlist")
        observed_type = self._run([self.rostopic, "type", topic]).strip()
        if observed_type != message_type:
            raise RuntimeError(
                f"ROS topic type mismatch for {topic}: expected {message_type}, got {observed_type}"
            )
        started = time.monotonic()
        command = [self.rostopic, "echo"]
        if message_type in {"sensor_msgs/Image", "sensor_msgs/PointCloud2"}:
            command.append("--noarr")
        command.extend(["-n", str(count), topic])
        output = self._run(command).strip()
        messages = [item for item in yaml.safe_load_all(output) if isinstance(item, dict)]
        if len(messages) != count:
            raise RuntimeError(
                f"rostopic echo returned {len(messages)} object messages, expected {count}"
            )
        self.last_sample_elapsed_sec = time.monotonic() - started
        return messages

    def subscribe_once(self, topic: str, message_type: str) -> dict[str, Any]:
        return self.subscribe_many(topic, message_type, count=1)[0]
