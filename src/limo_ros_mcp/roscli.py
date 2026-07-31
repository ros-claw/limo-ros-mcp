"""Read-only local ROS 1 CLI backend for systems without rosbridge."""

from __future__ import annotations

import re
import shutil
import subprocess
import time
import uuid
from typing import Any

import yaml


class RosCliReadOnlyClient:
    """Use fixed read-only ROS commands; no arbitrary command surface is exposed."""

    def __init__(
        self,
        topic_types: dict[str, str],
        *,
        timeout_sec: float = 5.0,
        max_output_bytes: int = 2_000_000,
    ) -> None:
        self.topic_types = dict(topic_types)
        self.timeout_sec = float(timeout_sec)
        self.max_output_bytes = int(max_output_bytes)
        self.last_sample_elapsed_sec = 0.0
        self.transport_generation = f"roscli-{uuid.uuid4()}"
        self._observed_topic_types: dict[str, str] = {}
        rostopic = shutil.which("rostopic")
        rosnode = shutil.which("rosnode")
        rosrun = shutil.which("rosrun")
        timeout_command = shutil.which("timeout")
        if not rostopic or not rosnode or not rosrun or not timeout_command:
            raise RuntimeError("ROS 1 rostopic/rosnode commands are unavailable")
        self.rostopic: str = rostopic
        self.rosnode: str = rosnode
        self.rosrun: str = rosrun
        self.timeout_command: str = timeout_command

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
        verbose = self._run([self.rostopic, "list", "-v"])
        observed: dict[str, str] = {}
        in_published = False
        for line in verbose.splitlines():
            stripped = line.strip()
            if stripped == "Published topics:":
                in_published = True
                continue
            if stripped == "Subscribed topics:":
                in_published = False
                continue
            if not in_published:
                continue
            match = re.match(r"^\*\s+(\S+)\s+\[([^]]+)]", stripped)
            if match:
                observed[match.group(1)] = match.group(2)
        if not observed:
            raise RuntimeError("rostopic list -v returned no published topic types")
        self._observed_topic_types = observed
        topics = sorted(observed)
        nodes = [line.strip() for line in self._run([self.rosnode, "list"]).splitlines()]
        return {"topics": topics, "types": [observed[topic] for topic in topics], "nodes": nodes}

    def _verified_type(self, topic: str, expected_type: str) -> str:
        observed_type = self._observed_topic_types.get(topic)
        if observed_type is None:
            observed_type = self._run([self.rostopic, "type", topic]).strip()
            self._observed_topic_types[topic] = observed_type
        if observed_type != expected_type:
            raise RuntimeError(
                f"ROS topic type mismatch for {topic}: expected {expected_type}, got {observed_type}"
            )
        return observed_type

    def topic_info(self, topic: str, expected_type: str) -> dict[str, Any]:
        if self.topic_types.get(topic) != expected_type:
            raise ValueError("topic is not present in the immutable LIMO observation allowlist")
        observed_type = self._verified_type(topic, expected_type)
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
        self._verified_type(topic, message_type)
        started = time.monotonic()
        command = [self.rostopic, "echo"]
        if message_type in {
            "sensor_msgs/Image",
            "sensor_msgs/PointCloud2",
            "nav_msgs/OccupancyGrid",
        }:
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

    def transform_available(self, parent_frame: str, child_frame: str) -> bool:
        """Resolve one fixed TF edge without publishing or exposing an arbitrary command."""

        allowed = {("map", "odom"), ("odom", "base_link")}
        if (parent_frame, child_frame) not in allowed:
            raise ValueError("transform is not present in the immutable readiness allowlist")
        timeout_sec = max(0.2, min(self.timeout_sec, 2.0))
        command = [
            self.timeout_command,
            "--signal=TERM",
            f"{timeout_sec:g}",
            self.rosrun,
            "tf",
            "tf_echo",
            parent_frame,
            child_frame,
        ]
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec + 1.0,
        )
        output = result.stdout
        if len(output.encode("utf-8")) > self.max_output_bytes:
            raise RuntimeError("ROS CLI output exceeds the configured size limit")
        if result.returncode not in {0, 124}:
            error = result.stderr.strip() or output.strip() or "tf_echo failed"
            raise RuntimeError(error)
        return "At time" in output and "Failure" not in output
