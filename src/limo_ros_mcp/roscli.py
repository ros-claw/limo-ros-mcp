"""Read-only local ROS 1 CLI backend for systems without rosbridge."""

from __future__ import annotations

import shutil
import subprocess
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

    def subscribe_once(self, topic: str, message_type: str) -> dict[str, Any]:
        if self.topic_types.get(topic) != message_type:
            raise ValueError("topic is not present in the immutable LIMO observation allowlist")
        observed_type = self._run([self.rostopic, "type", topic]).strip()
        if observed_type != message_type:
            raise RuntimeError(
                f"ROS topic type mismatch for {topic}: expected {message_type}, got {observed_type}"
            )
        output = self._run([self.rostopic, "echo", "-n", "1", topic]).strip()
        if output.endswith("---"):
            output = output[:-3].rstrip()
        message = yaml.safe_load(output)
        if not isinstance(message, dict):
            raise RuntimeError("rostopic echo did not return an object message")
        return message
