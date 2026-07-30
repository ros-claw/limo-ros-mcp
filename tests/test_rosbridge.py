"""Wire-level tests for the read-only rosbridge client."""

from __future__ import annotations

import json
from typing import Any

from limo_ros_mcp.rosbridge import RosbridgeReadOnlyClient


class FakeConnection:
    def __init__(self, responses: list[dict[str, Any]]) -> None:
        self.responses = [json.dumps(item) for item in responses]
        self.sent: list[dict[str, Any]] = []

    def send(self, payload: str) -> None:
        self.sent.append(json.loads(payload))

    def recv(self) -> str:
        response = json.loads(self.responses.pop(0))
        if response.get("id") == "REQUEST_ID":
            response["id"] = self.sent[-1]["id"]
        return json.dumps(response)

    def settimeout(self, timeout: float) -> None:
        assert timeout > 0

    def close(self) -> None:
        return None


class FakeSocket:
    def close(self) -> None:
        return None


def test_call_service_has_no_publish_primitive(monkeypatch: Any) -> None:
    connection = FakeConnection(
        [
            {
                "op": "service_response",
                "id": "REQUEST_ID",
                "result": True,
                "values": {"topics": ["/odom"], "types": ["nav_msgs/Odometry"]},
            }
        ]
    )
    connect_options: dict[str, Any] = {}

    def connect(*_args: Any, **kwargs: Any) -> FakeConnection:
        connect_options.update(kwargs)
        return connection

    monkeypatch.setattr("websocket.create_connection", connect)
    monkeypatch.setattr("socket.create_connection", lambda *_args, **_kwargs: FakeSocket())

    result = RosbridgeReadOnlyClient("ws://127.0.0.1:9090").call_service("/rosapi/topics")

    assert result["topics"] == ["/odom"]
    assert [item["op"] for item in connection.sent] == ["call_service"]
    assert isinstance(connect_options["socket"], FakeSocket)


def test_each_rosbridge_client_has_a_distinct_transport_generation() -> None:
    first = RosbridgeReadOnlyClient("ws://127.0.0.1:9090")
    second = RosbridgeReadOnlyClient("ws://127.0.0.1:9090")

    assert first.transport_generation.startswith("rosbridge-")
    assert first.transport_generation != second.transport_generation


def test_subscribe_once_always_unsubscribes(monkeypatch: Any) -> None:
    connection = FakeConnection(
        [{"op": "publish", "topic": "/imu", "msg": {"orientation": {"w": 1.0}}}]
    )
    monkeypatch.setattr("websocket.create_connection", lambda *_args, **_kwargs: connection)
    monkeypatch.setattr("socket.create_connection", lambda *_args, **_kwargs: FakeSocket())

    result = RosbridgeReadOnlyClient("ws://127.0.0.1:9090").subscribe_once(
        "/imu", "sensor_msgs/Imu"
    )

    assert result["orientation"]["w"] == 1.0
    assert [item["op"] for item in connection.sent] == ["subscribe", "unsubscribe"]
    assert not any(item["op"] in {"advertise", "publish"} for item in connection.sent)


def test_subscribe_many_collects_bounded_messages_then_unsubscribes(monkeypatch: Any) -> None:
    connection = FakeConnection(
        [
            {"op": "publish", "topic": "/odom", "msg": {"header": {"seq": 1}}},
            {"op": "publish", "topic": "/odom", "msg": {"header": {"seq": 2}}},
        ]
    )
    monkeypatch.setattr("websocket.create_connection", lambda *_args, **_kwargs: connection)
    monkeypatch.setattr("socket.create_connection", lambda *_args, **_kwargs: FakeSocket())

    messages = RosbridgeReadOnlyClient("ws://127.0.0.1:9090").subscribe_many(
        "/odom", "nav_msgs/Odometry", count=2
    )

    assert [message["header"]["seq"] for message in messages] == [1, 2]
    assert [item["op"] for item in connection.sent] == ["subscribe", "unsubscribe"]


def test_topic_info_reads_rosapi_metadata(monkeypatch: Any) -> None:
    connection = FakeConnection(
        [
            {
                "op": "service_response",
                "id": "REQUEST_ID",
                "result": True,
                "values": {"type": "sensor_msgs/LaserScan"},
            },
            {
                "op": "service_response",
                "id": "REQUEST_ID",
                "result": True,
                "values": {"publishers": ["/ydlidar_node"]},
            },
            {
                "op": "service_response",
                "id": "REQUEST_ID",
                "result": True,
                "values": {"subscribers": ["/move_base"]},
            },
        ]
    )
    monkeypatch.setattr("websocket.create_connection", lambda *_args, **_kwargs: connection)
    monkeypatch.setattr("socket.create_connection", lambda *_args, **_kwargs: FakeSocket())

    info = RosbridgeReadOnlyClient("ws://127.0.0.1:9090").topic_info(
        "/scan", "sensor_msgs/LaserScan"
    )

    assert info["observed_type"] == "sensor_msgs/LaserScan"
    assert info["publishers"] == ["/ydlidar_node"]
    assert info["subscribers"] == ["/move_base"]
