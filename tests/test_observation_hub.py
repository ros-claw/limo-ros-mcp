"""ObservationHub generation, cache, and readiness single-flight tests."""

from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
from types import SimpleNamespace
from typing import Any

from limo_ros_mcp.cache import ObservationHub
from limo_ros_mcp.evidence import seal_snapshot
from limo_ros_mcp.server import LimoMCPService


def test_hub_reuses_client_and_invalidates_generation_bound_cache() -> None:
    created: list[Any] = []

    def factory(transport: str, endpoint: str, timeout_sec: float) -> Any:
        client = SimpleNamespace(
            transport_generation=f"{transport}-{len(created) + 1}",
            endpoint=endpoint,
            timeout_sec=timeout_sec,
        )
        created.append(client)
        return client

    hub = ObservationHub(factory, capacity=2)
    first = hub.client("roscli", "local", 1.0)
    assert hub.client("roscli", "local", 2.0) is first
    record = {
        "summary": {"ok": True},
        "received_monotonic": time.monotonic(),
        "transport_generation": "roscli-1",
    }
    hub.remember("status", transport="roscli", endpoint="local", record=record)
    assert (
        hub.cached(
            "status",
            transport="roscli",
            endpoint="local",
            max_age_sec=1.0,
            cutoff_monotonic=time.monotonic(),
        )
        is not None
    )

    hub.invalidate("roscli", "local")
    second = hub.client("roscli", "local", 1.0)
    assert second is not first
    assert len(created) == 2
    assert (
        hub.cached(
            "status",
            transport="roscli",
            endpoint="local",
            max_age_sec=1.0,
            cutoff_monotonic=time.monotonic(),
        )
        is None
    )


def test_concurrent_readiness_calls_share_one_inflight_collection(monkeypatch: Any) -> None:
    service = LimoMCPService(gateway=object())
    call_lock = Lock()
    calls = 0

    def build(**kwargs: Any) -> dict[str, Any]:
        nonlocal calls
        with call_lock:
            calls += 1
        time.sleep(0.05)
        return {"ok": True, "body_snapshot_hash": kwargs["body_snapshot_hash"]}

    monkeypatch.setattr(service, "_build_patrol_readiness", build)
    with ThreadPoolExecutor(max_workers=4) as executor:
        futures = [
            executor.submit(
                service.patrol_readiness,
                "ws://127.0.0.1:9090",
                1.0,
                "roscli",
                "limo",
                "sha256:body",
            )
            for _ in range(4)
        ]
    assert [future.result()["ok"] for future in futures] == [True] * 4
    assert calls == 1


def test_navigation_reference_rejects_reconnected_transport(monkeypatch: Any) -> None:
    service = LimoMCPService(gateway=object())
    generations = iter(("roscli-1", "roscli-2"))

    def client(*args: Any) -> Any:
        return SimpleNamespace(transport_generation=next(generations), timeout_sec=args[-1])

    monkeypatch.setattr(service, "_client", client)
    endpoint = "ws://127.0.0.1:9090"
    service._observation_hub.client("roscli", endpoint, 1.0)
    readiness = seal_snapshot(
        {
            "schema_version": "limo.readiness.v1",
            "state": "READY",
            "ready": True,
            "blockers": [],
            "body_snapshot_hash": "sha256:body",
            "expires_wall_time": time.time() + 60.0,
            "transport_binding": {
                "endpoint": endpoint,
                "transport": "roscli",
                "generation": "roscli-1",
            },
        }
    )
    service._remember_readiness(readiness)
    assert (
        service._validate_readiness_reference(str(readiness["snapshot_hash"]), "sha256:body")["ok"]
        is True
    )

    service._observation_hub.invalidate("roscli", endpoint)
    service._observation_hub.client("roscli", endpoint, 1.0)
    blocked = service._validate_readiness_reference(str(readiness["snapshot_hash"]), "sha256:body")
    assert blocked["error_code"] == "LIMO_TRANSPORT_GENERATION_CHANGED"
