"""Persistent, bounded read-only ROS observation client and summary cache."""

from __future__ import annotations

import time
from collections import OrderedDict
from collections.abc import Callable
from threading import RLock
from typing import Any


class ObservationHub:
    """Reuse allowlisted transports and retain only bounded summarized observations."""

    def __init__(
        self,
        client_factory: Callable[[str, str, float], Any],
        *,
        capacity: int = 64,
    ) -> None:
        self._client_factory = client_factory
        self._capacity = capacity
        self._lock = RLock()
        self._clients: dict[tuple[str, str], Any] = {}
        self._records: OrderedDict[tuple[str, str, str], dict[str, Any]] = OrderedDict()
        self._epoch = 0
        self._active_generation: dict[tuple[str, str], str] = {}

    def client(self, transport: str, endpoint: str, timeout_sec: float) -> Any:
        key = (transport, endpoint)
        with self._lock:
            client = self._clients.get(key)
            if client is not None:
                if hasattr(client, "timeout_sec"):
                    client.timeout_sec = float(timeout_sec)
                return client
        client = self._client_factory(transport, endpoint, timeout_sec)
        with self._lock:
            existing = self._clients.get(key)
            if existing is not None:
                return existing
            self._epoch += 1
            generation = str(
                getattr(client, "transport_generation", None)
                or f"{transport}-generation-{self._epoch}"
            )
            self._clients[key] = client
            self._active_generation[key] = generation
            return client

    def generation(self, transport: str, endpoint: str) -> str | None:
        with self._lock:
            return self._active_generation.get((transport, endpoint))

    def invalidate(self, transport: str, endpoint: str) -> None:
        """Drop a failed connection and every observation bound to its generation."""

        key = (transport, endpoint)
        with self._lock:
            self._clients.pop(key, None)
            self._active_generation.pop(key, None)
            for record_key in tuple(self._records):
                if record_key[:2] == key:
                    self._records.pop(record_key, None)

    def remember(
        self,
        observation: str,
        *,
        transport: str,
        endpoint: str,
        record: dict[str, Any],
    ) -> None:
        key = (transport, endpoint, observation)
        with self._lock:
            self._records[key] = dict(record)
            self._records.move_to_end(key)
            while len(self._records) > self._capacity:
                self._records.popitem(last=False)

    def cached(
        self,
        observation: str,
        *,
        transport: str,
        endpoint: str,
        max_age_sec: float,
        cutoff_monotonic: float,
    ) -> dict[str, Any] | None:
        key = (transport, endpoint, observation)
        with self._lock:
            record = self._records.get(key)
            generation = self._active_generation.get((transport, endpoint))
            if record is None or record.get("transport_generation") != generation:
                return None
            received = record.get("received_monotonic")
            if not isinstance(received, (int, float)) or isinstance(received, bool):
                return None
            age = cutoff_monotonic - float(received)
            if age < 0 or age > max_age_sec:
                return None
            self._records.move_to_end(key)
            return dict(record)

    def stats(self) -> dict[str, Any]:
        with self._lock:
            return {
                "client_count": len(self._clients),
                "record_count": len(self._records),
                "capacity": self._capacity,
                "epoch": self._epoch,
                "generated_at_monotonic": time.monotonic(),
            }
