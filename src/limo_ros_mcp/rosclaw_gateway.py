"""Lazy integration with the local ROSClaw control plane."""

from __future__ import annotations

import asyncio
import os
from pathlib import Path
from typing import Any


class RosclawGateway:
    """Delegate all state-changing requests to rosclawd through ROSClaw."""

    def __init__(self, runtime_client: Any | None = None, daemon_client: Any | None = None) -> None:
        self._runtime_client = runtime_client
        self._daemon_client = daemon_client

    def _runtime(self) -> Any:
        if self._runtime_client is None:
            try:
                from rosclaw.mcp.adapters.runtime_client import RuntimeClient
            except ImportError as exc:
                raise RuntimeError(
                    "ROSClaw is not installed; install the sibling rosclaw checkout into "
                    "this environment before using control-plane tools"
                ) from exc
            project_root = Path(os.environ.get("ROSCLAW_PROJECT_ROOT", ".")).resolve()
            self._runtime_client = RuntimeClient(
                project_root=project_root,
                robot_id="limo",
                runtime_profile={},
            )
        return self._runtime_client

    def _daemon(self) -> Any:
        if self._daemon_client is None:
            try:
                from rosclaw.daemon.client import DaemonClient
            except ImportError as exc:
                raise RuntimeError("ROSClaw is not installed") from exc
            self._daemon_client = DaemonClient()
        return self._daemon_client

    @staticmethod
    def _result(value: Any) -> dict[str, Any]:
        if not isinstance(value, dict):
            raise RuntimeError("ROSClaw returned a non-object response")
        return value

    async def runtime_status(self) -> dict[str, Any]:
        return self._result(await self._runtime().get_runtime_status())

    async def request_navigation(self, **kwargs: Any) -> dict[str, Any]:
        return self._result(await self._runtime().request_action(**kwargs))

    async def request_tone(self, **kwargs: Any) -> dict[str, Any]:
        return self._result(await self._runtime().request_action(**kwargs))

    async def request_speech(self, **kwargs: Any) -> dict[str, Any]:
        return self._result(await self._runtime().request_action(**kwargs))

    async def request_initial_pose(self, **kwargs: Any) -> dict[str, Any]:
        return self._result(await self._runtime().request_action(**kwargs))

    def prepare_operator_action(self, **kwargs: Any) -> Any:
        """Build an exact REAL proposal for host-side confirmation without dispatching it."""

        return self._runtime().prepare_operator_action(**kwargs)

    async def confirm_operator_action(
        self,
        prepared: Any,
        *,
        principal_id: str,
        confirmation: dict[str, Any],
        wait_timeout_sec: float,
    ) -> dict[str, Any]:
        return self._result(
            await self._runtime().confirm_operator_action(
                prepared,
                principal_id=principal_id,
                confirmation=confirmation,
                wait_timeout_sec=wait_timeout_sec,
            )
        )

    async def action_status(self, action_id: str) -> dict[str, Any]:
        return self._result(await self._runtime().get_action_status(action_id))

    async def execution_receipt(self, action_id: str) -> dict[str, Any]:
        return self._result(
            await asyncio.to_thread(self._daemon().get_execution_receipt, action_id)
        )

    async def emergency_stop(self, reason: str) -> dict[str, Any]:
        return self._result(await self._runtime().emergency_stop(reason))
