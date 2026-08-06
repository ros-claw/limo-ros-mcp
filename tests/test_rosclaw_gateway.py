from __future__ import annotations

import pytest

from limo_ros_mcp.rosclaw_gateway import RosclawGateway


class _Runtime:
    async def get_action_status(self, action_id: str):
        return {
            "action_id": action_id,
            "receipt": {
                "authorization_decision": {
                    "authorized": True,
                    "approval_id": "internal-authorization-handle",
                    "provenance": {"decision_channel": "mcp_form_via_rosclaw_operatord"},
                }
            },
        }


class _Daemon:
    def get_execution_receipt(self, action_id: str):
        return {
            "action_id": action_id,
            "receipt": {
                "authorization_decision": {
                    "authorized": True,
                    "permit_id": "internal-permit-handle",
                    "permit": {"token": "internal-token"},
                }
            },
        }


@pytest.mark.asyncio
async def test_public_action_status_hides_internal_authorization_handles() -> None:
    result = await RosclawGateway(runtime_client=_Runtime()).action_status("action-1")

    assert result["receipt"]["authorization_decision"] == {
        "authorized": True,
        "provenance": {"decision_channel": "mcp_form_via_rosclaw_operatord"},
    }
    assert "internal-authorization-handle" not in str(result)


@pytest.mark.asyncio
async def test_public_execution_receipt_hides_internal_authorization_handles() -> None:
    result = await RosclawGateway(daemon_client=_Daemon()).execution_receipt("action-2")

    assert result["receipt"]["authorization_decision"] == {"authorized": True}
    assert "internal-permit-handle" not in str(result)
    assert "internal-token" not in str(result)
