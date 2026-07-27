# limo-ros-mcp

Safety-bounded MCP server for the AgileX LIMO ROS 1 platform. It converts the documented `limo_ros` interfaces into a small MCP surface for Codex and delegates every state-changing request to the ROSClaw daemon.

The ROSClaw embodiment assets live in `e-urdf-zoo/limo`, with the project template and ROS embodiment card under `configs/`.

[中文说明](README.zh.md)

## Safety model

- Read-only rosbridge or local ROS CLI access is limited to `/limo_status`, `/odom`, `/imu`, and optional `/scan`.
- No MCP tool can advertise or publish a ROS topic. `/cmd_vel`, serial, CAN, and vendor SDK access are intentionally absent.
- Navigation uses the high-level `/move_base` contract and is submitted through `rosclawd` only.
- REAL execution requires a body snapshot, daemon-issued authorization, a registered verified executor, and a canonical execution receipt.

## MCP tools

| Tool | Purpose |
| --- | --- |
| `limo_get_contract` | Return the immutable source and safety contract. |
| `limo_probe_ros` | Read the ROS graph and check required LIMO topics. |
| `limo_observe` | Read one allowlisted telemetry message. |
| `limo_validate_navigation_goal` | Validate a goal without dispatch. |
| `limo_get_runtime_status` | Read `rosclawd` readiness. |
| `limo_request_navigation` | Submit SHADOW or REAL navigation to `rosclawd`. |
| `limo_get_action_status` | Read action state. |
| `limo_get_execution_receipt` | Read the canonical execution receipt. |
| `limo_emergency_stop` | Request daemon E-stop; physical E-stop remains authoritative. |

## Installation

ROSClaw is not published under a safe PyPI package name, so install from its checkout rather than installing `rosclaw` from PyPI:

```bash
uv venv --python 3.12
uv pip install -e ../rosclaw
uv pip install -e '.[dev]'
```

Run the checks:

```bash
.venv/bin/ruff check .
.venv/bin/mypy src
.venv/bin/pytest -q
.venv/bin/python -m limo_ros_mcp.server --help
```

## Codex integration

Register the stdio server with absolute paths:

```bash
codex mcp add rosclaw-limo \
  --env ROSCLAW_PROJECT_ROOT=/absolute/path/to/rosclaw \
  -- /absolute/path/to/limo-ros-mcp/.venv/bin/python -m limo_ros_mcp.server
```

`codex mcp list` confirms the configuration. Start a new Codex session after adding the server so its tools are discovered.

## Read-only robot check

An operator starts the existing LIMO stack. rosbridge is optional because `transport=auto` falls back to fixed read-only `rostopic`/`rosnode` commands:

```bash
roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
# Optional remote websocket access:
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

Then call `limo_probe_ros`, followed by `limo_observe` for `status`, `odometry`, `imu`, and optionally `laser_scan`. These are observations only and do not prove motion.

Use SHADOW before considering REAL. Do not submit REAL unless `limo_get_runtime_status` proves the daemon boundary is ready and an operator has authorized the exact action. Verify the outcome through `limo_get_action_status` and `limo_get_execution_receipt`.

## Provenance

- [`agilexrobotics/limo_ros`](https://github.com/agilexrobotics/limo_ros), pinned in the contract to commit `4c78efc674cfc154012fe851fbda89c50be5b983`
- [`agilexrobotics/limo-doc`](https://github.com/agilexrobotics/limo-doc), pinned in the contract to commit `d78a730163f50e1a5e5631ffd651444e0ac6abc5`
- [`ros-claw/rosclaw`](https://github.com/ros-claw/rosclaw), used only for the guarded control-plane boundary

## License

MIT
