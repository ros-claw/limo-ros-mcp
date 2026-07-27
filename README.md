# limo-ros-mcp

Inspection-focused MCP server for the AgileX LIMO ROS 1 platform. It converts the documented `limo_ros` base, lidar, localization, navigation, mapping, diagnostics, TF, and RGB-D interfaces into 21 MCP tools for Codex. State-changing requests remain delegated to the ROSClaw daemon.

The ROSClaw embodiment assets live in `e-urdf-zoo/limo`, with the project template and ROS embodiment card under `configs/`.

The current real-robot interface verification and patrol preflight findings are recorded in
[`docs/ROS_INSPECTION_READINESS.md`](docs/ROS_INSPECTION_READINESS.md).

[中文说明](README.zh.md)

## ROS inspection surface

- Twenty-three named observation contracts cover the LIMO driver, AMCL, move_base, maps/costmaps, diagnostics, TF, logs, and documented RGB-D streams.
- `limo_observe` returns a compact message summary by default; `include_raw=true` is available for bounded message-level debugging.
- `limo_sample_topic` collects 1-10 messages and reports compact summaries plus an estimated message rate.
- Images, point clouds, paths, laser arrays, and occupancy grids are reduced to useful metadata and statistics by default.
- No MCP tool can advertise or publish a ROS topic. `/cmd_vel`, serial, CAN, and vendor SDK access are absent from the Agent process.
- Navigation uses the high-level `/move_base` contract and is submitted through `rosclawd` only.
- REAL execution requires a body snapshot, daemon-issued authorization, a registered verified executor, and a canonical execution receipt.

## MCP tools

| Group | Tools |
| --- | --- |
| Contract and ROS graph | `limo_get_contract`, `limo_list_observations`, `limo_probe_ros`, `limo_get_topic_info` |
| Message inspection | `limo_observe`, `limo_sample_topic` |
| Inspection snapshots | `limo_get_base_state`, `limo_get_laser_summary`, `limo_get_localization_state`, `limo_get_navigation_state`, `limo_get_map_summary`, `limo_get_diagnostics`, `limo_get_transform_state`, `limo_get_patrol_readiness` |
| Validation | `limo_validate_navigation_goal`, `limo_validate_velocity_command` |
| ROSClaw control plane | `limo_get_runtime_status`, `limo_request_navigation`, `limo_get_action_status`, `limo_get_execution_receipt`, `limo_emergency_stop` |

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

Run the opt-in read-only checks against an operator-started LIMO ROS graph:

```bash
LIMO_LIVE_ROS=1 .venv/bin/pytest tests/test_live_ros.py -q -m live_ros
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

Then call `limo_probe_ros`, inspect important endpoints with `limo_get_topic_info`, and sample status/navigation messages with `limo_sample_topic`. `limo_get_patrol_readiness` concurrently checks base state, lidar, AMCL, move_base, map metadata, and diagnostics while preserving partial failures. These are observations only and do not prove motion.

Use SHADOW before considering REAL. Do not submit REAL unless `limo_get_runtime_status` proves the daemon boundary is ready and an operator has authorized the exact action. Verify the outcome through `limo_get_action_status` and `limo_get_execution_receipt`.

## Provenance

- [`agilexrobotics/limo_ros`](https://github.com/agilexrobotics/limo_ros), pinned in the contract to commit `4c78efc674cfc154012fe851fbda89c50be5b983`
- [`agilexrobotics/limo-doc`](https://github.com/agilexrobotics/limo-doc), pinned in the contract to commit `d78a730163f50e1a5e5631ffd651444e0ac6abc5`
- [`ros-claw/rosclaw`](https://github.com/ros-claw/rosclaw), used only for the guarded control-plane boundary

## License

MIT
