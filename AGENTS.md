# LIMO ROS MCP engineering rules

- Treat the AgileX LIMO as physical equipment. Never bypass ROSClaw by publishing `/cmd_vel`, opening CAN/serial devices, or calling a motor SDK.
- Read-only ROS observations may use rosbridge. The allowlist is defined in `limo_ros_mcp.contract`.
- Submit SHADOW or REAL navigation only through `limo_request_navigation`, which delegates to `rosclawd` and requires an immutable body snapshot.
- REAL success requires a daemon-issued permit, a registered verified executor, and a terminal execution receipt. CLI output or a ROS topic observation is not proof of execution.
- Run unit and MCP protocol tests before any hardware probe. Hardware probes must remain read-only unless an operator has armed the daemon and separately approved the exact action.
