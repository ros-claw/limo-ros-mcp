# Navigation Contract v2 implementation report

## Delivered

- Removed the three caller-supplied readiness booleans from both navigation MCP schemas.
- Retained up to 32 hash-sealed readiness snapshots in the producing MCP process.
- Required unexpired `READY` evidence and exact body-snapshot binding.
- Added operator-owned map policy, SHA-256 map artifact checks, map/geofence/no-go,
  occupancy/unknown-cell, clearance, yaw, route-policy, and tolerance validation.
- Added structured `limo.navigation.v2` SHADOW arguments at the ROSClaw gateway.
- Kept REAL fail-closed pending daemon-owned trusted preflight and executor support.

## Robot evidence

The policy pins the running `Obstacles_2_916` map (1984 x 1984 at 0.05 m/cell,
origin `[-50, -50, 0]`). The YAML hash is
`c35cca5c76bee4b1724d3c45d72ec36184247cce8c03d6621da5be73787b634b` and the PGM
hash is `70c183410f939490f5abe109be2ddfcc4d86ccd3faae3483131f15eb3a7d3afe`.

All hardware interaction used read-only ROS graph/message sampling. No navigation
goal, velocity command, ROS service write, serial/CAN request, or motor SDK call was
issued.

## Remaining REAL gate

Dynamic `make_plan` validation, daemon-owned fresh ROS observations, permit/lease
enforcement, a verified LIMO executor, and terminal receipt validation remain
required before REAL may be enabled.

## Validation results

- Offline: 58 passed, 5 opt-in live tests skipped; Ruff and strict MyPy passed.
- Live read-only suite: 5 passed against the operator-started Melodic graph.
- MCP stdio exposed 21 tools and required both `body_snapshot_hash` and
  `readiness_snapshot_hash`; the three legacy booleans were absent.
- Same-process live request evidence was `BLOCKED` by `LIMO_AMCL_STALE` and
  `LIMO_TF_CHAIN_BROKEN` and the navigation request returned
  `LIMO_READINESS_NOT_READY` with `command_dispatched=false`.
- Built `limo_ros_mcp-0.4.0.tar.gz` (`sha256:842e2a051ad15c15bc6a42c42677125d1abfd5c7f1460fd0879b88c4a6bef72b`)
  and `limo_ros_mcp-0.4.0-py3-none-any.whl`
  (`sha256:6d2f5c33dfb35ef85ab739c8590272d6e37ead1ce454ec603f3ffc8f836fd6c1`).
- A clean wheel install reported version `0.4.0`, loaded the packaged
  `lab-default` route policy, and exposed 21 MCP tools.
