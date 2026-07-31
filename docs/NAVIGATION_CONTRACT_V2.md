# Navigation Contract v2

`limo_validate_navigation_goal` and `limo_request_navigation` use
`limo.navigation.v2`. The contract replaces caller-reported readiness booleans
with evidence and operator-owned policy.

## Evidence binding

A request is accepted for validation only when:

1. `readiness_snapshot_hash` identifies an unexpired, hash-valid snapshot that
   was generated and retained by the same MCP server process;
2. that snapshot state is `READY`;
3. `body_snapshot_hash` exactly matches the body snapshot recorded in the
   readiness snapshot; and
4. the requested route policy matches the operator configuration.

The cache is intentionally process-local and bounded to 32 snapshots. Restarting
the MCP server invalidates previous references.

## Static route checks

The default operator policy is `configs/patrol_lab.example.yaml`. It pins the
`Obstacles_2_916` YAML and PGM files by SHA-256 and defines:

- the only accepted frame (`map`), dimensions, resolution, and origin;
- a geofence and optional no-go polygons;
- free/occupied thresholds and unknown-cell policy;
- minimum obstacle clearance; and
- bounded XY/yaw goal tolerances and the expected LIMO motion mode.

Any missing map, hash mismatch, unsupported rotated origin, out-of-policy goal,
occupied/unknown clearance region, or non-finite value fails closed.

## Dispatch boundary

SHADOW requests carry the normalized goal, readiness hash, route-policy hash,
map-image hash, tolerances, and expected effect through `rosclawd.request_action`.
No MCP tool publishes a ROS topic or calls `move_base` directly.

REAL navigation uses an in-context confirmation for one exact target. After
confirmation, `rosclawd` injects an opaque one-shot permit and launches the
revision-locked `limo_navigation_worker.py`. The worker independently requires
fresh AMCL, lidar, odometry and chassis status, a zero chassis error code, the
`map -> odom -> base_link` TF chain, and a live `/move_base` action server before
dispatch. It accepts success only when move_base reaches `SUCCEEDED`, the final
AMCL pose is within the policy tolerance, and odometry reports the base stopped.
Timeout or non-success cancels the goal and fails the receipt.
