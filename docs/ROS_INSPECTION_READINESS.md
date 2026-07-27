# LIMO ROS inspection readiness

This document records the repeatable preflight work for LIMO patrol development. Live values
are a point-in-time observation, not a permanent robot claim or proof of motion.

## Implemented interface

- 21 MCP tools and 23 named ROS observation contracts.
- ROS graph discovery and per-topic type/publisher/subscriber inspection.
- Bounded 1-10 message sampling with ROS-header rate estimation.
- Compact parsers for LIMO status, odometry, IMU, LaserScan, AMCL pose, move_base status,
  paths, maps/costmaps, diagnostics, TF, logs, images, and point clouds.
- Concurrent patrol preflight across status, odometry, IMU, lidar, localization, navigation,
  map metadata, and diagnostics.
- High-level navigation remains connected to the ROSClaw daemon path; no Agent-side ROS
  publisher was added.

## Reproducible checks

Offline checks:

```bash
.venv/bin/ruff check .
.venv/bin/ruff format --check .
.venv/bin/mypy src
.venv/bin/pytest -q
```

Read-only live ROS checks, after the operator starts the LIMO ROS stack:

```bash
LIMO_LIVE_ROS=1 .venv/bin/pytest tests/test_live_ros.py -q -m live_ros
```

The live suite verifies:

1. The LIMO, AMCL, move_base, map, and lidar contracts are present in the ROS graph.
2. `/limo_status` fields decode into motion mode and driver error flags.
3. Two `/move_base/status` messages can be sampled and their rate derived from ROS stamps.
4. `/scan` has the expected message type and live graph endpoints.
5. A real MCP stdio client can call `limo_get_patrol_readiness` and receive structured evidence.

## 2026-07-27 live snapshot

The graph contained 13 nodes, 62 topics, and 49 services during the final probe. The patrol
snapshot read all eight requested observation groups without publishing a command.

| Evidence | Observed result |
| --- | --- |
| LIMO status | `error_code=0`, four-wheel differential mode, battery 10.3 V (dynamic) |
| Odometry | Fresh; measured linear and angular velocity were both zero |
| IMU | Fresh; acceleration and angular velocity decoded successfully |
| Lidar | 460 samples, 164 valid, nearest 1.30 m, front clearance about 2.02 m |
| move_base | Latest goal state was `SUCCEEDED` with `Goal reached.` |
| Map metadata | 1984 x 1984 cells at 0.05 m/cell |
| AMCL pose | Covariance decoded, but the message was stale by more than 300 seconds |
| Diagnostics | One WARN: `amcl: Standard deviation` / `Too large` |

The corrected patrol decision is therefore `BLOCKED`, with `localized_pose_stale` as a blocker
and `diagnostics_non_nominal` as a warning. This is intentional: topic presence alone is not
enough to declare localization ready.

## Next patrol experiment sequence

1. Restore continuous AMCL updates and resolve or explicitly investigate the standard-deviation
   warning. Re-run `limo_get_patrol_readiness` until localization is fresh.
2. Record the inspection map frame, named checkpoints, allowed route area, and return-home pose.
3. Validate each checkpoint and route segment without dispatching motion.
4. Submit the complete patrol to ROSClaw in SHADOW mode and inspect the action status and receipt.
5. For a supervised REAL run, verify the daemon boundary, executor, authorization, physical
   E-stop, operator line of sight, and exact route before submitting the same immutable action.
6. During the run, retain status, odometry, lidar clearance, localization covariance, move_base
   state, and diagnostics as the minimum inspection telemetry set.
