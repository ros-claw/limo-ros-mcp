# LIMO ROS inspection readiness

This document records the repeatable preflight work for LIMO patrol development. Live values
are a point-in-time observation, not a permanent robot claim or proof of motion.

## Implemented interface

- 29 MCP tools and 27 named ROS observation contracts.
- ROS graph discovery and per-topic type/publisher/subscriber inspection.
- Bounded 1-10 message sampling with ROS-header rate estimation.
- Compact parsers for LIMO status, odometry, IMU, LaserScan, AMCL pose, move_base status,
  paths, maps/costmaps, diagnostics, TF, logs, images, camera calibration, and point clouds.
- Live host-peripheral correlation for USB devices, ALSA playback/capture, microphone level,
  framebuffer, touchscreen, thermal zones, memory, swap, disk, and uptime.
- Concurrent patrol preflight across status, odometry, IMU, lidar, localization, navigation,
  map metadata, and diagnostics.
- High-level navigation remains connected to the ROSClaw daemon path; no Agent-side ROS
  publisher was added.
- REAL initial-pose setup uses MCP elicitation and internal one-shot authorization injection;
  permit identifiers are not part of the Agent-facing tool schema or result.

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
6. The Dabai color/depth core streams, optional IR endpoints, and getter-only identity/health
   services report their real active or inactive state.
7. A real MCP stdio client can call every host-peripheral tool, including a one-second
   no-retention microphone-level sample.

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

## 2026-07-31 live snapshot

The graph contained 16 nodes and 66 topics. All five opt-in live tests passed. The readiness
collector read all 11 requested observations, accepted the installed ROSClaw Body digest in its
native untagged form, and resolved the complete `map -> odom -> base_link` chain with bounded
read-only TF checks. The earlier TF-chain failure was a sampling false negative and is fixed.

| Evidence | Observed result |
| --- | --- |
| LIMO status | `error_code=0`, four-wheel differential mode, battery 12.2 V (dynamic) |
| Odometry | Zero linear and angular velocity at the observation instant |
| Lidar | Front 4.77 m, left 2.73 m, right 2.50 m minimum clearance (dynamic) |
| TF | `map -> odom` and `odom -> base_link` both resolved |
| AMCL | Still stale; the patrol decision remains `BLOCKED` |
| Diagnostics | One WARN: `amcl: Standard deviation` / `Too large` |

A SHADOW `limo.set_initial_pose` request for `(0.75, -1.25, 0.35 rad)` completed with a
`TASK_VERIFIED` receipt and `hardware_dispatched=false`. An MCP protocol test also rendered the
new REAL confirmation form, declined it, and verified that no daemon Session, Permit, or Action
was created.

### Peripheral expansion

The expanded opt-in suite completed with 19/19 live tests passing. This includes a real stdio MCP
client invoking every new host-peripheral tool, not only direct Python service calls.

The live host additionally confirmed the following patrol-relevant devices and interfaces:

| Peripheral | Live evidence | MCP result |
| --- | --- | --- |
| Dabai RGB/IR camera | USB `2bc5:0557`; color stream active | Color image and CameraInfo bound |
| Dabai depth sensor | USB `2bc5:0657`; depth and point cloud active | Depth, CameraInfo, and PointCloud2 bound |
| Dabai IR hardware | Getter services report device health and IR temperature | Endpoints bound; image stream truthfully reported inactive |
| Lidar | CP210x USB-UART plus `/scan` | Existing lidar summary remains active |
| Voice module | USB `0c76:161f`, ALSA playback and capture | Audio state and no-retention microphone level bound |
| Rear display | Active 1024x600 framebuffer | Display state bound |
| Rear touch panel | USB `1a86:e5e3` input device | Touchscreen presence bound |
| Jetson platform | Thermal, memory, swap, disk, load, uptime | Platform-health tool bound |
| Front OLED | Documented 128x64 device, no stable host/ROS interface found | `declared_unbound` |
| Chassis RGB lights | Firmware-owned, no upstream ROS API found | `declared_unbound` |

The USB sound card exposes Speaker gain/mute and microphone capture gain. No independent physical
amplifier power, temperature, or fault interface was enumerated. The MCP therefore observes these
controls but does not invent an amplifier-control contract. The one-second live microphone test
analyzed PCM samples in memory and returned only level statistics; no recording was retained or
returned.

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
