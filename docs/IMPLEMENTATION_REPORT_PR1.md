# PR-1 Implementation Report: Readiness Evidence v1

## Identity

- Date: 2026-07-30 (Asia/Shanghai)
- Repository: `ros-claw/limo-ros-mcp`
- Baseline: `main@1b45b648f3b566a81e000ab279fa7b804ffb6dd0`
- Branch: `agent/readiness-evidence-v1`
- PR: https://github.com/ros-claw/limo-ros-mcp/pull/3
- Target package version: `0.3.0`

## Implemented requirements

| Requirement | Implementation |
| --- | --- |
| Versioned readiness contract | `src/limo_ros_mcp/schemas.py`, `readiness.py` |
| Canonical JSON and SHA-256 seal | `src/limo_ros_mcp/evidence.py` |
| Five-second expiry and validation | `readiness.py::validate_readiness_snapshot` |
| Wall/ROS/receive time classification | `src/limo_ros_mcp/clocks.py` |
| Wall-clock jump invalidation | `clocks.py::ClockJumpTracker` and service integration |
| Operator policy and bounded thresholds | `src/limo_ros_mcp/policy.py`, `configs/limo_readiness_policy.yaml` |
| Structured checks and stable error codes | `readiness.py`, `src/limo_ros_mcp/errors.py` |
| Costmap and TF evidence | `server.py::patrol_readiness`, `readiness.py` |
| Per-observation receive time and transport generation | `server.py`, `rosbridge.py`, `roscli.py` |
| Single ROS graph preflight and shared batch transport | `server.py::_collect_summaries`, `roscli.py::probe` |
| Multi-message TF edge aggregation | `server.py::_observe_with_client` |
| YDLidar finite-hit/no-return/unsupported-slot distinction | `messages.py::summarize_laser_scan` |
| Non-finite values fail closed | `messages.py`, canonical serializer, regression tests |
| Raw binary/grid payload denial | `server.py`, contract tests |
| Legacy REAL self-report blocked | `server.py::request_navigation` |
| SHADOW migration warning | `server.py::request_navigation` |
| Packaged policy and v0.3 metadata | `pyproject.toml`, `uv.lock`, `manifest.json` |

The snapshot contains no permit, credential, image, point cloud, or occupancy-grid payload. Compact summaries are hashed and retained so a reviewer can explain each check. The snapshot is explicitly advisory and is never marked usable for REAL execution.

## Data contract changes

`limo_get_patrol_readiness` now returns `readiness.schema_version = limo.readiness.v1` with:

- snapshot/body/policy identity;
- creation and expiry wall times;
- clock-domain status and observation-window skew;
- per-observation topic/type, ROS stamp, receive time, age, freshness basis, transport generation, compact summary and summary hash;
- sorted checks with severity, operator, threshold, unit, error code and evidence references;
- sorted blockers/warnings and a hash excluding `snapshot_hash` itself.

The MCP tool count remains 21. Optional `body_id` and `body_snapshot_hash` inputs were added to readiness collection. An unbound body snapshot is a warning in client evidence, not authorization.

## Safety-boundary changes

- No ROS publish, `/cmd_vel`, serial, CAN, vendor SDK, local Runtime, executor registration, or permit creation was added.
- Raw Image, PointCloud2, map, and costmap responses are denied even when `include_raw=true`.
- Caller-provided `localization_ready/costmap_ready/obstacle_check_enabled` can no longer submit REAL. SHADOW keeps temporary compatibility and returns a deprecation warning.
- A readiness hash remains client evidence only. A future daemon-owned executor must repeat trusted preflight.

## Commands and results

```text
.venv/bin/ruff format .
.venv/bin/ruff check .
  PASS

.venv/bin/mypy src
  PASS: Success: no issues found in 13 source files

.venv/bin/python -m compileall -q src tests
  PASS

.venv/bin/pytest -q
  PASS: 52 passed, 5 skipped

.venv/bin/pytest -q tests/test_stdio.py tests/test_server.py tests/test_readiness.py
  PASS: 16 passed

.venv/bin/python -m build --outdir /tmp/limo-mcp-build-pr1-20260730-live
  PASS: sdist and wheel built

clean venv install of limo_ros_mcp-0.3.0-py3-none-any.whl
  PASS: packaged policy loaded; MCP tool count = 21

LIMO_LIVE_ROS=1 .venv/bin/pytest -vv -s tests/test_live_ros.py
  PASS: 5 passed in 27.42 s

real MCP stdio call: limo_get_patrol_readiness(transport=roscli)
  PASS: MCP protocol error=false; 11 available observations; 0 missing
  Snapshot: sha256:f7db5d5441cf9777d002d4c638ec4a5e7d9155e2cf30846c6399acbeb8000413
  State: BLOCKED
  Blockers: LIMO_AMCL_STALE, LIMO_TF_CHAIN_BROKEN
  Warnings: LIMO_BODY_SNAPSHOT_UNBOUND, LIMO_DIAGNOSTIC_WARN
  Observation-window skew: 693.38 ms
```

The first 2026-07-30 live attempt correctly failed while ROS master was unavailable. After the operator started LIMO bringup, Dabai, AMCL, map server, and move_base, the complete read-only suite passed. The final BLOCKED result is intentional: `/amcl_pose` remained over 1,000 seconds old, diagnostics reported `amcl: Standard deviation / Too large`, and the dynamic TF sample therefore had no `map→odom` edge. No localization initialization or motion was attempted by the Agent.

## Artifact hashes

```text
sdist  sha256:5f304634e4304fad2868793d7ab3d22e2948e67bb7e70a536b08629c24edc2ed
wheel  sha256:35fe0fb94c04377db37493c92c6a49e15529b9cecafae6254a4e8a7bc9fc5a2f
default readiness policy  sha256:92267704a0e4a199be8f3a1aca81ae465c4ede5cc68287c73db498d02c80c547
```

## Physical and Practice evidence

- `hardware_actions_executed`: **0**
- Read-only hardware checks on 2026-07-30: **5 passed**
- Final readiness snapshot observations: **11 available, 0 missing**
- Practice IDs: none
- Receipt IDs: none
- REAL permits: none
- SHADOW actions submitted: none

## Known gaps and claims that are not allowed

This PR does **not** complete Navigation Contract v2, map/geofence/make-plan validation, daemon-owned ROS 1 actionlib worker, lease guard, physical-stop proof, Robot Integration loader, Practice linkage, Gazebo chaos tests, Mission SR, H3/H4/H5 evidence, or canonical product promotion.

Additional P0 follow-ups remain:

- automatically detect `/use_sim_time` from an operator-owned observation source;
- reuse a bounded rosbridge connection for the multi-topic readiness window;
- split the remaining orchestration/control code out of `server.py` into `service.py` and transport packages;

## Next gate

PR-2 must implement Navigation Contract v2: replace legacy booleans with an evidence reference, validate expiry/tampering, add map/geofence/no-go/occupied/plan checks, and keep REAL blocked until the ROSClaw companion daemon preflight/executor exists.

Publication status: branch pushed, Draft PR opened, and the local branch tracks its remote.
