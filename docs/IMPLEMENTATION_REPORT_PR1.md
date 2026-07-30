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
  PASS: 50 passed, 5 skipped

.venv/bin/pytest -q tests/test_stdio.py tests/test_server.py tests/test_readiness.py
  PASS: 16 passed

.venv/bin/python -m build --outdir /tmp/limo-mcp-build-pr1-20260730
  PASS: sdist and wheel built

clean venv install of limo_ros_mcp-0.3.0-py3-none-any.whl
  PASS: packaged policy loaded; MCP tool count = 21

LIMO_LIVE_ROS=1 .venv/bin/pytest -vv -s tests/test_live_ros.py
  NOT RUNNABLE: 5 failed because ROS master was unavailable
  Structured probe error: "ERROR: Unable to communicate with master!"
  ROS_MASTER_URI: http://localhost:11311
  No roscore/rosmaster/limo/move_base/amcl process was running.
```

The live result is an infrastructure precondition failure, not hardware evidence and not counted as a passing test. It must be rerun when the operator starts the LIMO ROS graph.

## Artifact hashes

```text
sdist  sha256:2ef198cbe542182da97f62700de74a276108cde5fb567371ac284621234ba124
wheel  sha256:07946e60e5b42f1119cae3dcf1c1e0ddd4b6e05eea33426813c2956a7d4ced58
default readiness policy  sha256:0be89d067256ce30283e73d02b9883ca5f71cd642533a2d1a625505c3fabc3bd
```

## Physical and Practice evidence

- `hardware_actions_executed`: **0**
- Read-only hardware observations on 2026-07-30: **0**, ROS master unavailable
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
- rerun all five read-only live tests with the operator-started ROS graph and archive a real snapshot hash.

## Next gate

PR-2 must implement Navigation Contract v2: replace legacy booleans with an evidence reference, validate expiry/tampering, add map/geofence/no-go/occupied/plan checks, and keep REAL blocked until the ROSClaw companion daemon preflight/executor exists.

Publication status: branch pushed, Draft PR opened, and the local branch tracks its remote.
