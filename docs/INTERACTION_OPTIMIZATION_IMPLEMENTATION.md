# Interaction optimization implementation

This refactor implements the first deployable slice of
`rosclaw_limo_mcp_interaction_optimization_plan.md` across ROSClaw and LIMO MCP.

## Implemented

- All navigation, initial-pose, and tone confirmation flows delegate to
  `rosclaw.interaction.InteractionCoordinator`.
- Action semantics live in pure `action_specs` modules and use `rosclaw.action-display.v2`.
- Agent-visible results omit permits, sessions, daemon arming, approval hashes, and the internal
  approval request.
- `ObservationHub` reuses allowlisted read-only transports, retains bounded summary records, and
  invalidates generation-bound data after reconnect.
- Readiness collection uses a coherent monotonic cutoff, freshness-aware cache reuse, concurrent
  topic collection, and per-key single-flight deduplication.
- Readiness snapshots bind the endpoint, transport, and transport generation. Navigation rejects
  a snapshot after the observation transport generation changes.
- `core` (default), `inspection`, and `full` profiles bound MCP discovery. `--compat-tools`
  temporarily exposes the former `limo_get_patrol_readiness` name.
- `limo_get_readiness` defaults to a sub-10-KiB summary projection while retaining the sealed
  snapshot hash; `detail=full` is explicit.
- Core tools publish read-only/destructive/idempotent annotations and physical-execution metadata.

## Deliberately deferred

- The optional web Operator Console and URL completion callback.
- Mission/site-policy authorization grants beyond exact-action confirmation.
- A separately published lightweight `rosclaw-control-sdk` distribution. The current repository
  still installs ROSClaw from the sibling checkout.
- A long-running ROS Melodic worker over Unix sockets. The current hub persistently reuses the
  bounded rosbridge/ROS CLI client abstraction and cache without adding a publisher or shell API.

No test in this refactor publishes ROS messages or drives hardware. Real-robot validation must be
performed later with the operator-started graph and the existing verified `rosclawd` executor.
