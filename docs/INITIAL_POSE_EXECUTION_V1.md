# Initial-pose execution v1

`limo_request_initial_pose` submits `limo.set_initial_pose` through the local
ROSClaw control plane. The MCP process performs map, geofence, occupancy, frame,
and finite-number validation, then sends a bounded action envelope to
`rosclawd`. It never imports `rospy` and never publishes a ROS topic.

For REAL execution, the tool prepares one exact proposal and asks the MCP host
to render an operator confirmation form. The card includes the robot, action,
target, physical effect, deadline, and action-intent hash. If the operator
accepts, ROSClaw exchanges that exact proposal for a one-shot daemon permit,
injects it internally, and submits the action in the same tool call. The Agent
does not receive or provide a permit identifier or operator principal.

Clients that do not support MCP elicitation fail closed with
`MCP_ELICITATION_UNAVAILABLE` and the non-secret confirmation card. Decline and
cancel responses return `OPERATOR_CONFIRMATION_DECLINED`; neither path submits
an action. Codex must allow MCP elicitation prompts and keep the approval
reviewer set to the user for this interaction to be treated as operator input.

For REAL execution, the daemon validates the immutable LIMO Body snapshot and
an exact operator permit, then starts `worker/limo_initial_pose_worker.py` from
the revision-locked adapter checkout. The worker supports one operation only:
publishing a `geometry_msgs/PoseWithCovarianceStamped` message on
`/initialpose`. Covariance is fixed by the integration contract rather than
chosen by the Agent.

A successful worker result requires all of the following after dispatch:

- AMCL was subscribed to `/initialpose`;
- a `/amcl_pose` sample converged near the requested estimate within the bounded timeout;
- the `map -> odom` transform was available;
- the worker response matched the requested action and protocol.

Only the resulting daemon Execution Receipt is success evidence. SHADOW mode
does not publish and cannot be used as evidence that localization changed.
