# Initial-pose execution v1

`limo_request_initial_pose` submits `limo.set_initial_pose` through the local
ROSClaw control plane. The MCP process performs map, geofence, occupancy, frame,
and finite-number validation, then sends a bounded action envelope to
`rosclawd`. It never imports `rospy` and never publishes a ROS topic.

For REAL execution, the tool also accepts the exact operator-proposal
`deadline_at` ISO 8601 timestamp. Passing it back with the daemon-issued
`approval_id` keeps the submitted action intent identical to its one-shot
permit.

For REAL execution, the daemon validates the immutable LIMO Body snapshot and
an exact operator permit, then starts `worker/limo_initial_pose_worker.py` from
the revision-locked adapter checkout. The worker supports one operation only:
publishing a `geometry_msgs/PoseWithCovarianceStamped` message on
`/initialpose`. Covariance is fixed by the integration contract rather than
chosen by the Agent.

A successful worker result requires all of the following after dispatch:

- AMCL was subscribed to `/initialpose`;
- a new `/amcl_pose` message was observed;
- the `map -> odom` transform was available;
- the worker response matched the requested action and protocol.

Only the resulting daemon Execution Receipt is success evidence. SHADOW mode
does not publish and cannot be used as evidence that localization changed.
