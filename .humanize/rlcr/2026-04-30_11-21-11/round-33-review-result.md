- [P2] Emit recorded waypoints in the format the session UI parses — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:879-879
  When users record a trajectory manually, this log entry stores the waypoint fields at the top level. `SessionStore.read_session_waypoints()` only consumes events with a nested `waypoint` object or `waypoint_name`, so archived recording sessions end up looking empty in the UI even though the waypoints were recorded.

- [P3] Keep dry-run stages distinct in the workflow renderer — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:410-410
  `_workflow_for_status()` reports `stage: "dry_run"` for dry-run waypoint events, but this helper rewrites that to `movej`. As a result the flow widget never shows the dry-run branch or completion state, so operators see the same first-step highlight throughout a dry-run run.
2026-05-01T04:56:10.215500Z ERROR codex_core::session: failed to record rollout items: thread 019de1da-32c1-7da1-8337-d463073c1b71 not found
2026-05-01T04:56:10.269299Z ERROR codex_core::session: failed to record rollout items: thread 019de1da-32a0-7782-b27d-5b25dce59dd4 not found
The patch leaves the new manual-recording archive effectively unreadable by the session UI, and the dry-run workflow visualization is misleading because it collapses dry-run status back to the first motion step. These are both user-visible regressions in the new feature set.

Full review comments:

- [P2] Emit recorded waypoints in the format the session UI parses — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:879-879
  When users record a trajectory manually, this log entry stores the waypoint fields at the top level. `SessionStore.read_session_waypoints()` only consumes events with a nested `waypoint` object or `waypoint_name`, so archived recording sessions end up looking empty in the UI even though the waypoints were recorded.

- [P3] Keep dry-run stages distinct in the workflow renderer — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:410-410
  `_workflow_for_status()` reports `stage: "dry_run"` for dry-run waypoint events, but this helper rewrites that to `movej`. As a result the flow widget never shows the dry-run branch or completion state, so operators see the same first-step highlight throughout a dry-run run.
