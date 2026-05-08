- [P2] Start a manual session before logging waypoint edits — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:880-887
  If the operator records or deletes waypoints before any capture/semi-auto run, `run_log_path` is still `None`, so these edits are silently dropped. That means record-only sessions never get an archive the new Sessions UI can reconstruct after a restart, even though the trajectory was edited and saved.

- [P2] Honor the caller's full service wait timeout — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:526-526
  `timeout_s` is effectively capped at 2 seconds here, so `run_semi_auto` and the other UI commands can fail as "service unavailable" during normal startup/restart races even when the caller asked to wait much longer. This makes the new UI brittle whenever the calibration node advertises its services slowly.
2026-05-01T09:57:07.750405Z ERROR codex_core::session: failed to record rollout items: thread 019de2ea-f2af-72d1-a080-49a59c01d682 not found
2026-05-01T09:57:07.760134Z ERROR codex_core::session: failed to record rollout items: thread 019de2ea-f29a-7731-9181-958ad100a24e not found
The patch introduces at least one data-loss path for manual waypoint edits and a premature service timeout in the UI bridge, both of which can break expected workflows.

Full review comments:

- [P2] Start a manual session before logging waypoint edits — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:880-887
  If the operator records or deletes waypoints before any capture/semi-auto run, `run_log_path` is still `None`, so these edits are silently dropped. That means record-only sessions never get an archive the new Sessions UI can reconstruct after a restart, even though the trajectory was edited and saved.

- [P2] Honor the caller's full service wait timeout — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:526-526
  `timeout_s` is effectively capped at 2 seconds here, so `run_semi_auto` and the other UI commands can fail as "service unavailable" during normal startup/restart races even when the caller asked to wait much longer. This makes the new UI brittle whenever the calibration node advertises its services slowly.
