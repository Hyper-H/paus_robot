- [P2] Reject waypoint recording while semi-auto motion is active — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:842-844
  Unlike `capture/solve/save`, this callback never calls `_reject_manual_service_if_semi_auto_active()`. If an operator clicks "record current waypoint" while `/eye_to_hand/run_semi_auto_calibration` is moving through the trajectory, the node will sample whatever transient TCP pose happens to be in flight and append it to `trajectory_path`, silently corrupting the waypoint set used by later runs.

- [P2] Keep `get_status()` from recomputing live board detection — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:274-276
  `get_status()` now calls `get_latest_quality()`, which reruns chessboard detection and `solvePnP` on the latest frame. The frontend already polls `/api/handeye/quality` and `/api/image/latest.jpg`, so each `/api/status` poll adds a third full CV pass; on high-resolution streams this will noticeably slow the UI and delay other requests even though status itself should be cheap.
2026-04-30T19:23:50.730545Z ERROR codex_core::session: failed to record rollout items: thread 019ddfd2-3e79-7211-bc67-a9d0f7ead76d not found
2026-04-30T19:23:50.740307Z ERROR codex_core::session: failed to record rollout items: thread 019ddfd2-3e65-7330-85ae-42677feac1fc not found
The new functionality is broadly implemented, but it introduces at least one workflow-safety issue around trajectory editing during active semi-auto runs and a status path that does significantly more work than the UI polling pattern can safely afford.

Full review comments:

- [P2] Reject waypoint recording while semi-auto motion is active — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:842-844
  Unlike `capture/solve/save`, this callback never calls `_reject_manual_service_if_semi_auto_active()`. If an operator clicks "record current waypoint" while `/eye_to_hand/run_semi_auto_calibration` is moving through the trajectory, the node will sample whatever transient TCP pose happens to be in flight and append it to `trajectory_path`, silently corrupting the waypoint set used by later runs.

- [P2] Keep `get_status()` from recomputing live board detection — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:274-276
  `get_status()` now calls `get_latest_quality()`, which reruns chessboard detection and `solvePnP` on the latest frame. The frontend already polls `/api/handeye/quality` and `/api/image/latest.jpg`, so each `/api/status` poll adds a third full CV pass; on high-resolution streams this will noticeably slow the UI and delay other requests even though status itself should be cheap.
