- [P2] Serve in-memory waypoints while recording a trajectory — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:421-423
  `/api/handeye/waypoints` always reads `trajectory_path` from disk, but `record_waypoint`/`delete_last_waypoint` only update `EyeToHandCalibrationNode.recorded_trajectory` in memory until `save_trajectory` is called. In the new UI recording flow, this means the “当前示教轨迹” table keeps showing the old YAML contents (or an empty/error state if the file does not exist), so operators cannot verify newly recorded or deleted waypoints before saving.

- [P2] Drop stale backend status once the calibration node disconnects — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:273-275
  When the calibration node stops publishing and its services disappear, `_sync_backend_state()` correctly marks the backend disconnected, but `get_status()` still builds `workflow`, `current_waypoint`, and `session` from `self._last_status` before that connectivity check. In that scenario `/api/status` keeps returning the last waypoint pose/session as if it were live, so the UI can display stale calibration state for a dead backend.
2026-04-30T20:04:22.040675Z ERROR codex_core::session: failed to record rollout items: thread 019ddff9-31b9-7591-af8b-34755860ecab not found
2026-04-30T20:04:22.050681Z ERROR codex_core::session: failed to record rollout items: thread 019ddff9-31a5-7692-9e26-4265e75cc133 not found
The new UI/backend integration has at least two functional issues: the trajectory recording view does not reflect unsaved waypoint edits, and stale backend status remains visible after the calibration node disconnects. Both are user-visible regressions in the newly added hand-eye workflow.

Full review comments:

- [P2] Serve in-memory waypoints while recording a trajectory — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:421-423
  `/api/handeye/waypoints` always reads `trajectory_path` from disk, but `record_waypoint`/`delete_last_waypoint` only update `EyeToHandCalibrationNode.recorded_trajectory` in memory until `save_trajectory` is called. In the new UI recording flow, this means the “当前示教轨迹” table keeps showing the old YAML contents (or an empty/error state if the file does not exist), so operators cannot verify newly recorded or deleted waypoints before saving.

- [P2] Drop stale backend status once the calibration node disconnects — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:273-275
  When the calibration node stops publishing and its services disappear, `_sync_backend_state()` correctly marks the backend disconnected, but `get_status()` still builds `workflow`, `current_waypoint`, and `session` from `self._last_status` before that connectivity check. In that scenario `/api/status` keeps returning the last waypoint pose/session as if it were live, so the UI can display stale calibration state for a dead backend.
