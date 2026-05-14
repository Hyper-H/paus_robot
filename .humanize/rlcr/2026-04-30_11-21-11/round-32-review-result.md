- [P2] Normalize relative path overrides in ui.launch.py — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:59-61
  When an operator overrides `trajectory_path`, `output_path`, or `session_root_path` with a relative value (or a `~`/env-based path) on `ros2 launch paus_bringup ui.launch.py`, these values are forwarded verbatim instead of going through the same resolver used for config defaults. That makes the calibration node and UI server interpret them relative to the launch process CWD, so documented overrides like `trajectory_path:=src/paus_bringup/configs/eye_to_hand_trajectory.yaml` only work from the workspace root and fail from other directories.

- [P2] Normalize relative path overrides in calibration launch — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/eye_to_hand_calibration.launch.py:70-72
  `eye_to_hand_calibration.launch.py` has the same regression for `output_path`, `trajectory_path`, and `session_root_path`: explicit launch-arg overrides bypass the project/runtime path resolver and are passed through as raw strings. In practice, `ros2 launch ... output_path:=src/paus_bringup/configs/extrinsics.yaml` or similar now depends on the caller's CWD instead of resolving consistently against the config/workspace, so the node will read/write the wrong files when launched from elsewhere.

- [P3] Handle `waypoint_capture_disabled` in workflow mapping — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:650-651
  The backend now publishes `waypoint_capture_disabled` for supported `capture: false` waypoints, but `_workflow_for_status` does not recognize that status. During such runs the UI falls back to the default `idle` stage and shows the raw status string, so the live workflow/progress indicator regresses exactly when a no-capture waypoint completes.
2026-05-01T04:35:55.703910Z ERROR codex_core::session: failed to record rollout items: thread 019de1c3-22cb-7b92-9f2a-84cbadcbb168 not found
2026-05-01T04:35:55.760247Z ERROR codex_core::session: failed to record rollout items: thread 019de1c3-22a9-74d1-8108-e91e87c412d9 not found
The new launch files mishandle relative path overrides, so common `ros2 launch ... path:=...` usage can target the wrong files outside the workspace root. There is also a smaller UI-state regression for `capture:false` waypoints because the new backend status is not mapped in the workflow model.

Full review comments:

- [P2] Normalize relative path overrides in ui.launch.py — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:59-61
  When an operator overrides `trajectory_path`, `output_path`, or `session_root_path` with a relative value (or a `~`/env-based path) on `ros2 launch paus_bringup ui.launch.py`, these values are forwarded verbatim instead of going through the same resolver used for config defaults. That makes the calibration node and UI server interpret them relative to the launch process CWD, so documented overrides like `trajectory_path:=src/paus_bringup/configs/eye_to_hand_trajectory.yaml` only work from the workspace root and fail from other directories.

- [P2] Normalize relative path overrides in calibration launch — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/eye_to_hand_calibration.launch.py:70-72
  `eye_to_hand_calibration.launch.py` has the same regression for `output_path`, `trajectory_path`, and `session_root_path`: explicit launch-arg overrides bypass the project/runtime path resolver and are passed through as raw strings. In practice, `ros2 launch ... output_path:=src/paus_bringup/configs/extrinsics.yaml` or similar now depends on the caller's CWD instead of resolving consistently against the config/workspace, so the node will read/write the wrong files when launched from elsewhere.

- [P3] Handle `waypoint_capture_disabled` in workflow mapping — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:650-651
  The backend now publishes `waypoint_capture_disabled` for supported `capture: false` waypoints, but `_workflow_for_status` does not recognize that status. During such runs the UI falls back to the default `idle` stage and shows the raw status string, so the live workflow/progress indicator regresses exactly when a no-capture waypoint completes.
