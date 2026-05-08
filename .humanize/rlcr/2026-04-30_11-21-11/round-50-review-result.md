- [P2] Reset stale backend overrides on disconnect — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:248-252
  When the calibration node was previously publishing status and then disappears, this fallback branch only restores the path/threshold fields. `execute_motion`, `camera_config_path`, `board_rows`, `board_cols`, and `square_size_m` stay at their last backend-derived values, so the UI can keep showing the wrong motion mode and keep using stale detector settings after a node restart/disconnect. That makes the safety chip and live quality overlay inaccurate until a fresh status arrives.

- [P2] Show trajectory load failures in the current-trajectory view — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:277-279
  If `eye_to_hand_trajectory.yaml` is missing or malformed, `/api/handeye/waypoints` returns an `error`, but this code immediately maps `trajectory.waypoints` and never surfaces that failure. In that case the operator just sees `0` waypoints and an empty table, even though the real problem is that the trajectory could not be parsed; the next run/save attempt then fails without any prior explanation in the UI.
2026-05-01T12:41:56.176681Z ERROR codex_core::session: failed to record rollout items: thread 019de37e-fc3d-7dc1-8f65-de27a731bf4b not found
The new UI/state-sync logic has at least two functional gaps: it can retain stale backend-derived safety/calibration settings after disconnect, and it hides trajectory load errors from the operator by rendering them as an empty waypoint list. Those issues make the patch unreliable in real operator workflows.

Full review comments:

- [P2] Reset stale backend overrides on disconnect — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:248-252
  When the calibration node was previously publishing status and then disappears, this fallback branch only restores the path/threshold fields. `execute_motion`, `camera_config_path`, `board_rows`, `board_cols`, and `square_size_m` stay at their last backend-derived values, so the UI can keep showing the wrong motion mode and keep using stale detector settings after a node restart/disconnect. That makes the safety chip and live quality overlay inaccurate until a fresh status arrives.

- [P2] Show trajectory load failures in the current-trajectory view — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:277-279
  If `eye_to_hand_trajectory.yaml` is missing or malformed, `/api/handeye/waypoints` returns an `error`, but this code immediately maps `trajectory.waypoints` and never surfaces that failure. In that case the operator just sees `0` waypoints and an empty table, even though the real problem is that the trajectory could not be parsed; the next run/save attempt then fails without any prior explanation in the UI.
