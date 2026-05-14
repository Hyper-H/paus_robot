- [P2] Start a new archive before manual capture after semi-auto runs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:332-339
  If an operator does a dry-run or semi-auto run and then switches to the manual `capture_sample`/`solve` flow without restarting the node, `self.session_dir` is still set by `_begin_new_semi_auto_session()`, so `_ensure_session_started()` reuses that old archive. The next manual samples append to the previous run’s `samples.jsonl` and can overwrite its `report.yaml`, which merges separate calibrations into one session history.

- [P2] Avoid reporting queued run requests as successful — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:454-460
  This marks `POST /api/handeye/run` as `success=true` before `/eye_to_hand/run_semi_auto_calibration` has actually replied. When the backend immediately rejects the run (for example missing trajectory, concurrent run, or another validation failure), the frontend’s `commandTone()` in `static/app.js` still shows a successful start until the background thread overwrites it, so callers get a false positive that calibration began.
2026-04-30T17:11:45.933771Z ERROR codex_core::session: failed to record rollout items: thread 019ddf59-92da-74a0-b1c7-f34f6f3c6e39 not found
The new UI/session workflow is close, but it still has correctness issues around session archival and run-start reporting. Those bugs can corrupt calibration history or briefly tell operators that a run started when the backend actually rejected it.

Full review comments:

- [P2] Start a new archive before manual capture after semi-auto runs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:332-339
  If an operator does a dry-run or semi-auto run and then switches to the manual `capture_sample`/`solve` flow without restarting the node, `self.session_dir` is still set by `_begin_new_semi_auto_session()`, so `_ensure_session_started()` reuses that old archive. The next manual samples append to the previous run’s `samples.jsonl` and can overwrite its `report.yaml`, which merges separate calibrations into one session history.

- [P2] Avoid reporting queued run requests as successful — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:454-460
  This marks `POST /api/handeye/run` as `success=true` before `/eye_to_hand/run_semi_auto_calibration` has actually replied. When the backend immediately rejects the run (for example missing trajectory, concurrent run, or another validation failure), the frontend’s `commandTone()` in `static/app.js` still shows a successful start until the background thread overwrites it, so callers get a false positive that calibration began.
