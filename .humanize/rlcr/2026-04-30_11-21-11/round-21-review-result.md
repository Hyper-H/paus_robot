- [P2] Reset the auto-selected archive before the next waypoint-teaching pass — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:154-155
  Once a calibration run starts, this permanently switches the table into archived-session mode. After the run finishes, pressing Record/Delete still updates the backend trajectory, but `refreshWaypoints()` keeps reading `/api/sessions/{id}/waypoints`, so the operator sees the old archive instead of the live trajectory they are teaching. This is easy to hit on the second teaching pass unless the user manually switches the dropdown back to `当前示教轨迹` first.

- [P1] Make the new config-path test importable from a source checkout — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/tests/test_config_paths.py:3-5
  This new test imports `paus_perception` directly but never adds `src/paus_perception` to `sys.path` the way the existing test modules do. In a normal checkout run, collection fails immediately with `ModuleNotFoundError: paus_perception`, so the regression coverage for installed-path resolution never actually executes unless the package was installed first.

- [P1] Keep the new session test out of the ROS-only import path — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/tests/test_eye_to_hand_session.py:13-13
  Importing `EyeToHandCalibrationNode` here pulls in `ament_index_python`, `rclpy`, and the rest of the ROS stack during test collection. In the repo's lightweight source-checkout test environment, `python3 -m unittest discover -s src/paus_marker_ros2/tests` now fails before this test runs with `ModuleNotFoundError: ament_index_python`, even though the assertions only exercise the session helper methods.
2026-04-30T18:13:12.584283Z ERROR codex_core::session: failed to record rollout items: thread 019ddf92-43dc-79b0-b405-c5ab0441fb85 not found
2026-04-30T18:13:12.594128Z ERROR codex_core::session: failed to record rollout items: thread 019ddf92-43c9-7d42-b84b-f532e6d5f22c not found
The new UI flow has a state-selection regression after a run, and two newly added tests do not collect cleanly in the repository's source-checkout environment. Those issues are enough to make the patch unsafe to treat as correct.

Full review comments:

- [P2] Reset the auto-selected archive before the next waypoint-teaching pass — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:154-155
  Once a calibration run starts, this permanently switches the table into archived-session mode. After the run finishes, pressing Record/Delete still updates the backend trajectory, but `refreshWaypoints()` keeps reading `/api/sessions/{id}/waypoints`, so the operator sees the old archive instead of the live trajectory they are teaching. This is easy to hit on the second teaching pass unless the user manually switches the dropdown back to `当前示教轨迹` first.

- [P1] Make the new config-path test importable from a source checkout — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/tests/test_config_paths.py:3-5
  This new test imports `paus_perception` directly but never adds `src/paus_perception` to `sys.path` the way the existing test modules do. In a normal checkout run, collection fails immediately with `ModuleNotFoundError: paus_perception`, so the regression coverage for installed-path resolution never actually executes unless the package was installed first.

- [P1] Keep the new session test out of the ROS-only import path — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/tests/test_eye_to_hand_session.py:13-13
  Importing `EyeToHandCalibrationNode` here pulls in `ament_index_python`, `rclpy`, and the rest of the ROS stack during test collection. In the repo's lightweight source-checkout test environment, `python3 -m unittest discover -s src/paus_marker_ros2/tests` now fails before this test runs with `ModuleNotFoundError: ament_index_python`, even though the assertions only exercise the session helper methods.
