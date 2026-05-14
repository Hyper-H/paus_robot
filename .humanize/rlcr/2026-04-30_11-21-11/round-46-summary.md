## Round 46 Summary

Fixed the two review findings from round 46:

- Manual `save_trajectory` now archives the saved trajectory into the current session as `trajectory_used.yaml`, making manual sessions self-contained like semi-auto sessions.
- `semi_auto_started` is now mapped to the `movej` workflow stage so the UI progress bar shows a meaningful startup state before the first waypoint update.

## Validation

- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py`
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_ros_bridge.py`
  - `28 passed`
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`
- `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
  - `56 passed`

## BitLesson Delta

- Action: add
- Lesson ID(s): BL-20260501-handeye-session-workflow-completeness
- Notes: Captured the self-contained session archive rule and startup workflow mapping so manual and semi-auto UI feedback remain complete.
