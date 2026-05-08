## Round 44 Summary

Fixed the two review findings from round 44:

- Manual waypoint record/delete edits now start a manual session before logging, so record-only or edit-only sessions keep an archive that the Sessions UI can reconstruct after restart.
- UI bridge service waits now honor the caller's full timeout instead of being capped at two seconds, which makes startup/restart races less brittle.

## Validation

- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/tests/test_session_store.py`
  - `38 passed`
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`
- `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
  - `54 passed`

## BitLesson Delta

- Action: add
- Lesson ID(s): BL-20260501-handeye-manual-edit-archive, BL-20260501-handeye-ui-service-timeout
- Notes: Captured both the archive-first manual edit pattern and the timeout-serialization issue so future UI and ROS bridge work can reuse them.
