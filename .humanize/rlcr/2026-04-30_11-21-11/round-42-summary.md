## Round 42 Summary

Fixed the two review findings from round 42:

- Deleted waypoints are now logged with the normalized `waypoint_deleted` schema, and `SessionStore` removes them from archived session reconstruction so the archive view no longer shows phantom pending points.
- Semi-auto session creation is now deferred until the trajectory YAML has been validated, and validation failures no longer create empty session directories or append failure logs into an existing archive.

## Validation

- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_session_store.py`
  - `17 passed`
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`
- `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
  - `51 passed`

## BitLesson Delta

- Action: add
- Lesson ID(s): BL-20260501-handeye-archive-integrity
- Notes: Recorded the archive-integrity pattern so future rounds can reuse the same fix shape for validation-before-session and normalized deletion events.
