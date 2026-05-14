Round 11 summary

Changes made:
- Fixed backend connection state in `UiRosBridge`: a received backend status no longer expires after 10 seconds of idle time, and visible backend services also count as connected.
- Added `src/paus_ui/tests/conftest.py` so UI tests can import `paus_ui`, `paus_marker_ros2`, `paus_perception`, and `paus_motion_ros2` directly from a source checkout.
- Moved wrapped RPY delta calculation into ROS-free `semi_auto_calibration.py` and updated the trajectory test to import it from there instead of importing the ROS calibration node.
- Added a regression test confirming stale-but-valid backend status remains connected.

Validation:
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_ui/tests/test_operator_messages.py src/paus_ui/tests/test_overlay.py src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_session_store_shaping.py` -> 14 passed
- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_ui/tests` -> 17 passed
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `bitlesson-selector` is unavailable on the lab host in this environment, so no existing lesson was selected or updated.
