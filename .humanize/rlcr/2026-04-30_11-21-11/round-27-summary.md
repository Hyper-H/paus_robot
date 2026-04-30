# Review Round 27 Summary

## Work Completed
- Fixed `[P1] Let semi-auto runs call the solver while active` by splitting the manual solve service gate from the internal solve implementation.
- `_solve_callback()` still rejects external/manual solve requests while `_semi_auto_active` is true.
- `_run_semi_auto_callback()` now calls `_solve_samples()` directly, so a successful semi-auto run can solve and then save via `_save_current_solution()`.
- The repeated Round 26 findings for waypoint editing and cheap `get_status()` remain fixed in the current code.

## Files Changed
- `src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py`

## Validation
- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui src/paus_perception/paus_perception`
- `PYTHONPATH="$PWD/src/paus_ui:$PWD/src/paus_perception" python3 -m pytest -q src/paus_ui/tests/test_path_resolvers.py src/paus_ui/tests/test_ros_bridge.py` passed: 1 passed, 1 skipped without ROS setup.
- `PYTHONPATH="$PWD/src/paus_marker_ros2" python3 -m unittest discover -s src/paus_marker_ros2/tests` passed: 10 tests.
- `PYTHONPATH="$PWD/src/paus_perception" python3 -m pytest -q src/paus_perception/tests/test_config_paths.py` passed: 1 test.
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests src/paus_perception/tests/test_config_paths.py src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` passed: 39 tests.
- `colcon build --packages-select paus_perception paus_marker_ros2 paus_ui --symlink-install` passed: 3 packages.

## Remaining Items
- None.

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: The fix is a narrow separation between service-level guarding and internal workflow reuse; no durable new lesson is needed.
