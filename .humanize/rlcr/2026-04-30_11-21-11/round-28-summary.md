# Review Round 28 Summary

## Work Completed
- Fixed `[P1] Don't persist waypoints before the recording session is committed` by keeping recorded/deleted waypoints in node memory until `/eye_to_hand/save_trajectory` is called.
- Fixed `[P2] Enter record mode when --record-only is requested` by including `args.record_only` in the CLI `should_record` condition.

## Files Changed
- `src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py`
- `src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_semi_auto_cli.py`

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
- Notes: This was a narrow transactional recording fix; no durable new lesson is needed.
