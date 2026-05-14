## Fixed Issues

- Fixed `[P2] Reset the auto-selected archive before the next waypoint-teaching pass`.
  - Added UI state tracking for auto-selected sessions.
  - The UI can temporarily follow the active run session while a run is active, then returns to `Current trajectory` when the run is no longer active.
  - Explicit user session selections are preserved.

- Fixed `[P1] Make the new config-path test importable from a source checkout`.
  - Added the source package root to `sys.path` in `test_config_paths.py`, matching the existing lightweight test style.

- Fixed `[P1] Keep the new session test out of the ROS-only import path`.
  - Moved the session-owner comparison into the pure `semi_auto_calibration` module.
  - Rewrote the session test to exercise the pure helper instead of importing the ROS node and its ROS dependencies.
  - Updated the calibration node to use the shared helper.

## Validation

- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui src/paus_perception/paus_perception`
- `PYTHONPATH="$PWD/src/paus_marker_ros2" python3 -m unittest discover -s src/paus_marker_ros2/tests`
  - `Ran 8 tests`
- `PYTHONPATH="$PWD/src/paus_perception" python3 -m pytest -q src/paus_perception/tests/test_config_paths.py`
  - `1 passed`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests src/paus_perception/tests/test_config_paths.py src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
  - `34 passed`
- `colcon build --packages-select paus_perception paus_marker_ros2 paus_ui --symlink-install`
  - `3 packages finished`

## Unresolved Issues

- None.

## BitLesson Delta

- Action: none
- Lesson ID(s): NONE
- `.humanize/bitlesson.md` was read before coding.
- `bitlesson-selector` was attempted, but the command is unavailable on the lab host (`bitlesson-selector: missing`).
