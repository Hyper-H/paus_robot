Round 14 summary

Changes made:
- `eye_to_hand_calibration.launch.py` now treats empty config-backed launch arguments as "load from the selected config_path" during `_launch_setup()`, so custom calibration config files control trajectory/session paths, thresholds, dwell time, tool-to-board, and solver defaults.
- `ui.launch.py` uses the same selected-config resolution for calibration/UI parameters instead of baking values from the repository default YAML.
- Manual calibration sessions now clear the active session state after a successful save. The next `capture_sample` starts a fresh archive instead of appending samples/report data into the previous manual run.

Validation:
- `python3 -m py_compile src/paus_bringup/launch/eye_to_hand_calibration.launch.py src/paus_bringup/launch/ui.launch.py`
- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2`
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_ui/tests src/paus_motion_ros2/tests/test_control_config_defaults.py` -> 30 passed
- `colcon build --packages-select paus_marker_ros2 paus_ui paus_bringup --symlink-install`

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `bitlesson-selector` is unavailable on the lab host in this environment, so no existing lesson was selected or updated.
