Round 12 summary

Changes made:
- Routed relative `session_root_path` values from installed config files into the writable PAUS runtime directory, matching the install-safe handling for generated calibration artifacts.
- Updated the install-safe config regression test to expect `PAUS_ROBOT_RUNTIME_DIR/calibration_sessions`.
- Tightened UI backend connection detection: idle status no longer expires while services are ready, but stale transient-local status is treated as disconnected when backend services disappear.
- Added UI bridge regression coverage for both idle-service-ready and stale-services-gone backend states.

Validation:
- `python3 -m compileall -q src/paus_perception/paus_perception src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_motion_ros2/tests/test_control_config_defaults.py src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_ui/tests` -> 27 passed
- `colcon build --packages-select paus_perception paus_marker_ros2 paus_ui --symlink-install`

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `bitlesson-selector` is unavailable on the lab host in this environment, so no existing lesson was selected or updated.
