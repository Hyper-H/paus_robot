# Round 40 Summary

## Fixed Issues
- [P1] Restored installed-layout calibration artifacts to runtime storage for `output_path` and `trajectory_path`.
- [P2] Prevented live quality polling from overwriting the current waypoint pose in the UI.

## Resolution
- `resolve_config_artifact_path()` now resolves installed single-file calibration artifacts back into `${PAUS_ROBOT_RUNTIME_DIR}/configs`, matching the writable runtime layout again.
- `refreshQuality()` now only updates live quality metrics and no longer rewrites the current waypoint pose fields that are owned by `/api/status`.
- Added regression coverage for install-layout path resolution, UI path resolution, and the existing status snapshot tests continue to guard the waypoint pose rendering path.

## Validation
- `python3 -m compileall -q src/paus_perception/paus_perception/config.py src/paus_perception/tests/test_config_paths.py src/paus_ui/paus_ui/static/app.js src/paus_ui/tests/test_path_resolvers.py src/paus_ui/tests/test_ros_bridge.py`
- `node --check src/paus_ui/paus_ui/static/app.js`
- `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests/test_path_resolvers.py src/paus_ui/tests/test_ros_bridge.py`
- `colcon build --packages-select paus_perception paus_ui --symlink-install`
- `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`

## Result
- Selected regression tests: 21 passed
- Cross-package regression tests: 49 passed

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: No new reusable lesson was added; this round aligned install-safe write paths and preserved status-owned waypoint poses in the UI.
