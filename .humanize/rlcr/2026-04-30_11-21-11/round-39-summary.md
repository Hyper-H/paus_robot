# Round 39 Summary

## Fixed Issues
- [P1] Kept install-layout `extrinsics.yaml` on the same file the live stack reads.
- [P2] Prevented backend state from expiring during active semi-auto runs with long stable waits.
- [P3] Rendered workflow failures as error states instead of skipped waypoints.

## Resolution
- `resolve_config_artifact_path()` now resolves single-file config artifacts in installed layouts next to the installed config file, so calibration writes land on the live `share/.../configs/extrinsics.yaml` file.
- `UiRosBridge._sync_backend_state()` now treats an active run thread as a live backend signal, keeping `backend_connected`, `motion_state_known`, and live trajectory data stable during long waits.
- `renderFlow()` now preserves `error` as its own workflow state and paints the active outcome step red with an error label.
- Added regression coverage for installed config paths, active-run freshness, and the UI path resolver.

## Validation
- `python3 -m compileall -q src/paus_perception/paus_perception/config.py src/paus_perception/tests/test_config_paths.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/paus_ui/static/app.js src/paus_ui/paus_ui/static/styles.css src/paus_ui/tests/test_ros_bridge.py src/paus_ui/tests/test_path_resolvers.py src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
- `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/tests/test_path_resolvers.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
- `colcon build --packages-select paus_perception paus_ui paus_marker_ros2 --symlink-install`
- `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`

## Result
- Selected regression tests: 26 passed
- Cross-package regression tests: 49 passed

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: No new reusable lesson was added; this round aligned installed-path calibration outputs and long-running UI state handling.
