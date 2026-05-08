## Round 43 Summary

Fixed the two review findings from round 43:

- The waypoint preview now clears itself when the waypoint list becomes empty, so stale images and metrics no longer stick around after switching sessions or trajectory sources.
- Session totals now treat `waypoint_deleted` as removal from the active set, so run.log-only archives no longer overcount phantom pending waypoints.

## Validation

- `python3 -m compileall -q src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`
- `node --check src/paus_ui/paus_ui/static/app.js`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_path_resolvers.py src/paus_ui/tests/test_ros_bridge.py`
  - `31 passed`
- `colcon build --packages-select paus_ui --symlink-install`
- `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
  - `52 passed`

## BitLesson Delta

- Action: add
- Lesson ID(s): BL-20260501-handeye-empty-preview-active-count
- Notes: Captured the stale-preview and deletion-aware counting pattern so UI state and archive totals stay aligned with the active waypoint set.
