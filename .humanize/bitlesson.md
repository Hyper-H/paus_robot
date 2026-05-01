# BitLesson Knowledge Base

This file is project-specific. Keep entries precise and reusable for future rounds.

## Entry Template (Strict)

Use this exact field order for every entry:

```markdown
## Lesson: <unique-id>
Lesson ID: <BL-YYYYMMDD-short-name>
Scope: <component/subsystem/files>
Problem Description: <specific failure mode with trigger conditions>
Root Cause: <direct technical cause>
Solution: <exact fix that resolved the problem>
Constraints: <limits, assumptions, non-goals>
Validation Evidence: <tests/commands/logs/PR evidence>
Source Rounds: <round numbers where problem appeared and was solved>
```

## Entries

## Lesson: handeye-ui-archived-default-board-angle
Lesson ID: BL-20260501-handeye-ui-archived-default-board-angle
Scope: src/paus_ui/paus_ui/static/app.js; src/paus_ui/paus_ui/session_store.py; src/paus_ui/tests/test_session_store.py
Problem Description: On a page load with only archived handeye sessions available and no live run, the UI stayed on "current trajectory" until the operator manually chose a session. Sample preview details also omitted board-angle, so archived samples lost a useful visual quality cue.
Root Cause: Session selection only auto-followed live runs, and archived preview rendering depended on a field that was never propagated from stored sample data.
Solution: Auto-select the latest archived report session when there is no live run and no user-selected session, propagate `board_angle_deg` from sample storage or compute it from `camera_to_board_matrix`, and render it in the saved-sample preview panel.
Constraints: Do not override a user-selected session; only default-select archived reports when the session selector is otherwise empty. Keep the change compatible with existing live-run auto-follow behavior.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `node --check src/paus_ui/paus_ui/static/app.js`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_path_resolvers.py src/paus_ui/tests/test_ros_bridge.py` (`29 passed`); `colcon build --packages-select paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`49 passed`)
Source Rounds: 41

## Lesson: handeye-archive-integrity
Lesson ID: BL-20260501-handeye-archive-integrity
Scope: src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py; src/paus_ui/paus_ui/session_store.py; src/paus_marker_ros2/tests/test_eye_to_hand_session.py; src/paus_ui/tests/test_session_store.py
Problem Description: Semi-auto validation failures created empty archive sessions, and deleted recorded waypoints could still leak into the archived session view as pending entries.
Root Cause: The semi-auto flow opened a new session before trajectory validation completed, and delete events were written with an unnormalized payload that SessionStore could not recognize as a deletion marker.
Solution: Validate the trajectory before starting a semi-auto session, suppress run-log writes when the run never starts, emit `waypoint_deleted` with `waypoint_name` and `waypoint` fields, and teach SessionStore to drop deleted waypoints from archived reconstruction.
Constraints: Do not create archive directories for missing or malformed trajectories. Keep the remaining archived waypoints stable and preserve the existing capture/skip rendering for non-deleted items.
Validation Evidence: `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_session_store.py` (`17 passed`); `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`51 passed`)
Source Rounds: 42

## Lesson: handeye-empty-preview-active-count
Lesson ID: BL-20260501-handeye-empty-preview-active-count
Scope: src/paus_ui/paus_ui/static/app.js; src/paus_ui/paus_ui/session_store.py; src/paus_ui/tests/test_session_store.py
Problem Description: Switching to an empty waypoint list left the previous sample preview visible, and run.log-only sessions could still overcount pending waypoints after deletions.
Root Cause: The preview renderer was never cleared when `state.waypoints` became empty, and session totals counted every waypoint name in the log without treating `waypoint_deleted` as a removal.
Solution: Clear the preview panel when no waypoints are available, and count only active waypoint names by discarding deletions from the event-derived set.
Constraints: Keep the empty-state message explicit instead of showing stale sample details. Do not change accepted/skipped rendering for non-empty sessions.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `node --check src/paus_ui/paus_ui/static/app.js`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_path_resolvers.py src/paus_ui/tests/test_ros_bridge.py` (`31 passed`); `colcon build --packages-select paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`52 passed`)
Source Rounds: 43

## Lesson: handeye-manual-edit-archive
Lesson ID: BL-20260501-handeye-manual-edit-archive
Scope: src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py; src/paus_marker_ros2/tests/test_eye_to_hand_session.py
Problem Description: Manual waypoint record/delete edits made before any capture or semi-auto run were silently dropped because no archive session had been started yet.
Root Cause: The manual edit callbacks only wrote to `run.log` when `run_log_path` already existed, so the first edit in a fresh session had nowhere to persist.
Solution: Start a manual session before logging successful waypoint edits, so record-only or edit-only sessions are archived immediately and can be reconstructed after restart.
Constraints: Keep the semi-auto rejection guard intact. Only start the manual archive session after the edit succeeds, not before validation or robot reads.
Validation Evidence: `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/tests/test_session_store.py` (`38 passed`); `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`54 passed`)
Source Rounds: 44

## Lesson: handeye-ui-service-timeout
Lesson ID: BL-20260501-handeye-ui-service-timeout
Scope: src/paus_ui/paus_ui/ros_bridge.py; src/paus_ui/tests/test_ros_bridge.py
Problem Description: UI command calls could report service unavailability during normal startup or restart races even when the caller asked to wait longer.
Root Cause: The ROS service wait path truncated every timeout to two seconds before calling `wait_for_service()`.
Solution: Pass the caller's full timeout through to the ROS service wait so long startup windows are honored.
Constraints: Preserve the existing command result and event logging behavior. Do not change the user-facing timeout semantics beyond honoring the requested wait.
Validation Evidence: `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/tests/test_session_store.py` (`38 passed`); `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`54 passed`)
Source Rounds: 44
