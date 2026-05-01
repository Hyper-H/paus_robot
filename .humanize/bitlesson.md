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
