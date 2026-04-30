# Review Round 7 Summary

## Issues Fixed
- `[P1] Derive UI safety/config state from the backend, not local params`
- `[P2] Make --trajectory-path affect the node you are driving`

## Resolution
- Added backend calibration configuration fields to every `eye_to_hand_calibration_node` status payload: `execute_motion`, `trajectory_path`, `session_root_path`, `output_path`, and quality thresholds.
- Switched `/eye_to_hand/status` publisher and UI/CLI subscribers to transient-local reliable QoS so a UI or CLI started after the calibration node can still receive the latest backend state.
- Updated `UiRosBridge` to synchronize effective safety and path state from backend status before reporting status, reading sessions/waypoints, building motion summaries, and starting runs.
- Updated the browser flow so dry-run is blocked unless backend status confirms `execute_motion=false`; the start button now prompts when backend motion state is true or unknown.
- Changed CLI `--trajectory-path` semantics from a local-only check to a backend verification option. The CLI now reads backend status, uses the node trajectory path for local checks, and rejects mismatched custom paths with an explicit restart instruction.

## Validation
- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui`
- `node --check src/paus_ui/paus_ui/static/app.js`
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_ui/tests` passed: 15 tests.
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install` passed.

## Unresolved
- None known.

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `.humanize/bitlesson.md` only contains the template. `bitlesson-select.sh` was attempted but cannot run on the lab host because `jq` is unavailable and the default selector config is malformed.
