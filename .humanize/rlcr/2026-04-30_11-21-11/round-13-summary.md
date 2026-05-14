Round 13 summary

Changes made:
- Dry-run semi-auto runs now append one `waypoint_dry_run_complete` event per waypoint to `run.log`, so historical sessions reconstruct with pending count zero.
- `SessionStore` treats dry-run waypoint events as terminal skipped rows with `DRY` result text.
- Successful `run_semi_auto` command results now preserve the backend completion message instead of replacing it with a generic "request sent" text.
- `load_trajectory()` now rejects duplicate waypoint names before a run can collapse ambiguous rows in the session UI.
- Added regression tests for dry-run session reconstruction, successful run command text, and duplicate waypoint validation.

Validation:
- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_ui/tests` -> 21 passed
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `bitlesson-selector` is unavailable on the lab host in this environment, so no existing lesson was selected or updated.
