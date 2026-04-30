# Review Round 9 Summary

## Issues Fixed
- `[P1] Resolve runtime artifacts outside read-only install prefixes`
- `[P2] Mark queued /api/handeye/run requests as accepted`
- `[P3] Populate archived sample rotation from the stored transform`

## Resolution
- Changed install-package bare calibration artifact resolution to use a writable runtime root: `PAUS_ROBOT_RUNTIME_DIR/configs`, or `XDG_DATA_HOME/paus_robot/configs`, or `~/.local/share/paus_robot/configs`.
- Kept source-tree `default.yaml` behavior unchanged for bare artifact filenames, so lab worktree runs still write beside the source config.
- Changed accepted background `/api/handeye/run` responses to return `success=true` while retaining `accepted=true` and `queued=true`.
- Added archived sample rotation recovery from `camera_to_board_matrix` using `rotation_matrix_to_rpy_deg`, so historical sample and waypoint views can show board orientation.

## Validation
- `python3 -m compileall -q src/paus_perception/paus_perception src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_motion_ros2/tests/test_control_config_defaults.py src/paus_ui/tests` passed: 19 tests.
- `colcon build --packages-select paus_perception paus_ui --symlink-install` passed.

## Unresolved
- None known.

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `.humanize/bitlesson.md` only contains the template. `bitlesson-select.sh` was attempted but cannot run on the lab host because `jq` is unavailable and the default selector config is malformed.
