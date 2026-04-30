## Fixed Issues

- Fixed `[P2] Start a new archive before manual capture after semi-auto runs`.
  - Added a `session_owner` marker to distinguish `semi_auto` and `manual` session ownership.
  - `_begin_new_semi_auto_session()` now marks the active archive as `semi_auto`.
  - `_ensure_session_started()` now starts a fresh manual archive whenever the current archive is absent or not owned by the manual flow, preventing manual captures from appending to a previous dry-run or semi-auto archive.
  - `_clear_manual_session_after_save()` clears the ownership marker with the rest of the manual session state.

- Fixed `[P2] Avoid reporting queued run requests as successful`.
  - `start_semi_auto_run()` now reports the immediate queued response as `success: false` while preserving `accepted: true` and `queued: true`.
  - The UI can still show that the request was accepted for processing, but no longer presents the run as successful before the ROS backend replies.
  - Added a regression test covering the queued path.

## Validation

- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py`
  - `26 passed`
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`
  - `2 packages finished`

## Unresolved Issues

- None.

## BitLesson Delta

- Action: none
- Lesson ID(s): NONE
- `.humanize/bitlesson.md` was read before coding.
- `bitlesson-selector` was attempted, but the command is unavailable on the lab host (`bitlesson-selector: missing`).
