## Fixed Issues

- Fixed `[P1] Preserve the semi-auto session when capturing samples`.
  - `_ensure_session_started()` now accepts an explicit owner and only creates a new archive when the current archive does not match that owner.
  - Manual capture still defaults to `manual`, so manual sampling after a semi-auto or dry-run archive starts a fresh manual session.
  - Semi-auto capture now calls `_capture_one_sample(session_owner="semi_auto")`, so samples, `samples.jsonl`, `report.yaml`, `trajectory_used.yaml`, and `run.log` remain in the same semi-auto session directory.
  - Added regression coverage for both paths: semi-auto capture keeps its session, while manual capture after semi-auto starts a new one.

## Validation

- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
  - `28 passed`
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`
  - `2 packages finished`

## Unresolved Issues

- None.

## BitLesson Delta

- Action: none
- Lesson ID(s): NONE
- `.humanize/bitlesson.md` was read before coding.
- `bitlesson-selector` was attempted, but the command is unavailable on the lab host (`bitlesson-selector: missing`).
