## Fixed Issues

- Fixed `[P2] Keep live waypoint teaching reachable after sessions exist`.
  - The session selector now always includes a `Current trajectory` option.
  - `refreshSessions()` no longer auto-selects an archived session just because archives exist.
  - Live waypoint teaching remains visible by default, while users can still explicitly choose historical sessions.

- Fixed `[P2] Route installed nested artifact paths into the runtime dir`.
  - Installed-package relative artifact paths now resolve under `PAUS_ROBOT_RUNTIME_DIR` for both single-component and nested relative paths.
  - Single-component artifacts keep the existing `runtime/configs/<file>` behavior.
  - Nested artifacts preserve their relative directory under the runtime root.
  - Added regression coverage for `extrinsics.yaml`, `configs/eye_to_hand.yaml`, and `calibration/extrinsics.yaml`.

- Fixed `[P3] Map the dry-run waypoint status the backend actually emits`.
  - Added `waypoint_dry_run_complete` to the UI workflow status mapping.
  - Added a regression test for the backend dry-run waypoint status.

## Validation

- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui src/paus_perception/paus_perception`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests src/paus_perception/tests/test_config_paths.py src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
  - `34 passed`
- `colcon build --packages-select paus_perception paus_marker_ros2 paus_ui --symlink-install`
  - `3 packages finished`

## Unresolved Issues

- None.

## BitLesson Delta

- Action: none
- Lesson ID(s): NONE
- `.humanize/bitlesson.md` was read before coding.
- `bitlesson-selector` was attempted, but the command is unavailable on the lab host (`bitlesson-selector: missing`).
