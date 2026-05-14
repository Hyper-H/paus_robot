## Fixed Issues

- Fixed `[P2] Resolve relative calibration overrides with the runtime-path helpers`.
  - Exposed public `resolve_config_artifact_path()` and `resolve_runtime_data_path()` helpers from `paus_perception`.
  - Updated `eye_to_hand_calibration_node` launch-parameter overrides to use the same install-safe resolution logic as `default.yaml`.
  - `output_path` and `trajectory_path` now use artifact-path resolution; `session_root_path` and `sample_log_path` now use runtime-data resolution.
  - Added regression coverage for runtime data path resolution.

- Fixed `[P2] Raise missing archived sample images instead of returning placeholders`.
  - `get_sample_jpeg()` now raises `FileNotFoundError` for missing rows or missing image files.
  - It raises `ValueError` when OpenCV cannot decode an existing image file.
  - The existing FastAPI route already translates those exceptions into HTTP 404.
  - Added regression coverage for missing archived sample images.

## Validation

- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui src/paus_perception/paus_perception`
- `PYTHONPATH="$PWD/src/paus_marker_ros2" python3 -m unittest discover -s src/paus_marker_ros2/tests`
  - `Ran 8 tests`
- `PYTHONPATH="$PWD/src/paus_perception" python3 -m pytest -q src/paus_perception/tests/test_config_paths.py`
  - `1 passed`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests src/paus_perception/tests/test_config_paths.py src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
  - `35 passed`
- `colcon build --packages-select paus_perception paus_marker_ros2 paus_ui --symlink-install`
  - `3 packages finished`

## Unresolved Issues

- None.

## BitLesson Delta

- Action: none
- Lesson ID(s): NONE
- `.humanize/bitlesson.md` was read before coding.
- `bitlesson-selector` was attempted, but the command is unavailable on the lab host (`bitlesson-selector: missing`).
