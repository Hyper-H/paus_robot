Round 16 summary

Changes made:
- Calibration status now publishes `camera_config_path`, `board_rows`, `board_cols`, and `square_size_m` so detached UI instances can align overlay/PnP settings with the backend calibration node.
- UI backend sync now imports backend detector settings, invalidates the cached overlay detector when those settings change, and refreshes backend state before live quality/image scoring.
- Detector cache keys now include camera path, camera YAML mtime, board rows/cols, and square size.
- Archived samples now treat conventional fallback images at `session/images/sample_###.png` as present and expose that path for thumbnails/previews.
- Added regression tests for backend detector setting sync and fallback archived sample images.

Validation:
- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py` -> 25 passed
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `bitlesson-selector` is unavailable on the lab host in this environment, so no existing lesson was selected or updated.
