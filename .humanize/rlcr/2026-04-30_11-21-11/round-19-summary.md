## Fixed Issues

- Fixed `[P2] Treat stale frames as unavailable in quality APIs`.
  - Added a shared camera freshness window and stale-image handling for `get_latest_quality()`.
  - Live image rendering now returns a stale-stream placeholder instead of continuing to draw overlays on old cached frames.
  - Added a regression test that verifies stale cached images return `detected: false` with `reason_code: stale_image`.

- Fixed `[P2] Skip no-report sessions when choosing the latest archive`.
  - `latest_valid_session_id()` now returns the newest solution-backed session first, then the newest report-backed session, and returns `None` when only no-report archives exist.
  - Added regression tests for newer empty archives and all-empty archive lists.

- Fixed `[P3] Preserve archived sample overlays when current calibration changes`.
  - Archived sample previews now use saved sample metadata from `samples.jsonl` / `report.yaml` instead of re-running the current live detector and current `camera.yaml`.
  - Added a metadata overlay renderer for archived sample images.
  - Added a regression test that renders an archived overlay without requiring a live detector.

## Validation

- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`
  - `32 passed`
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`
  - `2 packages finished`

## Unresolved Issues

- None.

## BitLesson Delta

- Action: none
- Lesson ID(s): NONE
- `.humanize/bitlesson.md` was read before coding.
- `bitlesson-selector` was attempted, but the command is unavailable on the lab host (`bitlesson-selector: missing`).
