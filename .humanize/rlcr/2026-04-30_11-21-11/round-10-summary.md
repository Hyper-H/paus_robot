Round 10 summary

Changes made:
- Deferred calibration session creation until the first real capture or semi-auto run. Node startup now publishes `ready` without creating a reportless session directory.
- Added session guards around sample log, report writing, image saving, and extrinsics save paths so manual capture/solve/save fail clearly if the session state is not initialized.
- Checked `cv2.imwrite()` and raise a runtime error before accepting/logging a sample image when the write fails.
- Added a terminal `waypoint_capture_disabled` event for `capture:false` waypoints so UI/session parsing does not leave transit waypoints in `running`/`pending`.
- Updated `SessionStore` to count `waypoint_capture_disabled` as a terminal skipped waypoint and display it as `SKIP`.
- Added a regression test covering `capture:false` waypoint session parsing and pending count reconciliation.

Validation:
- `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_ui/tests` -> 16 passed
- `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: Selector still cannot run on the lab host because `jq` is missing and the installed humanize config JSON is malformed.
