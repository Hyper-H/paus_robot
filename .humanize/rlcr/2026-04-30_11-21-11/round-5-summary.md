# Review Round 5 Summary

## Issues Fixed
- `[P1] Keep install defaults valid in install-only workspaces`
- `[P1] Reject overlapping semi-auto runs inside the ROS service`
- `[P1] Measure TCP stability over the whole window, not step-to-step`
- `[P2] Do not report /api/handeye/run as successful before it starts`

## Resolution
- Changed default calibration artifact paths to `extrinsics.yaml` and `eye_to_hand_trajectory.yaml`, and resolved single-file calibration artifacts relative to the loaded config directory. Multi-part relative paths and `session_root_path` still resolve against the inferred workspace root.
- Added an install-style config regression test to ensure installed `default.yaml` resolves artifacts into the package share config directory instead of a missing source checkout.
- Added a node-level `_semi_auto_lock` and `_semi_auto_active` guard around `/eye_to_hand/run_semi_auto_calibration`, so direct service callers cannot overlap semi-auto runs or interleave robot motion commands.
- Changed TCP stability detection to compare all poses in the stable window against a fixed reference pose for that window, resetting when the total window deviation exceeds tolerance.
- Changed the UI `/api/handeye/run` path to preflight service availability and return an explicit `accepted/queued` state instead of `success=true` for asynchronous requests. When the service is unavailable, it returns a normal failure immediately.

## Validation
- `python3 -m compileall -q src/paus_perception/paus_perception src/paus_marker_ros2/paus_marker_ros2 src/paus_ui/paus_ui src/paus_bringup/launch`
- `node --check src/paus_ui/paus_ui/static/app.js`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests src/paus_motion_ros2/tests/test_control_config_defaults.py` passed: 16 tests.
- `colcon build --packages-select paus_perception paus_motion_ros2 paus_marker_ros2 paus_ui paus_bringup --symlink-install` passed.
- UI-only smoke passed with topic overrides and no calibration node; `POST /api/handeye/run` returned `success=false` plus a service-unavailable operator message instead of false success.

## Unresolved
- None known.

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `.humanize/bitlesson.md` only contains the template. `bitlesson-select.sh` was attempted but cannot run on the lab host because `jq` is unavailable and the default selector config is malformed.
