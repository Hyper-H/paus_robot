# Review Round 8 Summary

## Issues Fixed
- `[P1] Keep generated trajectory/extrinsics out of installed share dir`
- `[P2] Don't persist empty trajectories that the loader rejects`
- `[P2] Apply the recorded acceleration during MoveJ execution`

## Resolution
- Updated calibration artifact path resolution so bare `output_path` and `trajectory_path` still resolve beside source `default.yaml`, but installed `share/paus_bringup/configs/default.yaml` resolves runtime artifacts under the inferred workspace root `configs/` directory instead of the read-only install share directory.
- Changed `save_trajectory()` to reject empty waypoint lists before writing, and changed delete-last-waypoint behavior to remove the trajectory YAML when the final waypoint is deleted.
- Extended `FairinoLinuxClient.move_j()` and `move_j_pose()` to accept optional acceleration and pass `acc` through to the FAIRINO SDK, then wired semi-auto execution to use each recorded waypoint's `acc`.
- Added regression coverage for install artifact path resolution, empty trajectory save rejection, and MoveJ acceleration forwarding.

## Validation
- `python3 -m compileall -q src/paus_perception/paus_perception src/paus_motion_ros2/paus_motion_ros2 src/paus_marker_ros2/paus_marker_ros2`
- `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_motion_ros2/tests/test_control_config_defaults.py` passed: 14 tests.
- `colcon build --packages-select paus_perception paus_motion_ros2 paus_marker_ros2 --symlink-install` passed.

## Unresolved
- None known.

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `.humanize/bitlesson.md` only contains the template. `bitlesson-select.sh` was attempted but cannot run on the lab host because `jq` is unavailable and the default selector config is malformed.
