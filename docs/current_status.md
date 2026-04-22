# Current Status

## Migration summary
The repository has been migrated from the old mixed layout (`ag_repro + ag_marker_ros2 + ros2_ws`) to a workspace-root ROS2 structure.

Current mainline packages:
- `src/paus_perception`
- `src/paus_marker_ros2`
- `src/paus_motion_ros2`
- `src/paus_fine_ros2`
- `src/paus_interfaces`
- `src/paus_bringup`

## Completed
- `ag_repro` renamed and split into:
  - `paus_perception`
  - `paus_motion_ros2`
- `ag_marker_ros2` split into:
  - `paus_marker_ros2`
  - `paus_motion_ros2`
  - `paus_bringup`
- Linux launch entry changed to:
  - `ros2 launch paus_bringup online_stack.launch.py`
- CLI scripts moved into `paus_perception` console scripts
- old nested `ros2_ws` removed from the mainline
- legacy vendor / hybrid route isolated under `legacy/`

## Still intentionally empty / placeholder
- `paus_fine_ros2`
- `paus_interfaces`

These package slots are created now to keep future growth stable.

## Next recommended steps
1. Define formal custom messages instead of JSON strings in `std_msgs/String`.
2. Move fine-localization experiments into `paus_fine_ros2`.
3. Add package-level unit tests for `paus_marker_ros2` and `paus_motion_ros2` node utilities.
4. Add integration tests under `tests/integration/`.
