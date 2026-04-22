# Legacy Windows Route

The repository still keeps the old Windows / WSL hybrid path under `legacy/`:

- `legacy/hybrid_stack/`
- `legacy/windows_bridge/`
- `legacy/vendor/`

This route is preserved only for reference, rollback, or historical comparison. It is not part of the default Linux mainline anymore.

## Archived content
- Windows camera bridge implementation
- Windows FAIRINO execution bridge
- old ROS2 vendor packages formerly used inside the nested workspace

## Current recommendation
Use the Linux-native workspace root mainline instead:
- `paus_perception`
- `paus_marker_ros2`
- `paus_motion_ros2`
- `paus_bringup`
