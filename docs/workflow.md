# Workflow

## 1. Workspace-level package relationship

```text
paus_perception  -> shared perception / transform / camera helpers
paus_marker_ros2 -> coarse localization ROS2 nodes
paus_motion_ros2 -> approach decision + robot execution ROS2 node
paus_fine_ros2   -> future fine localization nodes
paus_interfaces  -> future custom msg/srv/action
paus_bringup     -> launch + runtime configuration
```

## 2. Online data flow

```text
camera_bridge.py
    -> /camera/image_bridge
        -> marker_pose_node
            -> /marker_pose
                -> target_transform_node
                    -> /target_point_base
                        -> fairino_control_node
                            -> /control_status
                            -> FAIRINO Linux SDK
```

## 3. Package responsibilities

### `paus_perception`
- camera calibration load/save
- ArUco detection
- pose estimation
- target-point planning in camera frame
- transform utilities
- camera SDK helpers
- bridge protocol helpers

### `paus_marker_ros2`
- `image_receiver_node`
- `marker_pose_node`
- `target_transform_node`
- `eye_to_hand_calibration_node`
- non-node `camera_bridge.py` runtime script

### `paus_motion_ros2`
- `fairino_control_node`
- approach decision logic
- Linux FAIRINO SDK adapter

### `paus_bringup`
- `online_stack.launch.py`
- packaged runtime configs (`default.yaml`, `extrinsics.yaml`)

## 4. Runtime modes

### Dry-run (default)
The full chain runs online, but `fairino_control_node` only publishes candidate poses and status. No robot motion command is sent.

### Real-run
Real motion is enabled only when launch or node parameters explicitly set `execute_motion=true`.

## 5. Manual startup order

1. `ros2 run paus_marker_ros2 image_receiver_node`
2. `camera_bridge.py`
3. `ros2 run paus_marker_ros2 marker_pose_node`
4. `ros2 run paus_marker_ros2 target_transform_node`
5. `ros2 run paus_motion_ros2 fairino_control_node`

## 6. Legacy route

The old Windows / WSL bridge path is preserved under `legacy/` for traceability only. It is no longer part of the default build or launch workflow.
