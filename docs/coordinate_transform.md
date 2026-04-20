# Coordinate Transform Pipeline

## Goal

This module turns:

- `camera -> marker`

into:

- `robot_base -> target_region`

through a complete transform chain.

## Frames

- `robot_base`: robot base frame
- `tool`: robot TCP frame
- `calib_board`: calibration board frame mounted on the robot tool
- `camera`: camera frame
- `marker`: working marker frame attached near the patient
- `target_region`: the coarse target region for the robot to approach

## Calibration Stage

The calibration stage only solves:

- `T_base_camera`

Workflow:

1. Mount a chessboard / calibration board rigidly on the robot tool
2. Detect the board in the camera image and estimate `T_camera_board`
3. Read the current robot TCP pose as `T_base_tool`
4. Use a fixed `T_tool_board`
5. Compute one sample:

```text
T_base_camera_i = T_base_tool_i * T_tool_board * inverse(T_camera_board_i)
```

6. Average multiple samples to obtain the final `T_base_camera`

Output:

- `configs/extrinsics.yaml`

## Runtime Stage

During runtime the calibration board is removed and only the working marker remains.

Workflow:

1. Detect the working marker
2. Estimate `T_camera_marker`
3. Load `T_base_camera`
4. Define a fixed `T_marker_target`
5. Compute:

```text
T_base_target = T_base_camera * T_camera_marker * T_marker_target
```

6. Publish:
   - `/target_pose_base`
   - `/target_point_base`

## Default Offset

The first version uses a fixed offset from the working marker to the coarse target region:

```yaml
transform:
  marker_to_target:
    translation_m: [0.0, -0.05, 0.0]
    rotation_rpy_deg: [0.0, 0.0, 0.0]
```

This means:

- the working marker is attached near the chin
- the coarse target is defined 5 cm below the marker in marker coordinates

## ROS2 Nodes

### `eye_to_hand_calibration_node`

Responsibilities:

- collect calibration samples
- solve `T_base_camera`
- save `configs/extrinsics.yaml`

Inputs:

- `/camera/image_bridge`
- `/nonrt_state_data` (currently assumed to be `PoseStamped`)

Services:

- `/eye_to_hand/capture_sample`
- `/eye_to_hand/solve`
- `/eye_to_hand/save`

### `target_transform_node`

Responsibilities:

- convert `camera -> marker` into `robot_base -> target_region`

Inputs:

- `/marker_pose`
- `configs/extrinsics.yaml`
- `configs/default.yaml`

Outputs:

- `/target_pose_base`
- `/target_point_base`
- `/transform_status`

## Current Scope

This module does not yet handle:

- occlusion robustness
- point-cloud refinement
- hand-eye algorithm innovation
- actual robot motion execution

The current goal is only:

**to close the transform chain from camera-frame coarse localization to robot-base coarse localization.**

## 8. First-stage robot approach control

The current first-stage execution strategy is intentionally conservative.

### Input

- `/target_point_base`

This topic already represents the coarse target point in the `robot_base` frame.

### Control strategy

The first version of `fairino_control_node` will:

1. Read the current TCP pose from the FAIRINO Python SDK
2. Keep the current TCP orientation unchanged
3. Convert the target point from `m` to `mm`
4. Generate a pre-approach point instead of directly touching the target
5. Apply safety checks
6. Only call `MoveL` when `execute_motion=true`

### Safety checks

The node checks:

- workspace limits
- minimum safe `z`
- maximum one-step motion distance
- finite input values
- correct `frame_id`

### Dry-run first

The default mode is:

```yaml
control:
  execute_motion: false
```

This means:

- the node prints and publishes the candidate command
- the robot does not move

Only after validating the command numerically should execution be enabled.
