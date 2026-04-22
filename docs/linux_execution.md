# Linux Execution

## Purpose
This note describes the Linux-native FAIRINO execution path used by the current mainline.

## Main components
- `src/paus_motion_ros2/paus_motion_ros2/fairino_linux_client.py`
- `src/paus_motion_ros2/paus_motion_ros2/fairino_control_node.py`
- `scripts/test_fairino_linux.py`

## Minimal direct checks
From the workspace root:

```bash
python3 scripts/test_fairino_linux.py --command connect
python3 scripts/test_fairino_linux.py --command set_speed --speed 5
python3 scripts/test_fairino_linux.py --command get_joints
python3 scripts/test_fairino_linux.py --command get_tcp
```

## Motion smoke checks
Move to current joint pose:

```bash
python3 scripts/test_fairino_linux.py --command move_j --joint-pos J1 J2 J3 J4 J5 J6 --tool-id 0 --user-id 0 --vel 5
```

Move to current TCP pose:

```bash
python3 scripts/test_fairino_linux.py --command move_l --pose-mmdeg X Y Z RX RY RZ --tool-id 0 --user-id 0 --vel 5
```

## ROS2 path
The ROS2 online chain uses `fairino_control_node` as the execution gateway:

```text
/target_point_base -> fairino_control_node -> linux_sdk -> robot
```

Default mode stays dry-run unless `execute_motion=true` is explicitly provided.
