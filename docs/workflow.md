# 工作流说明

## 1. Workspace 级包关系

```text
paus_perception  -> 感知 / 标定 / 坐标变换核心库
paus_marker_ros2 -> 粗定位视觉节点
paus_motion_ros2 -> 接近决策与机械臂执行节点
paus_fine_ros2   -> 未来细定位节点
paus_interfaces  -> 未来自定义接口
paus_bringup     -> launch 与运行配置
```

## 2. 在线数据流

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

## 3. 各包职责

### `paus_perception`
负责：
- ArUco 检测
- 相机标定读写
- marker 位姿估计
- 相机系目标点规划
- 坐标变换工具
- 相机 SDK 工具
- TCP 图像桥接协议工具

### `paus_marker_ros2`
负责：
- `image_receiver_node`
- `marker_pose_node`
- `target_transform_node`
- `eye_to_hand_calibration_node`
- `camera_bridge.py`

### `paus_motion_ros2`
负责：
- `fairino_control_node`
- 接近决策逻辑
- Linux FAIRINO SDK 适配层

### `paus_bringup`
负责：
- `online_stack.launch.py`
- 默认配置 `default.yaml`
- 默认外参 `extrinsics.yaml`

## 4. 运行模式

### Dry-run（默认）
整条链真实运行，但 `fairino_control_node` 只输出候选位姿和状态，不下发真实运动命令。

### Real-run
只有显式设置 `execute_motion=true` 时，控制节点才允许调用 Linux SDK 执行真实动作。

## 5. 最基本手动启动顺序
1. `ros2 run paus_marker_ros2 image_receiver_node`
2. `camera_bridge.py`
3. `ros2 run paus_marker_ros2 marker_pose_node`
4. `ros2 run paus_marker_ros2 target_transform_node`
5. `ros2 run paus_motion_ros2 fairino_control_node`

## 6. Legacy 路线
旧的 Windows / WSL 双桥方案仍保留在 `legacy/` 下，仅供回溯参考，不再参与主线构建和默认启动。