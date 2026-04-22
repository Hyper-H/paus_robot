# 坐标变换链说明

## 目标
这一模块的目标是把：

- `camera -> marker`

变成：

- `robot_base -> target_region`

也就是把相机系下的粗定位结果转换成机器人基座系下的目标区域。

## 主要坐标系
- `robot_base`：机械臂基座坐标系
- `tool`：当前工具 / TCP 坐标系
- `calib_board`：安装在工具上的标定板坐标系
- `camera`：相机坐标系
- `marker`：患者附近工作 marker 坐标系
- `target_region`：希望机器人靠近的粗目标区域

## 标定阶段
标定阶段的目标是求：

- `T_base_camera`

流程：
1. 将棋盘格或标定板刚性固定在工具上
2. 在相机图像中检测标定板，估计 `T_camera_board`
3. 读取机械臂当前 TCP 位姿 `T_base_tool`
4. 使用固定的 `T_tool_board`
5. 计算单次样本：

```text
T_base_camera_i = T_base_tool_i * T_tool_board * inverse(T_camera_board_i)
```

6. 对多组样本求平均，得到最终 `T_base_camera`

输出文件：
- `src/paus_bringup/configs/extrinsics.yaml`

## 运行阶段
运行阶段移除标定板，只保留工作 marker。

流程：
1. 检测工作 marker
2. 估计 `T_camera_marker`
3. 加载 `T_base_camera`
4. 定义固定的 `T_marker_target`
5. 计算：

```text
T_base_target = T_base_camera * T_camera_marker * T_marker_target
```

6. 发布：
- `/target_pose_base`
- `/target_point_base`

## 默认偏移
第一版默认从工作 marker 到粗目标区域使用固定偏移：

```yaml
transform:
  marker_to_target:
    translation_m: [0.0, -0.05, 0.0]
    rotation_rpy_deg: [0.0, 0.0, 0.0]
```

含义是：
- 工作 marker 贴在下巴附近
- 粗目标点定义在 marker 坐标系下方 5 cm 处

## ROS2 节点
### `eye_to_hand_calibration_node`
负责：
- 采集标定样本
- 求解 `T_base_camera`
- 保存 `extrinsics.yaml`

### `target_transform_node`
负责：
- 将 `camera -> marker` 转换成 `robot_base -> target_region`

输入：
- `/marker_pose`
- `extrinsics.yaml`
- `default.yaml`

输出：
- `/target_pose_base`
- `/target_point_base`
- `/transform_status`

## 当前范围
当前模块还不处理：
- 遮挡鲁棒性
- 点云精修
- 手眼算法创新
- 实际机械臂执行

当前目标只是：
**把相机系粗定位闭环到机器人基座系粗定位。**

## 第一阶段机器人接近控制
第一版执行策略刻意做得比较保守。

### 输入
- `/target_point_base`

### 控制策略
`fairino_control_node` 当前会：
1. 从 FAIRINO Linux SDK 读取当前 TCP 位姿
2. 保持当前 TCP 姿态不变
3. 将目标点从米转换为毫米
4. 生成预接近点，而不是直接贴向目标
5. 执行安全检查
6. 仅在 `execute_motion=true` 时真正调用 `MoveL`

### 安全检查
当前检查项包括：
- 工作空间范围
- 最低安全 `z`
- 单步最大位移
- 输入数值有限性
- `frame_id` 是否正确

### Dry-run 优先
默认配置：

```yaml
control:
  execute_motion: false
```

这意味着：
- 节点会输出并发布候选指令
- 机器人不会真实动作

只有在数值和逻辑确认无误后，才建议打开真实执行。