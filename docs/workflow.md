# AG 项目工作流说明

## 1. 目标工作流

后续推荐工作流已经明确改为：

```text
纯 Linux / 原生 Ubuntu 主线
```

目标结构是：

```text
Linux camera input
-> ROS2 image receiver
-> marker detection
-> base-frame target transform
-> control decision
-> robot execution
```

当前仓库中的 `Windows / WSL` 相关实现，主要用于：

- 复盘已有工作
- 保留过渡期参考
- 作为临时排查材料

不再是推荐长期架构。

## 2. 纯 Linux 主线中应保留的模块

### 2.1 感知层

- `image_receiver_node`
- `marker_pose_node`

职责：

1. 获取相机图像
2. 检测 `ArUco marker`
3. 输出 marker 在相机系下的位姿

### 2.2 坐标变换层

- `target_transform_node`

职责：

1. 读取 `camera -> robot_base` 外参
2. 读取 `marker -> target_region` 固定偏移
3. 输出机器人基座系下的目标点

### 2.3 控制决策层

- `fairino_control_node`

职责：

1. 订阅 `target_point_base`
2. 做工作空间和安全检查
3. 生成候选接近位姿
4. 为后续 Linux 执行层提供动作请求

### 2.4 执行层

后续目标：

```text
在 Linux / 原生 Ubuntu 中完成机械臂执行接口
```

当前还没有把这部分完全切干净，因此：

- Linux 执行层仍需单独验证
- 当前仓库里的 Windows 执行桥只作为历史过渡路线保留

## 3. 当前 Linux 主线最重要的 topic

### `/camera/image_bridge`

作用：

- 传输输入图像

### `/marker_pose`

作用：

- 发布 marker 相对相机的位姿

### `/target_point_base`

作用：

- 发布机器人基座坐标系下的目标点

### `/control_status`

作用：

- 发布控制层状态
- 发布候选位姿
- 发布执行结果

## 4. 当前 Linux 主线最重要的代码目录

### `src/ag_repro`

后续主线继续保留：

- `config.py`
- `detection.py`
- `pose.py`
- `transforms.py`
- `control.py`
- `pipeline.py`

### `ros2_ws/src/ag_marker_ros2`

后续主线继续保留：

- `image_receiver_node.py`
- `marker_pose_node.py`
- `target_transform_node.py`
- `fairino_control_node.py`

### `configs`

后续主线继续保留：

- `default.yaml`
- `extrinsics.yaml`

### `scripts`

后续主线继续保留：

- `run_single_image.py`
- `run_image_batch.py`
- `generate_marker.py`
- `calibrate_camera.py`

## 5. 当前应该降级为历史路线的内容

这些内容现在不建议继续作为主线依赖：

### 5.1 Windows 相机桥接

- `D:\Projects\auto_arm\camera_bridge_win.py`
- `D:\Projects\auto_arm\run_camera_bridge_win.ps1`

### 5.2 Windows FAIRINO 执行桥

- `D:\Projects\auto_arm\fairino_exec_bridge_win.py`
- `D:\Projects\auto_arm\run_fairino_exec_bridge_win.ps1`

### 5.3 WSL + Windows 混合桥接逻辑

- [legacy/hybrid_stack/src/ag_repro/execution_bridge.py](/root/projects/ag-repro/legacy/hybrid_stack/src/ag_repro/execution_bridge.py)
- [legacy/hybrid_stack/scripts/test_fairino_exec_bridge.py](/root/projects/ag-repro/legacy/hybrid_stack/scripts/test_fairino_exec_bridge.py)
- [legacy/hybrid_stack/docs/fairino_windows_bridge.md](/root/projects/ag-repro/legacy/hybrid_stack/docs/fairino_windows_bridge.md)

这些文件先不删，但它们现在的定位是：

```text
历史方案 / 过渡参考
```

## 6. 当前推荐的迁移顺序

### 第一步：统一路径和环境认知

把后续项目默认认知统一成：

```text
Linux 是主环境
```

不再把：

- Windows 相机桥
- Windows FAIRINO 执行桥

当成长期架构的一部分。

### 第二步：保留 ROS2 感知与控制链

继续推进：

```text
camera
-> marker_pose
-> target_point_base
-> control_status
```

### 第三步：重建 Linux 执行层

单独验证：

- FAIRINO Linux SDK 是否可用
- Linux 下的执行接口怎么接
- 是否需要新的桥接或适配层

## 7. 当前推荐的文档阅读顺序

1. [README.md](/root/projects/ag-repro/README.md)
2. [current_status.md](/root/projects/ag-repro/docs/current_status.md)
3. [coordinate_transform.md](/root/projects/ag-repro/docs/coordinate_transform.md)
4. [linux_execution.md](/root/projects/ag-repro/docs/linux_execution.md)

如果只是回顾历史 Windows 过渡路线，再看：

- [legacy_windows_route.md](/root/projects/ag-repro/docs/legacy_windows_route.md)
