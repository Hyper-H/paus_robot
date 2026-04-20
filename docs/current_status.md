# 当前状态

## 1. 当前方向

项目方向已经调整为：

```text
纯 Linux / 原生 Ubuntu 主线
```

当前仓库里的 `Windows / WSL` 双桥结构不再被视为长期方案。

## 2. 当前应继续保留的内容

### 2.1 Linux 侧代码骨架

以下内容是后续主线继续保留的核心：

- `src/ag_repro`
- `ros2_ws`
- `configs`
- `scripts`
- `tests`
- `docs`

### 2.2 感知与控制逻辑

以下逻辑仍然是项目主线的一部分：

- marker 检测
- 位姿估计
- `target_point_base`
- 候选接近位姿生成
- 控制状态输出

## 3. 当前应降级为历史/过渡方案的内容

### 3.1 Windows 相机桥

- `camera_bridge_win.py`
- `run_camera_bridge_win.ps1`

### 3.2 Windows FAIRINO 执行桥

- `fairino_exec_bridge_win.py`
- `run_fairino_exec_bridge_win.ps1`
- `legacy/hybrid_stack/src/ag_repro/execution_bridge.py`
- `legacy/hybrid_stack/scripts/test_fairino_exec_bridge.py`

这些内容当前保留，但定位改为：

```text
历史路线 / 过渡实现 / 参考资料
```

## 4. 已经验证过的事情

以下事情已经在过渡方案中得到验证：

- Windows 相机桥可连接工业相机
- WSL 中 ROS2 感知链可运行
- Windows FAIRINO Python SDK 可成功执行：
  - `SetSpeed`
  - `MoveJ`

这部分说明：

- 感知逻辑本身可行
- 机械臂控制链曾被验证过

但这些验证属于：

```text
历史混合架构下的验证
```

不代表这就是后续推荐主线。

## 5. 当前主要问题

### 5.1 环境复杂度过高

此前路线同时依赖：

- Windows
- WSL
- ROS2
- Windows 相机桥
- Windows FAIRINO 执行桥

带来了：

- 路径复杂
- 日志分散
- 进程容易重复启动
- 排查断点困难

### 5.2 图像输入链不稳定

此前在混合架构中观察到：

- `ConnectionRefusedError`
- `ConnectionAbortedError`
- `image_receiver_node` 未稳定常驻

导致：

- `/camera/image_bridge` 不稳定
- 后续 `/marker_pose`、`/target_point_base`、`/control_status` 无法持续输出

### 5.3 原始 `ros2_cmd_server` 路线不稳定

此前在 WSL 中测试时出现：

- `GetActualTCPPose(0)` -> `-2`
- `SetSpeed(5)` -> `-2`
- `MoveL(CART...)` -> `-1`

说明：

```text
WSL + ros2_cmd_server
不适合作为长期真机执行主线
```

## 6. 当前建议

### 6.1 短期

短期内你应该做的是：

1. 把文档和认知统一到 Linux 主线
2. 保留并继续整理 ROS2 感知与控制代码
3. 避免继续在混合架构上投入太多工程精力

### 6.2 中期

中期目标是：

1. 在原生 Linux 上跑通相机输入
2. 在 Linux 上重新验证：
   - `marker_pose`
   - `target_transform`
   - `fairino_control`
3. 再决定 Linux 下的机械臂执行接口怎么接

## 7. 当前推荐阅读

- [README.md](/root/projects/ag-repro/README.md)
- [workflow.md](/root/projects/ag-repro/docs/workflow.md)
- [coordinate_transform.md](/root/projects/ag-repro/docs/coordinate_transform.md)
- [linux_execution.md](/root/projects/ag-repro/docs/linux_execution.md)

Windows 过渡方案仅作参考：

- [legacy_windows_route.md](/root/projects/ag-repro/docs/legacy_windows_route.md)
