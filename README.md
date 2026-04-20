<<<<<<< HEAD
# paus_robot
=======
# AG Marker Localization

[中文版](#中文版) | [English](#english)

## 中文版

### 项目简介

这个项目当前用于搭建一条从相机图像到机械臂接近目标点的在线感知链路。  
接下来的主线方向已经确定为：

```text
纯 Linux / 原生 Ubuntu 主线
```

也就是说，后续开发和运行目标是：

- 在 Linux 上完成相机输入
- 在 Linux 上运行 `ROS2 Humble`
- 在 Linux 上完成 marker 检测、坐标变换、接近控制与机械臂执行

当前仓库里仍然保留了一部分 `Windows / WSL` 相关代码和文档，它们现在属于：

```text
历史路线 / 过渡方案 / 参考实现
```

不再是推荐长期主线。

### 当前核心目标

当前项目的核心目标是：

1. 检测单个 `ArUco marker`
2. 估计 marker 相对相机的位姿
3. 将 marker 转换为机器人基座坐标系下的目标点
4. 生成安全的候选接近位姿
5. 在 Linux 主线下完成机械臂执行接口整合

### 当前保留的 Linux 主线内容

以下内容是后续纯 Linux 主线应继续保留和完善的：

- `src/ag_repro`
  - 配置
  - 检测
  - 位姿估计
  - 坐标变换
  - 控制逻辑
- `ros2_ws`
  - `image_receiver_node`
  - `marker_pose_node`
  - `target_transform_node`
  - `fairino_control_node`
- `configs`
- `scripts`
- `docs`
- `tests`

### 当前 Windows / WSL 相关内容的定位

当前仓库里仍然存在：

- Windows 相机桥接：
  - `D:\Projects\auto_arm\camera_bridge_win.py`
  - `D:\Projects\auto_arm\run_camera_bridge_win.ps1`
- Windows FAIRINO 执行桥：
  - `D:\Projects\auto_arm\fairino_exec_bridge_win.py`
  - `D:\Projects\auto_arm\run_fairino_exec_bridge_win.ps1`
- WSL 相关启动与日志路径

这些内容目前不删除，但它们的角色改为：

- 用于回顾和参考
- 必要时作为临时过渡方案
- 不再作为推荐长期运行架构

### 当前建议的迁移原则

#### 1. 先统一开发环境

把后续开发重心统一到：

```text
Linux / 原生 Ubuntu
```

优先减少：

- Windows 路径
- WSL 与 Windows 双向 TCP 桥
- 多环境并存带来的调试复杂度

#### 2. 先保留感知和控制逻辑

优先保留并继续演进：

- marker 检测
- 位姿估计
- `target_point_base`
- `candidate_pose_mmdeg`
- ROS2 topic 链

#### 3. 再替换执行层

后续需要单独验证：

- FAIRINO 是否能在原生 Linux 下稳定执行
- 如果不能，需要怎样做 Linux 侧替代或接口适配

### 当前常用命令

#### Linux 基础环境

```bash
source /opt/ros/humble/setup.bash
source /root/projects/ag-repro/ros2_ws/install/setup.bash
```

#### 单图检测（离线）

```bash
source /root/miniconda3/etc/profile.d/conda.sh
conda activate ag-repro
cd /root/projects/ag-repro

python scripts/run_single_image.py \
  --input /path/to/image.png \
  --output-dir results/run_single_001 \
  --config configs/default.yaml \
  --camera-config configs/camera.yaml
```

#### 批处理（离线）

```bash
source /root/miniconda3/etc/profile.d/conda.sh
conda activate ag-repro
cd /root/projects/ag-repro

python scripts/run_image_batch.py \
  --input-dir /path/to/image_dir \
  --output-dir results/batch_001 \
  --config configs/default.yaml \
  --camera-config configs/camera.yaml
```

#### 棋盘格标定

```bash
source /root/miniconda3/etc/profile.d/conda.sh
conda activate ag-repro
cd /root/projects/ag-repro

python scripts/calibrate_camera.py \
  --input-dir /path/to/calib_images \
  --rows 6 \
  --cols 9 \
  --square-size-m 0.01 \
  --output configs/camera.yaml
```

### 当前文档说明

当前文档目录已经分成两类：

#### Linux 主线相关

- [workflow.md](/root/projects/ag-repro/docs/workflow.md)
- [coordinate_transform.md](/root/projects/ag-repro/docs/coordinate_transform.md)
- [current_status.md](/root/projects/ag-repro/docs/current_status.md)
- [linux_execution.md](/root/projects/ag-repro/docs/linux_execution.md)

#### 历史 / Windows 过渡方案

- [legacy_windows_route.md](/root/projects/ag-repro/docs/legacy_windows_route.md)

### 当前推荐

如果你准备继续做项目主线，推荐顺序是：

1. 先把文档和架构认知切到 Linux 主线
2. 继续保留并完善感知与几何链
3. 再单独验证 Linux 下的机械臂执行层

## English

The project is now being reorganized toward a **pure Linux / native Ubuntu mainline**.

Windows / WSL bridge components are still kept in the repository, but they are now considered:

- historical paths
- transitional tools
- reference implementations

The long-term recommended direction is:

- Linux for camera input
- Linux for ROS2
- Linux for marker detection, transforms, control, and robot execution
>>>>>>> 64a9a07 (Initial import of paus_robot project)
