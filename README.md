# paus_robot

[中文](#中文) | [English](#english)

## 中文

### 项目概览

`paus_robot` 当前以 **纯 Linux / 原生 Ubuntu** 为主线，目标是在 Linux 上完成：

- 相机输入
- ROS2 感知链
- ArUco marker 检测
- 相机到机器人基座的坐标变换
- 候选接近位姿生成
- FAIRINO 真机执行

仓库中的 Windows / WSL 双桥方案仍然保留，但已经降级为历史与参考路线，不再是默认运行方式。

### 当前主线

推荐工作流：

```text
Linux camera input
-> image_receiver_node
-> marker_pose_node
-> target_transform_node
-> fairino_control_node
-> fairino_linux_client
-> Robot
```

### 核心目录

- [configs](/root/projects/ag-repro/configs)
  - 主配置与外参
- [src/ag_repro](/root/projects/ag-repro/src/ag_repro)
  - 检测、位姿、坐标变换、控制、Linux FAIRINO 适配层
- [ros2_ws](/root/projects/ag-repro/ros2_ws)
  - ROS2 节点工作区
- [scripts](/root/projects/ag-repro/scripts)
  - 离线脚本与 Linux 执行测试脚本
- [docs](/root/projects/ag-repro/docs)
  - 工作流、执行说明、状态说明
- [legacy](/root/projects/ag-repro/legacy)
  - 历史 Windows / WSL 路线

### Linux 执行层

默认使用官方 FAIRINO Linux Python SDK。

适配层：

- [fairino_linux_client.py](/root/projects/ag-repro/src/ag_repro/fairino_linux_client.py)

测试脚本：

- [test_fairino_linux.py](/root/projects/ag-repro/scripts/test_fairino_linux.py)

默认配置项：

```yaml
control:
  robot_ip: "192.168.58.2"
  linux_fairino_sdk_root: "/opt/fairino_python_sdk/linux"
  execute_motion: false
```

### 常用命令

激活 ROS2：

```bash
source /opt/ros/humble/setup.bash
source /root/projects/ag-repro/ros2_ws/install/setup.bash
```

离线单图检测：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate paus_robot
cd ~/paus_robot

python scripts/run_single_image.py \
  --input /path/to/image.png \
  --output-dir results/run_single_001 \
  --config configs/default.yaml \
  --camera-config configs/camera.yaml
```

FAIRINO Linux SDK 连接测试：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate paus_robot
cd ~/paus_robot

python scripts/test_fairino_linux.py \
  --sdk-root /opt/fairino_python_sdk/linux \
  --robot-ip 192.168.58.2 \
  --command connect
```

设置速度测试：

```bash
python scripts/test_fairino_linux.py \
  --sdk-root /opt/fairino_python_sdk/linux \
  --robot-ip 192.168.58.2 \
  --command set_speed \
  --speed 5
```

### 一键启动（推荐）

默认 dry-run：

```bash
ros2 launch ag_marker_ros2 online_stack.launch.py
```

显式允许真实执行：

```bash
ros2 launch ag_marker_ros2 online_stack.launch.py execute_motion:=true
```

可选参数示例：

```bash
ros2 launch ag_marker_ros2 online_stack.launch.py \
  config_path:=/root/projects/ag-repro/configs/default.yaml \
  camera_config_output:=/root/projects/ag-repro/ros2_ws/install/ag_marker_ros2/share/ag_marker_ros2/configs/camera.yaml \
  camera_ip:=192.168.58.20 \
  execute_motion:=false
```

默认说明：

- launch 会同时启动：
  - `camera_bridge.py`
  - `image_receiver_node`
  - `marker_pose_node`
  - `target_transform_node`
  - `fairino_control_node`
- `execute_motion` 默认是 `false`
- 也就是说，默认只做 dry-run，不直接驱动机械臂

### 手动启动顺序

如果你不想使用 launch，可以按下面顺序手动启动。

#### 1. 启动图像接收节点

```bash
ros2 run ag_marker_ros2 image_receiver_node
```

#### 2. 启动 Linux 相机桥接脚本

```bash
python -u scripts/camera_bridge.py \
  --host 127.0.0.1 \
  --port 5001 \
  --camera-config-output /root/projects/ag-repro/ros2_ws/install/ag_marker_ros2/share/ag_marker_ros2/configs/camera.yaml
```

#### 3. 启动 marker 位姿节点

```bash
ros2 run ag_marker_ros2 marker_pose_node \
  --ros-args -p camera_config_path:=/root/projects/ag-repro/ros2_ws/install/ag_marker_ros2/share/ag_marker_ros2/configs/camera.yaml
```

#### 4. 启动目标变换节点

```bash
ros2 run ag_marker_ros2 target_transform_node
```

#### 5. 启动控制节点

```bash
ros2 run ag_marker_ros2 fairino_control_node
```

### 最基本运行顺序说明

1. 先用 `test_fairino_linux.py` 确认 Linux 执行层能连通机械臂。
2. 起 `image_receiver_node`。
3. 起 `camera_bridge.py`，确认相机图像进入 `/camera/image_bridge`。
4. 起 `marker_pose_node`，确认 `/detection_status` 和 `/marker_pose` 有输出。
5. 起 `target_transform_node`，确认 `/target_point_base` 有输出。
6. 起 `fairino_control_node`，确认 `/control_status` 有输出，且 `control_backend` 为 `linux_sdk`。
7. 若要真实执行，再把 `configs/default.yaml` 中的 `execute_motion` 改为 `true`，或 launch 时传 `execute_motion:=true`。

### 常用检查命令

查看节点：

```bash
ros2 node list
```

查看 marker 检测状态：

```bash
ros2 topic echo /detection_status
```

查看 marker 位姿：

```bash
ros2 topic echo /marker_pose
```

查看目标点：

```bash
ros2 topic echo /target_point_base
```

查看控制状态：

```bash
ros2 topic echo /control_status
```

### 文档入口

主线文档：

- [workflow.md](/root/projects/ag-repro/docs/workflow.md)
- [linux_execution.md](/root/projects/ag-repro/docs/linux_execution.md)
- [coordinate_transform.md](/root/projects/ag-repro/docs/coordinate_transform.md)
- [current_status.md](/root/projects/ag-repro/docs/current_status.md)

历史路线文档：

- [legacy_windows_route.md](/root/projects/ag-repro/docs/legacy_windows_route.md)

## English

### Overview

`paus_robot` is now organized around a **pure Linux / native Ubuntu mainline**.  
The target is to run the full stack on Linux:

- camera input
- ROS2 perception pipeline
- ArUco marker detection
- camera-to-base transform
- candidate approach pose generation
- FAIRINO robot execution

The previous Windows / WSL bridge route is still kept in the repository, but only as a legacy/reference path.

### Mainline Workflow

```text
Linux camera input
-> image_receiver_node
-> marker_pose_node
-> target_transform_node
-> fairino_control_node
-> fairino_linux_client
-> Robot
```

### Key Directories

- `configs/`: runtime config and extrinsics
- `src/ag_repro/`: perception, transforms, control, Linux FAIRINO adapter
- `ros2_ws/`: ROS2 workspace
- `scripts/`: offline tools and Linux execution tests
- `docs/`: workflow and execution docs
- `legacy/`: deprecated Windows / WSL route

### Linux FAIRINO Execution

Default execution path uses the official FAIRINO Linux Python SDK.

Adapter:

- [fairino_linux_client.py](/root/projects/ag-repro/src/ag_repro/fairino_linux_client.py)

Smoke test:

- [test_fairino_linux.py](/root/projects/ag-repro/scripts/test_fairino_linux.py)

### One-Command Launch

Default dry-run:

```bash
ros2 launch ag_marker_ros2 online_stack.launch.py
```

Enable real execution explicitly:

```bash
ros2 launch ag_marker_ros2 online_stack.launch.py execute_motion:=true
```

By default the launch file starts:

- `camera_bridge.py`
- `image_receiver_node`
- `marker_pose_node`
- `target_transform_node`
- `fairino_control_node`

Default behavior is dry-run. Real robot motion is disabled unless `execute_motion:=true` is provided.

### Manual Runtime Order

1. `ros2 run ag_marker_ros2 image_receiver_node`
2. `python -u scripts/camera_bridge.py --host 127.0.0.1 --port 5001 --camera-config-output ...`
3. `ros2 run ag_marker_ros2 marker_pose_node --ros-args -p camera_config_path:=...`
4. `ros2 run ag_marker_ros2 target_transform_node`
5. `ros2 run ag_marker_ros2 fairino_control_node`

### Useful Checks

```bash
ros2 node list
ros2 topic echo /detection_status
ros2 topic echo /marker_pose
ros2 topic echo /target_point_base
ros2 topic echo /control_status
```

### Documentation

Mainline docs:

- [workflow.md](/root/projects/ag-repro/docs/workflow.md)
- [linux_execution.md](/root/projects/ag-repro/docs/linux_execution.md)
- [current_status.md](/root/projects/ag-repro/docs/current_status.md)

Legacy docs:

- [legacy_windows_route.md](/root/projects/ag-repro/docs/legacy_windows_route.md)
