# paus_robot

## 中文

### 项目概述
`paus_robot` 现在采用 **仓库根目录即 ROS2 workspace** 的结构。主线面向 Linux / 原生 Ubuntu，围绕“粗定位 -> 运动接近 -> 细定位”的系统分层组织代码。

当前默认主线：
- `paus_perception`：纯 Python 感知与变换核心库
- `paus_marker_ros2`：粗定位视觉节点
- `paus_motion_ros2`：控制、运动、执行编排节点
- `paus_fine_ros2`：细定位预留包位
- `paus_interfaces`：自定义接口预留包位
- `paus_bringup`：launch 与运行配置

旧的 Windows / WSL 双桥方案已整体降级到 `legacy/`，不再作为默认运行入口。

### Workspace 结构
```text
paus_robot/
├── src/
│   ├── paus_perception/
│   ├── paus_marker_ros2/
│   ├── paus_motion_ros2/
│   ├── paus_fine_ros2/
│   ├── paus_interfaces/
│   └── paus_bringup/
├── docs/
├── test_data/
├── tests/
├── legacy/
├── README.md
├── build/      # 本地生成，不入库
├── install/    # 本地生成，不入库
└── log/        # 本地生成，不入库
```

### 包职责
- `paus_perception`
  感知、标定、位姿估计、坐标变换、相机桥接协议。
- `paus_marker_ros2`
  `image_receiver_node`、`marker_pose_node`、`target_transform_node`、`eye_to_hand_calibration_node`。
- `paus_motion_ros2`
  `fairino_control_node`、接近决策逻辑、Linux FAIRINO SDK 适配层。
- `paus_fine_ros2`
  后续超声/光声细定位与近场闭环控制。
- `paus_interfaces`
  预留给自定义 `msg / srv / action`。
- `paus_bringup`
  一键启动 launch 与默认运行配置。

### 构建
在 workspace 根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 一键启动
默认以 **dry-run** 启动整条在线链路：

```bash
ros2 launch paus_bringup online_stack.launch.py
```

显式允许真实机械臂执行：

```bash
ros2 launch paus_bringup online_stack.launch.py execute_motion:=true
```

可选参数：

```bash
ros2 launch paus_bringup online_stack.launch.py \
  camera_ip:=192.168.58.20 \
  camera_config_output:=/tmp/paus_robot/camera.yaml \
  execute_motion:=false
```

### 最基本运行顺序
如果你不走 launch，最小手动顺序是：

1. 启动图像接收节点
```bash
ros2 run paus_marker_ros2 image_receiver_node
```

2. 启动相机桥接脚本
```bash
python3 "$(ros2 pkg prefix paus_marker_ros2)/share/paus_marker_ros2/scripts/camera_bridge.py" \
  --host 127.0.0.1 \
  --port 5001 \
  --camera-config-output /tmp/paus_robot/camera.yaml
```

3. 启动 marker 位姿节点
```bash
ros2 run paus_marker_ros2 marker_pose_node \
  --ros-args -p camera_config_path:=/tmp/paus_robot/camera.yaml
```

4. 启动目标变换节点
```bash
ros2 run paus_marker_ros2 target_transform_node
```

5. 启动控制节点
```bash
ros2 run paus_motion_ros2 fairino_control_node
```

### 关键 Topics
- `/camera/image_bridge`
  Linux 相机桥接后的图像流。
- `/detection_status`
  marker 检测状态。
- `/marker_pose`
  marker 相对相机的位姿。
- `/target_point_base`
  机器人基座系下的目标点。
- `/control_status`
  控制节点状态、候选位姿与执行结果。

### CLI 工具
以下主线工具已经转为 console scripts：

```bash
ros2 run paus_perception generate_marker --marker-id 7 --output ./markers/id7.png
ros2 run paus_perception run_single_image --input ./test_data/images/sample.png --output-dir ./results/run_001
ros2 run paus_perception run_image_batch --input-dir ./test_data/images --output-dir ./results/batch_001
ros2 run paus_perception calibrate_camera --input-dir ./test_data/calib --rows 6 --cols 9 --square-size-m 0.01 --output ./camera.yaml
```

Linux FAIRINO SDK 最小直连测试脚本保留在根目录 `scripts/`：

```bash
python3 scripts/test_fairino_linux.py --command connect
python3 scripts/test_fairino_linux.py --command set_speed --speed 5
```

### 测试
```bash
python3 -m unittest discover -s src/paus_perception/tests -v
python3 -m unittest discover -s src/paus_motion_ros2/tests -v
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
```

### Legacy
以下目录只作历史保留，不参与默认构建与默认运行：
- `legacy/hybrid_stack/`
- `legacy/windows_bridge/`
- `legacy/vendor/`

---

## English

### Overview
`paus_robot` now uses a **workspace-at-repository-root** ROS2 layout. The Linux / native Ubuntu path is the default mainline, organized around coarse localization, robot approach, and future fine localization.

Current package split:
- `paus_perception`: pure Python perception and transform core
- `paus_marker_ros2`: coarse localization ROS2 nodes
- `paus_motion_ros2`: control, motion, and execution orchestration
- `paus_fine_ros2`: placeholder for fine localization
- `paus_interfaces`: placeholder for custom ROS interfaces
- `paus_bringup`: launch files and runtime configuration

The old Windows / WSL hybrid route has been moved to `legacy/` and is no longer the default runtime path.

### Build
From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### One-command launch
Dry-run by default:

```bash
ros2 launch paus_bringup online_stack.launch.py
```

Allow real robot motion explicitly:

```bash
ros2 launch paus_bringup online_stack.launch.py execute_motion:=true
```

### Manual startup order
1. `ros2 run paus_marker_ros2 image_receiver_node`
2. `python3 "$(ros2 pkg prefix paus_marker_ros2)/share/paus_marker_ros2/scripts/camera_bridge.py" --host 127.0.0.1 --port 5001 --camera-config-output /tmp/paus_robot/camera.yaml`
3. `ros2 run paus_marker_ros2 marker_pose_node --ros-args -p camera_config_path:=/tmp/paus_robot/camera.yaml`
4. `ros2 run paus_marker_ros2 target_transform_node`
5. `ros2 run paus_motion_ros2 fairino_control_node`

### Main topics
- `/camera/image_bridge`
- `/detection_status`
- `/marker_pose`
- `/target_point_base`
- `/control_status`

### Legacy
Historical-only content lives under `legacy/` and is excluded from the mainline workflow.

