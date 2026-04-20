# Linux 执行层说明

## 目标

后续纯 Linux 主线默认使用：

```text
官方 FAIRINO Linux Python SDK
```

不再默认依赖：

- Windows 执行桥
- WSL 与 Windows 之间的 TCP/JSON 桥接

## 当前默认实现

主线默认适配层：

- [fairino_linux_client.py](/root/projects/ag-repro/src/ag_repro/fairino_linux_client.py)

测试脚本：

- [test_fairino_linux.py](/root/projects/ag-repro/scripts/test_fairino_linux.py)

控制节点：

- [fairino_control_node.py](/root/projects/ag-repro/ros2_ws/src/ag_marker_ros2/ag_marker_ros2/fairino_control_node.py)

## 默认配置

配置文件：

- [default.yaml](/root/projects/ag-repro/configs/default.yaml)

关键字段：

```yaml
control:
  robot_ip: "192.168.58.2"
  linux_fairino_sdk_root: "/opt/fairino_python_sdk/linux"
  execute_motion: false
```

## Linux SDK 安装与布局要求

默认假设 Linux SDK 根目录下存在：

```text
linux_fairino_sdk_root/
  fairino/
    Robot.py
    build/lib.linux-*/
  libfairino/
```

当前实现会：

1. 把 SDK 根目录加入 `sys.path`
2. 把 `fairino/build/lib.linux-*` 目录加入 `sys.path`
3. 导入：

```python
from fairino import Robot
```

## 最小验证顺序

### 1. 连接测试

```bash
cd /root/projects/ag-repro
python3 scripts/test_fairino_linux.py --sdk-root /opt/fairino_python_sdk/linux --robot-ip 192.168.58.2 --command connect
```

### 2. 设置速度

```bash
cd /root/projects/ag-repro
python3 scripts/test_fairino_linux.py --sdk-root /opt/fairino_python_sdk/linux --robot-ip 192.168.58.2 --command set_speed --speed 5
```

### 3. 读取当前关节角

```bash
cd /root/projects/ag-repro
python3 scripts/test_fairino_linux.py --sdk-root /opt/fairino_python_sdk/linux --robot-ip 192.168.58.2 --command get_joints
```

### 4. 读取当前 TCP 位姿

```bash
cd /root/projects/ag-repro
python3 scripts/test_fairino_linux.py --sdk-root /opt/fairino_python_sdk/linux --robot-ip 192.168.58.2 --command get_tcp
```

### 5. 回当前关节位

```bash
cd /root/projects/ag-repro
python3 scripts/test_fairino_linux.py \
  --sdk-root /opt/fairino_python_sdk/linux \
  --robot-ip 192.168.58.2 \
  --command move_j \
  --joint-pos 8.024 -56.674 -82.216 -75.939 37.662 41.316 \
  --tool-id 0 \
  --user-id 0 \
  --vel 5
```

### 6. 单关节小幅偏移

```bash
cd /root/projects/ag-repro
python3 scripts/test_fairino_linux.py \
  --sdk-root /opt/fairino_python_sdk/linux \
  --robot-ip 192.168.58.2 \
  --command move_j \
  --joint-pos 10.024 -56.674 -82.216 -75.939 37.662 41.316 \
  --tool-id 0 \
  --user-id 0 \
  --vel 5
```

## 与控制节点的关系

`fairino_control_node` 当前默认逻辑：

- `use_mock_pose=true`
  - 使用 `mock_pose`
- 否则：
  - 优先初始化 Linux FAIRINO SDK
  - 成功则 `control_backend=linux_sdk`
  - 失败则退回 `mock_pose`

## 注意事项

- 在 Linux 主线下，只有 `linux_sdk` 和 `mock_pose` 是默认受支持的后端
- 原来的 Windows 执行桥相关内容已经移入 `legacy/`
