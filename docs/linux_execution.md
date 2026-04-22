# Linux 执行层说明

## 目的
这份文档说明当前 Linux 主线里 FAIRINO 执行链是怎么接的。

## 核心组件
- `src/paus_motion_ros2/paus_motion_ros2/fairino_linux_client.py`
- `src/paus_motion_ros2/paus_motion_ros2/fairino_control_node.py`
- `scripts/test_fairino_linux.py`

## 最小直连验证
在仓库根目录执行：

```bash
python3 scripts/test_fairino_linux.py --command connect
python3 scripts/test_fairino_linux.py --command set_speed --speed 5
python3 scripts/test_fairino_linux.py --command get_joints
python3 scripts/test_fairino_linux.py --command get_tcp
```

## 运动烟雾测试
回当前关节位：

```bash
python3 scripts/test_fairino_linux.py --command move_j --joint-pos J1 J2 J3 J4 J5 J6 --tool-id 0 --user-id 0 --vel 5
```

回当前 TCP 位姿：

```bash
python3 scripts/test_fairino_linux.py --command move_l --pose-mmdeg X Y Z RX RY RZ --tool-id 0 --user-id 0 --vel 5
```

## ROS2 路径
ROS2 在线控制链为：

```text
/target_point_base -> fairino_control_node -> linux_sdk -> robot
```

默认仍是 dry-run，只有显式传入 `execute_motion=true` 才允许真实动作。