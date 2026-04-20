# FAIRINO Windows 执行桥

当前推荐把 FAIRINO 真机执行层迁到 Windows，本项目里的 ROS2 / OpenCV / 坐标变换继续留在 WSL。

## 架构

```text
WSL candidate pose / command
-> TCP JSON
-> Windows fairino_exec_bridge_win.py
-> 官方 fairino-python-sdk
-> Robot controller
```

## Windows 侧

启动执行桥：

```powershell
powershell -ExecutionPolicy Bypass -File D:\Projects\auto_arm\run_fairino_exec_bridge_win.ps1
```

默认监听：

- `127.0.0.1:5002`

默认机器人 IP：

- `192.168.58.2`

当前桥支持的最小命令：

- `ping`
- `set_speed`
- `move_j`
- `move_l`

## WSL 侧

测试连通性：

```bash
cd /root/projects/ag-repro
python3 scripts/test_fairino_exec_bridge.py --host 127.0.0.1 --port 5002 --command ping
```

测试设置速度：

```bash
cd /root/projects/ag-repro
python3 scripts/test_fairino_exec_bridge.py --host 127.0.0.1 --port 5002 --command set_speed --speed 5
```

测试关节运动：

```bash
cd /root/projects/ag-repro
python3 scripts/test_fairino_exec_bridge.py \
  --host 127.0.0.1 \
  --port 5002 \
  --command move_j \
  --joint-pos 8.024 -56.674 -82.216 -75.939 37.662 41.316 \
  --tool-id 0 \
  --user-id 0 \
  --vel 5
```

## 建议

- 先用 `ping` 和 `set_speed` 验证桥接稳定
- 真机动作优先用 `move_j` 从当前安全关节位做最小测试
- 自动靠近 marker 前，先把 `fairino_control_node` 改成调用这个桥，而不是直接连 FAIRINO ROS2 command server
