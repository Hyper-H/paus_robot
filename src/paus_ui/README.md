# PAUS UI — 机器人视觉控制台

半自动手眼标定的 Web 界面，基于 FastAPI + ROS 2。

## 快速启动

```bash
ros2 launch paus_bringup ui.launch.py
```

浏览器打开 `http://localhost:8080`。

同时接收 Windows 超声窗口图像：

```bash
ros2 launch paus_bringup ui.launch.py execute_motion:=false \
  enable_ultrasound:=true ultrasound_listen_host:=<LINUX_IPV4>
```

页面顶部“超声图像”可展开独立预览，显示收帧数、到达率、图像年龄、
ROI 确认及显示标尺比例，不替换手眼相机。图像年龄按 `Image.header.stamp`
中的 Linux 收帧时间计算，不是声学采集到主控的端到端延迟。UI 还核对
接收器的帧年龄和会话标识，避免延迟到达的旧帧被误认为新帧。接收器失联或帧超时会清除
预览；“图像流可用”不构成机器人运动许可。

接收器默认未启用且只绑定回环地址；开启跨机连接需指定本机可达的局域网
地址。已单独启动接收器时，保留 `enable_ultrasound:=false` 可避免端口冲突。
可单独查询 `/api/ultrasound/status` 和 `/api/ultrasound/image.jpg`，后者在
无新鲜图像时返回 HTTP 204。单轴未知比例显示“未知”，不会推断为另一轴。

## 功能

- **实时图像**：WebSocket 二进制 JPEG raw 推流；断开时回退 HTTP raw；live 画面不再实时计算 overlay/pose
- **Waypoint 管理**：记录、删除、保存示教轨迹
- **半自动标定**：支持 dry-run 和真机两种模式
- **质量检查**：由状态事件、记录 waypoint 或手动检测触发，不再周期性重算
- **样本预览**：点击 waypoint 查看历史 raw 样本图像或详细指标
- **标定结果**：求解残差展示、阈值比较、极值高亮
- **事件通道**：WebSocket 优先，断开或 403 时自动回退 HTTP 轮询

## 目录结构

```
paus_ui/
  paus_ui/
    ros_bridge.py        # ROS 2 节点，HTTP API 后端
    web_server.py        # FastAPI 应用
    session_store.py     # 标定 session 数据管理
    overlay.py           # 棋盘检测和图像叠加
    operator_messages.py # 错误消息分类/翻译
    path_resolvers.py    # 路径解析
    static/
      index.html         # 主页面
      app.js             # 前端逻辑
      styles.css         # 样式表
  tests/                 # 单元测试
```

## 依赖

- ROS 2 (rclpy)
- FastAPI + uvicorn
- OpenCV (cv2)
- NumPy
- PyYAML
