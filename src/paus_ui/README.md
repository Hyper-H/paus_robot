# PAUS UI — 机器人视觉控制台

半自动手眼标定的 Web 界面，基于 FastAPI + ROS 2。

## 快速启动

```bash
ros2 launch paus_bringup ui.launch.py
```

浏览器打开 `http://localhost:8080`。

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
