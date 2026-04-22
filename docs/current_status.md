# 当前状态

## 迁移结论
仓库已经从旧的混合结构（`ag_repro + ag_marker_ros2 + ros2_ws`）迁移到“仓库根目录即 ROS2 workspace”的新结构。

当前主线包：
- `src/paus_perception`
- `src/paus_marker_ros2`
- `src/paus_motion_ros2`
- `src/paus_fine_ros2`
- `src/paus_interfaces`
- `src/paus_bringup`

## 已完成
- `ag_repro` 已拆分为：
  - `paus_perception`
  - `paus_motion_ros2`
- `ag_marker_ros2` 已拆分为：
  - `paus_marker_ros2`
  - `paus_motion_ros2`
  - `paus_bringup`
- Linux 主线 launch 已切换为：
  - `ros2 launch paus_bringup online_stack.launch.py`
- 原根目录 CLI 已迁到 `paus_perception` console scripts
- 嵌套式 `ros2_ws` 已退出主线
- 旧 vendor / hybrid 路线已隔离到 `legacy/`

## 当前仍为空骨架的包
- `paus_fine_ros2`
- `paus_interfaces`

它们现在先留空，但包位已经固定好，方便后面继续扩展。

## 当前建议的下一步
1. 用正式的自定义 `msg / srv / action` 替代 `std_msgs/String + JSON`
2. 把细定位相关实验逐步迁到 `paus_fine_ros2`
3. 给 `paus_marker_ros2` 与 `paus_motion_ros2` 增加更多包内测试
4. 在 `tests/integration/` 里补跨包联调测试