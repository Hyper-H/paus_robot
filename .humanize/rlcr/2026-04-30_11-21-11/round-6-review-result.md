- [P2] Normalize Euler-angle wraparound when checking TCP stability — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:833-835
  If a reported TCP angle crosses the ±180° boundary while the arm is actually stationary (for example `179.9` then `-179.9`), this raw component-wise subtraction turns a tiny orientation change into a ~360° delta. In that case `_wait_until_tcp_stable()` keeps resetting `stable_reference_pose` and can time out at otherwise valid waypoints, which is especially relevant because the supplied trajectory contains poses near the wrap boundary.

- [P2] Catch invalid `camera.yaml` instead of crashing UI endpoints — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:387-394
  When `camera_config_path` exists but is empty, partially written, or malformed (for example while `camera_bridge.py` is rewriting it), `load_camera_calibration()` raises here and `/api/status`, `/api/handeye/quality`, and the JPEG endpoints start returning 500s. The bridge already has a graceful fallback for a missing calibration file, so this path should degrade the same way instead of taking down the UI API.

- [P2] Declare the UI web-server runtime dependencies for ROS installs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/package.xml:20-21
  In a fresh ROS workspace built via `rosdep`/`colcon`, this package can be installed without `fastapi`, `uvicorn`, or a websocket backend because `package.xml` only declares OpenCV/YAML runtime deps. Launching `paus_ui_server` then fails immediately at the import guard in `web_server.py` unless users add manual pip installs, so these runtime dependencies need to be declared in the ROS package metadata as well.
2026-04-30T06:25:54.225728Z ERROR codex_core::session: failed to record rollout items: thread 019ddd0a-f0a9-7a23-b04f-6fef46ef541f not found
The new semi-auto/UI flow has a couple of functional edge cases that can break real runs or the UI, and the new ROS package metadata is incomplete for standard workspace installs. Those issues make the patch unsafe to treat as fully correct yet.

Full review comments:

- [P2] Normalize Euler-angle wraparound when checking TCP stability — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:833-835
  If a reported TCP angle crosses the ±180° boundary while the arm is actually stationary (for example `179.9` then `-179.9`), this raw component-wise subtraction turns a tiny orientation change into a ~360° delta. In that case `_wait_until_tcp_stable()` keeps resetting `stable_reference_pose` and can time out at otherwise valid waypoints, which is especially relevant because the supplied trajectory contains poses near the wrap boundary.

- [P2] Catch invalid `camera.yaml` instead of crashing UI endpoints — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:387-394
  When `camera_config_path` exists but is empty, partially written, or malformed (for example while `camera_bridge.py` is rewriting it), `load_camera_calibration()` raises here and `/api/status`, `/api/handeye/quality`, and the JPEG endpoints start returning 500s. The bridge already has a graceful fallback for a missing calibration file, so this path should degrade the same way instead of taking down the UI API.

- [P2] Declare the UI web-server runtime dependencies for ROS installs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/package.xml:20-21
  In a fresh ROS workspace built via `rosdep`/`colcon`, this package can be installed without `fastapi`, `uvicorn`, or a websocket backend because `package.xml` only declares OpenCV/YAML runtime deps. Launching `paus_ui_server` then fails immediately at the import guard in `web_server.py` unless users add manual pip installs, so these runtime dependencies need to be declared in the ROS package metadata as well.
