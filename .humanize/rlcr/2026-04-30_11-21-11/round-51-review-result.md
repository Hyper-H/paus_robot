- [P2] Respect explicit current-trajectory selection when live status arrives — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:165-168
  When the operator explicitly picks the blank “当前示教轨迹” option, `currentTrajectorySelected` becomes `true`, but this auto-follow branch still rewrites `selectedSession` to the live session as soon as `run_active` is reported. In that scenario the UI jumps away from the current trajectory even though the user explicitly asked to stay there, which defeats the new selection state this patch added.

- [P2] Keep the web server module importable without ROS installed — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/web_server.py:6-9
  In environments that run the new HTTP-layer tests outside a sourced ROS overlay, importing `paus_ui.web_server` now fails immediately because `rclpy` and `UiRosBridge` are imported at module load. That makes `_parse_confirmed_flag` and `create_app()` untestable in plain Python even though only `main()` actually needs ROS, and it already breaks `src/paus_ui/tests/test_web_server.py` in such setups.
2026-05-01T13:14:22.927350Z ERROR codex_core::session: failed to record rollout items: thread 019de3a1-1bde-7101-8047-05b2f2ac8255 not found
The patch adds substantial UI and session-management functionality, but it still overrides an explicit current-trajectory choice during live runs and the new web server module cannot be imported in non-ROS test environments. Those issues make the change set not fully correct yet.

Full review comments:

- [P2] Respect explicit current-trajectory selection when live status arrives — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:165-168
  When the operator explicitly picks the blank “当前示教轨迹” option, `currentTrajectorySelected` becomes `true`, but this auto-follow branch still rewrites `selectedSession` to the live session as soon as `run_active` is reported. In that scenario the UI jumps away from the current trajectory even though the user explicitly asked to stay there, which defeats the new selection state this patch added.

- [P2] Keep the web server module importable without ROS installed — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/web_server.py:6-9
  In environments that run the new HTTP-layer tests outside a sourced ROS overlay, importing `paus_ui.web_server` now fails immediately because `rclpy` and `UiRosBridge` are imported at module load. That makes `_parse_confirmed_flag` and `create_app()` untestable in plain Python even though only `main()` actually needs ROS, and it already breaks `src/paus_ui/tests/test_web_server.py` in such setups.
