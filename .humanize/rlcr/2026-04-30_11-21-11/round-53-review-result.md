- [P2] Keep blank trajectory selection eligible for live runs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:165-168
  If the operator leaves the dropdown on the blank "当前示教轨迹" option, `currentTrajectorySelected` stays true and this auto-follow branch never runs when a live `session_id` appears. In that case the UI stays detached from the active calibration session until the user manually reselects it, which defeats the expected live-follow behavior for an in-progress run.

- [P3] Tie camera-bridge startup to the local receiver — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:116-129
  When `start_image_receiver` is false, this still launches `camera_bridge.py` against the hard-coded `127.0.0.1:5001` socket. The bridge retries forever on connection failures, so `ros2 launch ... start_image_receiver:=false` leaves a noisy process that can never stream frames unless the receiver is also running locally.
2026-05-01T14:14:00.229285Z ERROR codex_core::session: failed to record rollout items: thread 019de3d0-50ef-7442-b6a1-728416fdff0a not found
2026-05-01T14:14:00.286871Z ERROR codex_core::session: failed to record rollout items: thread 019de3d0-50db-7662-8e98-af55f40dbb2f not found
The UI’s blank current-trajectory state now blocks live-session auto-follow, and the new UI launch can start an endlessly reconnecting camera bridge without its receiver. Both are user-visible regressions introduced by the patch.

Full review comments:

- [P2] Keep blank trajectory selection eligible for live runs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:165-168
  If the operator leaves the dropdown on the blank "当前示教轨迹" option, `currentTrajectorySelected` stays true and this auto-follow branch never runs when a live `session_id` appears. In that case the UI stays detached from the active calibration session until the user manually reselects it, which defeats the expected live-follow behavior for an in-progress run.

- [P3] Tie camera-bridge startup to the local receiver — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:116-129
  When `start_image_receiver` is false, this still launches `camera_bridge.py` against the hard-coded `127.0.0.1:5001` socket. The bridge retries forever on connection failures, so `ros2 launch ... start_image_receiver:=false` leaves a noisy process that can never stream frames unless the receiver is also running locally.
