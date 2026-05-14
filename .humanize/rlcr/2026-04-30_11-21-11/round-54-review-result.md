- [P2] Launch camera bridge independently of the image receiver — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:116-116
  `start_camera_bridge` is gated by `start_image_receiver`, so `start_camera_bridge:=true` does nothing whenever the receiver is managed elsewhere. In that setup the bridge never starts, `camera.yaml` is never produced, and the calibration node just times out waiting for it.
2026-05-01T14:45:51.477048Z ERROR codex_core::session: failed to record rollout items: thread 019de3f0-f00a-7443-854c-9726d8324ce6 not found
2026-05-01T14:45:51.486905Z ERROR codex_core::session: failed to record rollout items: thread 019de3f0-eff7-77e2-abd0-16370eb8a5b6 not found
The new UI launch file cannot start the camera bridge on its own, which breaks the advertised launch flag combination for setups that use an externally managed image receiver. That is a functional regression in a common deployment path.

Review comment:

- [P2] Launch camera bridge independently of the image receiver — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:116-116
  `start_camera_bridge` is gated by `start_image_receiver`, so `start_camera_bridge:=true` does nothing whenever the receiver is managed elsewhere. In that setup the bridge never starts, `camera.yaml` is never produced, and the calibration node just times out waiting for it.
