- [P2] Sync detector settings from the backend before scoring UI images — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:185-192
  When the web UI is launched against an already-running calibration stack (for example `start_calibration_node:=false` in `ui.launch.py`), this sync path only imports trajectory/session/motion thresholds from `/eye_to_hand/status`. The overlay detector keeps using the UI’s own `board_rows`/`board_cols`/`square_size_m`/camera-config settings, so a backend started with a different config can be happily capturing samples while `/api/handeye/quality` and the live overlay show false “chessboard not detected” or wrong pose data.

- [P3] Treat fallback sample images as present in archived sessions — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:108-110
  `has_image` is computed only from `sample["image_path"]`, but `sample_image_path()` later falls back to `session/images/sample_###.png`. For older/manual archives that only have the conventional image files, the backend can still serve the JPEG while every sample/waypoint is marked `has_image=false`, which makes the new UI hide thumbnails and preview links for valid archived images.
2026-04-30T16:53:11.670303Z ERROR codex_core::session: failed to record rollout items: thread 019ddf47-adc6-7950-a55e-389212f5e98e not found
The new UI stack works in the common in-process launch path, but there are still correctness gaps in detached-UI deployments and in archived-session browsing. Those issues are specific and user-visible enough that the patch should not be considered fully correct yet.

Full review comments:

- [P2] Sync detector settings from the backend before scoring UI images — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:185-192
  When the web UI is launched against an already-running calibration stack (for example `start_calibration_node:=false` in `ui.launch.py`), this sync path only imports trajectory/session/motion thresholds from `/eye_to_hand/status`. The overlay detector keeps using the UI’s own `board_rows`/`board_cols`/`square_size_m`/camera-config settings, so a backend started with a different config can be happily capturing samples while `/api/handeye/quality` and the live overlay show false “chessboard not detected” or wrong pose data.

- [P3] Treat fallback sample images as present in archived sessions — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:108-110
  `has_image` is computed only from `sample["image_path"]`, but `sample_image_path()` later falls back to `session/images/sample_###.png`. For older/manual archives that only have the conventional image files, the backend can still serve the JPEG while every sample/waypoint is marked `has_image=false`, which makes the new UI hide thumbnails and preview links for valid archived images.
