- [P2] Resolve relative calibration overrides with the runtime-path helpers — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:170-171
  When the node is launched from an installed workspace, relative overrides like `output_path:=foo.yaml` or `session_root_path:=runs` now bypass the new install-safe path logic and get resolved against the inferred project/install root here. That sends saves/archives to unexpected or read-only locations instead of `PAUS_ROBOT_RUNTIME_DIR`, so semi-auto runs can fail at save time even though the same values work when they come from `default.yaml`.

- [P2] Raise missing archived sample images instead of returning placeholders — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:396-402
  This API path is wired to translate `FileNotFoundError`/`ValueError` into a 404, but `get_sample_jpeg()` never raises for a missing row or missing file and returns a synthetic JPEG instead. In sessions created with `save_sample_images:=false`, after manual file cleanup, or for an invalid `row_index`, callers receive HTTP 200 with a fake image and cannot tell that the archived sample is actually unavailable.
2026-04-30T18:26:11.878860Z ERROR codex_core::session: failed to record rollout items: thread 019ddf9c-e755-73e2-a4c8-74d81e9a2d58 not found
The new UI/calibration flow has at least two functional issues: install-time relative path overrides no longer resolve to the runtime data area, and archived sample image requests silently succeed with placeholder content instead of surfacing missing data. Both are behavior regressions that can break real usage scenarios.

Full review comments:

- [P2] Resolve relative calibration overrides with the runtime-path helpers — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:170-171
  When the node is launched from an installed workspace, relative overrides like `output_path:=foo.yaml` or `session_root_path:=runs` now bypass the new install-safe path logic and get resolved against the inferred project/install root here. That sends saves/archives to unexpected or read-only locations instead of `PAUS_ROBOT_RUNTIME_DIR`, so semi-auto runs can fail at save time even though the same values work when they come from `default.yaml`.

- [P2] Raise missing archived sample images instead of returning placeholders — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:396-402
  This API path is wired to translate `FileNotFoundError`/`ValueError` into a 404, but `get_sample_jpeg()` never raises for a missing row or missing file and returns a synthetic JPEG instead. In sessions created with `save_sample_images:=false`, after manual file cleanup, or for an invalid `row_index`, callers receive HTTP 200 with a fake image and cannot tell that the archived sample is actually unavailable.
