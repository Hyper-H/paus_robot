- [P2] Ignore malformed archive YAML instead of failing sessions list — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:36-39
  If a session’s `report.yaml` or `trajectory_used.yaml` is truncated/corrupted (which is easy if a run is interrupted while writing), this path will raise out of `list_sessions()`/`read_report()` and make the whole sessions API return 500. It would be safer to catch parse errors per session and mark that archive invalid rather than letting one bad file break the UI.

- [P2] Wait longer than 2s before rejecting backend trajectory validation — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_semi_auto_cli.py:173-173
  When the calibration node is slow to publish `/eye_to_hand/status`, this hard-coded 2s probe can fail even though the service is about to come up and the requested trajectory path is valid. In that case the CLI exits with code 2 just because status was late, which makes the new semi-auto flow flaky on slower startups.
2026-05-01T05:36:36.172088Z ERROR codex_core::session: failed to record rollout items: thread 019de1ff-4018-7cb3-816b-ad059c4c6814 not found
2026-05-01T05:36:36.182178Z ERROR codex_core::session: failed to record rollout items: thread 019de1ff-3ff1-7060-a187-d0c1eb271aba not found
The patch introduces at least two user-visible regressions: archive browsing can fail on a single malformed session file, and the new semi-auto CLI can reject valid runs if backend status arrives a little late. These are actionable and likely to affect real usage.

Full review comments:

- [P2] Ignore malformed archive YAML instead of failing sessions list — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:36-39
  If a session’s `report.yaml` or `trajectory_used.yaml` is truncated/corrupted (which is easy if a run is interrupted while writing), this path will raise out of `list_sessions()`/`read_report()` and make the whole sessions API return 500. It would be safer to catch parse errors per session and mark that archive invalid rather than letting one bad file break the UI.

- [P2] Wait longer than 2s before rejecting backend trajectory validation — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_semi_auto_cli.py:173-173
  When the calibration node is slow to publish `/eye_to_hand/status`, this hard-coded 2s probe can fail even though the service is about to come up and the requested trajectory path is valid. In that case the CLI exits with code 2 just because status was late, which makes the new semi-auto flow flaky on slower startups.
