- [P2] Clear the preview when the waypoint list is empty — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:292-296
  When `state.waypoints` is empty (for example after switching to an empty archive or back to the live trajectory), this branch never calls `renderPreview()` or clears `samplePreview`, so the previous session’s image/details stay visible. That leaves the right-hand preview panel showing stale data instead of an empty state.

- [P2] Count only active waypoints when deriving session totals — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:355-364
  If a session only has `run.log` data, this helper treats every `waypoint_name` it sees as active, including `waypoint_deleted`. In a record/delete/re-record sequence that makes `pending_count` too large, so `list_sessions()` reports phantom pending waypoints even though `read_session_waypoints()` has already dropped the deleted ones.
2026-05-01T09:37:13.937505Z ERROR codex_core::session: failed to record rollout items: thread 019de2d3-3e26-7441-bed1-f95d441e257a not found
2026-05-01T09:37:13.990114Z ERROR codex_core::session: failed to record rollout items: thread 019de2d3-3e11-7cc3-a5d5-d0bcc4a89915 not found
The patch introduces at least two user-visible regressions: the UI can keep showing a stale sample preview after switching to an empty waypoint set, and session summaries can overcount pending waypoints because deleted names are still counted as active.

Full review comments:

- [P2] Clear the preview when the waypoint list is empty — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:292-296
  When `state.waypoints` is empty (for example after switching to an empty archive or back to the live trajectory), this branch never calls `renderPreview()` or clears `samplePreview`, so the previous session’s image/details stay visible. That leaves the right-hand preview panel showing stale data instead of an empty state.

- [P2] Count only active waypoints when deriving session totals — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:355-364
  If a session only has `run.log` data, this helper treats every `waypoint_name` it sees as active, including `waypoint_deleted`. In a record/delete/re-record sequence that makes `pending_count` too large, so `list_sessions()` reports phantom pending waypoints even though `read_session_waypoints()` has already dropped the deleted ones.
