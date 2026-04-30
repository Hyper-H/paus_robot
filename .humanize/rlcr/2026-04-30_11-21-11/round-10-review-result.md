- [P2] Defer session directory creation until first capture/run — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:195-198
  Creating `self.session_dir` during node startup means every restart immediately produces a new reportless session, and `_append_run_log("node_started", ...)` makes it show up in `/api/sessions` as if it were a real calibration run. In practice the UI will surface this bogus "latest" session and hide the last actual report until the operator manually re-selects it, while `calibration_sessions/` accumulates empty archives on every restart.

- [P2] Mark `capture: false` waypoints as completed — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:944-944
  This branch only records a terminal outcome when `waypoint.capture` is true. If a trajectory contains non-capturing transit/safety waypoints, their last event is `waypoint_reached`, so `SessionStore.read_session_waypoints()` leaves them stuck in a running/pending state and the session counts never reconcile. Emitting a dedicated completed/skipped event for `capture: false` waypoints would keep the history and UI accurate.

- [P3] Check `cv2.imwrite()` before accepting the sample — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:455-455
  `cv2.imwrite()` fails by returning `False`, not by throwing, so permission or disk-space problems here silently produce samples whose `image_path` is logged even though no PNG was written. In those environments the run appears successful but later session review has missing thumbnails/evidence for accepted captures.
2026-04-30T08:10:57.980235Z ERROR codex_core::session: failed to record rollout items: thread 019ddd65-ae85-7f80-b6d2-74a3eee48b3c not found
2026-04-30T08:10:57.990326Z ERROR codex_core::session: failed to record rollout items: thread 019ddd65-ae61-7cf0-8eae-6a43ecbb6b83 not found
The patch introduces a few workflow regressions in the new semi-auto/session features: startup now creates bogus sessions, non-capturing waypoints never reach a terminal state, and sample-image save failures are silently ignored. These issues can mislead operators and leave archived calibration data inconsistent.

Full review comments:

- [P2] Defer session directory creation until first capture/run — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:195-198
  Creating `self.session_dir` during node startup means every restart immediately produces a new reportless session, and `_append_run_log("node_started", ...)` makes it show up in `/api/sessions` as if it were a real calibration run. In practice the UI will surface this bogus "latest" session and hide the last actual report until the operator manually re-selects it, while `calibration_sessions/` accumulates empty archives on every restart.

- [P2] Mark `capture: false` waypoints as completed — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:944-944
  This branch only records a terminal outcome when `waypoint.capture` is true. If a trajectory contains non-capturing transit/safety waypoints, their last event is `waypoint_reached`, so `SessionStore.read_session_waypoints()` leaves them stuck in a running/pending state and the session counts never reconcile. Emitting a dedicated completed/skipped event for `capture: false` waypoints would keep the history and UI accurate.

- [P3] Check `cv2.imwrite()` before accepting the sample — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:455-455
  `cv2.imwrite()` fails by returning `False`, not by throwing, so permission or disk-space problems here silently produce samples whose `image_path` is logged even though no PNG was written. In those environments the run appears successful but later session review has missing thumbnails/evidence for accepted captures.
