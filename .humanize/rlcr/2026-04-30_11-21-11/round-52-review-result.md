- [P2] Recognize legacy `sample_captured` events in session shaping — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:229-229
  This branch only marks a waypoint accepted for `waypoint_sample_captured`. Sessions created by the existing `/eye_to_hand/capture_sample` flow still log `sample_captured`, so older/manual archives will show every waypoint as `pending`/`-` in the new UI even after a successful capture and solve.

- [P3] Refresh the trajectory path when an archive session is selected — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:287-287
  When `state.selectedSession` is set, this branch loads the archived waypoints but never updates `els.trajectoryPath`, so the header keeps showing whichever live trajectory path was rendered previously. Browsing an archived session therefore leaves the trajectory label inconsistent with the selected session.
2026-05-01T13:41:30.729052Z ERROR codex_core::session: failed to record rollout items: thread 019de3b5-9be9-7590-8227-9ac9ad6c7a63 not found
The new session store does not handle legacy manual-capture archives, so existing sessions can render as entirely pending, and the UI also leaves the trajectory path stale when switching to an archived session. Those are user-visible regressions in the new browsing workflow.

Full review comments:

- [P2] Recognize legacy `sample_captured` events in session shaping — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:229-229
  This branch only marks a waypoint accepted for `waypoint_sample_captured`. Sessions created by the existing `/eye_to_hand/capture_sample` flow still log `sample_captured`, so older/manual archives will show every waypoint as `pending`/`-` in the new UI even after a successful capture and solve.

- [P3] Refresh the trajectory path when an archive session is selected — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:287-287
  When `state.selectedSession` is set, this branch loads the archived waypoints but never updates `els.trajectoryPath`, so the header keeps showing whichever live trajectory path was rendered previously. Browsing an archived session therefore leaves the trajectory label inconsistent with the selected session.
