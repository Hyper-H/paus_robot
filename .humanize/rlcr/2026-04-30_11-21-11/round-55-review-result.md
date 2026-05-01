- [P2] Count legacy `sample_captured` events as accepted — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:343-346
  When an archived session only has the older `sample_captured` run-log entries, `read_session_waypoints()` already marks those rows accepted, but `_counts_for_session()` still counts only `waypoint_sample_captured`. That leaves `accepted_count` at 0 and inflates `pending_count` for legacy/manual sessions even though the displayed waypoint is accepted.

- [P2] Map manual capture statuses into the workflow state — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:681-700
  The new manual `Record current point` path publishes `sample_captured`/`capture_failed`, but this mapping only recognizes the semi-auto `waypoint_*` statuses. In a manual session the workflow ribbon will stay on the idle branch and never show that a point was captured or failed, which makes the new record/delete/save flow look broken to the operator.
2026-05-01T15:16:49.946499Z ERROR codex_core::session: failed to record rollout items: thread 019de40f-12cb-7983-8fe7-3a6e314b5845 not found
The new UI/session layer misreports legacy manual sessions and fails to surface manual capture progress in the workflow view, so the new feature does not behave correctly for existing and manual recording paths.

Full review comments:

- [P2] Count legacy `sample_captured` events as accepted — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:343-346
  When an archived session only has the older `sample_captured` run-log entries, `read_session_waypoints()` already marks those rows accepted, but `_counts_for_session()` still counts only `waypoint_sample_captured`. That leaves `accepted_count` at 0 and inflates `pending_count` for legacy/manual sessions even though the displayed waypoint is accepted.

- [P2] Map manual capture statuses into the workflow state — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:681-700
  The new manual `Record current point` path publishes `sample_captured`/`capture_failed`, but this mapping only recognizes the semi-auto `waypoint_*` statuses. In a manual session the workflow ribbon will stay on the idle branch and never show that a point was captured or failed, which makes the new record/delete/save flow look broken to the operator.
