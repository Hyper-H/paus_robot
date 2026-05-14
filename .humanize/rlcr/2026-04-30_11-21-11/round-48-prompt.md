# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Recompute pending counts from the edited waypoint set — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:347-350
  If a session has already written `trajectory_used.yaml` and the operator later records or deletes additional waypoints before finishing, this branch still trusts the saved file length and ignores the later `run.log` edits. `read_session_waypoints()` already replays those edits, so `list_sessions()` ends up reporting a stale `pending_count` for the same archived session.

- [P2] Allow the empty-trajectory path to be cleared after the last delete — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:937-941
  When the operator deletes the final waypoint after a trajectory has already been saved, `recorded_trajectory` becomes empty but the old YAML is still left on disk. Because this guard rejects empty saves, `saveTrajectoryIfDirty()`/the Run button can never clear that stale file again, so the session stays dirty until the node is restarted or the file is removed manually.
2026-05-01T11:42:48.673742Z ERROR codex_core::session: failed to record rollout items: thread 019de34a-be55-7533-9d97-c2735f01a665 not found
2026-05-01T11:42:48.683494Z ERROR codex_core::session: failed to record rollout items: thread 019de34a-be41-7bd2-ad61-a54dd2b1049f not found
The patch introduces user-visible regressions in the new trajectory workflow: archived session counts can drift from the actual edited waypoint list, and deleting the last waypoint leaves the trajectory in an unrecoverable dirty state. Both affect normal calibration use.

Full review comments:

- [P2] Recompute pending counts from the edited waypoint set — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:347-350
  If a session has already written `trajectory_used.yaml` and the operator later records or deletes additional waypoints before finishing, this branch still trusts the saved file length and ignores the later `run.log` edits. `read_session_waypoints()` already replays those edits, so `list_sessions()` ends up reporting a stale `pending_count` for the same archived session.

- [P2] Allow the empty-trajectory path to be cleared after the last delete — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:937-941
  When the operator deletes the final waypoint after a trajectory has already been saved, `recorded_trajectory` becomes empty but the old YAML is still left on disk. Because this guard rejects empty saves, `saveTrajectoryIfDirty()`/the Run button can never clear that stale file again, so the session stays dirty until the node is restarted or the file is removed manually.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-48-summary.md`

## Summary Template

Your summary should include:
- Which issues were fixed
- How each issue was resolved
- Any issues that could not be resolved (with explanation)

## Important Notes

- The COMPLETE signal has no effect during the review phase
- You must address the code review findings to proceed
- After you commit and write your summary, Codex will perform another code review
- The loop continues until no `[P0-9]` issues are found

## Task Tag Routing Reminder

Follow the plan's per-task routing tags strictly:
- `coding` task -> Claude executes directly
- `analyze` task -> execute via `/humanize:ask-codex`, then integrate the result
- Keep Goal Tracker Active Tasks columns `Tag` and `Owner` aligned with execution
