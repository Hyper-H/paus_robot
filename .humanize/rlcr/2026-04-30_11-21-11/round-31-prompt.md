# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Expose trajectory save before letting the UI start a run — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/web_server.py:49-60
  The new UI exposes `record_waypoint` and `delete_last_waypoint`, but there is no matching `save_trajectory` action here even though the calibration node only runs from `trajectory_path` on disk (`_save_trajectory_callback` is the only code that persists the in-memory edits). In practice, after an operator records or deletes waypoints in the web UI, `run_semi_auto_calibration` will still use the old YAML file—or fail with `Trajectory YAML does not exist` on a fresh setup—so the record/run workflow added in this patch cannot actually use the points the operator just taught.

- [P3] Defer session creation until a manual capture succeeds — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:558-564
  `_capture_one_sample()` allocates a new session directory before waiting for a fresh frame or checking whether the chessboard can be detected. If the operator presses capture while the camera is stale or the board is out of frame, the exception path leaves behind an empty archive under `calibration_sessions`, and the new Sessions UI will accumulate these phantom runs even though no sample was ever captured.
2026-04-30T20:17:58.106390Z ERROR codex_core::session: failed to record rollout items: thread 019de003-e296-73a0-842a-74085a693332 not found
2026-04-30T20:17:58.116062Z ERROR codex_core::session: failed to record rollout items: thread 019de003-e281-7613-bd28-339653b2da58 not found
The web UI's primary record-and-run flow is broken because recorded waypoints are never persisted before the run consumes the trajectory file. The patch also leaves empty session artifacts on common manual-capture failures.

Full review comments:

- [P1] Expose trajectory save before letting the UI start a run — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/web_server.py:49-60
  The new UI exposes `record_waypoint` and `delete_last_waypoint`, but there is no matching `save_trajectory` action here even though the calibration node only runs from `trajectory_path` on disk (`_save_trajectory_callback` is the only code that persists the in-memory edits). In practice, after an operator records or deletes waypoints in the web UI, `run_semi_auto_calibration` will still use the old YAML file—or fail with `Trajectory YAML does not exist` on a fresh setup—so the record/run workflow added in this patch cannot actually use the points the operator just taught.

- [P3] Defer session creation until a manual capture succeeds — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:558-564
  `_capture_one_sample()` allocates a new session directory before waiting for a fresh frame or checking whether the chessboard can be detected. If the operator presses capture while the camera is stale or the board is out of frame, the exception path leaves behind an empty archive under `calibration_sessions`, and the new Sessions UI will accumulate these phantom runs even though no sample was ever captured.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-31-summary.md`

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
