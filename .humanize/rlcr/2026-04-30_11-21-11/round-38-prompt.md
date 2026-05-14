# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Return a usable UI URL instead of `0.0.0.0` — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:289-293
  When the server is launched with the default bind host, this field becomes `http://0.0.0.0:PORT`, which is not a connectable browser URL. Any client that uses `/api/status` to open the UI will fail unless it manually rewrites the host; returning `localhost` (or omitting the URL) would avoid advertising an unusable address.

- [P2] Reset semi-auto session state before validating the trajectory — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:988-989
  If `load_trajectory()` raises (for example, a malformed `trajectory_path` after a previous run left `self.session_dir`/`run_log_path` set), the exception handler appends `semi_auto_failed` to the old session log and reports that stale session directory in status. That misattributes the failure to the previous archive instead of the new attempt, so the session should be created/reset before loading or the stale state should be cleared on this path.
2026-05-01T06:55:14.698920Z ERROR codex_core::session: failed to record rollout items: thread 019de243-1562-7fd1-9dc9-0349e44643db not found
2026-05-01T06:55:14.755386Z ERROR codex_core::session: failed to record rollout items: thread 019de243-154e-75c0-92cd-40375b1eb098 not found
The UI now advertises an unusable 0.0.0.0 URL, and semi-auto startup can log failures into a stale previous session when trajectory validation fails. Both are user-visible regressions introduced by the patch.

Full review comments:

- [P2] Return a usable UI URL instead of `0.0.0.0` — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:289-293
  When the server is launched with the default bind host, this field becomes `http://0.0.0.0:PORT`, which is not a connectable browser URL. Any client that uses `/api/status` to open the UI will fail unless it manually rewrites the host; returning `localhost` (or omitting the URL) would avoid advertising an unusable address.

- [P2] Reset semi-auto session state before validating the trajectory — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:988-989
  If `load_trajectory()` raises (for example, a malformed `trajectory_path` after a previous run left `self.session_dir`/`run_log_path` set), the exception handler appends `semi_auto_failed` to the old session log and reports that stale session directory in status. That misattributes the failure to the previous archive instead of the new attempt, so the session should be created/reset before loading or the stale state should be cleared on this path.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-38-summary.md`

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
