# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Keep unsaved recorded waypoints after the last status is older than 10s — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:433-436
  If an operator records a few waypoints and then pauses for more than 10 seconds before clicking run/save, `get_waypoints()` stops returning `recorded_trajectory` and falls back to `session_store.read_waypoints()`. In that state the UI loses the in-memory edits and `saveTrajectoryIfDirty()` can skip `/api/handeye/save_trajectory`, so the subsequent semi-auto run uses the stale on-disk YAML (or fails because no file exists) instead of the just-recorded trajectory.

- [P2] Preserve backend config overrides after the status message becomes stale — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:191-195
  `_sync_backend_state()` discards `trajectory_path`, `session_root_path`, threshold, and camera-config values as soon as the last status message is older than 10 seconds, even if the calibration node and its services are still up. When the node was launched with non-default paths or thresholds, the UI silently reverts to its local config after a short idle period and starts reading the wrong trajectory/session directories until another status publish happens.
2026-05-01T11:09:07.607334Z ERROR codex_core::session: failed to record rollout items: thread 019de32d-4e2f-7d00-bb7c-8a3c99869183 not found
The new UI bridge expires backend state too aggressively. After a brief idle period it can both hide unsaved recorded trajectories and revert to the wrong backend paths/config, which breaks core manual-recording and custom-path workflows.

Full review comments:

- [P1] Keep unsaved recorded waypoints after the last status is older than 10s — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:433-436
  If an operator records a few waypoints and then pauses for more than 10 seconds before clicking run/save, `get_waypoints()` stops returning `recorded_trajectory` and falls back to `session_store.read_waypoints()`. In that state the UI loses the in-memory edits and `saveTrajectoryIfDirty()` can skip `/api/handeye/save_trajectory`, so the subsequent semi-auto run uses the stale on-disk YAML (or fails because no file exists) instead of the just-recorded trajectory.

- [P2] Preserve backend config overrides after the status message becomes stale — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:191-195
  `_sync_backend_state()` discards `trajectory_path`, `session_root_path`, threshold, and camera-config values as soon as the last status message is older than 10 seconds, even if the calibration node and its services are still up. When the node was launched with non-default paths or thresholds, the UI silently reverts to its local config after a short idle period and starts reading the wrong trajectory/session directories until another status publish happens.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-47-summary.md`

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
