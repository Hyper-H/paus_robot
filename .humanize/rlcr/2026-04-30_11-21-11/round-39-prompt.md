# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Keep install-layout extrinsics on the file the stack reads — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:204-206
  In installed workspaces this resolver sends `output_path` to the runtime dir, but the existing online stack/`target_transform_node` still load `share/paus_bringup/configs/extrinsics.yaml`. After a calibration run on an installed system, the new extrinsics are written to a different file than the live stack consumes, so the refreshed solve never takes effect.

- [P2] Avoid expiring backend status during long calibration waits — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:188-191
  If a waypoint spends more than 10s in `_wait_until_tcp_stable()` (the default timeout is also 10s), the last status goes stale before the service returns. That makes `backend_connected` flip back to `ui_local_fallback` mid-run even though the calibration node is still active, which drops motion/trajectory state in the UI and can re-enable confirmation prompts incorrectly.

- [P3] Render failure states as errors, not skipped waypoints — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:421-429
  Statuses that `_workflow_for_status()` marks as `error` (`semi_auto_failed`, `semi_auto_insufficient_samples`, `record_waypoint_failed`) are normalized to `skipped` here. In failure cases the workflow strip will highlight the skipped branch instead of an error state, which makes failed runs look like intentional skips.
2026-05-01T07:20:34.343017Z ERROR codex_core::session: failed to record rollout items: thread 019de25c-498e-7f03-a53e-7c05c6de64a3 not found
The patch introduces an install-layout path mismatch for calibration outputs and misrenders backend failures in the UI. It also contains a stale-status cutoff that can misclassify a still-running calibration as disconnected during long waits.

Full review comments:

- [P1] Keep install-layout extrinsics on the file the stack reads — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:204-206
  In installed workspaces this resolver sends `output_path` to the runtime dir, but the existing online stack/`target_transform_node` still load `share/paus_bringup/configs/extrinsics.yaml`. After a calibration run on an installed system, the new extrinsics are written to a different file than the live stack consumes, so the refreshed solve never takes effect.

- [P2] Avoid expiring backend status during long calibration waits — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:188-191
  If a waypoint spends more than 10s in `_wait_until_tcp_stable()` (the default timeout is also 10s), the last status goes stale before the service returns. That makes `backend_connected` flip back to `ui_local_fallback` mid-run even though the calibration node is still active, which drops motion/trajectory state in the UI and can re-enable confirmation prompts incorrectly.

- [P3] Render failure states as errors, not skipped waypoints — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:421-429
  Statuses that `_workflow_for_status()` marks as `error` (`semi_auto_failed`, `semi_auto_insufficient_samples`, `record_waypoint_failed`) are normalized to `skipped` here. In failure cases the workflow strip will highlight the skipped branch instead of an error state, which makes failed runs look like intentional skips.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-39-summary.md`

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
