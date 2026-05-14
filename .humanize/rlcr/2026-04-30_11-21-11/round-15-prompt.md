# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Do not trust local `execute_motion` before any backend status arrives — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:181-193
  If the UI starts against an already running calibration node, ROS service discovery can succeed before a `/eye_to_hand/status` message has been received. In that window this code marks the backend as connected but falls back to the UI's own `execute_motion` value, so `/api/handeye/run` can skip the safety confirmation even when the real node was launched with `execute_motion:=true` and the UI config still says `false`.

- [P2] Avoid binding archived manual sessions to the current trajectory file — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:316-318
  Manual `/eye_to_hand/capture_sample` sessions never write `trajectory_used.yaml`, so this fallback makes `read_session_waypoints()` and `_counts_for_session()` interpret old manual sessions using whatever trajectory YAML happens to be current today. After the operator records or edits a different trajectory, previously saved manual sessions will show phantom pending/failed waypoints and incorrect counts in the UI.
2026-04-30T09:46:21.616587Z ERROR codex_core::session: failed to record rollout items: thread 019dddc1-ea4f-7a01-a4e0-d8cb2b57ff29 not found
2026-04-30T09:46:21.625006Z ERROR codex_core::session: failed to record rollout items: thread 019dddc1-ea3d-7510-b423-6f6dfd479d26 not found
The patch adds useful functionality, but it also introduces a motion-safety gap in the UI's confirmation logic and misreports archived manual sessions by attaching them to the wrong trajectory metadata. Those are correctness issues that can affect operators and historical calibration records.

Full review comments:

- [P1] Do not trust local `execute_motion` before any backend status arrives — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:181-193
  If the UI starts against an already running calibration node, ROS service discovery can succeed before a `/eye_to_hand/status` message has been received. In that window this code marks the backend as connected but falls back to the UI's own `execute_motion` value, so `/api/handeye/run` can skip the safety confirmation even when the real node was launched with `execute_motion:=true` and the UI config still says `false`.

- [P2] Avoid binding archived manual sessions to the current trajectory file — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:316-318
  Manual `/eye_to_hand/capture_sample` sessions never write `trajectory_used.yaml`, so this fallback makes `read_session_waypoints()` and `_counts_for_session()` interpret old manual sessions using whatever trajectory YAML happens to be current today. After the operator records or edits a different trajectory, previously saved manual sessions will show phantom pending/failed waypoints and incorrect counts in the UI.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-15-summary.md`

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
