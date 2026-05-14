# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Preserve deleted waypoints in the session log schema — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:917-918
  When `delete_last_waypoint` is used after a session has started, this appends `removed.to_payload()` to `run.log`, but that payload does not carry the `waypoint_name`/`waypoint` shape used everywhere else. `SessionStore.read_session_waypoints()` only reconstructs entries from those fields, so deleted waypoints will still appear in the archived session view as pending instead of disappearing.

- [P3] Defer session creation until the trajectory is validated — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:983-989
  If the trajectory YAML is missing or malformed, this creates a new `calibration_sessions/<timestamp>` directory before the failure is detected, then returns an error without cleaning it up. That leaves empty sessions behind in `/api/sessions`, which makes the archive look like a run happened when it never started.
2026-05-01T08:59:11.578895Z ERROR codex_core::session: failed to record rollout items: thread 019de2b5-bb45-7f90-86a5-2e8d8b0afb69 not found
2026-05-01T08:59:11.588737Z ERROR codex_core::session: failed to record rollout items: thread 019de2b5-bb30-71f0-bab0-9882142e35b6 not found
The new calibration/session flow has at least two user-visible regressions: deleted waypoints are not represented correctly in archived session data, and failed semi-auto launches leave behind empty session directories. Both affect the accuracy of the UI archive and are likely to need fixes.

Full review comments:

- [P2] Preserve deleted waypoints in the session log schema — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:917-918
  When `delete_last_waypoint` is used after a session has started, this appends `removed.to_payload()` to `run.log`, but that payload does not carry the `waypoint_name`/`waypoint` shape used everywhere else. `SessionStore.read_session_waypoints()` only reconstructs entries from those fields, so deleted waypoints will still appear in the archived session view as pending instead of disappearing.

- [P3] Defer session creation until the trajectory is validated — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:983-989
  If the trajectory YAML is missing or malformed, this creates a new `calibration_sessions/<timestamp>` directory before the failure is detected, then returns an error without cleaning it up. That leaves empty sessions behind in `/api/sessions`, which makes the archive look like a run happened when it never started.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-42-summary.md`

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
