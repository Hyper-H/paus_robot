# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Wait for a valid camera.yaml instead of any non-empty file — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:242-248
  When the default launch starts `camera_bridge.py` and the calibration node together, this loop returns as soon as `camera.yaml` exists and has a non-zero size. The bridge writes that YAML in place, so the node can race here and call `load_camera_calibration()` while the file is only partially written, leading to intermittent startup failures even though the bridge finishes a moment later. This needs to wait for a parseable calibration file (or rely on an atomic writer) rather than just `st_size > 0`.

- [P2] Clear stale board pose widgets when live status has no pose — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:142-153
  If a previous status included `camera_to_board_*` data and a later status does not (for example the board leaves view, the node returns to idle, or a stale/no-image state arrives), these branches leave the old pose and angle on screen. In that scenario the operator keeps seeing the last successful board pose as if it were current, which is misleading for a live calibration UI.

- [P3] Keep the auto-selected session visible after a run finishes — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:155-160
  When the user starts a semi-auto run without manually picking a session first, the UI auto-selects the active archive during the run and then immediately clears that selection as soon as `run_active` flips to false. That means the report and waypoint results for the just-finished calibration disappear right after completion unless the operator manually re-selects the session, which defeats the new session review flow.
2026-04-30T19:55:08.173404Z ERROR codex_core::session: failed to record rollout items: thread 019ddfed-85c1-7b01-aa84-64bda5775f3b not found
The patch introduces at least one startup race in the calibration node and a couple of user-visible UI state issues that make the new semi-auto/session workflow unreliable or misleading. These should be addressed before considering the change correct.

Full review comments:

- [P2] Wait for a valid camera.yaml instead of any non-empty file — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:242-248
  When the default launch starts `camera_bridge.py` and the calibration node together, this loop returns as soon as `camera.yaml` exists and has a non-zero size. The bridge writes that YAML in place, so the node can race here and call `load_camera_calibration()` while the file is only partially written, leading to intermittent startup failures even though the bridge finishes a moment later. This needs to wait for a parseable calibration file (or rely on an atomic writer) rather than just `st_size > 0`.

- [P2] Clear stale board pose widgets when live status has no pose — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:142-153
  If a previous status included `camera_to_board_*` data and a later status does not (for example the board leaves view, the node returns to idle, or a stale/no-image state arrives), these branches leave the old pose and angle on screen. In that scenario the operator keeps seeing the last successful board pose as if it were current, which is misleading for a live calibration UI.

- [P3] Keep the auto-selected session visible after a run finishes — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:155-160
  When the user starts a semi-auto run without manually picking a session first, the UI auto-selects the active archive during the run and then immediately clears that selection as soon as `run_active` flips to false. That means the report and waypoint results for the just-finished calibration disappear right after completion unless the operator manually re-selects the session, which defeats the new session review flow.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-29-summary.md`

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
