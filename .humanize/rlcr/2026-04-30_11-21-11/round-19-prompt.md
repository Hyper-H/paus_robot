# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Treat stale frames as unavailable in quality APIs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:328-352
  If the camera stream freezes or disconnects after at least one image arrives, `get_latest_quality()` still re-runs detection on that last cached frame and `get_status()` propagates the old pose/quality into `handeye.current_waypoint`. In that scenario the UI correctly marks `camera.connected=false`, but it still shows a seemingly valid board pose and quality metrics from stale data, which can mislead operators during alignment or troubleshooting. Consider expiring cached images here using the same freshness window as the camera status.

- [P2] Skip no-report sessions when choosing the latest archive — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:61-69
  After a dry-run or failed semi-auto run, a new session directory is created without `report.yaml`, but `latest_valid_session_id()` still falls back to `sessions[0]`. Because the frontend seeds its initial selection from this method, a page reload will jump to that empty archive instead of the newest solved/unsolved report-backed session, hiding the last actionable calibration result behind a blank report view.

- [P3] Preserve archived sample overlays when current calibration changes — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:379-392
  `get_sample_jpeg()` redraws archived session images with the current detector and current `camera.yaml` instead of the pose/calibration data saved with that session. If the operator recalibrates the camera, changes board dimensions, or simply removes the live `camera.yaml`, historical sample previews will show the wrong overlay—or silently degrade to the raw image—even though the accepted sample already has stored pose metadata. That makes past sessions non-reproducible in the UI.
2026-04-30T17:36:45.910519Z ERROR codex_core::session: failed to record rollout items: thread 019ddf70-38b3-7671-9f7f-21adcc9a8ff3 not found
The new UI/session features mostly hang together, but there are still user-visible correctness issues around stale live data and session/history selection. Those cases can mislead operators or hide the most relevant calibration results, so the patch should not be considered fully correct yet.

Full review comments:

- [P2] Treat stale frames as unavailable in quality APIs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:328-352
  If the camera stream freezes or disconnects after at least one image arrives, `get_latest_quality()` still re-runs detection on that last cached frame and `get_status()` propagates the old pose/quality into `handeye.current_waypoint`. In that scenario the UI correctly marks `camera.connected=false`, but it still shows a seemingly valid board pose and quality metrics from stale data, which can mislead operators during alignment or troubleshooting. Consider expiring cached images here using the same freshness window as the camera status.

- [P2] Skip no-report sessions when choosing the latest archive — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:61-69
  After a dry-run or failed semi-auto run, a new session directory is created without `report.yaml`, but `latest_valid_session_id()` still falls back to `sessions[0]`. Because the frontend seeds its initial selection from this method, a page reload will jump to that empty archive instead of the newest solved/unsolved report-backed session, hiding the last actionable calibration result behind a blank report view.

- [P3] Preserve archived sample overlays when current calibration changes — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:379-392
  `get_sample_jpeg()` redraws archived session images with the current detector and current `camera.yaml` instead of the pose/calibration data saved with that session. If the operator recalibrates the camera, changes board dimensions, or simply removes the live `camera.yaml`, historical sample previews will show the wrong overlay—or silently degrade to the raw image—even though the accepted sample already has stored pose metadata. That makes past sessions non-reproducible in the UI.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-19-summary.md`

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
