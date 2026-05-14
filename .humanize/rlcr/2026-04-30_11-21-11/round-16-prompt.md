# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Sync detector settings from the backend before scoring UI images — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:185-192
  When the web UI is launched against an already-running calibration stack (for example `start_calibration_node:=false` in `ui.launch.py`), this sync path only imports trajectory/session/motion thresholds from `/eye_to_hand/status`. The overlay detector keeps using the UI’s own `board_rows`/`board_cols`/`square_size_m`/camera-config settings, so a backend started with a different config can be happily capturing samples while `/api/handeye/quality` and the live overlay show false “chessboard not detected” or wrong pose data.

- [P3] Treat fallback sample images as present in archived sessions — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:108-110
  `has_image` is computed only from `sample["image_path"]`, but `sample_image_path()` later falls back to `session/images/sample_###.png`. For older/manual archives that only have the conventional image files, the backend can still serve the JPEG while every sample/waypoint is marked `has_image=false`, which makes the new UI hide thumbnails and preview links for valid archived images.
2026-04-30T16:53:11.670303Z ERROR codex_core::session: failed to record rollout items: thread 019ddf47-adc6-7950-a55e-389212f5e98e not found
The new UI stack works in the common in-process launch path, but there are still correctness gaps in detached-UI deployments and in archived-session browsing. Those issues are specific and user-visible enough that the patch should not be considered fully correct yet.

Full review comments:

- [P2] Sync detector settings from the backend before scoring UI images — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:185-192
  When the web UI is launched against an already-running calibration stack (for example `start_calibration_node:=false` in `ui.launch.py`), this sync path only imports trajectory/session/motion thresholds from `/eye_to_hand/status`. The overlay detector keeps using the UI’s own `board_rows`/`board_cols`/`square_size_m`/camera-config settings, so a backend started with a different config can be happily capturing samples while `/api/handeye/quality` and the live overlay show false “chessboard not detected” or wrong pose data.

- [P3] Treat fallback sample images as present in archived sessions — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:108-110
  `has_image` is computed only from `sample["image_path"]`, but `sample_image_path()` later falls back to `session/images/sample_###.png`. For older/manual archives that only have the conventional image files, the backend can still serve the JPEG while every sample/waypoint is marked `has_image=false`, which makes the new UI hide thumbnails and preview links for valid archived images.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-16-summary.md`

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
