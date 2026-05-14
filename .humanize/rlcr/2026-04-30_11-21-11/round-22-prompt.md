# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Resolve relative calibration overrides with the runtime-path helpers — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:170-171
  When the node is launched from an installed workspace, relative overrides like `output_path:=foo.yaml` or `session_root_path:=runs` now bypass the new install-safe path logic and get resolved against the inferred project/install root here. That sends saves/archives to unexpected or read-only locations instead of `PAUS_ROBOT_RUNTIME_DIR`, so semi-auto runs can fail at save time even though the same values work when they come from `default.yaml`.

- [P2] Raise missing archived sample images instead of returning placeholders — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:396-402
  This API path is wired to translate `FileNotFoundError`/`ValueError` into a 404, but `get_sample_jpeg()` never raises for a missing row or missing file and returns a synthetic JPEG instead. In sessions created with `save_sample_images:=false`, after manual file cleanup, or for an invalid `row_index`, callers receive HTTP 200 with a fake image and cannot tell that the archived sample is actually unavailable.
2026-04-30T18:26:11.878860Z ERROR codex_core::session: failed to record rollout items: thread 019ddf9c-e755-73e2-a4c8-74d81e9a2d58 not found
The new UI/calibration flow has at least two functional issues: install-time relative path overrides no longer resolve to the runtime data area, and archived sample image requests silently succeed with placeholder content instead of surfacing missing data. Both are behavior regressions that can break real usage scenarios.

Full review comments:

- [P2] Resolve relative calibration overrides with the runtime-path helpers — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:170-171
  When the node is launched from an installed workspace, relative overrides like `output_path:=foo.yaml` or `session_root_path:=runs` now bypass the new install-safe path logic and get resolved against the inferred project/install root here. That sends saves/archives to unexpected or read-only locations instead of `PAUS_ROBOT_RUNTIME_DIR`, so semi-auto runs can fail at save time even though the same values work when they come from `default.yaml`.

- [P2] Raise missing archived sample images instead of returning placeholders — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:396-402
  This API path is wired to translate `FileNotFoundError`/`ValueError` into a 404, but `get_sample_jpeg()` never raises for a missing row or missing file and returns a synthetic JPEG instead. In sessions created with `save_sample_images:=false`, after manual file cleanup, or for an invalid `row_index`, callers receive HTTP 200 with a fake image and cannot tell that the archived sample is actually unavailable.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-22-summary.md`

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
