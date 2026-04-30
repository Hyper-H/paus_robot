# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Start a new archive before manual capture after semi-auto runs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:332-339
  If an operator does a dry-run or semi-auto run and then switches to the manual `capture_sample`/`solve` flow without restarting the node, `self.session_dir` is still set by `_begin_new_semi_auto_session()`, so `_ensure_session_started()` reuses that old archive. The next manual samples append to the previous run’s `samples.jsonl` and can overwrite its `report.yaml`, which merges separate calibrations into one session history.

- [P2] Avoid reporting queued run requests as successful — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:454-460
  This marks `POST /api/handeye/run` as `success=true` before `/eye_to_hand/run_semi_auto_calibration` has actually replied. When the backend immediately rejects the run (for example missing trajectory, concurrent run, or another validation failure), the frontend’s `commandTone()` in `static/app.js` still shows a successful start until the background thread overwrites it, so callers get a false positive that calibration began.
2026-04-30T17:11:45.933771Z ERROR codex_core::session: failed to record rollout items: thread 019ddf59-92da-74a0-b1c7-f34f6f3c6e39 not found
The new UI/session workflow is close, but it still has correctness issues around session archival and run-start reporting. Those bugs can corrupt calibration history or briefly tell operators that a run started when the backend actually rejected it.

Full review comments:

- [P2] Start a new archive before manual capture after semi-auto runs — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:332-339
  If an operator does a dry-run or semi-auto run and then switches to the manual `capture_sample`/`solve` flow without restarting the node, `self.session_dir` is still set by `_begin_new_semi_auto_session()`, so `_ensure_session_started()` reuses that old archive. The next manual samples append to the previous run’s `samples.jsonl` and can overwrite its `report.yaml`, which merges separate calibrations into one session history.

- [P2] Avoid reporting queued run requests as successful — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:454-460
  This marks `POST /api/handeye/run` as `success=true` before `/eye_to_hand/run_semi_auto_calibration` has actually replied. When the backend immediately rejects the run (for example missing trajectory, concurrent run, or another validation failure), the frontend’s `commandTone()` in `static/app.js` still shows a successful start until the background thread overwrites it, so callers get a false positive that calibration began.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-17-summary.md`

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
