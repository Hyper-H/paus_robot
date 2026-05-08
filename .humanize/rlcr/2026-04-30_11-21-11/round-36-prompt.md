# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Ignore stale backend status when only services are still reachable — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:188-191
  If `/eye_to_hand/status` stops updating but the service graph still reports the calibration node as reachable, this branch still reuses `last_status` because it only checks `backend_connected`. That means the 10s freshness cutoff is effectively ignored for `execute_motion`, `trajectory_path`, `session_root_path`, etc., so the UI can show stale run state and even stop requiring confirmation based on an old `execute_motion=false` payload. Please gate the payload and `motion_state_known` on `status_recent`, not just service availability.

- [P3] Treat non-mapping report YAML as invalid instead of empty — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:400-408
  When an archived `report.yaml` parses successfully but produces a scalar or list, `_read_yaml()` silently returns `{}` here. In that case the session is shown as a normal unsolved report (`has_report=true`, `is_invalid=false`) instead of being flagged as a corrupt archive, and `latest_valid_session_id()` can even prefer it when there is no solved report. Returning an error for non-dict payloads would keep report validation consistent with the new malformed-archive handling elsewhere in this patch.
2026-05-01T05:57:20.657103Z ERROR codex_core::session: failed to record rollout items: thread 019de211-7964-7683-9e60-afbef9e87684 not found
2026-05-01T05:57:20.667286Z ERROR codex_core::session: failed to record rollout items: thread 019de211-7950-7ed1-aa1a-a54041c0bfec not found
The new UI/session handling has at least one behavioral regression around stale backend status, and archive validation misses some malformed-but-parseable report files. These issues can lead to misleading UI state and incorrect session classification.

Full review comments:

- [P2] Ignore stale backend status when only services are still reachable — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:188-191
  If `/eye_to_hand/status` stops updating but the service graph still reports the calibration node as reachable, this branch still reuses `last_status` because it only checks `backend_connected`. That means the 10s freshness cutoff is effectively ignored for `execute_motion`, `trajectory_path`, `session_root_path`, etc., so the UI can show stale run state and even stop requiring confirmation based on an old `execute_motion=false` payload. Please gate the payload and `motion_state_known` on `status_recent`, not just service availability.

- [P3] Treat non-mapping report YAML as invalid instead of empty — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:400-408
  When an archived `report.yaml` parses successfully but produces a scalar or list, `_read_yaml()` silently returns `{}` here. In that case the session is shown as a normal unsolved report (`has_report=true`, `is_invalid=false`) instead of being flagged as a corrupt archive, and `latest_valid_session_id()` can even prefer it when there is no solved report. Returning an error for non-dict payloads would keep report validation consistent with the new malformed-archive handling elsewhere in this patch.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-36-summary.md`

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
