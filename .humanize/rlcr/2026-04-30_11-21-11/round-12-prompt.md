# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Route session_root_path out of install trees — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:209-212
  When this config is loaded from an installed workspace (for example under `/opt/.../install/...`), `output_path` and `trajectory_path` are redirected to a writable runtime directory but `session_root_path` is still resolved with `resolve_config_path(...)` to `<workspace>/calibration_sessions`. In a read-only install this makes the first waypoint-record or semi-auto run fail inside `create_session_dir(...)` with a permission error, so the new session archiving workflow does not work unless operators override the parameter manually.

- [P2] Treat stale latched status as a disconnected backend — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:181-183
  Because `/eye_to_hand/status` is subscribed with transient-local QoS, `last_status` stays non-`None` after the calibration node exits. `_sync_backend_state()` therefore keeps `backend_connected` true forever once a single status has been seen, even if all services have disappeared. In a crash/restart scenario the UI will still show the backend as connected and keep using the old `execute_motion`/path values until a new status arrives, although `/eye_to_hand/run_semi_auto_calibration` is already unavailable.
2026-04-30T09:04:20.570747Z ERROR codex_core::session: failed to record rollout items: thread 019ddd9c-af30-79d1-8ff2-4a9b2edf2500 not found
2026-04-30T09:04:20.624265Z ERROR codex_core::session: failed to record rollout items: thread 019ddd9c-af1e-7701-8e71-02ead187c691 not found
The patch introduces a deploy-time path resolution regression for `session_root_path`, and the new UI backend state logic can report a dead calibration node as connected indefinitely. Both issues affect real usage of the new semi-auto/session UI flow.

Full review comments:

- [P1] Route session_root_path out of install trees — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:209-212
  When this config is loaded from an installed workspace (for example under `/opt/.../install/...`), `output_path` and `trajectory_path` are redirected to a writable runtime directory but `session_root_path` is still resolved with `resolve_config_path(...)` to `<workspace>/calibration_sessions`. In a read-only install this makes the first waypoint-record or semi-auto run fail inside `create_session_dir(...)` with a permission error, so the new session archiving workflow does not work unless operators override the parameter manually.

- [P2] Treat stale latched status as a disconnected backend — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:181-183
  Because `/eye_to_hand/status` is subscribed with transient-local QoS, `last_status` stays non-`None` after the calibration node exits. `_sync_backend_state()` therefore keeps `backend_connected` true forever once a single status has been seen, even if all services have disappeared. In a crash/restart scenario the UI will still show the backend as connected and keep using the old `execute_motion`/path values until a new status arrives, although `/eye_to_hand/run_semi_auto_calibration` is already unavailable.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-12-summary.md`

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
