# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P2] Respect `config_path` when deriving semi-auto launch defaults — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/eye_to_hand_calibration.launch.py:161-170
  If an operator starts `eye_to_hand_calibration.launch.py` with `config_path:=/path/to/custom.yaml`, the new semi-auto defaults here (`trajectory_path`, `session_root_path`, quality/stability thresholds, `dwell_s`, etc.) are still baked from the repository's default YAML and then forwarded as explicit node parameters. In that scenario the calibration node silently ignores the values from the selected config file and can read/write the wrong trajectory/session locations or use the wrong acceptance thresholds.

- [P2] Load UI launch defaults from the selected config file — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:228-239
  `ui.launch.py` has the same regression: passing `config_path:=...` does not actually switch the UI/calibration profile unless every related launch arg is duplicated on the command line. These defaults are computed from `default_config_file` here, then `_launch_setup()` pushes them into both the calibration node and the web server as explicit overrides, so custom `trajectory_path`/`session_root_path`/board settings from the chosen config file are ignored.

- [P2] Start a fresh archive for each manual calibration run — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:327-332
  The documented manual flow (`capture_sample` → `solve` → `save`) now archives into `session_dir`, but `_ensure_session_started()` only creates that directory once and nothing clears it after a successful save. If the operator performs a second manual calibration without restarting the node, new samples are appended to the first run's `samples.jsonl` and its `report.yaml` is overwritten, so the session history no longer represents individual calibrations.
2026-04-30T09:31:26.484754Z ERROR codex_core::session: failed to record rollout items: thread 019dddb5-7ebd-7453-b219-bb179c0b060f not found
The patch introduces launch/config regressions for custom calibration profiles and a session-archiving bug in the existing manual calibration workflow. These issues can make runs use the wrong files/settings and merge separate calibrations into one archived session.

Full review comments:

- [P2] Respect `config_path` when deriving semi-auto launch defaults — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/eye_to_hand_calibration.launch.py:161-170
  If an operator starts `eye_to_hand_calibration.launch.py` with `config_path:=/path/to/custom.yaml`, the new semi-auto defaults here (`trajectory_path`, `session_root_path`, quality/stability thresholds, `dwell_s`, etc.) are still baked from the repository's default YAML and then forwarded as explicit node parameters. In that scenario the calibration node silently ignores the values from the selected config file and can read/write the wrong trajectory/session locations or use the wrong acceptance thresholds.

- [P2] Load UI launch defaults from the selected config file — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:228-239
  `ui.launch.py` has the same regression: passing `config_path:=...` does not actually switch the UI/calibration profile unless every related launch arg is duplicated on the command line. These defaults are computed from `default_config_file` here, then `_launch_setup()` pushes them into both the calibration node and the web server as explicit overrides, so custom `trajectory_path`/`session_root_path`/board settings from the chosen config file are ignored.

- [P2] Start a fresh archive for each manual calibration run — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:327-332
  The documented manual flow (`capture_sample` → `solve` → `save`) now archives into `session_dir`, but `_ensure_session_started()` only creates that directory once and nothing clears it after a successful save. If the operator performs a second manual calibration without restarting the node, new samples are appended to the first run's `samples.jsonl` and its `report.yaml` is overwritten, so the session history no longer represents individual calibrations.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-14-summary.md`

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
