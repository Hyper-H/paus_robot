# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Delay calibration node until camera.yaml exists — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:108-110
  When `ui.launch.py` is used on a clean machine or after `/tmp` has been cleared, `eye_to_hand_calibration_node` starts immediately here while `camera_bridge.py` is only started later. The node loads `camera_config_output` during `__init__`, so it can exit before the bridge has written `camera.yaml`; the default launch then fails unless a stale calibration file already happens to exist.

- [P2] Forward topic overrides to the calibration stack — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:115-123
  This launch file exposes `image_topic` and `status_topic`, but those overrides are not passed into `eye_to_hand_calibration_node` here, and `image_receiver_node` above is also left on its default topic. If an operator launches `ui.launch.py image_topic:=...` or `status_topic:=...`, the UI listens on the new topics while the recorder/calibration node stays on `/camera/image_bridge` and `/eye_to_hand/status`, so the dashboard appears disconnected.

- [P1] Start each semi-auto run with a fresh session — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:183-186
  Because `session_dir`, `report_path`, and `run_log_path` are initialized once at node startup, later `/eye_to_hand/run_semi_auto_calibration` calls on the same node reuse the previous run’s archive; `self.samples` is also kept alive across runs. After a failed attempt or a retry without restarting the node, the new solve can mix stale accepted samples into the result and the session browser no longer represents one calibration run per session.

- [P2] Default the UI to the live session instead of the last solved one — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:147-148
  If a previous solved session exists, opening the UI during a fresh calibration run sets `selectedSession` to `latest_valid_session_id` rather than the `session_id` from the live status. In that case the report and waypoint panes keep showing the old solved run until the operator manually changes the dropdown, which hides the current run’s skips and samples.
2026-04-30T05:41:57.819243Z ERROR codex_core::session: failed to record rollout items: thread 019ddce0-c622-7292-a631-1fda6b11ffc7 not found
The new UI/semi-auto workflow has multiple functional issues: fresh default launches can fail before `camera.yaml` exists, launch topic overrides are not wired through the backend, retries reuse stale calibration state, and the UI can default to the wrong session. These issues can break startup or cause the operator to see or save misleading calibration results.

Full review comments:

- [P1] Delay calibration node until camera.yaml exists — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:108-110
  When `ui.launch.py` is used on a clean machine or after `/tmp` has been cleared, `eye_to_hand_calibration_node` starts immediately here while `camera_bridge.py` is only started later. The node loads `camera_config_output` during `__init__`, so it can exit before the bridge has written `camera.yaml`; the default launch then fails unless a stale calibration file already happens to exist.

- [P2] Forward topic overrides to the calibration stack — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/launch/ui.launch.py:115-123
  This launch file exposes `image_topic` and `status_topic`, but those overrides are not passed into `eye_to_hand_calibration_node` here, and `image_receiver_node` above is also left on its default topic. If an operator launches `ui.launch.py image_topic:=...` or `status_topic:=...`, the UI listens on the new topics while the recorder/calibration node stays on `/camera/image_bridge` and `/eye_to_hand/status`, so the dashboard appears disconnected.

- [P1] Start each semi-auto run with a fresh session — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:183-186
  Because `session_dir`, `report_path`, and `run_log_path` are initialized once at node startup, later `/eye_to_hand/run_semi_auto_calibration` calls on the same node reuse the previous run’s archive; `self.samples` is also kept alive across runs. After a failed attempt or a retry without restarting the node, the new solve can mix stale accepted samples into the result and the session browser no longer represents one calibration run per session.

- [P2] Default the UI to the live session instead of the last solved one — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:147-148
  If a previous solved session exists, opening the UI during a fresh calibration run sets `selectedSession` to `latest_valid_session_id` rather than the `session_id` from the live status. In that case the report and waypoint panes keep showing the old solved run until the operator manually changes the dropdown, which hides the current run’s skips and samples.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-4-summary.md`

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
