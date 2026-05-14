# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Stop expiring backend status after 10 seconds of inactivity — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:176-182
  The calibration node only publishes `/eye_to_hand/status` on state changes, so after 10 seconds of idle time `backend_connected` flips to false even though the node and its services are still alive. In that state the UI falls back to local config, shows the backend as disconnected, blocks the Dry-run button, and forces an extra motion-confirmation prompt until another status event happens.

- [P1] Make the new paus_ui tests importable from a source checkout — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/tests/test_operator_messages.py:1-1
  These new tests import `paus_ui` directly without adding `src/paus_ui` to `sys.path`, unlike the existing test modules in this repo. In a normal checkout run (`python3 -m pytest ...`) collection fails with `ModuleNotFoundError: paus_ui`, so `test_operator_messages.py`, `test_overlay.py`, `test_session_store.py`, and `test_session_store_shaping.py` never run unless the package was installed first.

- [P1] Avoid pulling ROS-only dependencies into the pure trajectory test — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/tests/test_semi_auto_calibration.py:24-24
  Importing `_wrapped_rotation_delta_norm_deg` from `eye_to_hand_calibration_node` drags in `ament_index_python`, `rclpy`, and the rest of the ROS node stack during test collection. In the lightweight Python test environment used here that raises `ModuleNotFoundError: ament_index_python` before any of the new semi-auto tests execute, so this helper needs to live in a ROS-free module or be duplicated in the test.
2026-04-30T08:31:00.873570Z ERROR codex_core::session: failed to record rollout items: thread 019ddd7f-238f-7be1-ad2f-61bb5ff146da not found
2026-04-30T08:31:00.883259Z ERROR codex_core::session: failed to record rollout items: thread 019ddd7f-236b-74d3-befd-cc659e9c75d9 not found
The UI backend-connection logic regresses to a false-disconnected state whenever the calibration node is idle, and the newly added test files do not collect successfully in the repo's current source-checkout test setup. Those issues make the patch unsafe to treat as correct.

Full review comments:

- [P1] Stop expiring backend status after 10 seconds of inactivity — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:176-182
  The calibration node only publishes `/eye_to_hand/status` on state changes, so after 10 seconds of idle time `backend_connected` flips to false even though the node and its services are still alive. In that state the UI falls back to local config, shows the backend as disconnected, blocks the Dry-run button, and forces an extra motion-confirmation prompt until another status event happens.

- [P1] Make the new paus_ui tests importable from a source checkout — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/tests/test_operator_messages.py:1-1
  These new tests import `paus_ui` directly without adding `src/paus_ui` to `sys.path`, unlike the existing test modules in this repo. In a normal checkout run (`python3 -m pytest ...`) collection fails with `ModuleNotFoundError: paus_ui`, so `test_operator_messages.py`, `test_overlay.py`, `test_session_store.py`, and `test_session_store_shaping.py` never run unless the package was installed first.

- [P1] Avoid pulling ROS-only dependencies into the pure trajectory test — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/tests/test_semi_auto_calibration.py:24-24
  Importing `_wrapped_rotation_delta_norm_deg` from `eye_to_hand_calibration_node` drags in `ament_index_python`, `rclpy`, and the rest of the ROS node stack during test collection. In the lightweight Python test environment used here that raises `ModuleNotFoundError: ament_index_python` before any of the new semi-auto tests execute, so this helper needs to live in a ROS-free module or be duplicated in the test.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-11-summary.md`

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
