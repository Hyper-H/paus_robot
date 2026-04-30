# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Block manual capture while a semi-auto run is active — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:584-587
  If `/eye_to_hand/run_semi_auto_calibration` is in progress, this callback can still run concurrently because the node uses a `ReentrantCallbackGroup` and never checks `_semi_auto_active` here. In that case `_capture_one_sample()` calls `_ensure_session_started("manual")`, which sees the owner mismatch, creates a new session directory, and clears `self.samples`/`self.current_solution`, so the ongoing semi-auto run starts writing into the wrong archive and may fail its final solve/save. Please reject or serialize the manual calibration services while a semi-auto run owns the session state.

- [P2] Derive the node's default `output_path` from `config_path` — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:136-136
  `load_config()` now resolves `calibration.output_path` to a source/runtime-safe location, but this parameter is still declared with `bringup_share/configs/extrinsics.yaml`. Any caller that only sets `config_path` and leaves `output_path` unset (for example `ros2 run` or another launch file) will therefore ignore the YAML setting and keep writing into the package share tree, which is read-only in installed deployments and bypasses the new runtime-path logic. `output_path` needs to be initialized from the loaded calibration config just like `trajectory_path` and `session_root_path`.

- [P1] Avoid ROS-only imports at `test_ros_bridge` module scope — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/tests/test_ros_bridge.py:19-19
  In a plain source checkout without sourced ROS Python packages, this new test file now fails during collection because importing `paus_ui.ros_bridge` immediately requires `ament_index_python`. The rest of the added unit tests are written to run in that environment, so `python3 -m pytest src/paus_ui/tests/test_ros_bridge.py` currently aborts before executing any tests. If this test is meant to stay in the normal unit-test suite, the ROS-dependent import needs to be isolated or guarded.
2026-04-30T18:56:55.172946Z ERROR codex_core::session: failed to record rollout items: thread 019ddfb9-fcd0-7213-a07c-c5382e807368 not found
2026-04-30T18:56:55.228488Z ERROR codex_core::session: failed to record rollout items: thread 019ddfb9-fcaa-7e53-b757-1f829de0df71 not found
The patch introduces at least one runtime state-corruption path in the new semi-auto flow, and it also regresses a direct-node configuration path plus unit-test collection in a non-ROS source checkout. Those issues are enough to make the change unsafe to treat as correct.

Full review comments:

- [P1] Block manual capture while a semi-auto run is active — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:584-587
  If `/eye_to_hand/run_semi_auto_calibration` is in progress, this callback can still run concurrently because the node uses a `ReentrantCallbackGroup` and never checks `_semi_auto_active` here. In that case `_capture_one_sample()` calls `_ensure_session_started("manual")`, which sees the owner mismatch, creates a new session directory, and clears `self.samples`/`self.current_solution`, so the ongoing semi-auto run starts writing into the wrong archive and may fail its final solve/save. Please reject or serialize the manual calibration services while a semi-auto run owns the session state.

- [P2] Derive the node's default `output_path` from `config_path` — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:136-136
  `load_config()` now resolves `calibration.output_path` to a source/runtime-safe location, but this parameter is still declared with `bringup_share/configs/extrinsics.yaml`. Any caller that only sets `config_path` and leaves `output_path` unset (for example `ros2 run` or another launch file) will therefore ignore the YAML setting and keep writing into the package share tree, which is read-only in installed deployments and bypasses the new runtime-path logic. `output_path` needs to be initialized from the loaded calibration config just like `trajectory_path` and `session_root_path`.

- [P1] Avoid ROS-only imports at `test_ros_bridge` module scope — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/tests/test_ros_bridge.py:19-19
  In a plain source checkout without sourced ROS Python packages, this new test file now fails during collection because importing `paus_ui.ros_bridge` immediately requires `ament_index_python`. The rest of the added unit tests are written to run in that environment, so `python3 -m pytest src/paus_ui/tests/test_ros_bridge.py` currently aborts before executing any tests. If this test is meant to stay in the normal unit-test suite, the ROS-dependent import needs to be isolated or guarded.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-24-summary.md`

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
