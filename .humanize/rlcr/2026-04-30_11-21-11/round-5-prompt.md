# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Keep install defaults valid in install-only workspaces — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/configs/default.yaml:35-36
  When this config is loaded from an installed workspace (`.../install/paus_bringup/share/...`), `load_config()` now resolves these relative values to `<ws>/src/paus_bringup/...`. On machines that only have the installed artifacts, that source tree does not exist, so the semi-auto node cannot find `trajectory_path` and will try to write `extrinsics.yaml` into a non-existent checkout path. Please keep these defaults install-safe or resolve them relative to the package share instead of the inferred workspace root.

- [P1] Reject overlapping semi-auto runs inside the ROS service — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:837-846
  If two clients call `/eye_to_hand/run_semi_auto_calibration` while a run is already in progress (for example the CLI plus the UI, or a retry before the first call returns), both requests enter this callback because there is no node-level active-run guard. Each invocation resets `self.session_dir`/`self.samples` and drives the same `FairinoLinuxClient`, so logs from the two runs can be interleaved and overlapping `MoveJ` commands can be sent to the robot. The UI thread check does not protect direct service callers here.

- [P1] Measure TCP stability over the whole window, not step-to-step — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:824-830
  This logic only compares the current TCP pose to the immediately previous sample. A robot that is still drifting slowly can stay under the per-sample thresholds for `stable_window_s` and be accepted as "stable" even though it moved well beyond the tolerance across the whole window, which means semi-auto capture can happen before the arm has actually settled. The stability check needs to bound motion against a fixed reference or the max deviation seen during the window.

- [P2] Do not report `/api/handeye/run` as successful before it starts — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:316-319
  This endpoint returns `success=true` immediately after spawning a background thread, even if the ROS service is unavailable or the run fails validation right away (missing trajectory, concurrent run, etc.). The current frontend treats that response as a green success notice, so an operator can be told that calibration started when nothing actually began. If the call remains asynchronous, it should return an explicit queued/accepted state instead of `success=true`, or wait until the trigger service is known to have accepted the request.
2026-04-30T06:07:41.610790Z ERROR codex_core::session: failed to record rollout items: thread 019ddcfb-9cbf-7663-b8da-38c2234ddc13 not found
2026-04-30T06:07:41.620822Z ERROR codex_core::session: failed to record rollout items: thread 019ddcfb-9c9e-71d2-97a1-c1520c9007fe not found
The patch adds useful functionality, but it introduces several functional issues: install-only deployments now resolve default paths into a missing source tree, semi-auto runs are not serialized at the service layer, and the TCP stability gate can accept a robot that is still drifting. There is also an API-level false-success response in the UI bridge.

Full review comments:

- [P1] Keep install defaults valid in install-only workspaces — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_bringup/configs/default.yaml:35-36
  When this config is loaded from an installed workspace (`.../install/paus_bringup/share/...`), `load_config()` now resolves these relative values to `<ws>/src/paus_bringup/...`. On machines that only have the installed artifacts, that source tree does not exist, so the semi-auto node cannot find `trajectory_path` and will try to write `extrinsics.yaml` into a non-existent checkout path. Please keep these defaults install-safe or resolve them relative to the package share instead of the inferred workspace root.

- [P1] Reject overlapping semi-auto runs inside the ROS service — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:837-846
  If two clients call `/eye_to_hand/run_semi_auto_calibration` while a run is already in progress (for example the CLI plus the UI, or a retry before the first call returns), both requests enter this callback because there is no node-level active-run guard. Each invocation resets `self.session_dir`/`self.samples` and drives the same `FairinoLinuxClient`, so logs from the two runs can be interleaved and overlapping `MoveJ` commands can be sent to the robot. The UI thread check does not protect direct service callers here.

- [P1] Measure TCP stability over the whole window, not step-to-step — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:824-830
  This logic only compares the current TCP pose to the immediately previous sample. A robot that is still drifting slowly can stay under the per-sample thresholds for `stable_window_s` and be accepted as "stable" even though it moved well beyond the tolerance across the whole window, which means semi-auto capture can happen before the arm has actually settled. The stability check needs to bound motion against a fixed reference or the max deviation seen during the window.

- [P2] Do not report `/api/handeye/run` as successful before it starts — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:316-319
  This endpoint returns `success=true` immediately after spawning a background thread, even if the ROS service is unavailable or the run fails validation right away (missing trajectory, concurrent run, etc.). The current frontend treats that response as a green success notice, so an operator can be told that calibration started when nothing actually began. If the call remains asynchronous, it should return an explicit queued/accepted state instead of `success=true`, or wait until the trigger service is known to have accepted the request.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-5-summary.md`

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
