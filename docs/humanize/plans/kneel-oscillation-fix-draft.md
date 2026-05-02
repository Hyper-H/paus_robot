# Kneel Oscillation Fix Draft

## Context

The robot arm oscillates near the marker: it approaches, reaches or nearly reaches the marker, lifts away, and repeats. Logs show candidate_stage switching among pre_approach, reorient, and final_hover. A common sequence is control_executed at final_hover, followed by a later frame returning to pre_approach, often with tracking_target_lost and completion_reason=target_lost_timeout_before_reach.

The baseline must be the latest eye-to-hand semi-auto workflow at /home/chen_lab/worktrees/paus_robot_handeye on branch codex/eye-to-hand-semi-auto, not main. The fix must be implemented in a new isolated worktree and branch. Do not modify the existing semi-auto worktree.

## Desired Behavior

Stage progress should be monotonic during normal tracking: once tracking has reached reorient or final_hover, later visible frames must not drive execution back to pre_approach. Safety exceptions such as safe_lift must still be allowed to interrupt.

Per-frame geometry should continue to use the newest visible marker pose, so marker movement can still update candidate positions. The latch should constrain stage regression, not freeze the target pose while the marker remains visible.

When the marker is briefly lost, the node should keep the last valid target, final hover pose, surface normal, and stage progress. This treats short loss as likely occlusion by the approaching arm. If loss exceeds the hold timeout before completion, the node should report target lost and stop active progression rather than resetting to a high pre-approach command.

The existing repeat command threshold should remain a command de-duplication threshold. If needed, add a separate stage_switch_buffer_mm config value for geometry-stage transitions.

## Constraints

- Do not run real robot motion.
- Do not contact the lab host or execute FAIRINO movement commands.
- Preserve existing completion latch and tracking status behavior where possible.
- Do not revert unrelated or user-owned changes.
- Add focused non-hardware unit tests.

## Likely Files

- src/paus_motion_ros2/paus_motion_ros2/control_logic.py
- src/paus_motion_ros2/paus_motion_ros2/fairino_control_node.py
- src/paus_bringup/configs/default.yaml
- src/paus_motion_ros2/tests/

## Initial Test Ideas

- A visible sequence that reaches final_hover must not later produce executable pre_approach.
- A visible sequence that reaches reorient must not later regress to pre_approach.
- Short target loss keeps last valid target/final hover/stage progress.
- Long target loss reports target lost without issuing a reset-to-pre-approach command.
- safe_lift still overrides monotonic stage latching.

