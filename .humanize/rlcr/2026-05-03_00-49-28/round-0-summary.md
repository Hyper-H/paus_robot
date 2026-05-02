# Round 0 Summary

## What was implemented

Stage latch to prevent kneel oscillation — added monotonic stage progression to the robot arm approach pipeline. Once the arm reaches reorient or final_hover, geometric candidate calculations can no longer regress the stage back to pre_approach during normal tracking. The safe_lift safety exception still overrides the latch.

Root cause: build_approach_decision() computed stage purely from geometry each frame. When TCP was near final_hover position but orientation was slightly off, the second elif branch (distance_to_pre_approach > stage_switch_buffer_mm) would trigger PRE_APPROACH because pre_approach is ~110mm away while stage_switch_buffer_mm was only 5mm — making the condition always true near the marker.

## Files created/modified

| File | Change |
|------|--------|
| src/paus_motion_ros2/paus_motion_ros2/control_logic.py | +38 lines: STAGE_ORDER dict, stage_latch parameter, latch application after geometric stage computation |
| src/paus_motion_ros2/paus_motion_ros2/fairino_control_node.py | +14 lines: stage_switch_buffer_mm config, stage_latch tracking, pass to decision, update on execution |
| src/paus_bringup/configs/default.yaml | +1 line: stage_switch_buffer_mm: 10.0 |
| src/paus_motion_ros2/tests/test_stage_latch_oscillation.py | New file: 9 unit tests |
| .humanize/rlcr/2026-05-03_00-49-28/goal-tracker.md | Initialized: 8 ACs, 6 tasks completed |
| .humanize/bitlesson.md | Added BL-20260503-stage-latch-monotonic |

## Tests added/passed

- New: 9/9 pass (StageLatchPreventsRegressionTests x3, StageLatchGeometryUpdateTests x1, SafeLiftOverrideTests x1, StageLatchSequenceTests x2, StageSwitchBufferTests x2)
- Existing: 7/7 pass (no regressions)
- Baseline worktree: /home/chen_lab/worktrees/paus_robot_handeye has zero diffs from our changes

## Remaining items

1. Real-robot validation not performed (per AC-8 constraint)
2. stage_switch_buffer_mm: 10.0 default may need tuning on physical hardware
3. ROS 2 node integration test (mock node with full callback interaction) not implemented

## BitLesson Delta

- Action: add
- Lesson ID(s): BL-20260503-stage-latch-monotonic
- Notes: Multi-stage geometric approach pipelines need temporal state (stage_latch) to prevent regression when intermediate geometric conditions are ambiguous. Without a latch, pure per-frame geometry can oscillate because proximity checks use distance to a reference point rather than region membership. Fix pattern: add optional latch parameter to the pure-function decision engine; track latch in the stateful caller; allow safety overrides to bypass latch; keep latest-geometry position updates flowing through the latched path.
