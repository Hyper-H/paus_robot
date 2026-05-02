# BitLesson Knowledge Base

This file is project-specific. Keep entries precise and reusable for future rounds.

## Entry Template (Strict)

Use this exact field order for every entry:

## Lesson: <unique-id>
Lesson ID: <BL-YYYYMMDD-short-name>
Scope: <component/subsystem/files>
Problem Description: <specific failure mode with trigger conditions>
Root Cause: <direct technical cause>
Solution: <exact fix that resolved the problem>
Constraints: <limits, assumptions, non-goals>
Validation Evidence: <tests/commands/logs/PR evidence>
Source Rounds: <round numbers where problem appeared and was solved>

## Entries

## Lesson: Stage Latch for Monotonic Multi-Stage Approach
Lesson ID: BL-20260503-stage-latch-monotonic
Scope: paus_motion_ros2/control_logic.py, paus_motion_ros2/fairino_control_node.py
Problem Description: Robot arm oscillates between pre_approach/reorient/final_hover when approaching marker. Per-frame geometry-only stage computation has no temporal memory. When TCP is at final_hover position but orientation delta exceeds tolerance, the distance_to_pre_approach check triggers PRE_APPROACH since pre_approach is ~110mm away and stage_switch_buffer_mm was only 5mm.
Root Cause: Pure geometric stage decisions lack cross-frame memory. The elif branch checking distance_to_pre_approach catches cases that should go to REORIENT or FINAL_HOVER.
Solution: Added optional stage_latch parameter to build_approach_decision(). After geometric stage computation, if stage_latch is set and geometric stage would regress, candidate is recomputed at latched stage level using latest marker geometry. safe_lift always overrides latch. In node, stage_latch advances monotonically on execution, clears on safe_lift.
Constraints: Latch only constrains stage progression, not target pose. Marker position updates still flow through. Does not handle target-loss-driven regression separately.
Validation Evidence: 9 unit tests pass. 7 existing tests no regression. Baseline worktree zero diffs.
Source Rounds: 0
