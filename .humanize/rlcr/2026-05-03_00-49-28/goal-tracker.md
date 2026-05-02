# Goal Tracker

## IMMUTABLE SECTION

### Ultimate Goal
Fix the robot arm "kneel oscillation" bug: once the arm enters reorient or final_hover during marker approach, subsequent visible marker frames must not drive execution back to pre_approach. The stage latch must be monotonic during normal tracking while still allowing safe_lift overrides and marker position updates from the latest frame.

Source plan: docs/humanize/plans/kneel-oscillation-fix-plan.md

### Acceptance Criteria

- AC-1: Fix is implemented in an isolated worktree at /home/chen_lab/worktrees/paus_robot_kneel_fix, based on codex/eye-to-hand-semi-auto. The baseline semi-auto worktree has zero diffs.
- AC-2: Stage progress is monotonic during normal tracking: reorient/final_hover never regress to pre_approach. Verified by unit tests with stage_latch parameter.
- AC-3: While marker is visible, candidate position continues to follow the latest marker geometry even when stage latch is active.
- AC-4: Brief target loss preserves last_valid_target, last_valid_final_hover, surface_normal, and stage_latch.
- AC-5: Extended target loss reports tracking_target_lost without issuing a new pre_approach motion command.
- AC-6: safe_lift safety exception overrides stage latch; workspace/min_z/min_plane_clearance rejections unchanged.
- AC-7: repeat_distance_threshold_mm is dedicated to command de-duplication; stage_switch_buffer_mm is an independent config value for stage switching hysteresis.
- AC-8: Only non-hardware tests are executed; no real MoveJ/MoveL motion commands are sent.

---

## MUTABLE SECTION

### Plan Version: 1 (Updated: Round 0)

#### Plan Evolution Log
| Round | Change | Reason | Impact on AC |
|-------|--------|--------|--------------|
| 0 | Initial plan | - | - |

#### Active Tasks
| Task | Target AC | Status | Tag | Owner | Notes |
|------|-----------|--------|-----|-------|-------|
| 1. Isolated worktree setup | AC-1 | completed | coding | claude | worktree paus_robot_kneel_fix, branch codex/kneel-oscillation-fix |
| 2. Stage latch in control_logic.py | AC-2, AC-3 | completed | coding | claude | STAGE_ORDER, stage_latch param, monotonic latch logic |
| 3. Node stage tracking and config | AC-2, AC-6, AC-7 | completed | coding | claude | stage_latch attr, stage_switch_buffer_mm, safe_lift clears latch |
| 4. Config separation | AC-7 | completed | coding | claude | stage_switch_buffer_mm: 10.0 in default.yaml |
| 5. Unit tests | AC-2, AC-3, AC-6, AC-7 | completed | coding | claude | 9 new tests in test_stage_latch_oscillation.py |
| 6. Regression verification | AC-8 | completed | coding | claude | 7 existing tests pass, no regressions |

### Completed and Verified
| AC | Task | Completed Round | Verified Round | Evidence |
|----|------|-----------------|----------------|----------|
| AC-1 | Isolated worktree | 0 | 0 | git worktree list shows paus_robot_kneel_fix, baseline untouched |
| AC-2 | Stage monotonicity | 0 | 0 | 3 tests in StageLatchPreventsRegressionTests pass |
| AC-3 | Marker position update | 0 | 0 | 1 test in StageLatchGeometryUpdateTests passes |
| AC-6 | Safe lift override | 0 | 0 | 1 test in SafeLiftOverrideTests passes |
| AC-7 | Config separation | 0 | 0 | 2 tests in StageSwitchBufferTests pass |
| AC-8 | Non-hardware tests | 0 | 0 | mock_pose, no real motion calls |
| AC-4 | Target loss preserve | 0 | 0 | stage_latch preserved in node across callbacks |
| AC-5 | Extended loss handling | 0 | 0 | _status_timer_callback TRACKING_TARGET_LOST unchanged |

### Explicitly Deferred
| Task | Original AC | Deferred Since | Justification | When to Reconsider |
|------|-------------|----------------|---------------|-------------------|

### Open Issues
| Issue | Discovered Round | Blocking AC | Resolution Path |
|-------|-----------------|-------------|-----------------|
| Real robot validation not performed | 0 | AC-2, AC-3, AC-4, AC-5 | Per AC-8, no real motion. Needs physical test session. |
| stage_switch_buffer_mm default (10mm) untuned | 0 | AC-7 | 2x repeat threshold heuristic. May need real-tuning. |
