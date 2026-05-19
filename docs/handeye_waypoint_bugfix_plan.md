# Hand-Eye Waypoint Workflow Bugfix Plan

## Goal Description

Fix the hand-eye semi-auto waypoint workflow reported from lab UI usage. Recorded waypoints must immediately show per-waypoint record quality, semi-auto calibration must skip failed chessboard captures instead of aborting the whole run, and the UI must let the operator select and delete any current-trajectory waypoint by name. The change must stay focused on current trajectory editing and semi-auto execution; live stream black/half-black frame optimization is deferred.

## Acceptance Criteria

- AC-1: Recorded waypoint quality is visible immediately
  - Positive: After `/api/handeye/record_waypoint`, `/api/handeye/waypoints` returns rows shaped from `record_quality.accepted/status/reject_reason/reprojection_error_px/board_margin_px`.
  - Positive: Accepted rows show accepted/OK and numeric quality; rejected rows show rejected/FAIL and a friendly reason such as chessboard not found.
  - Negative: Rows with `record_quality.accepted=false` are not left as pending just because they were manually recorded.

- AC-2: Semi-auto skips failed captures and continues
  - Positive: If a waypoint capture raises `SampleRejectedError`, the node logs/publishes the waypoint as skipped/rejected and continues to later waypoints.
  - Positive: Final solve still runs when enough accepted samples remain; if not enough samples remain, the final status clearly reports insufficient samples after completing the trajectory attempt.
  - Negative: A single `chessboard_not_found` at the first waypoint must not immediately stop the semi-auto run.

- AC-3: Delete selected current waypoint by name
  - Positive: Backend exposes a delete-by-name/current-trajectory service/API that removes the named waypoint and persists the edited trajectory.
  - Positive: UI row selection enables a shared delete-selected control and deletes the selected current waypoint, not just the last waypoint.
  - Negative: Archived session/history waypoint rows are not mutated by delete-selected.

- AC-4: Existing delete-last behavior and archive reconstruction remain compatible
  - Positive: Existing delete-last API still works.
  - Positive: Session/run-log reconstruction handles `waypoint_deleted` for selected deletes.
  - Negative: Deleting one waypoint must not renumber unrelated waypoint identities in archived history.

- AC-5: Focused tests/build checks pass on lab host
  - Positive: Relevant Python unit tests for `paus_ui` and `paus_marker_ros2` pass.
  - Positive: `compileall` passes for changed Python files and `node --check` passes if frontend JavaScript changes.

## Path Boundaries

Allowed changes:
- `src/paus_ui/paus_ui/ros_bridge.py`
- `src/paus_ui/paus_ui/web_server.py`
- `src/paus_ui/paus_ui/static/app.js`
- `src/paus_ui/paus_ui/static/index.html`
- `src/paus_ui/paus_ui/static/styles.css` if needed for the delete button state only
- `src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py`
- `src/paus_marker_ros2/paus_marker_ros2/semi_auto_calibration.py`
- Focused tests under `src/paus_ui/tests` and `src/paus_marker_ros2/tests`

Out of scope:
- Live stream black/half-black frame performance or camera bridge reconnect changes.
- Solver algorithm replacement.
- New frontend framework or large UI redesign.
- Mutating archived session directories from the delete-selected UI.

## Dependencies and Sequence

1. Data shaping for recorded waypoint quality
   - Inspect current `record_quality` schema and waypoint row shaping.
   - Fix row status/result/reason/quality fields to use `accepted/status/reject_reason/reprojection_*`.
   - Add unit tests for accepted and rejected recorded qualities.

2. Semi-auto skip-and-continue behavior
   - Inspect semi-auto capture loop and existing `SampleRejectedError` handling.
   - Convert per-waypoint capture rejection into skipped waypoint status/log event and continue.
   - Preserve final solve behavior based on accepted sample count.
   - Add tests for first-waypoint rejection followed by accepted later sample, plus insufficient accepted samples.

3. Delete selected waypoint by name
   - Add backend node/service helper to delete a named waypoint from the current recorded trajectory.
   - Add FastAPI/UI bridge route to call it.
   - Add frontend selected-row state and delete-selected button behavior, disabled when no current waypoint row is selected.
   - Ensure archived session selection does not expose current-trajectory mutation.

4. Validation and RLCR close
   - Run focused pytest/compile/node checks.
   - Commit changes and run RLCR stop gate until clean.

## Task Breakdown

| Task ID | Description | Target AC | Tag | Depends On |
|---------|-------------|-----------|-----|------------|
| task1 | Audit current waypoint shaping, semi-auto loop, delete service/API, and frontend row selection | AC-1, AC-2, AC-3 | analyze | - |
| task2 | Fix recorded waypoint quality shaping in UI bridge/API and add tests | AC-1 | coding | task1 |
| task3 | Implement skip-and-continue for per-waypoint capture rejection and add tests | AC-2, AC-5 | coding | task1 |
| task4 | Add delete-by-name backend/API support and tests | AC-3, AC-4 | coding | task1 |
| task5 | Add frontend selected-row delete-selected control and guarded current-trajectory behavior | AC-3 | coding | task4 |
| task6 | Run focused validation, commit, and complete RLCR review loop | AC-5 | analyze | task2, task3, task5 |

## Notes

The previous branch accidentally picked up a tracked `.humanize` directory from `dev`. This plan assumes `.humanize/` is ignored and not tracked in the implementation worktree.
