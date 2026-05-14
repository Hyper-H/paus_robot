# Goal Tracker

<!--
This file tracks the ultimate goal, acceptance criteria, and plan evolution.
It prevents goal drift by maintaining a persistent anchor across all rounds.

RULES:
- IMMUTABLE SECTION: Do not modify after initialization
- MUTABLE SECTION: Update each round, but document all changes
- Every task must be in one of: Active, Completed, or Deferred
- Deferred items require explicit justification
-->

## IMMUTABLE SECTION
<!-- Do not modify after initialization -->

### Ultimate Goal

Upgrade the semi-auto eye-to-hand calibration UI from the current vertically stacked page into a single-screen calibration workbench close to the target reference UI. The operator should be able to open the UI from a Windows browser and, without page-level scrolling, see the live camera view, current waypoint state, motion controls, waypoint quality table, sample preview, and calibration result summary.

The implementation must keep the existing calibration algorithm and ROS2 node contracts. The UI should reorganize, enrich, and clarify the existing data from FastAPI, ROS2 status events, camera frames, trajectory YAML, session files, and report files.

Execution and verification must be planned around the lab Linux host. Local WSL is only the storage, editing, and static-analysis workspace. Any ROS2 launch, `colcon build`, camera bridge, FAIRINO SDK access, robot motion, and browser acceptance run must happen on `chen_lab@192.168.58.183:/home/chen_lab/worktrees/paus_robot_handeye`, using the `paus_robot` conda environment. The Windows operator should access the UI through `http://192.168.58.183:8080` after the lab host binds the UI server to `0.0.0.0`.

### Acceptance Criteria
<!-- Each criterion must be independently verifiable -->

- AC-1: Single-screen calibration workbench layout
  - Positive Tests:
    - Opening `http://192.168.58.183:8080` shows the live image, current waypoint panel, control buttons, waypoint table, sample preview, and result summary in the first viewport on a desktop browser.
    - The body/page itself does not require vertical scrolling for the main calibration workflow; only table/log subpanels may scroll internally.
    - The left navigation remains visible and the hand-eye page is highlighted.
  - Negative Tests:
    - The waypoint table or sample preview can only be reached by scrolling far below the live image.
    - Large blank regions push critical controls or results off-screen.

- AC-2: Live image and overlay are useful for calibration decisions
  - Positive Tests:
    - When the chessboard is detected, the live image displays corners, board outline, board coordinate axes, `reprojection_error_px`, `board_margin_px`, `image_sequence`, `T_camera_board`, and board angle.
    - Raw, Overlay, and Pose view modes update the main image.
    - A coordinate-axis toggle can hide/show board axes without changing the raw image endpoint.
  - Negative Tests:
    - A detected chessboard is shown without visible corners, outline, or pose axis.
    - Detection failure appears only as a raw Python exception or as an unexplained black panel.

- AC-3: Current waypoint state is clear and operational
  - Positive Tests:
    - The current waypoint panel shows the waypoint name, workflow state, actual TCP pose, `T_camera_board`, and board angle.
    - During a run, the panel maps node status to a workflow progression: MoveJ, waiting stable, capture, detect, accepted/skipped, solve, finished.
    - Missing data is represented by a clear empty state, not silent `--` values when a reason is available.
  - Negative Tests:
    - The current waypoint remains blank while `/eye_to_hand/status` contains waypoint data.
    - The operator cannot tell whether the system is moving, waiting for stability, capturing, solving, or idle.

- AC-4: Motion controls are safe and understandable
  - Positive Tests:
    - `execute_motion=false/true` is visible in the top status bar.
    - In `execute_motion=true`, starting calibration requires a confirmation dialog that states real robot motion, waypoint count, MoveJ mode, and speed/acceleration.
    - The stop control explains the current limitation if immediate cancellation is not supported by the backend node.
  - Negative Tests:
    - Real robot motion can be started without a UI confirmation.
    - The UI presents stop as guaranteed immediate emergency stop when the backend cannot guarantee it.

- AC-5: Waypoint table manages sampling quality
  - Positive Tests:
    - The table shows index, waypoint name, status, `reprojection_error_px`, `board_margin_px`, `T_camera_board z`, capture flag, result, reason, and thumbnail.
    - Accepted, skipped, pending, and running states have distinct visual treatments.
    - Values that exceed `max_reprojection_error_px` or fall below `min_board_margin_px` are highlighted.
    - Clicking a waypoint row updates the sample preview and selected-row state.
  - Negative Tests:
    - Recorded waypoints remain only `pending` after accepted/skipped events arrive.
    - Failed waypoints show raw exception strings as the primary user-facing reason.

- AC-6: Sample preview supports review and diagnosis
  - Positive Tests:
    - Selecting a waypoint with an image displays the corresponding sample preview.
    - Preview supports Raw, Overlay, and Pose modes.
    - Preview details include reprojection error, board margin, `T_camera_board`, board angle, image sequence, and capture time when available.
  - Negative Tests:
    - A waypoint without a sample image displays a blank black area instead of an explanatory empty state.
    - The preview remains stale after selecting a different row.

- AC-7: Calibration result summary makes quality easy to judge
  - Positive Tests:
    - The UI selects the latest valid session with a report by default when available.
    - The result panel shows session path, report path, sample count, accepted, skipped, pending, and residual metrics.
    - Residual metrics include translation RMS/mean/max and rotation RMS/mean/max.
    - A compact threshold comparison is shown for current residuals versus configured target values.
  - Negative Tests:
    - An empty or unsolved session renders only `--` without explaining that no report exists.
    - Completed calibration results are not reflected until the page is manually reloaded.

- AC-8: Events, errors, and API behavior are reliable
  - Positive Tests:
    - WebSocket `/ws/events` connects successfully; if it fails, the frontend falls back to polling.
    - `POST /api/handeye/run` does not return 422 for the UI request body.
    - User-facing errors are translated into operator language such as "棋盘未检测到", "没有收到新图像", "重投影误差过大", or "标定节点服务未连接".
    - Duplicate event spam is reduced in the visible log.
  - Negative Tests:
    - WebSocket failure causes the page to stop updating entirely.
    - The main UI displays full tracebacks or long `RuntimeError(...)` text as the primary message.

- AC-9: Lab-host execution boundary is explicit and preserved
  - Positive Tests:
    - Build, launch, and UI acceptance instructions target `chen_lab@192.168.58.183:/home/chen_lab/worktrees/paus_robot_handeye`.
    - Commands that need Python, ROS2, `colcon`, camera, or robot SDK explicitly activate the `paus_robot` conda environment on the lab host.
    - UI access instructions use the Windows browser URL `http://192.168.58.183:8080` and assume the server binds `0.0.0.0`.
    - Local WSL instructions are limited to editing, git operations, static analysis, and non-ROS offline checks.
  - Negative Tests:
    - The plan or README tells the user to run ROS2 launch, camera bridge, FAIRINO SDK, or true robot validation in local WSL.
    - A test plan treats `localhost:8080` on Windows as equivalent to the lab host without SSH forwarding or a lab-host browser.

---

## MUTABLE SECTION
<!-- Update each round with justification for changes -->

### Plan Version: 1 (Updated: Round 3)

#### Plan Evolution Log
<!-- Document any changes to the plan with justification -->
| Round | Change | Reason | Impact on AC |
|-------|--------|--------|--------------|
| 0 | Initial tracker populated from `docs/handeye_ui_improvement_plan_humanize.md` | Required RLCR initialization | Establishes AC and task mapping |
| 1 | Marked task1, task6, task8, task10, and task14 completed after direct Codex verification; kept the remaining completion claims pending and logged the blocking gaps discovered in review | Round 1 made real progress, but several completion claims were not fully delivered | Keeps AC-2, AC-4, AC-6, AC-8, and AC-9 progress accurate while AC-3, AC-5, AC-7, and part of AC-8 remain open |
| 2 | Marked task2, task3, task11, and task12 completed after direct Codex verification; kept task4, task5, task7, task9, and task13 pending because the current-waypoint panel is still clipped/misleading in the first viewport and the waypoint table still misses the required index column | Round 2 closed the Round 1 fallback, status-shaping, result-selection, and artifact gaps, but it did not fully satisfy the remaining layout and table requirements from the original plan | Advances AC-7, AC-8, and lab-host verification while AC-1, AC-3, and AC-5 remain partially blocked |
| 3 | Marked task4, task5, task7, task9, and task13 completed after direct Codex verification, and removed stale task2/task3 duplicates from Active Tasks | Round 3 resolved the remaining current-panel clipping/workflow-branching and waypoint-index gaps, and the tracker needed cleanup so Active Tasks matched the verified state | Closes the remaining AC-1, AC-3, and AC-5 blockers; tracker now reflects full plan completion |

#### Active Tasks
<!-- Map each task to its target Acceptance Criterion and routing tag -->
| Task | Target AC | Status | Tag | Owner | Notes |
|------|-----------|--------|-----|-------|-------|
| None | - | - | - | - | All planned tasks are completed and verified through Round 3 |

### Completed and Verified
<!-- Only move tasks here after Codex verification -->
| AC | Task | Completed Round | Verified Round | Evidence |
|----|------|-----------------|----------------|----------|
| AC-8 | task1: Audit current UI route behavior and document which endpoints provide each panel's data | 1 | 1 | `.humanize/skill/2026-04-30_11-59-52-4050280-1ed5b0bc/output.md` captures the route audit and missing-field analysis used in Round 1 |
| AC-3, AC-7, AC-8 | task2: Add/adjust backend response shaping for status, current waypoint, progress, sessions, and friendly errors | 2 | 2 | `src/paus_ui/paus_ui/ros_bridge.py` now shapes current waypoint quality fields and latest valid session metadata; the status smoke in `.humanize/rlcr/2026-04-30_11-21-11/artifacts/round-2/ui-smoke.log` shows `current_waypoint.empty_reason`, `quality_reason_code`, and motion summary on the lab host |
| AC-8 | task3: Add frontend WebSocket fallback and remove visible duplicate event spam | 2 | 2 | `src/paus_ui/paus_ui/static/app.js` now refreshes status/waypoints/report from `/api/events` polling and keeps a periodic refresh path while WebSocket is disconnected |
| AC-1 | task4: Refactor page markup into the target single-screen workbench regions | 3 | 3 | `src/paus_ui/paus_ui/static/index.html`, `src/paus_ui/paus_ui/static/styles.css`, and `.humanize/rlcr/2026-04-30_11-21-11/artifacts/round-3/paus_ui_round3_1600x1000.png` show the live view, current waypoint, controls, table, preview, result, and event panels within the first 1600x1000 viewport |
| AC-1, AC-4 | task5: Rework CSS for fixed viewport layout, internal scrolling, compact panels, and status chips | 3 | 3 | `src/paus_ui/paus_ui/static/styles.css` keeps page-level scrolling disabled, routes scrolling into table/log/detail subpanels, and compacts the workbench so the full current-waypoint content remains visible in the Round 3 screenshot |
| AC-2 | task6: Improve live image overlay rendering and detection failure empty states | 1 | 1 | `src/paus_ui/paus_ui/overlay.py` now separates `raw`/`overlay`/`pose`, supports the axes switch, and maps failures through `operator_messages.py`; `src/paus_ui/tests/test_overlay.py` covers the friendly-failure and raw-render behavior |
| AC-3 | task7: Implement current waypoint panel with workflow progression, TCP table, `T_camera_board`, and board angle | 3 | 3 | `src/paus_ui/paus_ui/static/index.html`, `src/paus_ui/paus_ui/static/app.js`, and the Round 3 screenshot show the workflow chips, TCP card, all six `T_camera_board` fields, and board-angle/empty-state text without clipping |
| AC-4 | task8: Strengthen control behavior and confirmation copy for true motion | 1 | 1 | `src/paus_ui/paus_ui/static/app.js` adds a true-motion confirmation dialog with waypoint count, motion mode, velocity, and acceleration; `src/paus_ui/paus_ui/ros_bridge.py` exposes motion summary and honest stop limitations |
| AC-5 | task9: Rebuild waypoint table rendering with state colors, threshold highlighting, selected row, and stats | 3 | 3 | `src/paus_ui/paus_ui/static/index.html` and `src/paus_ui/paus_ui/static/app.js` render the leading index column and full AC-5 schema, while `src/paus_ui/paus_ui/session_store.py` plus `src/paus_ui/tests/test_session_store_shaping.py` preserve stable ordered indices for session-backed rows |
| AC-6 | task10: Link waypoint selection to sample preview and add preview metrics/empty states | 1 | 1 | `src/paus_ui/paus_ui/static/app.js` selects rows, switches preview image modes, shows metrics, and renders explicit empty-state text when no sample image is available |
| AC-7 | task11: Rework result summary to choose latest valid report and display residual comparisons | 2 | 2 | `src/paus_ui/paus_ui/session_store.py` now prefers solved sessions via `has_solution`, and `src/paus_ui/paus_ui/static/app.js` renders an explicit unsolved-report state instead of blank residual cards |
| AC-1, AC-2, AC-8, AC-9 | task12: Run UI-only API smoke tests and browser screenshot comparison on the lab host against the reference target | 2 | 2 | `.humanize/rlcr/2026-04-30_11-21-11/artifacts/round-2/ui-smoke.log`, `paus_ui_round2_1600x1000.png`, and `latest-overlay.jpg` provide persistent lab-host evidence for the API smoke and first-viewport screenshot pass |
| AC-1, AC-4, AC-8 | task13: Polish spacing, color hierarchy, button risk levels, and log presentation | 3 | 3 | `src/paus_ui/paus_ui/static/styles.css`, `src/paus_ui/paus_ui/static/app.js`, and `.humanize/rlcr/2026-04-30_11-21-11/artifacts/round-3/paus_ui_round3_1600x1000.png` show compact workflow chips, distinct record/run/stop risk levels, and a cleaner event panel without the earlier wasted vertical space |
| AC-9 | task14: Update operator docs and launch notes so all ROS2/camera/robot/UI acceptance commands target the lab host, while WSL is documented as edit/static-analysis only | 1 | 1 | `README.md` documents lab-host login/build/launch flow, `conda activate paus_robot`, and Windows access via `http://192.168.58.183:8080` |

### Explicitly Deferred
<!-- Items here require strong justification -->
| Task | Original AC | Deferred Since | Justification | When to Reconsider |
|------|-------------|----------------|---------------|-------------------|
| None | - | - | - | - |

### Open Issues
<!-- Issues discovered during implementation -->
| Issue | Discovered Round | Blocking AC | Resolution Path |
|-------|-----------------|-------------|-----------------|
| None | - | - | - |
