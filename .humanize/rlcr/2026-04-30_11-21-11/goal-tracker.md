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

### Plan Version: 1 (Updated: Round 1)

#### Plan Evolution Log
<!-- Document any changes to the plan with justification -->
| Round | Change | Reason | Impact on AC |
|-------|--------|--------|--------------|
| 0 | Initial tracker populated from `docs/handeye_ui_improvement_plan_humanize.md` | Required RLCR initialization | Establishes AC and task mapping |
| 1 | Marked task1, task6, task8, task10, and task14 completed after direct Codex verification; kept the remaining completion claims pending and logged the blocking gaps discovered in review | Round 1 made real progress, but several completion claims were not fully delivered | Keeps AC-2, AC-4, AC-6, AC-8, and AC-9 progress accurate while AC-3, AC-5, AC-7, and part of AC-8 remain open |

#### Active Tasks
<!-- Map each task to its target Acceptance Criterion and routing tag -->
| Task | Target AC | Status | Tag | Owner | Notes |
|------|-----------|--------|-----|-------|-------|
| task2: Add/adjust backend response shaping for status, current waypoint, progress, sessions, and friendly errors | AC-3, AC-7, AC-8 | pending | coding | claude | From plan task2 |
| task3: Add frontend WebSocket fallback and remove visible duplicate event spam | AC-8 | pending | coding | claude | From plan task3 |
| task4: Refactor page markup into the target single-screen workbench regions | AC-1 | pending | coding | claude | From plan task4 |
| task5: Rework CSS for fixed viewport layout, internal scrolling, compact panels, and status chips | AC-1, AC-4 | pending | coding | claude | From plan task5 |
| task7: Implement current waypoint panel with workflow progression, TCP table, `T_camera_board`, and board angle | AC-3 | pending | coding | claude | From plan task7 |
| task9: Rebuild waypoint table rendering with state colors, threshold highlighting, selected row, and stats | AC-5 | pending | coding | claude | From plan task9 |
| task11: Rework result summary to choose latest valid report and display residual comparisons | AC-7 | pending | coding | claude | From plan task11 |
| task12: Run UI-only API smoke tests and browser screenshot comparison on the lab host against the reference target | AC-1, AC-2, AC-8, AC-9 | pending | analyze | codex | From plan task12 |
| task13: Polish spacing, color hierarchy, button risk levels, and log presentation | AC-1, AC-4, AC-8 | pending | coding | claude | From plan task13 |

### Completed and Verified
<!-- Only move tasks here after Codex verification -->
| AC | Task | Completed Round | Verified Round | Evidence |
|----|------|-----------------|----------------|----------|
| AC-8 | task1: Audit current UI route behavior and document which endpoints provide each panel's data | 1 | 1 | `.humanize/skill/2026-04-30_11-59-52-4050280-1ed5b0bc/output.md` captures the route audit and missing-field analysis used in Round 1 |
| AC-2 | task6: Improve live image overlay rendering and detection failure empty states | 1 | 1 | `src/paus_ui/paus_ui/overlay.py` now separates `raw`/`overlay`/`pose`, supports the axes switch, and maps failures through `operator_messages.py`; `src/paus_ui/tests/test_overlay.py` covers the friendly-failure and raw-render behavior |
| AC-4 | task8: Strengthen control behavior and confirmation copy for true motion | 1 | 1 | `src/paus_ui/paus_ui/static/app.js` adds a true-motion confirmation dialog with waypoint count, motion mode, velocity, and acceleration; `src/paus_ui/paus_ui/ros_bridge.py` exposes motion summary and honest stop limitations |
| AC-6 | task10: Link waypoint selection to sample preview and add preview metrics/empty states | 1 | 1 | `src/paus_ui/paus_ui/static/app.js` selects rows, switches preview image modes, shows metrics, and renders explicit empty-state text when no sample image is available |
| AC-9 | task14: Update operator docs and launch notes so all ROS2/camera/robot/UI acceptance commands target the lab host, while WSL is documented as edit/static-analysis only | 1 | 1 | `README.md` documents lab-host login/build/launch flow, `conda activate paus_robot`, and Windows access via `http://192.168.58.183:8080` |

### Explicitly Deferred
<!-- Items here require strong justification -->
| Task | Original AC | Deferred Since | Justification | When to Reconsider |
|------|-------------|----------------|---------------|-------------------|

### Open Issues
<!-- Issues discovered during implementation -->
| Issue | Discovered Round | Blocking AC | Resolution Path |
|-------|-----------------|-------------|-----------------|
| Polling fallback is incomplete: when `/ws/events` is down, the UI still polls the event log but does not refresh waypoint or report data | 1 | AC-8 | Make the polling path trigger the same `refreshStatus`/`refreshWaypoints`/`refreshReport` updates that the WebSocket path performs, or add equivalent periodic fallback refreshes |
| The current-waypoint workflow display is incomplete and `/api/status` still omits `T_camera_board`, `board_angle`, and empty-state fields for the panel | 1 | AC-3 | Expand shaped status payloads and update the flow renderer to cover detect, skipped, solve, and finished states with explicit empty reasons |
| The waypoint table still omits the required `result` column even though backend rows already provide `result` | 1 | AC-5 | Add the `result` column to the table markup and row renderer, and keep its styling aligned with accepted/skipped/running states |
| Result-session selection still treats any `report.yaml` as a valid solved session and does not render an explicit unsolved-session state | 1 | AC-7 | Prefer `has_solution` when selecting the default session and render a clear unsolved/report-incomplete message instead of blank residual metrics |
