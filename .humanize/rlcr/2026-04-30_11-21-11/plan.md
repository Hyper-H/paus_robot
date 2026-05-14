# Semi-Auto Eye-to-Hand UI Workbench Plan

## Goal Description

Upgrade the semi-auto eye-to-hand calibration UI from the current vertically stacked page into a single-screen calibration workbench close to the target reference UI. The operator should be able to open the UI from a Windows browser and, without page-level scrolling, see the live camera view, current waypoint state, motion controls, waypoint quality table, sample preview, and calibration result summary.

The implementation must keep the existing calibration algorithm and ROS2 node contracts. The UI should reorganize, enrich, and clarify the existing data from FastAPI, ROS2 status events, camera frames, trajectory YAML, session files, and report files.

Execution and verification must be planned around the lab Linux host. Local WSL is only the storage, editing, and static-analysis workspace. Any ROS2 launch, `colcon build`, camera bridge, FAIRINO SDK access, robot motion, and browser acceptance run must happen on `chen_lab@192.168.58.183:/home/chen_lab/worktrees/paus_robot_handeye`, using the `paus_robot` conda environment. The Windows operator should access the UI through `http://192.168.58.183:8080` after the lab host binds the UI server to `0.0.0.0`.

## Acceptance Criteria

Following TDD philosophy, each criterion includes positive and negative tests for deterministic verification.

- AC-1: Single-screen calibration workbench layout
  - Positive Tests (expected to PASS):
    - Opening `http://192.168.58.183:8080` shows the live image, current waypoint panel, control buttons, waypoint table, sample preview, and result summary in the first viewport on a desktop browser.
    - The body/page itself does not require vertical scrolling for the main calibration workflow; only table/log subpanels may scroll internally.
    - The left navigation remains visible and the hand-eye page is highlighted.
  - Negative Tests (expected to FAIL):
    - The waypoint table or sample preview can only be reached by scrolling far below the live image.
    - Large blank regions push critical controls or results off-screen.

- AC-2: Live image and overlay are useful for calibration decisions
  - Positive Tests (expected to PASS):
    - When the chessboard is detected, the live image displays corners, board outline, board coordinate axes, `reprojection_error_px`, `board_margin_px`, `image_sequence`, `T_camera_board`, and board angle.
    - Raw, Overlay, and Pose view modes update the main image.
    - A coordinate-axis toggle can hide/show board axes without changing the raw image endpoint.
  - Negative Tests (expected to FAIL):
    - A detected chessboard is shown without visible corners, outline, or pose axis.
    - Detection failure appears only as a raw Python exception or as an unexplained black panel.

- AC-3: Current waypoint state is clear and operational
  - Positive Tests (expected to PASS):
    - The current waypoint panel shows the waypoint name, workflow state, actual TCP pose, `T_camera_board`, and board angle.
    - During a run, the panel maps node status to a workflow progression: MoveJ, waiting stable, capture, detect, accepted/skipped, solve, finished.
    - Missing data is represented by a clear empty state, not silent `--` values when a reason is available.
  - Negative Tests (expected to FAIL):
    - The current waypoint remains blank while `/eye_to_hand/status` contains waypoint data.
    - The operator cannot tell whether the system is moving, waiting for stability, capturing, solving, or idle.

- AC-4: Motion controls are safe and understandable
  - Positive Tests (expected to PASS):
    - `execute_motion=false/true` is visible in the top status bar.
    - In `execute_motion=true`, starting calibration requires a confirmation dialog that states real robot motion, waypoint count, MoveJ mode, and speed/acceleration.
    - The stop control explains the current limitation if immediate cancellation is not supported by the backend node.
  - Negative Tests (expected to FAIL):
    - Real robot motion can be started without a UI confirmation.
    - The UI presents stop as guaranteed immediate emergency stop when the backend cannot guarantee it.

- AC-5: Waypoint table manages sampling quality
  - Positive Tests (expected to PASS):
    - The table shows index, waypoint name, status, `reprojection_error_px`, `board_margin_px`, `T_camera_board z`, capture flag, result, reason, and thumbnail.
    - Accepted, skipped, pending, and running states have distinct visual treatments.
    - Values that exceed `max_reprojection_error_px` or fall below `min_board_margin_px` are highlighted.
    - Clicking a waypoint row updates the sample preview and selected-row state.
  - Negative Tests (expected to FAIL):
    - Recorded waypoints remain only `pending` after accepted/skipped events arrive.
    - Failed waypoints show raw exception strings as the primary user-facing reason.

- AC-6: Sample preview supports review and diagnosis
  - Positive Tests (expected to PASS):
    - Selecting a waypoint with an image displays the corresponding sample preview.
    - Preview supports Raw, Overlay, and Pose modes.
    - Preview details include reprojection error, board margin, `T_camera_board`, board angle, image sequence, and capture time when available.
  - Negative Tests (expected to FAIL):
    - A waypoint without a sample image displays a blank black area instead of an explanatory empty state.
    - The preview remains stale after selecting a different row.

- AC-7: Calibration result summary makes quality easy to judge
  - Positive Tests (expected to PASS):
    - The UI selects the latest valid session with a report by default when available.
    - The result panel shows session path, report path, sample count, accepted, skipped, pending, and residual metrics.
    - Residual metrics include translation RMS/mean/max and rotation RMS/mean/max.
    - A compact threshold comparison is shown for current residuals versus configured target values.
  - Negative Tests (expected to FAIL):
    - An empty or unsolved session renders only `--` without explaining that no report exists.
    - Completed calibration results are not reflected until the page is manually reloaded.

- AC-8: Events, errors, and API behavior are reliable
  - Positive Tests (expected to PASS):
    - WebSocket `/ws/events` connects successfully; if it fails, the frontend falls back to polling.
    - `POST /api/handeye/run` does not return 422 for the UI request body.
    - User-facing errors are translated into operator language such as "棋盘未检测到", "没有收到新图像", "重投影误差过大", or "标定节点服务未连接".
    - Duplicate event spam is reduced in the visible log.
  - Negative Tests (expected to FAIL):
    - WebSocket failure causes the page to stop updating entirely.
    - The main UI displays full tracebacks or long `RuntimeError(...)` text as the primary message.

- AC-9: Lab-host execution boundary is explicit and preserved
  - Positive Tests (expected to PASS):
    - Build, launch, and UI acceptance instructions target `chen_lab@192.168.58.183:/home/chen_lab/worktrees/paus_robot_handeye`.
    - Commands that need Python, ROS2, `colcon`, camera, or robot SDK explicitly activate the `paus_robot` conda environment on the lab host.
    - UI access instructions use the Windows browser URL `http://192.168.58.183:8080` and assume the server binds `0.0.0.0`.
    - Local WSL instructions are limited to editing, git operations, static analysis, and non-ROS offline checks.
  - Negative Tests (expected to FAIL):
    - The plan or README tells the user to run ROS2 launch, camera bridge, FAIRINO SDK, or true robot validation in local WSL.
    - A test plan treats `localhost:8080` on Windows as equivalent to the lab host without SSH forwarding or a lab-host browser.

## Path Boundaries

### Upper Bound (Maximum Acceptable Scope)

The implementation may substantially refactor the `paus_ui` frontend and supporting FastAPI endpoints, including richer status aggregation, stronger session parsing, improved OpenCV overlay rendering, row-to-preview linking, friendly error translation, and compact result visualizations. It may add targeted tests for API shape, session parsing, frontend data shaping helpers, and overlay behavior on synthetic images.

### Lower Bound (Minimum Acceptable Scope)

The minimum acceptable implementation provides a single-screen desktop workbench, working overlay display, current waypoint details, safe control state, waypoint table status/quality coloring, sample preview linking, result summary from report files, and friendly empty/error states. It must fix interface-level failures such as 422 run requests and WebSocket-only update dependence.

### Allowed Choices

- Can use:
  - FastAPI backend routes already in `paus_ui`
  - Native HTML/CSS/JS
  - OpenCV backend overlay rendering
  - WebSocket with polling fallback
  - CSS Grid/Flexbox
  - Existing ROS2 services, topics, trajectory YAML, session files, and report files
- Cannot use:
  - React, Vite, or a new JavaScript build pipeline for this version
  - A rewritten calibration solver or new motion planner
  - Default real robot motion
  - Silent overwrites of calibration outputs
  - UI text that promises immediate stop if the backend cannot guarantee it
  - Local WSL for ROS2/camera/FAIRINO SDK execution or true UI acceptance against live robot data

## Feasibility Hints and Suggestions

### Conceptual Approach

One feasible implementation path is to keep the current FastAPI server and replace the page structure with a fixed-height dashboard:

```text
sidebar | top status bar
        | live image        | current waypoint + controls
        | waypoint table    | sample preview | result summary
```

The backend can continue serving JPEG endpoints for images and sample previews. The frontend can poll `/api/status` and `/api/handeye/quality` while also listening to `/ws/events`; when WebSocket fails, polling remains sufficient for status and table refresh.

Friendly error mapping can be implemented as a small helper shared by API responses and frontend rendering:

```text
"Chessboard was not detected" -> "棋盘未检测到"
"No fresh image arrived" -> "没有收到新图像"
"reprojection error ... exceeds" -> "重投影误差过大"
"Service is unavailable" -> "标定节点服务未连接"
```

### Relevant References

- `src/paus_ui/paus_ui/static/index.html` - current page structure
- `src/paus_ui/paus_ui/static/styles.css` - current layout and visual system
- `src/paus_ui/paus_ui/static/app.js` - current frontend polling, commands, and WebSocket handling
- `src/paus_ui/paus_ui/web_server.py` - FastAPI routes and WebSocket endpoint
- `src/paus_ui/paus_ui/ros_bridge.py` - ROS2 bridge, status cache, image endpoints, service calls
- `src/paus_ui/paus_ui/session_store.py` - session, report, sample, and waypoint parsing
- `src/paus_ui/paus_ui/overlay.py` - OpenCV chessboard overlay rendering
- `src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py` - status and run log event source
- `src/paus_bringup/launch/ui.launch.py` - UI launch parameters and defaults

## Dependencies and Sequence

### Milestones

0. Lab-host execution setup
   - Treat `/root/projects/paus_robot_eyehand_worktree` or local WSL as edit/static-analysis only.
   - Run build, ROS2 launch, camera bridge, FastAPI server, and browser acceptance from `/home/chen_lab/worktrees/paus_robot_handeye` on the lab host.
   - Use `conda activate paus_robot` before Python, ROS2, `colcon`, and UI commands on the lab host.
   - Access the UI from Windows with `http://192.168.58.183:8080`, not Windows `localhost`, unless SSH forwarding is explicitly configured.

1. Data reliability and API shaping
   - Verify all UI routes under a UI-only launch.
   - Ensure WebSocket works and add polling fallback in the browser.
   - Normalize status payloads for current waypoint, current session, progress, and command results.
   - Add friendly error translation.

2. Single-screen layout
   - Rebuild the HTML structure around fixed-height dashboard regions.
   - Move scrolling into waypoint and event panels only.
   - Add a top status bar for camera, node, motion, extrinsics, and user actions.

3. Live image and current waypoint
   - Strengthen overlay rendering and visible detection failure states.
   - Add current waypoint title, workflow progression, TCP table, `T_camera_board` table, and board angle.
   - Link quality updates to both the image overlay and the detail panel.

4. Waypoint table and sample preview
   - Add state colors, threshold highlighting, selected row, thumbnails, and row click behavior.
   - Add preview empty states and sample detail metrics.
   - Keep table statistics in sync with run events and session data.

5. Result summary and session behavior
   - Select the latest valid report by default.
   - Display residual metrics and threshold comparisons.
   - Show unsolved session state clearly.

6. Visual polish and acceptance pass
   - Reduce spacing and blank regions.
   - Align button hierarchy with risk level.
   - De-duplicate event log display.
   - Run API smoke tests and browser screenshot review.

## Task Breakdown

Each task includes exactly one routing tag.

| Task ID | Description | Target AC | Tag (`coding`/`analyze`) | Depends On |
|---------|-------------|-----------|----------------------------|------------|
| task1 | Audit current UI route behavior and document which endpoints provide each panel's data | AC-8 | analyze | - |
| task2 | Add/adjust backend response shaping for status, current waypoint, progress, sessions, and friendly errors | AC-3, AC-7, AC-8 | coding | task1 |
| task3 | Add frontend WebSocket fallback and remove visible duplicate event spam | AC-8 | coding | task2 |
| task4 | Refactor page markup into the target single-screen workbench regions | AC-1 | coding | task2 |
| task5 | Rework CSS for fixed viewport layout, internal scrolling, compact panels, and status chips | AC-1, AC-4 | coding | task4 |
| task6 | Improve live image overlay rendering and detection failure empty states | AC-2 | coding | task2 |
| task7 | Implement current waypoint panel with workflow progression, TCP table, `T_camera_board`, and board angle | AC-3 | coding | task2, task4 |
| task8 | Strengthen control behavior and confirmation copy for true motion | AC-4 | coding | task7 |
| task9 | Rebuild waypoint table rendering with state colors, threshold highlighting, selected row, and stats | AC-5 | coding | task2, task4 |
| task10 | Link waypoint selection to sample preview and add preview metrics/empty states | AC-6 | coding | task9 |
| task11 | Rework result summary to choose latest valid report and display residual comparisons | AC-7 | coding | task2, task4 |
| task12 | Run UI-only API smoke tests and browser screenshot comparison on the lab host against the reference target | AC-1, AC-2, AC-8, AC-9 | analyze | task3, task11 |
| task13 | Polish spacing, color hierarchy, button risk levels, and log presentation | AC-1, AC-4, AC-8 | coding | task12 |
| task14 | Update operator docs and launch notes so all ROS2/camera/robot/UI acceptance commands target the lab host, while WSL is documented as edit/static-analysis only | AC-9 | coding | task12 |

## Claude-Codex Deliberation

### Agreements

- The current UI already has the right broad modules, but the information architecture is not yet a calibration workbench.
- Data reliability and status clarity must be addressed before visual polish.
- The frontend should remain native HTML/CSS/JS for this iteration.
- The UI must never imply real robot motion is safe by default.
- The reference UI's strongest traits are single-screen layout, clear current waypoint state, visible quality metrics, and table-preview-result linkage.
- All ROS2, camera, FAIRINO SDK, and live UI acceptance work must run on the lab Linux host; local WSL is not a valid runtime for those checks.

### Resolved Disagreements

- Topic: Whether to prioritize visual layout or data plumbing first  
  Resolution: Start with data/API shaping and fallback behavior, then refactor layout. Layout without reliable data would make the page look closer to the target while still failing during calibration.

- Topic: Whether to introduce a frontend framework  
  Resolution: Keep native HTML/CSS/JS. The current scope is layout/data binding and does not justify a new build pipeline on the lab machine.

- Topic: Whether to make stop appear as an emergency stop  
  Resolution: Do not present it as an emergency stop unless backend support exists. Use honest operator copy and keep physical emergency stop as the real immediate safety mechanism.

### Convergence Status

- Final Status: `converged`

## Pending User Decisions

- DEC-1: Target residual thresholds shown in the result comparison
  - Claude Position: Use conservative default visual targets such as translation RMS 10 mm and rotation RMS 2 deg, but make them configurable later.
  - Codex Position: Show configurable thresholds from UI/default config if available; otherwise label defaults as "reference only".
  - Tradeoff Summary: Hardcoded targets make the UI immediately useful but can imply a quality standard not yet validated for this setup.
  - Decision Status: `PENDING`

- DEC-2: Whether to disable recording when chessboard is not detected
  - Claude Position: Disable by default to prevent bad waypoints.
  - Codex Position: Keep record available but warn strongly, because users may want to record safe motion waypoints that are not capture waypoints.
  - Tradeoff Summary: Disabling prevents accidental bad calibration data; allowing supports trajectory-only waypoint recording.
  - Decision Status: `PENDING`

- DEC-3: Whether to expose session deletion in this iteration
  - Claude Position: Include it in session management if convenient.
  - Codex Position: Defer deletion to avoid accidental loss of calibration evidence.
  - Tradeoff Summary: Cleanup is useful, but calibration sessions are audit artifacts and deletion should be designed carefully.
  - Decision Status: `PENDING`

## Implementation Notes

### Code Style Requirements

- Implementation code and comments must NOT contain plan-specific terminology such as "AC-", "Milestone", "Step", "Phase", or similar workflow markers.
- These terms are for plan documentation only, not for the resulting codebase.
- Use descriptive, domain-appropriate names in code, such as `renderWaypointTable`, `formatOperatorError`, `selectedSample`, `motionConfirmation`, and `qualityThresholds`.

### Operational Notes

- Build and test on the lab host from `/home/chen_lab/worktrees/paus_robot_handeye`.
- Use the `paus_robot` conda environment before Python, ROS2, or build commands.
- Do not run ROS2 launch files, camera bridge, FAIRINO SDK calls, or robot validation from local WSL. WSL is for code editing, git, static analysis, and offline file inspection.
- When testing from Windows, open `http://192.168.58.183:8080`; `localhost:8080` refers to the Windows machine unless SSH forwarding is configured.
- Do not run real robot motion unless explicitly launching with `execute_motion:=true` and confirming through the UI.
- Keep UI smoke tests capable of running with `start_image_receiver:=false`, `start_camera_bridge:=false`, and `start_calibration_node:=false`.
