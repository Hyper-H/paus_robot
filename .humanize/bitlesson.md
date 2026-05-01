# BitLesson Knowledge Base

This file is project-specific. Keep entries precise and reusable for future rounds.

## Entry Template (Strict)

Use this exact field order for every entry:

```markdown
## Lesson: <unique-id>
Lesson ID: <BL-YYYYMMDD-short-name>
Scope: <component/subsystem/files>
Problem Description: <specific failure mode with trigger conditions>
Root Cause: <direct technical cause>
Solution: <exact fix that resolved the problem>
Constraints: <limits, assumptions, non-goals>
Validation Evidence: <tests/commands/logs/PR evidence>
Source Rounds: <round numbers where problem appeared and was solved>
```

## Entries

## Lesson: handeye-ui-archived-default-board-angle
Lesson ID: BL-20260501-handeye-ui-archived-default-board-angle
Scope: src/paus_ui/paus_ui/static/app.js; src/paus_ui/paus_ui/session_store.py; src/paus_ui/tests/test_session_store.py
Problem Description: On a page load with only archived handeye sessions available and no live run, the UI stayed on "current trajectory" until the operator manually chose a session. Sample preview details also omitted board-angle, so archived samples lost a useful visual quality cue.
Root Cause: Session selection only auto-followed live runs, archived preview rendering depended on a field that was never propagated from stored sample data, and later UI state used the same flag for explicit archived selection and returning to the current trajectory option.
Solution: Auto-select the latest archived report session when there is no live run and no user-selected/current-trajectory selection, propagate `board_angle_deg` from sample storage or compute it from `camera_to_board_matrix`, render it in the saved-sample preview panel, keep blank "current trajectory" selection eligible for live auto-follow, and avoid overwriting archived session metadata with live backend paths.
Constraints: Do not override a user-selected archived session or explicit current-trajectory selection. Keep the change compatible with existing live-run auto-follow behavior, and surface invalid archive `empty_reason`/parse errors instead of generic unsolved copy.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `node --check src/paus_ui/paus_ui/static/app.js`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_path_resolvers.py src/paus_ui/tests/test_ros_bridge.py` (`29 passed`); `colcon build --packages-select paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`49 passed`); round 45 `node --check src/paus_ui/paus_ui/static/app.js`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_ros_bridge.py` (`19 passed`); full regression (`54 passed`)
Source Rounds: 41, 45

## Lesson: handeye-archive-integrity
Lesson ID: BL-20260501-handeye-archive-integrity
Scope: src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py; src/paus_ui/paus_ui/session_store.py; src/paus_marker_ros2/tests/test_eye_to_hand_session.py; src/paus_ui/tests/test_session_store.py
Problem Description: Semi-auto validation failures created empty archive sessions, and deleted recorded waypoints could still leak into the archived session view as pending entries.
Root Cause: The semi-auto flow opened a new session before trajectory validation completed, and delete events were written with an unnormalized payload that SessionStore could not recognize as a deletion marker.
Solution: Validate the trajectory before starting a semi-auto session, suppress run-log writes when the run never starts, emit `waypoint_deleted` with `waypoint_name` and `waypoint` fields, and teach SessionStore to drop deleted waypoints from archived reconstruction.
Constraints: Do not create archive directories for missing or malformed trajectories. Keep the remaining archived waypoints stable and preserve the existing capture/skip rendering for non-deleted items.
Validation Evidence: `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_session_store.py` (`17 passed`); `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`51 passed`)
Source Rounds: 42

## Lesson: handeye-empty-preview-active-count
Lesson ID: BL-20260501-handeye-empty-preview-active-count
Scope: src/paus_ui/paus_ui/static/app.js; src/paus_ui/paus_ui/session_store.py; src/paus_ui/tests/test_session_store.py
Problem Description: Switching to an empty waypoint list left the previous sample preview visible, and run.log-only sessions could still overcount pending waypoints after deletions.
Root Cause: The preview renderer was never cleared when `state.waypoints` became empty, and session totals counted every waypoint name in the log without treating `waypoint_deleted` as a removal.
Solution: Clear the preview panel when no waypoints are available, and count only active waypoint names by discarding deletions from the event-derived set.
Constraints: Keep the empty-state message explicit instead of showing stale sample details. Do not change accepted/skipped rendering for non-empty sessions.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `node --check src/paus_ui/paus_ui/static/app.js`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_path_resolvers.py src/paus_ui/tests/test_ros_bridge.py` (`31 passed`); `colcon build --packages-select paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`52 passed`)
Source Rounds: 43

## Lesson: handeye-manual-edit-archive
Lesson ID: BL-20260501-handeye-manual-edit-archive
Scope: src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py; src/paus_marker_ros2/tests/test_eye_to_hand_session.py
Problem Description: Manual waypoint record/delete edits made before any capture or semi-auto run were silently dropped because no archive session had been started yet.
Root Cause: The manual edit callbacks only wrote to `run.log` when `run_log_path` already existed, so the first edit in a fresh session had nowhere to persist.
Solution: Start a manual session before logging successful waypoint edits, so record-only or edit-only sessions are archived immediately and can be reconstructed after restart.
Constraints: Keep the semi-auto rejection guard intact. Only start the manual archive session after the edit succeeds, not before validation or robot reads.
Validation Evidence: `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/tests/test_session_store.py` (`38 passed`); `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`54 passed`)
Source Rounds: 44

## Lesson: handeye-ui-service-timeout
Lesson ID: BL-20260501-handeye-ui-service-timeout
Scope: src/paus_ui/paus_ui/ros_bridge.py; src/paus_ui/tests/test_ros_bridge.py
Problem Description: UI command calls could report service unavailability during normal startup or restart races even when the caller asked to wait longer.
Root Cause: The ROS service wait path truncated every timeout to two seconds before calling `wait_for_service()`.
Solution: Pass the caller's full timeout through to the ROS service wait so long startup windows are honored.
Constraints: Preserve the existing command result and event logging behavior. Do not change the user-facing timeout semantics beyond honoring the requested wait.
Validation Evidence: `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_ros_bridge.py src/paus_ui/tests/test_session_store.py` (`38 passed`); `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`54 passed`)
Source Rounds: 44

## Lesson: handeye-session-workflow-completeness
Lesson ID: BL-20260501-handeye-session-workflow-completeness
Scope: src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py; src/paus_marker_ros2/tests/test_eye_to_hand_session.py; src/paus_ui/paus_ui/ros_bridge.py; src/paus_ui/tests/test_ros_bridge.py
Problem Description: Manual trajectory archives were not self-contained after save, and the UI workflow bar stayed blank during the initial `semi_auto_started` phase before the first waypoint event.
Root Cause: The manual save path wrote only the shared trajectory YAML and did not copy it into the active session, while the UI workflow status map had no entry for `semi_auto_started`.
Solution: Copy the saved manual trajectory into `trajectory_used.yaml` inside the active manual session, and map `semi_auto_started` to the `movej` workflow stage with an explicit startup label.
Constraints: Keep manual session archives consistent with semi-auto archives. Do not add a new workflow step; reuse the existing first motion stage for startup feedback.
Validation Evidence: `python3 -m compileall -q src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py`; `/usr/bin/python3 -m pytest -q src/paus_marker_ros2/tests/test_eye_to_hand_session.py src/paus_ui/tests/test_ros_bridge.py` (`28 passed`); `colcon build --packages-select paus_marker_ros2 paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`56 passed`)
Source Rounds: 46

## Lesson: handeye-ui-backend-state-preservation
Lesson ID: BL-20260501-handeye-ui-backend-state-preservation
Scope: src/paus_ui/paus_ui/ros_bridge.py; src/paus_ui/tests/test_ros_bridge.py
Problem Description: After the last backend status aged past the freshness window, the UI hid unsaved recorded waypoints and reverted backend trajectory/session/threshold overrides to local defaults even while ROS services were still available.
Root Cause: `get_waypoints()` only trusted `recorded_trajectory` while status was recent or a run was active, and `_sync_backend_state()` recomputed effective config from local defaults whenever the live payload was empty.
Solution: Keep dirty in-memory `recorded_trajectory` available regardless of status age, preserve the last effective backend trajectory/session/threshold settings while the backend remains connected, and fall back to local defaults only when backend services are gone.
Constraints: Do not keep stale clean recordings ahead of the saved trajectory YAML. Preserve the existing disconnected fallback and session-store rebuild behavior.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_ros_bridge.py` (`22 passed`); `colcon build --packages-select paus_ui --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`58 passed`)
Source Rounds: 47

## Lesson: handeye-session-edited-trajectory-consistency
Lesson ID: BL-20260501-handeye-session-edited-trajectory-consistency
Scope: src/paus_ui/paus_ui/session_store.py; src/paus_ui/tests/test_session_store.py; src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py; src/paus_marker_ros2/tests/test_eye_to_hand_session.py
Problem Description: Archived session pending counts could disagree with the edited waypoint table, and deleting the last waypoint after a save left the stale live trajectory YAML on disk.
Root Cause: Session summaries counted the saved `trajectory_used.yaml` length instead of the run-log-replayed waypoint set, while the trajectory save service rejected empty recordings before it could clear the stale live file.
Solution: Compute pending totals from `read_session_waypoints()` so run-log edits are replayed consistently, and let an empty manual save remove the existing live `trajectory_path` while publishing a clean dirty state.
Constraints: Keep clean empty saves without an existing file rejected. Do not delete archived `trajectory_used.yaml`; archive reconstruction still relies on it plus run-log delete events.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`22 passed`); `colcon build --packages-select paus_ui paus_marker_ros2 --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`60 passed`)
Source Rounds: 48

## Lesson: handeye-ui-confirmed-boolean
Lesson ID: BL-20260501-handeye-ui-confirmed-boolean
Scope: src/paus_ui/paus_ui/web_server.py; src/paus_ui/tests/test_web_server.py
Problem Description: The `/api/handeye/run` endpoint treated any non-empty `confirmed` value as truthy, so API callers could accidentally bypass or trigger the motion-confirmation gate by sending strings like `"false"`.
Root Cause: The request handler coerced `body["confirmed"]` with Python `bool(...)` instead of requiring a JSON boolean type.
Solution: Parse `confirmed` through a dedicated validator that accepts only real JSON booleans, returns 422 for non-boolean values, and leaves the missing-field default at `False`.
Constraints: Keep the existing API shape and dry-run behavior. Reject malformed request bodies instead of coercing them.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/web_server.py src/paus_ui/tests/test_web_server.py`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_web_server.py src/paus_ui/tests/test_ros_bridge.py` (`23 passed`); `colcon build --packages-select paus_ui paus_marker_ros2 --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`61 passed`)
Source Rounds: 49

## Lesson: handeye-ui-disconnect-and-trajectory-error-visibility
Lesson ID: BL-20260501-handeye-ui-disconnect-and-trajectory-error-visibility
Scope: src/paus_ui/paus_ui/ros_bridge.py; src/paus_ui/tests/test_ros_bridge.py; src/paus_ui/paus_ui/static/app.js
Problem Description: After the calibration backend disappeared, the UI could retain backend-derived motion and overlay detector settings. Separately, malformed or unreadable current trajectory YAML was rendered as an empty waypoint table.
Root Cause: The disconnected fallback reset only trajectory/session/threshold fields, while `execute_motion`, `camera_config_path`, `board_rows`, `board_cols`, and `square_size_m` stayed on the last backend values. The waypoint refresh path ignored `/api/handeye/waypoints` `error` payloads.
Solution: Store the UI-local detector and motion defaults during bridge initialization, restore them when backend services/status disappear, reset the detector cache on fallback changes, extend the disconnect test to cover those fields, and render current-trajectory load errors directly in the waypoint table and preview panel.
Constraints: Preserve backend-derived overrides while services remain connected. Keep archived session waypoint rendering unchanged and only show the trajectory load error in the current-trajectory view.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py`; `node --check src/paus_ui/paus_ui/static/app.js`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_ros_bridge.py` (`22 passed`); `colcon build --packages-select paus_ui paus_marker_ros2 --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`61 passed`)
Source Rounds: 50

## Lesson: handeye-ui-selection-and-web-import-laziness
Lesson ID: BL-20260501-handeye-ui-selection-and-web-import-laziness
Scope: src/paus_ui/paus_ui/static/app.js; src/paus_ui/paus_ui/web_server.py; src/paus_ui/tests/test_web_server.py
Problem Description: The UI could auto-follow a live session even after the operator explicitly chose the blank current-trajectory view, and importing the web server module in a plain Python environment failed before helper-only tests could run because ROS modules were loaded at import time.
Root Cause: The live-status auto-follow branch only checked `userSelectedSession`, not the explicit current-trajectory selection flag, and `rclpy`/`UiRosBridge` were imported at module scope even though only `main()` needed them.
Solution: Gate the live auto-follow branch on `!state.currentTrajectorySelected` as well, move ROS imports into `main()`, and make the bridge type annotation import-time safe with a `TYPE_CHECKING` guard.
Constraints: Keep archived-session auto-follow intact. Preserve `create_app()` and `_parse_confirmed_flag()` as plain-Python-importable helpers without requiring a ROS overlay.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/web_server.py src/paus_ui/tests/test_web_server.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py`; `node --check src/paus_ui/paus_ui/static/app.js`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_web_server.py src/paus_ui/tests/test_ros_bridge.py` (`23 passed`); `colcon build --packages-select paus_ui paus_marker_ros2 --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`61 passed`)
Source Rounds: 51

## Lesson: handeye-legacy-session-compatibility-and-archive-path
Lesson ID: BL-20260501-handeye-legacy-session-compatibility-and-archive-path
Scope: src/paus_ui/paus_ui/session_store.py; src/paus_ui/tests/test_session_store.py; src/paus_ui/paus_ui/static/app.js
Problem Description: Legacy manual archives created by the older capture flow still logged `sample_captured`, which rendered accepted samples as pending in the new UI, and switching to an archived session left the trajectory label showing the previous live path.
Root Cause: The session-shaping code only recognized `waypoint_sample_captured`, and the archived-session branch in the waypoint refresh path never updated the displayed trajectory path.
Solution: Treat `sample_captured` as a legacy alias for `waypoint_sample_captured`, add a regression test for that event name, and refresh the displayed trajectory path whenever an archived session is selected.
Constraints: Keep the current current-trajectory rendering unchanged. Do not rewrite archived session contents; only normalize event interpretation and the displayed header path.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `node --check src/paus_ui/paus_ui/static/app.js`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_web_server.py src/paus_ui/tests/test_ros_bridge.py` (`37 passed`); `colcon build --packages-select paus_ui paus_marker_ros2 --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`62 passed`)
Source Rounds: 52

## Lesson: handeye-live-follow-and-camera-bridge-gating
Lesson ID: BL-20260501-handeye-live-follow-and-camera-bridge-gating
Scope: src/paus_ui/paus_ui/static/app.js; src/paus_bringup/launch/ui.launch.py
Problem Description: The blank current-trajectory selection could stop the UI from following an in-progress live run, and `ui.launch.py` could leave camera bridge startup tied to assumptions about a single local receiver socket.
Root Cause: The live-session auto-follow branch was gated by the current-trajectory selection flag, and the launch file treated the camera bridge destination as an implicit hard-coded local receiver instead of an explicit launch-time target.
Solution: Let live runs reselect the active session regardless of the blank current-trajectory flag, while keeping archived-session auto-follow intact. Keep `start_camera_bridge` independently effective and expose the bridge target socket as launch arguments.
Constraints: Do not disturb the existing current-trajectory workflow for archived sessions. Preserve the default local receiver path, but support externally managed receiver setups without source edits.
Validation Evidence: Round 53: `python3 -m compileall -q src/paus_bringup/launch/ui.launch.py src/paus_ui/paus_ui/session_store.py src/paus_ui/tests/test_session_store.py`; `node --check src/paus_ui/paus_ui/static/app.js`; `colcon build --packages-select paus_bringup paus_ui paus_marker_ros2 --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_web_server.py src/paus_ui/tests/test_ros_bridge.py` (`37 passed`). Round 54: `python3 -m compileall -q src/paus_bringup/launch/ui.launch.py`; `colcon build --packages-select paus_bringup paus_ui paus_marker_ros2 --symlink-install`; `ros2 launch paus_bringup ui.launch.py --show-args` confirmed the receiver socket arguments; full related regression (`62 passed`).
Source Rounds: 53, 54

## Lesson: handeye-camera-bridge-target-socket-config
Lesson ID: BL-20260501-handeye-camera-bridge-target-socket-config
Scope: src/paus_bringup/launch/ui.launch.py
Problem Description: The UI launch needed to support both the default local image-receiver pipeline and the lab-host external receiver setup, but a strict coupling between `start_camera_bridge` and `start_image_receiver` blocked the bridge from starting in valid external-receiver deployments.
Root Cause: The launch file treated the camera bridge as if it could only connect to the receiver launched in the same file, so disabling the local receiver also disabled the bridge and left `camera.yaml` unwritten.
Solution: Keep `start_camera_bridge` independent, expose `image_receiver_host` and `image_receiver_port` launch arguments, and let the bridge target be selected at launch time while preserving the default local socket for the one-click workflow.
Constraints: Preserve the existing default `ros2 launch paus_bringup ui.launch.py` path. Do not force callers to edit source just to point the bridge at a different receiver socket.
Validation Evidence: `python3 -m compileall -q src/paus_bringup/launch/ui.launch.py`; `colcon build --packages-select paus_bringup paus_ui paus_marker_ros2 --symlink-install`; `ros2 launch paus_bringup ui.launch.py --show-args` (confirmed `image_receiver_host`, `image_receiver_port`, `start_image_receiver`, and `start_camera_bridge`); `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_web_server.py src/paus_ui/tests/test_ros_bridge.py` (`37 passed`); `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`62 passed`)
Source Rounds: 54

## Lesson: handeye-install-safe-single-segment-config-paths
Lesson ID: BL-20260501-handeye-install-safe-single-segment-config-paths
Scope: src/paus_perception/paus_perception/config.py; src/paus_perception/tests/test_config_paths.py
Problem Description: Single-segment relative paths loaded from an install-layout `default.yaml` could still resolve into read-only package/share prefixes instead of a writable runtime directory.
Root Cause: `resolve_config_path()` only anchored relative paths against the config file location, even when the config file lived under an installed share tree.
Solution: Treat single-segment relative paths as runtime artifacts in install-layout configs and route them under the runtime root, while leaving source-tree config resolution unchanged.
Constraints: Keep source-tree relative path behavior unchanged. Do not regress `resolve_config_artifact_path()` or runtime-data resolution.
Validation Evidence: `python3 -m compileall -q src/paus_perception/paus_perception/config.py src/paus_perception/tests/test_config_paths.py`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_ros_bridge.py` (`39 passed`); `colcon build --packages-select paus_perception paus_ui paus_bringup paus_marker_ros2 --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`64 passed`)
Source Rounds: 55

## Lesson: handeye-queued-run-request-accepted-success
Lesson ID: BL-20260501-handeye-queued-run-request-accepted-success
Scope: src/paus_ui/paus_ui/ros_bridge.py; src/paus_ui/tests/test_ros_bridge.py
Problem Description: The semi-auto run API could report an accepted request as unsuccessful, which made normal queued starts look like errors to callers that only inspect the `success` flag.
Root Cause: `start_semi_auto_run()` shaped the queued request result with `success=False` even after the worker thread had been started and the request had been accepted.
Solution: Mark the queued run result as `success=True` while keeping the `accepted`/`queued` metadata so the UI still shows the request as pending execution.
Constraints: Preserve the existing confirmation gate and error returns for unavailable services or duplicate runs. Do not change the queued message semantics.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_ros_bridge.py`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_ros_bridge.py` (`37 passed`); `colcon build --packages-select paus_perception paus_ui paus_bringup paus_marker_ros2 --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`64 passed`)
Source Rounds: 55

## Lesson: handeye-manual-capture-status-compatibility
Lesson ID: BL-20260501-handeye-manual-capture-status-compatibility
Scope: src/paus_ui/paus_ui/session_store.py; src/paus_ui/paus_ui/ros_bridge.py; src/paus_ui/tests/test_session_store.py; src/paus_ui/tests/test_ros_bridge.py
Problem Description: Legacy and manual capture flows now emit `sample_captured`/`capture_failed`, but session counts and workflow ribbons still only recognized the semi-auto `waypoint_*` variants.
Root Cause: `_counts_for_session()` only counted `waypoint_sample_captured`, and `_workflow_for_status()` lacked manual capture status aliases.
Solution: Count both legacy and semi-auto accepted events, and map manual `sample_captured` / `capture_failed` statuses into the workflow stage model.
Constraints: Keep the existing semi-auto status names unchanged. Do not alter the accepted/skipped semantics for archived waypoint rendering.
Validation Evidence: `python3 -m compileall -q src/paus_ui/paus_ui/session_store.py src/paus_ui/paus_ui/ros_bridge.py src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_ros_bridge.py`; `/usr/bin/python3 -m pytest -q src/paus_ui/tests/test_session_store.py src/paus_ui/tests/test_ros_bridge.py` (`37 passed`); `colcon build --packages-select paus_perception paus_ui paus_bringup paus_marker_ros2 --symlink-install`; `/usr/bin/python3 -m pytest -q src/paus_perception/tests/test_config_paths.py src/paus_ui/tests src/paus_marker_ros2/tests/test_semi_auto_calibration.py src/paus_marker_ros2/tests/test_eye_to_hand_session.py` (`64 passed`)
Source Rounds: 55
