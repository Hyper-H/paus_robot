Audit based on [web_server.py](/home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/web_server.py:28), [ros_bridge.py](/home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:152), [session_store.py](/home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:17), [overlay.py](/home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/overlay.py:26), [index.html](/home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/index.html:33), [app.js](/home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:85), and [eye_to_hand_calibration_node.py](/home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:244). No `run.log` files were present in this worktree, so event names below are from node source.

**Panel Map**
- Top status strip: `/api/status` -> `camera.connected/image_sequence`, `handeye.calibration_node_connected`, `handeye.execute_motion`, `handeye.last_status.session_dir`.
- Live image: `/api/image/latest.jpg?mode=overlay|pose|raw`; metrics row from `/api/handeye/quality` -> `detected`, `reprojection_error_px`, `board_margin_px`, `camera_to_board_translation_m`, `board_angle_deg`. Overlay rendering is server-side.
- Control/current waypoint: `/api/status` -> `handeye.last_status.status/message`, `waypoint_name|waypoint.name`, `stable_tcp_pose_mmdeg|tcp_pose_mmdeg`; `T_camera_board` in this panel actually comes from `/api/handeye/quality`, not from the current status.
- Commands: `POST /api/handeye/record_waypoint`, `delete_last_waypoint`, `run`, `stop`; command notice from `/api/status.handeye.last_command_result`.
- Result summary: `/api/sessions` for selector; `/api/sessions/{id}/report` for `sample_count` and `residuals`; skipped count is taken from the waypoint table response, not the report.
- Waypoint table: `/api/sessions/{id}/waypoints` when a session is selected, otherwise `/api/handeye/waypoints`; thumbnails from `/api/sessions/{id}/sample-image/{row}.jpg`. Session rows are reconstructed from current trajectory YAML + session `samples.jsonl` + session `run.log`.
- Sample preview: `/api/sessions/{id}/sample-image/{row}.jpg?mode=*`; title is local UI state only, no metadata endpoint is used.
- Event log: `/ws/events`; only bridge-buffered events are exposed, not raw session `run.log`.

Relevant node names:
- Status topic names: `ready`, `waypoint_recorded`, `record_waypoint_failed`, `waypoint_deleted`, `delete_waypoint_failed`, `trajectory_saved`, `sample_captured`, `capture_failed`, `solved`, `solve_failed`, `saved`, `save_failed`, `semi_auto_dry_run_waypoint`, `semi_auto_dry_run_complete`, `waypoint_motion_started`, `waypoint_waiting_stable`, `waypoint_reached`, `waypoint_capture_started`, `waypoint_capture_skipped`, `waypoint_sample_captured`, `semi_auto_insufficient_samples`, `semi_auto_finished`, `semi_auto_finished_with_skips`, `semi_auto_failed`.
- `run.log` event names: `node_started`, `waypoint_recorded`, `waypoint_deleted`, `semi_auto_started`, `semi_auto_dry_run_complete`, `waypoint_motion_started`, `waypoint_reached`, `waypoint_capture_skipped`, `waypoint_sample_captured`, `sample_captured`, `solved`, `extrinsics_saved`, `semi_auto_insufficient_samples`, `semi_auto_failed`.
- Important mismatch: `waypoint_waiting_stable` and `waypoint_capture_started` are status-only, not `run.log` events.

**Missing Shaped Fields**
- AC-2: no axis-toggle field/param; `pose` is not actually distinct from `overlay`; failure reasons are raw `repr(exc)` strings.
- AC-3: no normalized `current_waypoint` object, no `workflow_phase`, no panel-ready `camera_to_board_translation_m` or `board_angle_deg` in current status, no `empty_reason`.
- AC-4: no shaped run-confirmation payload with `waypoint_count`, `motion`, `vel`, `acc`; no structured `stop_supported` or `stop_explanation`.
- AC-5: no `result` field, no threshold flags like `reprojection_error_exceeds_limit` / `board_margin_below_limit`, no friendly `reason_display`, no thumbnail URL field, no session-specific trajectory source indicator. Historical rows are also built from the current trajectory file, not `trajectory_used.yaml`.
- AC-6: no preview metadata shape for `image_sequence`, `capture_time`, `board_angle_deg`, `camera_to_board_rotation_rpy_deg`, or `empty_reason`. Sample preview image uses `row_index` as displayed `image_sequence`.
- AC-7: no `latest_valid_session` selection hint, no shaped `accepted/skipped/pending` counts in session summary, no `has_report/has_solution/empty_reason` summary payload, no threshold-comparison fields.
- AC-8: no polling fallback for waypoint/report refresh when WebSocket is down, no direct events polling endpoint, no shared `error_code/operator_message`, no dedupe key. `POST /api/handeye/run` itself is fine and should not 422.

**Recommendations**
- Add a backend-shaped status model under `/api/status`: `current_waypoint`, `workflow`, `session`, `motion`, `stop`, `command`, plus friendly `operator_message`.
- Add session-shaped endpoints or enrich existing ones: `summary`, `waypoints`, and sample metadata. Build session waypoints from `trajectory_used.yaml` when present, not the live trajectory file.
- Make `pose` mode real or remove it; add an axis toggle separate from `raw`.
- Translate known backend errors once, centrally: chessboard missing, no fresh image, reprojection too large, board margin too small, service unavailable.
- Add reliable fallback when WS is down: either timer-refresh `report/waypoints` or expose `/api/events?since=`.
- Dedupe event spam: `_run_semi_auto_worker` and `_call_trigger` both emit `ui_command_result`, and every status message is logged verbatim today.
