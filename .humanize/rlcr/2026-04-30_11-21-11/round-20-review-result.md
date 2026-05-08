- [P2] Keep live waypoint teaching reachable after sessions exist — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:198-200
  Once any archived session exists, this auto-selects a session permanently, which means `refreshWaypoints()` never falls back to `/api/handeye/waypoints`. In that state, pressing Record/Delete still edits the current trajectory on the backend, but the table keeps showing the previously selected archive, so operators cannot verify newly taught waypoints or preview the trajectory they are building.

- [P2] Route installed nested artifact paths into the runtime dir — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:193-198
  For configs loaded from an installed package, `_resolve_config_artifact_path()` only redirects single-component relative paths like `extrinsics.yaml` into `PAUS_ROBOT_RUNTIME_DIR`. If a deployment uses a nested relative path such as `trajectory_path: configs/eye_to_hand.yaml` or `output_path: calibration/extrinsics.yaml`, it falls through to `resolve_config_path()` and resolves under the install prefix's parent instead, so semi-auto recording/save writes into the wrong tree (often read-only).

- [P3] Map the dry-run waypoint status the backend actually emits — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:619-620
  The dry-run path publishes `waypoint_dry_run_complete` for every waypoint, but the UI bridge only recognizes `semi_auto_dry_run_waypoint`. Whenever operators use the dedicated dry-run flow, the workflow stepper falls back to the raw status string and never highlights the dry-run/progress stage, so the UI looks stuck even though the backend is advancing through the trajectory.
2026-04-30T17:59:36.915468Z ERROR codex_core::session: failed to record rollout items: thread 019ddf7c-8ff3-7971-b317-30112c93aa49 not found
2026-04-30T17:59:36.925435Z ERROR codex_core::session: failed to record rollout items: thread 019ddf7c-8fe0-77a1-9b9e-13d8251956f2 not found
The new UI and path-resolution work introduce user-visible workflow regressions and an install-path bug that can misroute saved artifacts. Those issues make important parts of the added semi-auto workflow unreliable in common scenarios.

Full review comments:

- [P2] Keep live waypoint teaching reachable after sessions exist — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:198-200
  Once any archived session exists, this auto-selects a session permanently, which means `refreshWaypoints()` never falls back to `/api/handeye/waypoints`. In that state, pressing Record/Delete still edits the current trajectory on the backend, but the table keeps showing the previously selected archive, so operators cannot verify newly taught waypoints or preview the trajectory they are building.

- [P2] Route installed nested artifact paths into the runtime dir — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:193-198
  For configs loaded from an installed package, `_resolve_config_artifact_path()` only redirects single-component relative paths like `extrinsics.yaml` into `PAUS_ROBOT_RUNTIME_DIR`. If a deployment uses a nested relative path such as `trajectory_path: configs/eye_to_hand.yaml` or `output_path: calibration/extrinsics.yaml`, it falls through to `resolve_config_path()` and resolves under the install prefix's parent instead, so semi-auto recording/save writes into the wrong tree (often read-only).

- [P3] Map the dry-run waypoint status the backend actually emits — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:619-620
  The dry-run path publishes `waypoint_dry_run_complete` for every waypoint, but the UI bridge only recognizes `semi_auto_dry_run_waypoint`. Whenever operators use the dedicated dry-run flow, the workflow stepper falls back to the raw status string and never highlights the dry-run/progress stage, so the UI looks stuck even though the backend is advancing through the trajectory.
