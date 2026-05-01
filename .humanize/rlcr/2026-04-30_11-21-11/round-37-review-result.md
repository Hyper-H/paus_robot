- [P2] Ignore stale `recorded_trajectory` when status has aged out — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:429-430
  When `/eye_to_hand/status` stops updating but the service clients are still reachable, `_sync_backend_state()` reports `backend_connected=True` and `status_recent=False`. `get_waypoints()` only checks `backend_connected`, so it keeps returning `last_status["recorded_trajectory"]` from the stale message instead of falling back to the on-disk trajectory. In that state the main waypoint table can show an old unsaved recording after a node restart or status gap, even though `get_status()` has already dropped the same payload as stale.

- [P2] Base the motion summary on the live trajectory state — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:661-668
  During manual waypoint recording, `/api/handeye/waypoints` is populated from `last_status["recorded_trajectory"]`, but `_motion_summary()` always re-reads `trajectory_path` from disk. Until the operator saves the edited trajectory, the status card and run-confirmation dialog can report the wrong waypoint count/motion defaults (often 0) even though the table already shows the in-memory waypoints that are about to be saved and run.

- [P3] Count recorded waypoints in recording-only session archives — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:336-343
  `_counts_for_session()` derives `waypoint_count` only from `trajectory_used.yaml`, but manual recording sessions never create that file; they only persist `waypoint_recorded` events in `run.log`. Because of that, archived recording sessions show `accepted/skipped/pending = 0` in the session list even though `read_session_waypoints()` can reconstruct pending waypoints from the same log, so those sessions appear empty in the UI summary.
2026-05-01T06:25:40.867688Z ERROR codex_core::session: failed to record rollout items: thread 019de22b-f9bc-7f00-8f86-1349ce763f39 not found
2026-05-01T06:25:40.877699Z ERROR codex_core::session: failed to record rollout items: thread 019de22b-f999-70f3-a0b1-2967c1860ae0 not found
The new UI/session handling has state-consistency bugs: it can serve stale in-memory trajectories after status updates stop, and it reports mismatched or empty waypoint counts in common recording workflows. These issues are user-visible and affect the correctness of the new calibration UI.

Full review comments:

- [P2] Ignore stale `recorded_trajectory` when status has aged out — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:429-430
  When `/eye_to_hand/status` stops updating but the service clients are still reachable, `_sync_backend_state()` reports `backend_connected=True` and `status_recent=False`. `get_waypoints()` only checks `backend_connected`, so it keeps returning `last_status["recorded_trajectory"]` from the stale message instead of falling back to the on-disk trajectory. In that state the main waypoint table can show an old unsaved recording after a node restart or status gap, even though `get_status()` has already dropped the same payload as stale.

- [P2] Base the motion summary on the live trajectory state — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:661-668
  During manual waypoint recording, `/api/handeye/waypoints` is populated from `last_status["recorded_trajectory"]`, but `_motion_summary()` always re-reads `trajectory_path` from disk. Until the operator saves the edited trajectory, the status card and run-confirmation dialog can report the wrong waypoint count/motion defaults (often 0) even though the table already shows the in-memory waypoints that are about to be saved and run.

- [P3] Count recorded waypoints in recording-only session archives — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:336-343
  `_counts_for_session()` derives `waypoint_count` only from `trajectory_used.yaml`, but manual recording sessions never create that file; they only persist `waypoint_recorded` events in `run.log`. Because of that, archived recording sessions show `accepted/skipped/pending = 0` in the session list even though `read_session_waypoints()` can reconstruct pending waypoints from the same log, so those sessions appear empty in the UI summary.
