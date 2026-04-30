- [P1] Resolve runtime artifacts outside read-only install prefixes — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:183-186
  When `default.yaml` is loaded from an installed package (`.../install/paus_bringup/share/...`) and the path is just `extrinsics.yaml` or `eye_to_hand_trajectory.yaml`, this branch rewrites it to `<install-prefix>/../configs/...` (for example `/opt/ros/configs/...`). In an install-only deployment that location is typically read-only or absent, so waypoint recording and final extrinsics save fail as soon as they try to write their outputs.

- [P2] Mark queued `/api/handeye/run` requests as accepted — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:400-404
  This is the normal happy-path return from `POST /api/handeye/run`, but it sets `success` to `false` even after the request has been accepted and the worker thread has started. Any client that keys off `success` will treat a valid start as an error, which is especially easy to hit because this response is all the caller sees before the background ROS service finishes.

- [P3] Populate archived sample rotation from the stored transform — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:110-111
  Archived samples written by the calibration node only persist `camera_to_board_matrix`, not a precomputed Euler-angle field. Because this code never derives `camera_to_board_rotation_rpy_deg` from that matrix, every historical sample/waypoint preview shows `board rx/ry/rz` as `--` even though the orientation data is already present in the log record.
2026-04-30T07:32:37.952117Z ERROR codex_core::session: failed to record rollout items: thread 019ddd43-0319-74f3-98b9-f455038c7630 not found
The patch adds substantial functionality, but it still has user-visible regressions: install-only deployments can no longer write runtime artifacts, the run-start API reports accepted requests as failures, and archived sample pose details are incomplete. Those issues are enough to treat the change as not fully correct yet.

Full review comments:

- [P1] Resolve runtime artifacts outside read-only install prefixes — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:183-186
  When `default.yaml` is loaded from an installed package (`.../install/paus_bringup/share/...`) and the path is just `extrinsics.yaml` or `eye_to_hand_trajectory.yaml`, this branch rewrites it to `<install-prefix>/../configs/...` (for example `/opt/ros/configs/...`). In an install-only deployment that location is typically read-only or absent, so waypoint recording and final extrinsics save fail as soon as they try to write their outputs.

- [P2] Mark queued `/api/handeye/run` requests as accepted — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/ros_bridge.py:400-404
  This is the normal happy-path return from `POST /api/handeye/run`, but it sets `success` to `false` even after the request has been accepted and the worker thread has started. Any client that keys off `success` will treat a valid start as an error, which is especially easy to hit because this response is all the caller sees before the background ROS service finishes.

- [P3] Populate archived sample rotation from the stored transform — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/session_store.py:110-111
  Archived samples written by the calibration node only persist `camera_to_board_matrix`, not a precomputed Euler-angle field. Because this code never derives `camera_to_board_rotation_rpy_deg` from that matrix, every historical sample/waypoint preview shows `board rx/ry/rz` as `--` even though the orientation data is already present in the log record.
