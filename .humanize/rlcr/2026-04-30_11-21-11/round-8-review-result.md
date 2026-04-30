- [P1] Keep generated trajectory/extrinsics out of installed share dir — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:178-185
  When `default.yaml` is loaded from `install/.../share/paus_bringup/configs`, this resolver sends bare `output_path`/`trajectory_path` values back into that same installed config directory. That makes `record_waypoint`, `save_trajectory`, and the final extrinsics save write runtime artifacts into the installed package, which is read-only in non-workspace deployments, so semi-auto recording/saving breaks as soon as the node is launched from an installed package.

- [P2] Don't persist empty trajectories that the loader rejects — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/semi_auto_calibration.py:156-160
  `save_trajectory()` happily writes `waypoints: []`, but `load_trajectory()` later rejects that file. In practice this happens after deleting the last recorded waypoint: the file still exists, so the CLI/UI treat it as a usable trajectory, and the next semi-auto run fails with `Trajectory must contain at least one waypoint` instead of returning to recording mode. Either reject empty saves here or remove the file when the last waypoint is deleted.

- [P2] Apply the recorded acceleration during MoveJ execution — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:919-923
  The new trajectory schema records per-waypoint `acc`, and the UI confirmation/reporting surfaces that value, but the actual semi-auto motion call ignores it and forwards only `vel`. As a result, changing `acc` in the saved trajectory has no effect on the robot motion, so the executed run can differ from what the operator reviewed in the YAML/UI.
2026-04-30T07:07:06.923169Z ERROR codex_core::session: failed to record rollout items: thread 019ddd2f-cf0d-7673-b66b-21bd6d1f296e not found
The patch introduces a couple of runtime regressions in the new semi-auto flow: generated artifacts are resolved into the installed package directory, empty trajectory files can be saved even though the loader rejects them, and the recorded acceleration is not applied during execution. These issues are user-visible and can break or mislead normal calibration runs.

Full review comments:

- [P1] Keep generated trajectory/extrinsics out of installed share dir — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:178-185
  When `default.yaml` is loaded from `install/.../share/paus_bringup/configs`, this resolver sends bare `output_path`/`trajectory_path` values back into that same installed config directory. That makes `record_waypoint`, `save_trajectory`, and the final extrinsics save write runtime artifacts into the installed package, which is read-only in non-workspace deployments, so semi-auto recording/saving breaks as soon as the node is launched from an installed package.

- [P2] Don't persist empty trajectories that the loader rejects — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/semi_auto_calibration.py:156-160
  `save_trajectory()` happily writes `waypoints: []`, but `load_trajectory()` later rejects that file. In practice this happens after deleting the last recorded waypoint: the file still exists, so the CLI/UI treat it as a usable trajectory, and the next semi-auto run fails with `Trajectory must contain at least one waypoint` instead of returning to recording mode. Either reject empty saves here or remove the file when the last waypoint is deleted.

- [P2] Apply the recorded acceleration during MoveJ execution — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:919-923
  The new trajectory schema records per-waypoint `acc`, and the UI confirmation/reporting surfaces that value, but the actual semi-auto motion call ignores it and forwards only `vel`. As a result, changing `acc` in the saved trajectory has no effect on the robot motion, so the executed run can differ from what the operator reviewed in the YAML/UI.
