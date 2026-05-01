- [P1] Handle non-colcon ROS install prefixes when resolving paths — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:193-199
  When the package is installed under a normal prefix such as `/opt/ros/.../share/paus_bringup/configs/default.yaml`, this branch never sees an `install/` segment and falls back to the share directory. That sends relative `output_path`/`trajectory_path` (and the matching runtime-path helper below for `session_root_path`) back into a typically read-only package tree, so deployed installs will fail when they try to save trajectories or session archives.
2026-05-01T05:19:41.904134Z ERROR codex_core::session: failed to record rollout items: thread 019de1ed-f03b-7792-b6dd-469ced10de6d not found
2026-05-01T05:19:41.914167Z ERROR codex_core::session: failed to record rollout items: thread 019de1ed-f026-7710-81bf-a83a1fd6e877 not found
The new path-resolution logic only works for workspaces whose paths literally contain `install/`. Standard ROS install prefixes will misroute calibration and session files into the package share directory, which breaks saving in deployed environments.

Review comment:

- [P1] Handle non-colcon ROS install prefixes when resolving paths — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:193-199
  When the package is installed under a normal prefix such as `/opt/ros/.../share/paus_bringup/configs/default.yaml`, this branch never sees an `install/` segment and falls back to the share directory. That sends relative `output_path`/`trajectory_path` (and the matching runtime-path helper below for `session_root_path`) back into a typically read-only package tree, so deployed installs will fail when they try to save trajectories or session archives.
