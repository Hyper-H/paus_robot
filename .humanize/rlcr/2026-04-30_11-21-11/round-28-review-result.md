- [P1] Don't persist waypoints before the recording session is committed — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:867-868
  If an operator records a few waypoints and then aborts the interactive recorder (for example by pressing `q` in `eye_to_hand_semi_auto_cli`), this write has already updated `trajectory_path` on disk. That means the "cancel" path cannot discard a partial trajectory, and a later `--run-only` or UI-triggered run can execute stale/incomplete waypoints that the operator thought had been abandoned.

- [P2] Enter record mode when `--record-only` is requested — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_semi_auto_cli.py:194-200
  When a trajectory file already exists, `--record-only` currently becomes a no-op because `should_record` is false unless `--record` is also passed, and the function returns at line 199 without ever entering `_interactive_record`. In practice this breaks the advertised "record/save trajectory and do not run calibration" flow for re-recording an existing trajectory.
2026-04-30T19:43:20.963403Z ERROR codex_core::session: failed to record rollout items: thread 019ddfe5-3217-7601-a3e1-847596c6794a not found
2026-04-30T19:43:21.015430Z ERROR codex_core::session: failed to record rollout items: thread 019ddfe5-3203-7580-9297-23c2dc521009 not found
The semi-auto recording flow has user-visible behavioral bugs: cancelling a recording still leaves a partially saved trajectory behind, and `--record-only` does not actually enter recording mode when a trajectory already exists. Both issues affect the safety and usability of the new workflow.

Full review comments:

- [P1] Don't persist waypoints before the recording session is committed — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py:867-868
  If an operator records a few waypoints and then aborts the interactive recorder (for example by pressing `q` in `eye_to_hand_semi_auto_cli`), this write has already updated `trajectory_path` on disk. That means the "cancel" path cannot discard a partial trajectory, and a later `--run-only` or UI-triggered run can execute stale/incomplete waypoints that the operator thought had been abandoned.

- [P2] Enter record mode when `--record-only` is requested — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_semi_auto_cli.py:194-200
  When a trajectory file already exists, `--record-only` currently becomes a no-op because `should_record` is false unless `--record` is also passed, and the function returns at line 199 without ever entering `_interactive_record`. In practice this breaks the advertised "record/save trajectory and do not run calibration" flow for re-recording an existing trajectory.
