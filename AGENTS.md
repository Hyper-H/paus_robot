# Agent Rules for paus_robot

Current execution context:
- Codex threads for this workspace run directly on the lab Linux host.
- Do not ssh to `chen_lab@192.168.58.183` for normal PAUS work from this workspace.
- Do not use old WSL paths such as `\\wsl.localhost\Ubuntu-22.04\root\projects\ag-repro`.
- Do not treat `/home/chen_lab/paus_robot` as the canonical repository.

Canonical repo:
- Path: `/mnt/data/projects/paus_robot/repo`
- Default base branch: `dev`
- Release branch: `main`

Worktrees:
- Feature and cleanup branches live under `/mnt/data/projects/paus_robot/worktrees`.
- Before editing, inspect the current worktree with:
  `git status --short --branch && git branch --show-current`
- Preserve dirty worktree changes unless the user explicitly asks to discard them.
- New implementation work should start from the canonical repo's latest `dev` in a new `codex/<task>` branch or dedicated worktree.

Runtime and data directories:
- Runtime outputs belong under `/mnt/data/projects/paus_robot/runs`.
- Calibration assets belong under `/mnt/data/projects/paus_robot/calibration`.
- Rosbags belong under `/mnt/data/projects/paus_robot/rosbags`.
- Datasets belong under `/mnt/data/projects/paus_robot/datasets`.
- Migration snapshots and recovery material belong under `/mnt/data/projects/paus_robot/migration_backups`.

Hard rules:
- Never run real robot motion unless the user explicitly approves the hardware safety flow.
- Default ROS launch and smoke tests must use `execute_motion:=false`.
- Source ROS before ROS commands:
  `source /opt/ros/humble/setup.bash && source install/setup.bash`
- Stop stale PAUS runtime before hardware or launch checks:
  `ros2 run paus_bringup stop_paus_runtime --kill --force`
- Never commit `runtime_logs/`, `runs/`, `rosbags/`, `*.bag`, `*.db3`, `*.bak_*.yaml`, `.humanize/`, `.codex/`, `build/`, `install/`, `log/`, or one-off debug scripts unless the user explicitly asks.

Preferred workflow:
1. Work in the relevant directory under `/mnt/data/projects/paus_robot/worktrees`.
2. Keep `/mnt/data/projects/paus_robot/repo` as the clean canonical `dev` checkout unless intentionally updating it.
3. Run focused tests or dry-run ROS checks from the active worktree.
4. Commit only source/config/test/documentation changes that belong to the active branch.
5. Push from the active worktree when the user asks for a branch update.
