# Agent Rules for paus_robot

Canonical repo:
- Host: `chen_lab@192.168.58.183`
- Path: `/home/chen_lab/paus_robot`
- Default base branch: `dev`
- Release branch: `main`

Hard rules:
- Do not edit this WSL checkout for `paus_robot` work.
- All code edits, git operations, builds, tests, ROS commands, and hardware checks must run on the lab host.
- Before any task, inspect the lab repo:
  `ssh chen_lab@192.168.58.183 'cd /home/chen_lab/paus_robot && git status --short --branch && git branch --show-current'`
- New implementation work starts from `origin/dev` in a new `codex/<task>` branch on the lab host.
- Never commit rosbag directories, `*.bak_*.yaml`, `.humanize/`, `.codex/`, `build/`, `install/`, `log/`, or one-off debug scripts unless the user explicitly asks.

Preferred lab workflow:
1. `ssh chen_lab@192.168.58.183 'cd /home/chen_lab/paus_robot && git fetch origin && git switch dev && git pull origin dev'`
2. `ssh chen_lab@192.168.58.183 'cd /home/chen_lab/paus_robot && git switch -c codex/<task>'`
3. Make edits only on the lab host.
4. Run focused tests/builds on the lab host.
5. Commit and push from the lab host.
