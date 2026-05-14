# Code Review Findings

You are in the **Review Phase**. Codex has performed a code review and found issues that need to be addressed.

## Review Results

## Codex Review Issues

- [P1] Resolve installed output/trajectory paths under runtime storage — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:236-239
  When `load_config()` reads a `default.yaml` from an installed `share/paus_bringup/configs` tree, this loop resolves bare `output_path`/`trajectory_path` values back into the package share directory instead of `${PAUS_ROBOT_RUNTIME_DIR}`. The new install-safe test already demonstrates the regression: `extrinsics.yaml` becomes `.../share/paus_bringup/configs/extrinsics.yaml` instead of `.../runtime/configs/extrinsics.yaml`. In an installed deployment, saving extrinsics or recording a trajectory will therefore try to write into a read-only install location unless the operator overrides both paths manually.

- [P2] Stop quality polling from overwriting the current waypoint pose — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:191-197
  `refreshStatus()` already populates `camera-board-grid`/`board-angle` from the backend status snapshot, but every `refreshQuality()` call rewrites those same DOM nodes from `/api/handeye/quality`. As a result, whenever the latest live image is stale or chessboard detection fails, the current-waypoint card is cleared or replaced with unrelated live-quality data even though `/api/status` still has a valid waypoint pose. Operators reviewing a run or an archived session will see that panel flicker away from the actual status payload.
2026-05-01T07:58:50.595625Z ERROR codex_core::session: failed to record rollout items: thread 019de27c-e8ae-7be2-b9ff-fb92c7039fe8 not found
2026-05-01T07:58:50.605374Z ERROR codex_core::session: failed to record rollout items: thread 019de27c-e89a-7b53-89ba-e726f6d83d1d not found
The patch introduces at least one concrete regression in installed deployments by resolving writable calibration artifacts back into the package share directory, and the new UI polling logic also overwrites the current-waypoint pose with live quality data. These issues make the patch unsafe to consider correct as-is.

Full review comments:

- [P1] Resolve installed output/trajectory paths under runtime storage — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_perception/paus_perception/config.py:236-239
  When `load_config()` reads a `default.yaml` from an installed `share/paus_bringup/configs` tree, this loop resolves bare `output_path`/`trajectory_path` values back into the package share directory instead of `${PAUS_ROBOT_RUNTIME_DIR}`. The new install-safe test already demonstrates the regression: `extrinsics.yaml` becomes `.../share/paus_bringup/configs/extrinsics.yaml` instead of `.../runtime/configs/extrinsics.yaml`. In an installed deployment, saving extrinsics or recording a trajectory will therefore try to write into a read-only install location unless the operator overrides both paths manually.

- [P2] Stop quality polling from overwriting the current waypoint pose — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:191-197
  `refreshStatus()` already populates `camera-board-grid`/`board-angle` from the backend status snapshot, but every `refreshQuality()` call rewrites those same DOM nodes from `/api/handeye/quality`. As a result, whenever the latest live image is stale or chessboard detection fails, the current-waypoint card is cleared or replaced with unrelated live-quality data even though `/api/status` still has a valid waypoint pose. Operators reviewing a run or an archived session will see that panel flicker away from the actual status payload.

## Instructions

1. **Read `.humanize/bitlesson.md` and run `bitlesson-selector`** for each fix task before coding
2. **Address all issues** marked with `[P0-9]` severity markers
3. **Focus on fixes only** - do not add new features or make unrelated changes
4. **Commit your changes** after fixing the issues
5. **Write your summary** to: `/home/chen_lab/worktrees/paus_robot_handeye_rlcr/.humanize/rlcr/2026-04-30_11-21-11/round-40-summary.md`

## Summary Template

Your summary should include:
- Which issues were fixed
- How each issue was resolved
- Any issues that could not be resolved (with explanation)

## Important Notes

- The COMPLETE signal has no effect during the review phase
- You must address the code review findings to proceed
- After you commit and write your summary, Codex will perform another code review
- The loop continues until no `[P0-9]` issues are found

## Task Tag Routing Reminder

Follow the plan's per-task routing tags strictly:
- `coding` task -> Claude executes directly
- `analyze` task -> execute via `/humanize:ask-codex`, then integrate the result
- Keep Goal Tracker Active Tasks columns `Tag` and `Owner` aligned with execution
