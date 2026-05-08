- [P2] Select the latest archived report session by default — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:194-209
  When the page loads with archived sessions and no live run, this refresh only rebuilds the dropdown; it never initializes `state.selectedSession` from the latest valid report session. The UI therefore stays on “当前示教轨迹” and leaves the report/waypoint panes empty until the operator manually picks a session, which breaks the documented default-selection behavior for calibration results.

- [P2] Include board angle in saved-sample preview details — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:348-358
  For archived sessions, the preview detail panel never shows the board-angle field because it only renders reprojection error, margin, translation, rotation, image sequence, and capture time. As a result, selecting any saved sample misses one of the required diagnosis metrics even though the live quality path already computes `board_angle_deg`.
2026-05-01T08:24:56.189217Z ERROR codex_core::session: failed to record rollout items: thread 019de291-c1ed-7663-8bdd-6ed243b6b438 not found
2026-05-01T08:24:56.199128Z ERROR codex_core::session: failed to record rollout items: thread 019de291-c1d9-73b0-a1e7-c6add3d1e146 not found
The new UI/archive flow has two user-visible functional gaps: it does not auto-open the latest archived result, and saved sample previews omit board-angle diagnostics. Those issues mean the patch does not fully satisfy the intended behavior.

Full review comments:

- [P2] Select the latest archived report session by default — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:194-209
  When the page loads with archived sessions and no live run, this refresh only rebuilds the dropdown; it never initializes `state.selectedSession` from the latest valid report session. The UI therefore stays on “当前示教轨迹” and leaves the report/waypoint panes empty until the operator manually picks a session, which breaks the documented default-selection behavior for calibration results.

- [P2] Include board angle in saved-sample preview details — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/static/app.js:348-358
  For archived sessions, the preview detail panel never shows the board-angle field because it only renders reprojection error, margin, translation, rotation, image sequence, and capture time. As a result, selecting any saved sample misses one of the required diagnosis metrics even though the live quality path already computes `board_angle_deg`.
