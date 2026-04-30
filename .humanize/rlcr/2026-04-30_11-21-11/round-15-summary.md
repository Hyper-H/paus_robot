Round 15 summary

Changes made:
- UI backend state now treats "services are ready but no backend status has arrived" as motion-state unknown and forces motion confirmation by setting the effective execute-motion risk state conservatively.
- `/api/status` and `/api/handeye/run` both include the motion-state-known guard when deciding whether confirmation is required.
- Archived sessions no longer fall back to the current trajectory YAML when `trajectory_used.yaml` is absent, so manual sessions do not get phantom waypoints after the live trajectory changes.
- Added regression tests for service-ready/no-status safety behavior and manual sessions without trajectory metadata.

Validation:
- `python3 -m compileall -q src/paus_ui/paus_ui`
- `/usr/bin/python3 -m pytest -q src/paus_ui/tests` -> 17 passed
- `colcon build --packages-select paus_ui --symlink-install`

## BitLesson Delta
- Action: none
- Lesson ID(s): NONE
- Notes: `bitlesson-selector` is unavailable on the lab host in this environment, so no existing lesson was selected or updated.
