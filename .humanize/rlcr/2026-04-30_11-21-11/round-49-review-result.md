- [P3] Parse `/api/handeye/run` confirmation as a real boolean — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/web_server.py:63-63
  If a client sends `{"confirmed": "false"}` (or any non-empty string), `bool(...)` still becomes `True`, so the motion-confirmation gate in `UiRosBridge.start_semi_auto_run` can be bypassed or trigger unexpectedly. Please validate that `confirmed` is actually a JSON boolean, or reject non-boolean values.
2026-05-01T12:14:15.084714Z ERROR codex_core::session: failed to record rollout items: thread 019de366-01e4-76b1-9702-1bd0069efd29 not found
2026-05-01T12:14:15.095988Z ERROR codex_core::session: failed to record rollout items: thread 019de366-01d0-7c71-af7e-ddebc15e4e81 not found
The new run endpoint can misinterpret non-empty string values as confirmation, which can cause the semi-auto motion gate to behave incorrectly for API callers. That is a concrete regression in the new UI backend.

Review comment:

- [P3] Parse `/api/handeye/run` confirmation as a real boolean — /home/chen_lab/worktrees/paus_robot_handeye_rlcr/src/paus_ui/paus_ui/web_server.py:63-63
  If a client sends `{"confirmed": "false"}` (or any non-empty string), `bool(...)` still becomes `True`, so the motion-confirmation gate in `UiRosBridge.start_semi_auto_run` can be bypassed or trigger unexpectedly. Please validate that `confirmed` is actually a JSON boolean, or reject non-boolean values.
