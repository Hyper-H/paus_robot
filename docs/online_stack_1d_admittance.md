# online_stack 1D Fz admittance

The online path keeps one robot owner and one external force-control path:

```text
fairino_control_node
  -> existing visual approach
  -> final_hover
  -> AdmittanceSession
  -> FzAdmittanceController.update()
  -> FT_GetForceTorqueRCS()
  -> ServoCart
```

`FzAdmittanceController` is pure Python. It filters positive compressive Fz,
computes a deadbanded proportional velocity, integrates a bounded Tool-Z
travel, and returns a displacement increment. It does not import ROS or the
FAIRINO SDK.

`AdmittanceSession` represents one task. It uses the node's already-created
`FairinoLinuxClient`, owns the CONTACTING, SETTLING, HOLDING, optional STROKING, RELEASING and
RETURNING lifecycle, and applies the unchanged force, lateral-force, torque,
contact, timing and ServoCart gates. Fx/Fy are monitored only in this first
1D implementation and never enter the position feedback command.

The ROS node remains the sole robot command owner. Its force worker is a
thread inside that node, not another ROS node or process. The worker reads FT,
calls the pure controller, sends ServoCart, and performs cleanup. The stop
service only sets a stop event; the worker performs the SDK calls.

`execute_motion: false` is a hard dry-run gate: the node does not import or
connect the real FAIRINO SDK and does not send MoveJ, MoveL or ServoCart.
The default `admittance_1d.enabled: false` prevents accidental contact motion.
The default target remains `final_hover` with 50 mm clearance. Contact
commissioning requires an explicit reviewed configuration with
`control.max_execution_stage: force_hold`, a verified near-surface
`control.hover_clearance_mm`, and `admittance_1d.enabled: true`.

This is distinct from FAIRINO controller-internal FT/impedance/compliance
interfaces. No `FT_Control`, `ImpedanceControlStartStop`,
`FT_ComplianceStart`, or `FT_ComplianceStop` call is used. Future ROS2 or
`ros2_control` integration can reuse the pure controller, but is outside this
first online_stack change.
