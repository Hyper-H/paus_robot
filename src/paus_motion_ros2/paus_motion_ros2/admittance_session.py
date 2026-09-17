"""A single externally controlled Fz contact task.

This is a regular Python task object, not a ROS node and not a robot client.
The owning node supplies its already-connected client and invokes ``run`` from
its worker thread.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import threading
import time
from typing import Any, Callable

from .admittance_1d import FzAdmittanceConfig, FzAdmittanceController


class AdmittanceSessionState(str, Enum):
    CONTACTING = "CONTACTING"
    SETTLING = "SETTLING"
    HOLDING = "HOLDING"
    STROKING = "STROKING"
    RELEASING = "RELEASING"
    RETURNING = "RETURNING"
    DONE = "DONE"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class AdmittanceSessionConfig:
    contact_detect_force_n: float = 1.0
    target_force_n: float = 2.0
    release_force_n: float = 0.2
    hold_s: float = 10.0
    force_stable_s: float = 1.0
    loop_hz: float = 20.0
    servo_cmd_t: float = 0.008
    max_control_cycle_s: float = 0.15
    max_servo_call_s: float = 0.15
    contact_speed_mm_s: float = 0.05
    contact_direction_sign: float = -1.0
    max_press_mm: float = 10.0
    max_release_mm: float = 2.0
    deadband_n: float = 0.2
    gain_mm_s_n: float = 0.2
    max_speed_mm_s: float = 0.3
    release_speed_mm_s: float = 0.3
    force_filter_s: float = 0.25
    contact_timeout_s: float = 20.0
    settling_timeout_s: float = 20.0
    release_timeout_s: float = 10.0
    force_hard_limit_n: float = 5.0
    lateral_force_quality_limit_n: float = 1.0
    lateral_force_hard_limit_n: float = 2.0
    torque_hard_limit_nm: float = 0.5
    stop_after_contact: bool = False
    stroke_enabled: bool = False
    stroke_axes: tuple[str, ...] = ()
    stroke_distances_mm: tuple[float, ...] = ()
    stroke_speed_mm_s: float = 0.3
    stroke_return_to_origin: bool = True

    def __post_init__(self) -> None:
        for name in ("contact_detect_force_n", "target_force_n", "release_force_n", "hold_s", "force_stable_s", "loop_hz"):
            if not math.isfinite(float(getattr(self, name))) or float(getattr(self, name)) < 0.0:
                raise ValueError(f"{name} must be finite and >= 0")
        if self.contact_detect_force_n <= 0.0 or self.target_force_n < self.contact_detect_force_n:
            raise ValueError("target_force_n must be >= contact_detect_force_n > 0")
        if self.loop_hz <= 0.0:
            raise ValueError("loop_hz must be > 0")
        if float(self.contact_direction_sign) not in (-1.0, 1.0):
            raise ValueError("contact_direction_sign must be -1 or 1")
        if not math.isfinite(float(self.release_speed_mm_s)) or self.release_speed_mm_s <= 0.0:
            raise ValueError("release_speed_mm_s must be finite and > 0")
        if not math.isfinite(float(self.stroke_speed_mm_s)) or self.stroke_speed_mm_s < 0.0:
            raise ValueError("stroke_speed_mm_s must be finite and >= 0")
        if len(self.stroke_axes) != len(self.stroke_distances_mm):
            raise ValueError("stroke_axes and stroke_distances_mm must have the same length")
        for axis, distance in zip(self.stroke_axes, self.stroke_distances_mm):
            if str(axis).lower() not in {"x", "y", "xy"}:
                raise ValueError(f"unsupported stroke axis: {axis}")
            if not math.isfinite(float(distance)):
                raise ValueError("stroke distances must be finite")


@dataclass
class AdmittanceSessionResult:
    state: AdmittanceSessionState
    success: bool
    reason: str
    samples: int = 0
    max_abs_fx_n: float = 0.0
    max_abs_fy_n: float = 0.0
    max_abs_torque_nm: float = 0.0
    contact_travel_mm: float = 0.0
    stroke_completed_segments: int = 0
    stroke_total_distance_mm: float = 0.0
    events: list[dict[str, Any]] = field(default_factory=list)


class AdmittanceSession:
    def __init__(
        self,
        client: Any,
        *,
        config: AdmittanceSessionConfig,
        start_pose_mmdeg: list[float],
        return_callback: Callable[[], None] | None = None,
        release_callback: Callable[[], None] | None = None,
        trace_callback: Callable[[dict[str, Any]], None] | None = None,
        stop_event: threading.Event | None = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.client = client
        self.config = config
        self.start_pose_mmdeg = list(start_pose_mmdeg)
        self.return_callback = return_callback
        self.release_callback = release_callback
        self.trace_callback = trace_callback
        self.stop_event = stop_event or threading.Event()
        self.clock = clock
        self.sleep = sleep
        self.state = AdmittanceSessionState.CONTACTING
        self._servo_started = False
        self._cleanup_done = False
        self._contact_detected = False
        self._contact_travel_mm = 0.0
        self._controller = self._make_controller(config.contact_detect_force_n)
        self._hold_since: float | None = None

    def run(self) -> AdmittanceSessionResult:
        result = AdmittanceSessionResult(self.state, False, "not_started")
        try:
            self.client.ft_activate(True)
            self.client.ft_set_zero(True)
            self.client.ft_set_rcs(ref=0)
            self._start_servo()
            deadline = self.clock() + self.config.contact_timeout_s
            while self.state == AdmittanceSessionState.CONTACTING:
                sample = self._read_sample(result)
                if self.stop_event.is_set():
                    return self._cancel(result)
                if sample["fz"] >= self.config.contact_detect_force_n:
                    self._contact_detected = True
                    self._record(result, "contact_detected", state=self.state.value)
                    if self.config.stop_after_contact:
                        self.state = AdmittanceSessionState.RELEASING
                        self._record(result, "contact_target_reached", state=self.state.value)
                    else:
                        self.state = AdmittanceSessionState.SETTLING
                        self._controller = self._make_controller(self.config.target_force_n)
                        deadline = self.clock() + self.config.settling_timeout_s
                    continue
                step_mm = min(
                    self.config.contact_speed_mm_s / self.config.loop_hz,
                    self.config.max_press_mm - self._contact_travel_mm,
                )
                if step_mm <= 1e-9:
                    return self._fail(result, "max_press_limit_without_force")
                self._send_delta([0.0, 0.0, self.config.contact_direction_sign * step_mm])
                self._contact_travel_mm += step_mm
                result.contact_travel_mm = self._contact_travel_mm
                self._trace(
                    "force_sample",
                    state=self.state.value,
                    fz_raw_n=sample["fz_raw"],
                    fz_filtered_n=None,
                    target_force_n=self.config.contact_detect_force_n,
                    force_error_n=self.config.contact_detect_force_n - sample["fz"],
                    command_speed_mm_s=self.config.contact_speed_mm_s,
                    press_travel_mm=self._contact_travel_mm,
                    delta_z_mm=self.config.contact_direction_sign * step_mm,
                    fx_n=sample["fx"],
                    fy_n=sample["fy"],
                    torque_nm=sample["torque_nm"],
                    force_settled=False,
                )
                if self.clock() > deadline:
                    return self._fail(result, "contact_timeout")
                self._wait_cycle()

            while self.state == AdmittanceSessionState.SETTLING:
                sample = self._read_sample(result)
                if self.stop_event.is_set():
                    return self._cancel(result)
                step = self._controller.update(sample["fz"], self._dt())
                delta_z_mm = -self.config.contact_direction_sign * step.delta_z_mm
                self._send_delta([0.0, 0.0, delta_z_mm])
                self._trace_step(
                    sample,
                    step,
                    force_settled=step.state.force_in_band,
                    actual_delta_z_mm=delta_z_mm,
                )
                if step.state.force_in_band:
                    self.state = AdmittanceSessionState.HOLDING
                    self._hold_since = self.clock()
                    self._record(result, "force_settled", state=self.state.value)
                elif step.state.at_press_limit:
                    return self._fail(result, "max_press_limit_without_force")
                if self.clock() > deadline:
                    return self._fail(result, "settling_timeout")
                self._wait_cycle()

            while self.state == AdmittanceSessionState.HOLDING:
                sample = self._read_sample(result)
                if self.stop_event.is_set():
                    return self._cancel(result)
                step = self._controller.update(sample["fz"], self._dt())
                delta_z_mm = -self.config.contact_direction_sign * step.delta_z_mm
                self._send_delta([0.0, 0.0, delta_z_mm])
                self._trace_step(
                    sample,
                    step,
                    force_settled=True,
                    actual_delta_z_mm=delta_z_mm,
                )
                if step.state.force_in_band:
                    if self._hold_since is None:
                        self._hold_since = self.clock()
                    elif self.clock() - self._hold_since >= self.config.hold_s:
                        self.state = (
                            AdmittanceSessionState.STROKING
                            if self._stroke_requested()
                            else AdmittanceSessionState.RELEASING
                        )
                        self._record(result, "hold_complete", state=self.state.value)
                else:
                    self._hold_since = None
                self._wait_cycle()

            if self.state == AdmittanceSessionState.HOLDING:
                self.state = AdmittanceSessionState.RELEASING

            if self.state == AdmittanceSessionState.STROKING:
                if not self._run_stroke(result):
                    return self._cancel(result)
                self.state = AdmittanceSessionState.RELEASING
                self._record(result, "stroke_sequence_completed", state=self.state.value)

            if self.state == AdmittanceSessionState.RELEASING:
                self._release()
                self.state = AdmittanceSessionState.RETURNING
                if self._servo_started:
                    self.client.servo_move_end(com_type=0, call_timeout_s=self.config.max_servo_call_s)
                    self._servo_started = False
                if self.return_callback is not None:
                    self.return_callback()
                self.state = AdmittanceSessionState.DONE
                return self._finish(result, "completed")
            return self._finish(result, "completed")
        except Exception as exc:
            return self._fail(result, f"{type(exc).__name__}: {exc}")

    def _make_controller(
        self,
        target_force_n: float,
        *,
        max_speed_mm_s: float | None = None,
    ) -> FzAdmittanceController:
        return FzAdmittanceController(
            FzAdmittanceConfig(
                target_force_n=target_force_n,
                deadband_n=self.config.deadband_n,
                gain_mm_s_n=self.config.gain_mm_s_n,
                max_speed_mm_s=(
                    self.config.max_speed_mm_s
                    if max_speed_mm_s is None
                    else max_speed_mm_s
                ),
                max_press_mm=self.config.max_press_mm,
                max_release_mm=self.config.max_release_mm,
                force_filter_s=self.config.force_filter_s,
            )
        )

    def _read_sample(self, result: AdmittanceSessionResult) -> dict[str, float]:
        started = self.clock()
        code, wrench = self.client.ft_get_force_torque_rcs()
        elapsed = self.clock() - started
        if elapsed > self.config.max_control_cycle_s:
            raise RuntimeError("control_cycle_timeout")
        if code != 0 or len(wrench) < 6 or not all(math.isfinite(float(v)) for v in wrench[:6]):
            raise RuntimeError(f"invalid_ft_read:{code}")
        fx, fy, fz_raw = (float(wrench[0]), float(wrench[1]), float(wrench[2]))
        fz = -fz_raw
        torque = [float(value) for value in wrench[3:6]]
        abs_torque = max(abs(value) for value in torque)
        result.samples += 1
        result.max_abs_fx_n = max(result.max_abs_fx_n, abs(fx))
        result.max_abs_fy_n = max(result.max_abs_fy_n, abs(fy))
        result.max_abs_torque_nm = max(result.max_abs_torque_nm, abs_torque)
        if abs(fx) > self.config.lateral_force_hard_limit_n or abs(fy) > self.config.lateral_force_hard_limit_n:
            raise RuntimeError("lateral_force_hard_limit")
        if abs(fx) > self.config.lateral_force_quality_limit_n or abs(fy) > self.config.lateral_force_quality_limit_n:
            raise RuntimeError("lateral_force_quality_gate")
        if abs(fz) > self.config.force_hard_limit_n or abs_torque > self.config.torque_hard_limit_nm:
            raise RuntimeError("force_or_torque_hard_limit")
        return {
            "fx": fx,
            "fy": fy,
            "fz": fz,
            "fz_raw": fz_raw,
            "torque_nm": abs_torque,
        }

    def _send_delta(self, delta: list[float]) -> None:
        started = self.clock()
        code = self.client.servo_cart_tool_delta(
            delta,
            cmd_t=self.config.servo_cmd_t,
            mode=2,
            call_timeout_s=self.config.max_servo_call_s,
        )
        if self.clock() - started > self.config.max_servo_call_s:
            raise RuntimeError("servo_call_timeout")
        if code != 0:
            raise RuntimeError(f"servocart_failed:{code}")

    def _start_servo(self) -> None:
        if self.client.servo_move_start(com_type=0, call_timeout_s=self.config.max_servo_call_s) != 0:
            raise RuntimeError("servo_move_start_failed")
        self._servo_started = True

    def _release(self) -> None:
        self._record_dummy("release_started")
        if self.release_callback is not None:
            self.release_callback()
        controller = self._make_controller(
            self.config.release_force_n,
            max_speed_mm_s=self.config.release_speed_mm_s,
        )
        deadline = self.clock() + self.config.release_timeout_s
        while True:
            sample_result = AdmittanceSessionResult(self.state, False, "release")
            sample = self._read_sample(sample_result)
            if sample["fz"] <= self.config.release_force_n + self.config.deadband_n:
                return
            step = controller.update(sample["fz"], self._dt())
            delta_z_mm = -self.config.contact_direction_sign * abs(step.delta_z_mm)
            self._send_delta([0.0, 0.0, delta_z_mm])
            self._trace_step(
                sample,
                step,
                force_settled=step.state.force_in_band,
                actual_delta_z_mm=delta_z_mm,
                phase="RELEASING",
            )
            if step.state.at_release_limit or self.clock() > deadline:
                raise RuntimeError("release_timeout_or_limit")
            self._wait_cycle()

    def _stroke_requested(self) -> bool:
        return bool(
            self.config.stroke_enabled
            and self.config.stroke_speed_mm_s > 0.0
            and any(abs(float(distance)) > 1e-9 for distance in self.config.stroke_distances_mm)
        )

    def _stroke_axis_vector(self, axis: str) -> tuple[float, float]:
        normalized = str(axis).lower()
        if normalized == "x":
            return 1.0, 0.0
        if normalized == "y":
            return 0.0, 1.0
        if normalized == "xy":
            diagonal = 1.0 / math.sqrt(2.0)
            return diagonal, diagonal
        raise ValueError(f"unsupported stroke axis: {axis}")

    def _run_stroke(self, result: AdmittanceSessionResult) -> bool:
        """Run configured Tool-X/Y strokes while Fz remains under admittance control."""
        for segment_index, (axis, distance_mm) in enumerate(
            zip(self.config.stroke_axes, self.config.stroke_distances_mm),
            start=1,
        ):
            distance_mm = float(distance_mm)
            axis_name = str(axis).lower()
            if abs(distance_mm) <= 1e-9:
                self._record(
                    result,
                    "stroke_segment_skipped",
                    state=self.state.value,
                    stroke_index=segment_index,
                    stroke_axis=axis_name,
                    stroke_distance_mm=distance_mm,
                )
                continue

            axis_x, axis_y = self._stroke_axis_vector(axis_name)
            self._record(
                result,
                "stroke_segment_started",
                state=self.state.value,
                stroke_index=segment_index,
                stroke_axis=axis_name,
                stroke_distance_mm=distance_mm,
            )
            if not self._run_stroke_leg(
                result,
                axis_name=axis_name,
                axis_x=axis_x,
                axis_y=axis_y,
                distance_mm=distance_mm,
                stroke_phase="OUTBOUND",
            ):
                return False

            if self.config.stroke_return_to_origin:
                self._record(
                    result,
                    "stroke_return_started",
                    state=self.state.value,
                    stroke_index=segment_index,
                    stroke_axis=axis_name,
                    stroke_distance_mm=distance_mm,
                )
                if not self._run_stroke_leg(
                    result,
                    axis_name=axis_name,
                    axis_x=axis_x,
                    axis_y=axis_y,
                    distance_mm=-distance_mm,
                    stroke_phase="RETURN",
                ):
                    return False
                self._record(
                    result,
                    "stroke_return_completed",
                    state=self.state.value,
                    stroke_index=segment_index,
                    stroke_axis=axis_name,
                )

            result.stroke_completed_segments += 1
            self._record(
                result,
                "stroke_segment_completed",
                state=self.state.value,
                stroke_index=segment_index,
                stroke_axis=axis_name,
                stroke_distance_mm=distance_mm,
            )
        return True

    def _run_stroke_leg(
        self,
        result: AdmittanceSessionResult,
        *,
        axis_name: str,
        axis_x: float,
        axis_y: float,
        distance_mm: float,
        stroke_phase: str,
    ) -> bool:
        signed_progress_mm = 0.0
        target_abs_mm = abs(float(distance_mm))
        direction = 1.0 if distance_mm >= 0.0 else -1.0
        step_limit_mm = self.config.stroke_speed_mm_s / self.config.loop_hz

        while abs(signed_progress_mm) < target_abs_mm - 1e-9:
            if self.stop_event.is_set():
                return False
            sample = self._read_sample(result)
            step = self._controller.update(sample["fz"], self._dt())
            tangential_step_mm = min(step_limit_mm, target_abs_mm - abs(signed_progress_mm))
            # Preserve contact: pause tangential travel while Fz is outside
            # the quality band, while continuing normal force correction.
            if not step.state.force_in_band:
                tangential_step_mm = 0.0
            signed_step_mm = direction * tangential_step_mm
            delta_z_mm = -self.config.contact_direction_sign * step.delta_z_mm
            self._send_delta(
                [
                    axis_x * signed_step_mm,
                    axis_y * signed_step_mm,
                    delta_z_mm,
                ]
            )
            signed_progress_mm += signed_step_mm
            result.stroke_total_distance_mm += abs(signed_step_mm)
            self._trace_step(
                sample,
                step,
                force_settled=step.state.force_in_band,
                actual_delta_z_mm=delta_z_mm,
                phase="STROKING",
                stroke_axis=axis_name,
                stroke_distance_mm=distance_mm,
                stroke_progress_mm=abs(signed_progress_mm),
                stroke_phase=stroke_phase,
                stroke_delta_x_mm=axis_x * signed_step_mm,
                stroke_delta_y_mm=axis_y * signed_step_mm,
            )
            self._wait_cycle()
        return True

    def _wait_cycle(self) -> None:
        self.sleep(1.0 / self.config.loop_hz)

    def _dt(self) -> float:
        return 1.0 / self.config.loop_hz

    def _record(self, result: AdmittanceSessionResult, event: str, **fields: Any) -> None:
        payload = {"event": event, "stamp_monotonic": self.clock(), "state": self.state.value, **fields}
        result.events.append(payload)
        self._trace_payload(payload)

    def _trace_step(
        self,
        sample: dict[str, float],
        step: Any,
        *,
        force_settled: bool,
        actual_delta_z_mm: float,
        phase: str | None = None,
        stroke_axis: str | None = None,
        stroke_distance_mm: float | None = None,
        stroke_progress_mm: float | None = None,
        stroke_phase: str | None = None,
        stroke_delta_x_mm: float | None = None,
        stroke_delta_y_mm: float | None = None,
    ) -> None:
        payload = {
            "event": "force_sample",
            "stamp_monotonic": self.clock(),
            "state": phase or self.state.value,
            "fz_raw_n": sample["fz_raw"],
            "fz_filtered_n": step.state.filtered_fz_n,
            "target_force_n": self._controller.config.target_force_n,
            "force_error_n": step.state.force_error_n,
            "command_speed_mm_s": step.state.command_speed_mm_s,
            "press_travel_mm": step.state.press_travel_mm,
            "delta_z_mm": actual_delta_z_mm,
            "controller_delta_z_mm": step.delta_z_mm,
            "fx_n": sample["fx"],
            "fy_n": sample["fy"],
            "torque_nm": sample["torque_nm"],
            "force_settled": bool(force_settled),
            "at_press_limit": bool(step.state.at_press_limit),
            "at_release_limit": bool(step.state.at_release_limit),
            "dt_s": step.dt_s,
        }
        if stroke_axis is not None:
            payload.update(
                {
                    "stroke_axis": stroke_axis,
                    "stroke_distance_mm": stroke_distance_mm,
                    "stroke_progress_mm": stroke_progress_mm,
                    "stroke_phase": stroke_phase,
                    "stroke_delta_x_mm": stroke_delta_x_mm,
                    "stroke_delta_y_mm": stroke_delta_y_mm,
                }
            )
        self._trace_payload(payload)

    def _trace(self, event: str, **fields: Any) -> None:
        self._trace_payload(
            {
                "event": event,
                "stamp_monotonic": self.clock(),
                **fields,
            }
        )

    def _trace_payload(self, payload: dict[str, Any]) -> None:
        if self.trace_callback is None:
            return
        try:
            self.trace_callback(dict(payload))
        except Exception:
            # Diagnostics must never change force-control behavior.
            return

    def _record_dummy(self, event: str) -> None:
        self._trace_payload({"event": event, "stamp_monotonic": self.clock(), "state": self.state.value})

    def _finish(self, result: AdmittanceSessionResult, reason: str) -> AdmittanceSessionResult:
        self._cleanup()
        result.state = self.state
        result.success = self.state == AdmittanceSessionState.DONE
        result.reason = reason
        result.contact_travel_mm = self._contact_travel_mm
        return result

    def _cancel(self, result: AdmittanceSessionResult) -> AdmittanceSessionResult:
        """Gracefully cancel after contact, while remaining fail-closed."""
        if not self._contact_detected or not self._servo_started:
            self.state = AdmittanceSessionState.CANCELLED
            return self._finish(result, "stop_requested_no_contact")
        try:
            self.state = AdmittanceSessionState.RELEASING
            self._release()
            self.state = AdmittanceSessionState.RETURNING
            if self._servo_started:
                self.client.servo_move_end(com_type=0, call_timeout_s=self.config.max_servo_call_s)
                self._servo_started = False
            if self.return_callback is not None:
                self.return_callback()
            self.state = AdmittanceSessionState.CANCELLED
            return self._finish(result, "stop_requested_released")
        except Exception as exc:
            return self._fail(result, f"cancel_release_failed:{type(exc).__name__}: {exc}")

    def _fail(self, result: AdmittanceSessionResult, reason: str) -> AdmittanceSessionResult:
        self.state = AdmittanceSessionState.FAILED
        try:
            if self._servo_started:
                self.client.stop_motion(call_timeout_s=self.config.max_servo_call_s)
        finally:
            self._cleanup()
        result.state = self.state
        result.success = False
        result.reason = reason
        return result

    def _cleanup(self) -> None:
        if self._cleanup_done:
            return
        self._cleanup_done = True
        if self._servo_started:
            self.client.servo_move_end(com_type=0, call_timeout_s=self.config.max_servo_call_s)
            self._servo_started = False
        self.client.ft_activate(False, call_timeout_s=self.config.max_servo_call_s)
