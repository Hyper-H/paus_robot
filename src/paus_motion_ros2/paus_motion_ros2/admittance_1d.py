"""Pure one-dimensional Fz admittance control.

The module deliberately has no ROS or FAIRINO dependency.  It converts a
force error into a bounded Tool-Z displacement increment; a caller owns the
robot connection and decides how that increment is sent.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FzAdmittanceConfig:
    target_force_n: float = 2.0
    deadband_n: float = 0.2
    gain_mm_s_n: float = 0.2
    max_speed_mm_s: float = 0.3
    max_press_mm: float = 10.0
    max_release_mm: float = 2.0
    force_filter_s: float = 0.25

    def __post_init__(self) -> None:
        for name in (
            "target_force_n",
            "deadband_n",
            "gain_mm_s_n",
            "max_speed_mm_s",
            "max_press_mm",
            "max_release_mm",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and >= 0")
        if self.target_force_n <= 0.0:
            raise ValueError("target_force_n must be > 0")
        if self.deadband_n <= 0.0 or self.gain_mm_s_n <= 0.0 or self.max_speed_mm_s <= 0.0:
            raise ValueError("deadband, gain and max_speed must be > 0")
        if not math.isfinite(float(self.force_filter_s)) or self.force_filter_s < 0.0:
            raise ValueError("force_filter_s must be finite and >= 0")


@dataclass(frozen=True)
class FzAdmittanceState:
    filtered_fz_n: float
    force_error_n: float
    command_speed_mm_s: float
    press_travel_mm: float
    force_in_band: bool
    at_press_limit: bool
    at_release_limit: bool


@dataclass(frozen=True)
class FzAdmittanceStep:
    state: FzAdmittanceState
    delta_z_mm: float
    dt_s: float


class FzAdmittanceController:
    """First-order velocity admittance for positive compressive Fz."""

    def __init__(self, config: FzAdmittanceConfig) -> None:
        self.config = config
        self._filtered_fz_n: float | None = None
        self._press_travel_mm = 0.0

    @property
    def state(self) -> FzAdmittanceState | None:
        if self._filtered_fz_n is None:
            return None
        return self._state(self._filtered_fz_n, 0.0)

    @property
    def press_travel_mm(self) -> float:
        return self._press_travel_mm

    def reset(self) -> None:
        self._filtered_fz_n = None
        self._press_travel_mm = 0.0

    def update(self, measured_fz_n: float, dt_s: float) -> FzAdmittanceStep:
        measured = float(measured_fz_n)
        dt = float(dt_s)
        if not math.isfinite(measured):
            raise ValueError("measured_fz_n must be finite")
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt_s must be finite and > 0")

        if self._filtered_fz_n is None:
            filtered = measured
        elif self.config.force_filter_s <= 0.0:
            filtered = measured
        else:
            alpha = min(1.0, dt / (self.config.force_filter_s + dt))
            filtered = self._filtered_fz_n + alpha * (measured - self._filtered_fz_n)
        self._filtered_fz_n = filtered

        error = self.config.target_force_n - filtered
        if abs(error) <= self.config.deadband_n:
            speed = 0.0
        else:
            speed = max(-self.config.max_speed_mm_s, min(self.config.max_speed_mm_s, self.config.gain_mm_s_n * error))

        previous = self._press_travel_mm
        requested = previous + speed * dt
        self._press_travel_mm = max(-self.config.max_release_mm, min(self.config.max_press_mm, requested))
        delta_travel = self._press_travel_mm - previous
        # Positive controller speed means press deeper.  Tool-Z press is
        # represented by negative Cartesian Z in the existing client path.
        delta_z_mm = -delta_travel
        return FzAdmittanceStep(
            state=self._state(filtered, error),
            delta_z_mm=delta_z_mm,
            dt_s=dt,
        )

    def _state(self, filtered: float, error: float) -> FzAdmittanceState:
        return FzAdmittanceState(
            filtered_fz_n=float(filtered),
            force_error_n=float(error),
            command_speed_mm_s=(
                0.0
                if abs(error) <= self.config.deadband_n
                else max(
                    -self.config.max_speed_mm_s,
                    min(self.config.max_speed_mm_s, self.config.gain_mm_s_n * error),
                )
            ),
            press_travel_mm=float(self._press_travel_mm),
            force_in_band=abs(error) <= self.config.deadband_n,
            at_press_limit=self._press_travel_mm >= self.config.max_press_mm - 1e-9,
            at_release_limit=self._press_travel_mm <= -self.config.max_release_mm + 1e-9,
        )
