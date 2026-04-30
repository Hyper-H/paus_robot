from __future__ import annotations

from dataclasses import asdict
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


TRAJECTORY_SCHEMA_VERSION = 1
SUPPORTED_MOTION_MODES = {"movej"}


class TrajectoryValidationError(ValueError):
    pass


@dataclass
class CalibrationWaypoint:
    name: str
    motion: str
    joint_deg: list[float]
    expected_tcp_pose_mmdeg: list[float]
    vel: float
    acc: float
    dwell_s: float
    capture: bool = True
    record_quality: dict[str, Any] | None = None

    def to_payload(self) -> dict[str, Any]:
        payload = asdict(self)
        if payload["record_quality"] is None:
            payload.pop("record_quality")
        return payload


@dataclass
class CalibrationTrajectory:
    version: int
    tool_id: int
    user_id: int
    default_motion: str
    default_vel: float
    default_acc: float
    default_dwell_s: float
    waypoints: list[CalibrationWaypoint]

    def to_payload(self) -> dict[str, Any]:
        return {
            "version": self.version,
            "tool_id": self.tool_id,
            "user_id": self.user_id,
            "defaults": {
                "motion": self.default_motion,
                "vel": self.default_vel,
                "acc": self.default_acc,
                "dwell_s": self.default_dwell_s,
            },
            "waypoints": [waypoint.to_payload() for waypoint in self.waypoints],
        }


def _coerce_float_list(value: Any, *, field_name: str, length: int) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise TrajectoryValidationError(f"{field_name} must be a list with {length} numbers.")
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise TrajectoryValidationError(f"{field_name} must contain only numbers.") from exc


def _coerce_bool(value: Any, *, field_name: str) -> bool:
    if isinstance(value, bool):
        return value
    raise TrajectoryValidationError(f"{field_name} must be true or false.")


def _parse_waypoint(payload: dict[str, Any], defaults: dict[str, Any], index: int) -> CalibrationWaypoint:
    if not isinstance(payload, dict):
        raise TrajectoryValidationError(f"waypoints[{index}] must be a mapping.")
    motion = str(payload.get("motion", defaults.get("motion", "movej"))).strip().lower()
    if motion not in SUPPORTED_MOTION_MODES:
        raise TrajectoryValidationError(f"waypoints[{index}].motion unsupported: {motion!r}.")
    return CalibrationWaypoint(
        name=str(payload.get("name", f"waypoint_{index + 1:03d}")),
        motion=motion,
        joint_deg=_coerce_float_list(payload.get("joint_deg"), field_name=f"waypoints[{index}].joint_deg", length=6),
        expected_tcp_pose_mmdeg=_coerce_float_list(
            payload.get("expected_tcp_pose_mmdeg"),
            field_name=f"waypoints[{index}].expected_tcp_pose_mmdeg",
            length=6,
        ),
        vel=float(payload.get("vel", defaults.get("vel", 10.0))),
        acc=float(payload.get("acc", defaults.get("acc", 10.0))),
        dwell_s=float(payload.get("dwell_s", defaults.get("dwell_s", 0.5))),
        capture=_coerce_bool(payload.get("capture", True), field_name=f"waypoints[{index}].capture"),
        record_quality=payload.get("record_quality") if isinstance(payload.get("record_quality"), dict) else None,
    )


def load_trajectory(path: str | Path) -> CalibrationTrajectory:
    trajectory_path = Path(path)
    with trajectory_path.open("r", encoding="utf-8") as handle:
        payload = yaml.safe_load(handle) or {}
    if not isinstance(payload, dict):
        raise TrajectoryValidationError("Trajectory YAML must contain a mapping.")

    version = int(payload.get("version", TRAJECTORY_SCHEMA_VERSION))
    if version != TRAJECTORY_SCHEMA_VERSION:
        raise TrajectoryValidationError(f"Unsupported trajectory version: {version}.")

    defaults = payload.get("defaults", {})
    if defaults is None:
        defaults = {}
    if not isinstance(defaults, dict):
        raise TrajectoryValidationError("defaults must be a mapping.")

    default_motion = str(defaults.get("motion", "movej")).strip().lower()
    if default_motion not in SUPPORTED_MOTION_MODES:
        raise TrajectoryValidationError(f"defaults.motion unsupported: {default_motion!r}.")

    waypoint_payloads = payload.get("waypoints", [])
    if not isinstance(waypoint_payloads, list):
        raise TrajectoryValidationError("waypoints must be a list.")
    waypoints = [_parse_waypoint(item, defaults, index) for index, item in enumerate(waypoint_payloads)]
    if not waypoints:
        raise TrajectoryValidationError("Trajectory must contain at least one waypoint.")

    return CalibrationTrajectory(
        version=version,
        tool_id=int(payload.get("tool_id", 0)),
        user_id=int(payload.get("user_id", 0)),
        default_motion=default_motion,
        default_vel=float(defaults.get("vel", 10.0)),
        default_acc=float(defaults.get("acc", 10.0)),
        default_dwell_s=float(defaults.get("dwell_s", 0.5)),
        waypoints=waypoints,
    )


def empty_trajectory(*, tool_id: int, user_id: int, default_vel: float, default_acc: float, default_dwell_s: float) -> CalibrationTrajectory:
    return CalibrationTrajectory(
        version=TRAJECTORY_SCHEMA_VERSION,
        tool_id=int(tool_id),
        user_id=int(user_id),
        default_motion="movej",
        default_vel=float(default_vel),
        default_acc=float(default_acc),
        default_dwell_s=float(default_dwell_s),
        waypoints=[],
    )


def save_trajectory(trajectory: CalibrationTrajectory, path: str | Path) -> None:
    if not trajectory.waypoints:
        raise TrajectoryValidationError("Trajectory must contain at least one waypoint.")
    trajectory_path = Path(path)
    trajectory_path.parent.mkdir(parents=True, exist_ok=True)
    with trajectory_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(trajectory.to_payload(), handle, sort_keys=False, allow_unicode=True)


def build_recorded_waypoint(
    *,
    index: int,
    joint_deg: list[float],
    tcp_pose_mmdeg: list[float],
    vel: float,
    acc: float,
    dwell_s: float,
    capture: bool = True,
    record_quality: dict[str, Any] | None = None,
) -> CalibrationWaypoint:
    return CalibrationWaypoint(
        name=f"waypoint_{index:03d}",
        motion="movej",
        joint_deg=[float(value) for value in joint_deg],
        expected_tcp_pose_mmdeg=[float(value) for value in tcp_pose_mmdeg],
        vel=float(vel),
        acc=float(acc),
        dwell_s=float(dwell_s),
        capture=bool(capture),
        record_quality=record_quality,
    )


def create_session_dir(root_path: str | Path, *, now: datetime | None = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y-%m-%d_%H%M%S")
    root = Path(root_path)
    candidate = root / timestamp
    suffix = 2
    while candidate.exists():
        candidate = root / f"{timestamp}_{suffix:02d}"
        suffix += 1
    candidate.mkdir(parents=True, exist_ok=False)
    (candidate / "images").mkdir(parents=True, exist_ok=True)
    return candidate
