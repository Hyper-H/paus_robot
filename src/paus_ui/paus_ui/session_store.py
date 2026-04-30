from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml

from paus_marker_ros2.semi_auto_calibration import TrajectoryValidationError, load_trajectory


class SessionStore:
    def __init__(self, *, session_root_path: str | Path, trajectory_path: str | Path) -> None:
        self.session_root_path = Path(session_root_path)
        self.trajectory_path = Path(trajectory_path)

    def list_sessions(self) -> list[dict[str, Any]]:
        if not self.session_root_path.exists():
            return []
        sessions: list[dict[str, Any]] = []
        for path in sorted((item for item in self.session_root_path.iterdir() if item.is_dir()), reverse=True):
            report = self._read_yaml(path / "report.yaml")
            samples_path = path / "samples.jsonl"
            sample_count = int(report.get("sample_count", 0) or self._count_jsonl(samples_path))
            sessions.append(
                {
                    "id": path.name,
                    "path": str(path),
                    "report_path": str(path / "report.yaml"),
                    "sample_count": sample_count,
                    "has_report": (path / "report.yaml").exists(),
                    "has_samples": samples_path.exists(),
                    "method": report.get("method"),
                    "residuals": report.get("residuals", {}),
                }
            )
        return sessions

    def read_report(self, session_id: str) -> dict[str, Any]:
        return self._read_yaml(self._session_path(session_id) / "report.yaml")

    def read_samples(self, session_id: str) -> list[dict[str, Any]]:
        session_path = self._session_path(session_id)
        samples = self._read_jsonl(session_path / "samples.jsonl")
        if not samples:
            report_samples = self.read_report(session_id).get("samples", [])
            if isinstance(report_samples, list):
                samples = [item for item in report_samples if isinstance(item, dict)]
        for index, sample in enumerate(samples, start=1):
            sample["row_index"] = index
            image_path = sample.get("image_path")
            sample["has_image"] = bool(image_path and Path(str(image_path)).exists())
        return samples

    def read_run_events(self, session_id: str) -> list[dict[str, Any]]:
        return self._read_jsonl(self._session_path(session_id) / "run.log")

    def read_session_waypoints(self, session_id: str) -> list[dict[str, Any]]:
        trajectory = self.read_waypoints()
        samples_by_index: dict[int, dict[str, Any]] = {}
        for sample in self.read_samples(session_id):
            for key in ("sample_index", "row_index"):
                try:
                    sample_index = int(sample.get(key))
                except (TypeError, ValueError):
                    continue
                samples_by_index.setdefault(sample_index, sample)
        records: dict[str, dict[str, Any]] = {}
        for waypoint in trajectory.get("waypoints", []):
            if not isinstance(waypoint, dict):
                continue
            name = str(waypoint.get("name", ""))
            records[name] = {
                "waypoint": waypoint,
                "name": name,
                "status": "pending",
                "reprojection_error_px": _nested_get(waypoint, ["record_quality", "reprojection_error_px"]),
                "board_margin_px": _nested_get(waypoint, ["record_quality", "board_margin_px"]),
                "capture": waypoint.get("capture", True),
                "reason": "",
                "sample_index": None,
                "image_path": None,
            }

        for event in self.read_run_events(session_id):
            waypoint = event.get("waypoint")
            waypoint_name = event.get("waypoint_name")
            if isinstance(waypoint, dict):
                waypoint_name = waypoint.get("name", waypoint_name)
            if not waypoint_name:
                continue
            name = str(waypoint_name)
            record = records.setdefault(
                name,
                {
                    "waypoint": waypoint if isinstance(waypoint, dict) else {"name": name},
                    "name": name,
                    "status": "pending",
                    "capture": True,
                    "reason": "",
                    "sample_index": None,
                    "image_path": None,
                },
            )
            event_name = str(event.get("event", ""))
            if event_name == "waypoint_capture_skipped":
                record.update(
                    {
                        "status": "skipped",
                        "reason": str(event.get("reason", "")),
                    }
                )
            elif event_name == "waypoint_sample_captured":
                try:
                    sample_index = int(event.get("sample_index"))
                except (TypeError, ValueError):
                    sample_index = None
                sample = samples_by_index.get(sample_index) if sample_index is not None else None
                record.update(
                    {
                        "status": "accepted",
                        "sample_index": sample_index,
                        "reprojection_error_px": _first_present(sample, event, "reprojection_error_px"),
                        "board_margin_px": _first_present(sample, event, "board_margin_px"),
                        "image_path": _first_present(sample, event, "image_path"),
                        "camera_to_board_translation_m": _matrix_translation(sample.get("camera_to_board_matrix") if sample else None),
                        "tcp_pose_mmdeg": sample.get("tcp_pose_mmdeg") if sample else None,
                        "reason": "",
                    }
                )
            elif event_name in {"waypoint_motion_started", "waypoint_reached", "waypoint_capture_started"} and record.get("status") == "pending":
                record["status"] = "running"

        return list(records.values())

    def read_waypoints(self) -> dict[str, Any]:
        if not self.trajectory_path.exists():
            return {"trajectory_path": str(self.trajectory_path), "waypoints": [], "error": "Trajectory YAML does not exist."}
        try:
            trajectory = load_trajectory(self.trajectory_path)
        except TrajectoryValidationError as exc:
            return {"trajectory_path": str(self.trajectory_path), "waypoints": [], "error": str(exc)}
        return {
            "trajectory_path": str(self.trajectory_path),
            "tool_id": trajectory.tool_id,
            "user_id": trajectory.user_id,
            "defaults": {
                "motion": trajectory.default_motion,
                "vel": trajectory.default_vel,
                "acc": trajectory.default_acc,
                "dwell_s": trajectory.default_dwell_s,
            },
            "waypoints": [waypoint.to_payload() for waypoint in trajectory.waypoints],
            "error": None,
        }

    def sample_image_path(self, session_id: str, row_index: int) -> Path | None:
        samples = self.read_samples(session_id)
        if row_index < 1 or row_index > len(samples):
            return None
        sample = samples[row_index - 1]
        image_path = sample.get("image_path")
        if image_path:
            candidate = Path(str(image_path))
            if candidate.exists():
                return candidate
        fallback = self._session_path(session_id) / "images" / f"sample_{row_index:03d}.png"
        return fallback if fallback.exists() else None

    def _session_path(self, session_id: str) -> Path:
        if "/" in session_id or "\\" in session_id or session_id in {"", ".", ".."}:
            raise ValueError(f"Invalid session id: {session_id!r}")
        path = self.session_root_path / session_id
        if not path.exists() or not path.is_dir():
            raise FileNotFoundError(f"Session does not exist: {path}")
        return path

    def _read_yaml(self, path: Path) -> dict[str, Any]:
        if not path.exists():
            return {}
        with path.open("r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}
        return payload if isinstance(payload, dict) else {}

    def _read_jsonl(self, path: Path) -> list[dict[str, Any]]:
        if not path.exists():
            return []
        rows: list[dict[str, Any]] = []
        with path.open("r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if not line:
                    continue
                try:
                    payload = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(payload, dict):
                    rows.append(payload)
        return rows

    def _count_jsonl(self, path: Path) -> int:
        return len(self._read_jsonl(path))


def _nested_get(payload: dict[str, Any], keys: list[str]) -> Any:
    value: Any = payload
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _first_present(primary: dict[str, Any] | None, fallback: dict[str, Any], key: str) -> Any:
    if primary and primary.get(key) is not None:
        return primary.get(key)
    return fallback.get(key)


def _matrix_translation(matrix: Any) -> list[float] | None:
    if not isinstance(matrix, list) or len(matrix) < 3:
        return None
    try:
        return [float(matrix[0][3]), float(matrix[1][3]), float(matrix[2][3])]
    except (TypeError, ValueError, IndexError):
        return None
