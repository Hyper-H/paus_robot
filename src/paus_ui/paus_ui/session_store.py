from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import yaml

import numpy as np

from paus_marker_ros2.semi_auto_calibration import TrajectoryValidationError, load_trajectory
from paus_perception import make_transform_matrix, rotation_matrix_to_rpy_deg, rpy_deg_to_rotation_matrix

from .operator_messages import classify_operator_message


class SessionStore:
    def __init__(
        self,
        *,
        session_root_path: str | Path,
        trajectory_path: str | Path,
        max_reprojection_error_px: float = 0.0,
        min_board_margin_px: float = 10.0,
    ) -> None:
        self.session_root_path = Path(session_root_path)
        self.trajectory_path = Path(trajectory_path)
        self.max_reprojection_error_px = float(max_reprojection_error_px)
        self.min_board_margin_px = float(min_board_margin_px)

    def list_sessions(self) -> list[dict[str, Any]]:
        if not self.session_root_path.exists():
            return []
        sessions: list[dict[str, Any]] = []
        for path in sorted((item for item in self.session_root_path.iterdir() if item.is_dir()), reverse=True):
            report_path = path / "report.yaml"
            samples_path = path / "samples.jsonl"
            report = self._read_yaml(report_path)
            samples = self._read_jsonl(samples_path)
            events = self._read_jsonl(path / "run.log")
            counts = self._counts_for_session(path=path, report=report, samples=samples, events=events)
            report_error = report.get("error")
            trajectory_error = counts.get("trajectory_error")
            invalid_reason = report_error or trajectory_error
            has_report = report_path.exists()
            has_solution = bool(report.get("base_to_camera") or report.get("residuals")) and not invalid_reason
            sessions.append(
                {
                    "id": path.name,
                    "path": str(path),
                    "report_path": str(report_path),
                    "sample_count": counts["sample_count"],
                    "accepted_count": counts["accepted"],
                    "skipped_count": counts["skipped"],
                    "pending_count": counts["pending"],
                    "has_report": has_report,
                    "has_solution": has_solution,
                    "is_invalid": bool(invalid_reason),
                    "report_error": report_error,
                    "trajectory_error": trajectory_error,
                    "has_samples": samples_path.exists(),
                    "method": report.get("method"),
                    "residuals": report.get("residuals", {}),
                    "empty_reason": invalid_reason or (None if has_report else "该 session 暂无 report.yaml。"),
                }
            )
        return sessions

    def latest_valid_session_id(self) -> str | None:
        sessions = self.list_sessions()
        for session in sessions:
            if session.get("has_solution"):
                return str(session["id"])
        for session in sessions:
            if session.get("has_report") and not session.get("is_invalid"):
                return str(session["id"])
        return None

    def read_report(self, session_id: str) -> dict[str, Any]:
        session_path = self._session_path(session_id)
        report_path = session_path / "report.yaml"
        report = self._read_yaml(report_path)
        samples = self._read_samples_without_report_fallback(session_path)
        events = self._read_jsonl(session_path / "run.log")
        counts = self._counts_for_session(path=session_path, report=report, samples=samples, events=events)
        residuals = report.get("residuals", {}) if isinstance(report.get("residuals"), dict) else {}
        report_error = report.get("error")
        trajectory_error = counts.get("trajectory_error")
        invalid_reason = report_error or trajectory_error
        has_report = report_path.exists()
        has_solution = bool(report.get("base_to_camera") or residuals) and not invalid_reason
        shaped = dict(report)
        report_samples = shaped.get("samples")
        if isinstance(report_samples, list):
            shaped["samples"] = [
                self._normalize_sample_fields(s) for s in report_samples if isinstance(s, dict)
            ]
        shaped.update(
            {
                "session_id": session_id,
                "session_dir": str(session_path),
                "report_path": str(report_path),
                "has_report": has_report,
                "has_solution": has_solution,
                "is_invalid": bool(invalid_reason),
                "report_error": report_error,
                "trajectory_error": trajectory_error,
                "sample_count": int(report.get("sample_count", counts["sample_count"]) or counts["sample_count"]),
                "accepted_count": counts["accepted"],
                "skipped_count": counts["skipped"],
                "pending_count": counts["pending"],
                "residuals": residuals,
                "residual_comparison": self._residual_comparison(residuals),
                "empty_reason": invalid_reason or (None if has_solution else ("report.yaml 存在，但该 session 尚未完成求解。" if has_report else "该 session 暂无 report.yaml。")),
            }
        )
        return shaped

    def read_samples(self, session_id: str) -> list[dict[str, Any]]:
        session_path = self._session_path(session_id)
        samples = self._read_samples_without_report_fallback(session_path)
        if not samples and (session_path / "report.yaml").exists():
            report_samples = self._read_yaml(session_path / "report.yaml").get("samples", [])
            if isinstance(report_samples, list):
                samples = [item for item in report_samples if isinstance(item, dict)]
        for index, sample in enumerate(samples, start=1):
            sample["row_index"] = index
            image_path = sample.get("image_path")
            explicit_image_path = Path(str(image_path)) if image_path else None
            fallback_image_path = session_path / "images" / f"sample_{index:03d}.png"
            resolved_image_path = None
            if explicit_image_path is not None and explicit_image_path.exists():
                resolved_image_path = explicit_image_path
            elif fallback_image_path.exists():
                resolved_image_path = fallback_image_path
            sample["has_image"] = resolved_image_path is not None
            if resolved_image_path is not None:
                sample["image_path"] = str(resolved_image_path)
            sample["camera_to_board_translation_m"] = sample.get("camera_to_board_translation_m") or _matrix_translation(sample.get("camera_to_board_matrix"))
            sample["camera_to_board_rotation_rpy_deg"] = sample.get("camera_to_board_rotation_rpy_deg") or _matrix_rotation_rpy_deg(sample.get("camera_to_board_matrix"))
            sample["board_angle_deg"] = sample.get("board_angle_deg") if sample.get("board_angle_deg") is not None else _matrix_board_angle_deg(sample.get("camera_to_board_matrix"))
            sample["capture_time_s"] = sample.get("image_header_time_s") or sample.get("image_received_time_s")
            sample["thresholds"] = self._quality_flags(sample.get("reprojection_error_px"), sample.get("board_margin_px"))
        return samples

    def per_sample_residuals(self, session_id: str) -> list[dict[str, Any]]:
        session_path = self._session_path(session_id)
        report = self._read_yaml(session_path / "report.yaml")
        base_to_camera = report.get("base_to_camera")
        tool_to_board_cfg = report.get("tool_to_board")
        if not base_to_camera or not tool_to_board_cfg:
            return []
        if isinstance(base_to_camera, dict):
            base_to_camera_matrix = _matrix_from_dict(base_to_camera)
        else:
            base_to_camera_matrix = _parse_4x4(base_to_camera)
        tool_to_board_matrix = _build_4x4_from_transform(tool_to_board_cfg)
        if base_to_camera_matrix is None or tool_to_board_matrix is None:
            return []
        samples = self.read_samples(session_id)
        result: list[dict[str, Any]] = []
        for sample in samples:
            base_to_tool = _parse_4x4(sample.get("base_to_tool_matrix"))
            camera_to_board = _parse_4x4(sample.get("camera_to_board_matrix"))
            if base_to_tool is None or camera_to_board is None:
                result.append({"row_index": sample.get("row_index"), "error": "Missing matrix data"})
                continue
            measured_board = base_to_tool @ tool_to_board_matrix
            estimated_board = base_to_camera_matrix @ camera_to_board
            t_residual_mm = float(np.linalg.norm(estimated_board[:3, 3] - measured_board[:3, 3]) * 1000.0)
            R_diff = estimated_board[:3, :3] @ measured_board[:3, :3].T
            trace_val = float(np.clip((np.trace(R_diff) - 1.0) / 2.0, -1.0, 1.0))
            r_residual_deg = float(np.degrees(np.arccos(trace_val)))
            result.append({
                "row_index": sample.get("row_index"),
                "name": sample.get("waypoint_name", ""),
                "translation_residual_mm": round(t_residual_mm, 3),
                "rotation_residual_deg": round(r_residual_deg, 4),
            })
        return result

    def sample_for_row(self, session_id: str, row_index: int) -> dict[str, Any] | None:
        samples = self.read_samples(session_id)
        if row_index < 1 or row_index > len(samples):
            return None
        return samples[row_index - 1]

    def read_run_events(self, session_id: str) -> list[dict[str, Any]]:
        return self._read_jsonl(self._session_path(session_id) / "run.log")

    def read_session_waypoints(self, session_id: str) -> list[dict[str, Any]]:
        session_path = self._session_path(session_id)
        trajectory = self._read_waypoints_from_path(self._trajectory_path_for_session(session_path))
        samples = self.read_samples(session_id)
        sample_residuals = self.per_sample_residuals(session_id)
        residuals_by_row: dict[int, dict[str, Any]] = {}
        for res in sample_residuals:
            row_index = res.get("row_index")
            if row_index is not None:
                residuals_by_row[row_index] = res
        samples_by_index: dict[int, dict[str, Any]] = {}
        for sample in samples:
            for key in ("sample_index", "row_index"):
                try:
                    sample_index = int(sample.get(key))
                except (TypeError, ValueError):
                    continue
                samples_by_index.setdefault(sample_index, sample)

        records: dict[str, dict[str, Any]] = {}
        for display_index, waypoint in enumerate(trajectory.get("waypoints", []), start=1):
            if not isinstance(waypoint, dict):
                continue
            name = str(waypoint.get("name", ""))
            record_quality = waypoint.get("record_quality") if isinstance(waypoint.get("record_quality"), dict) else {}
            records[name] = self._waypoint_record(
                session_id=session_id,
                display_index=display_index,
                name=name,
                waypoint=waypoint,
                status="pending",
                result="-",
                reason="",
                sample=None,
                reprojection_error_px=record_quality.get("reprojection_error_px"),
                board_margin_px=record_quality.get("board_margin_px"),
            )

        events = self.read_run_events(session_id)
        for event in events:
            waypoint = event.get("waypoint")
            waypoint_name = event.get("waypoint_name")
            if isinstance(waypoint, dict):
                waypoint_name = waypoint.get("name", waypoint_name)
            if not waypoint_name:
                continue
            name = str(waypoint_name)
            event_name = str(event.get("event", ""))
            if event_name == "waypoint_deleted":
                records.pop(name, None)
                continue
            record_quality = waypoint.get("record_quality") if isinstance(waypoint, dict) and isinstance(waypoint.get("record_quality"), dict) else {}
            record = records.setdefault(
                name,
                self._waypoint_record(
                    session_id=session_id,
                    display_index=len(records) + 1,
                    name=name,
                    waypoint=waypoint if isinstance(waypoint, dict) else {"name": name},
                    status="pending",
                    result="-",
                    reason="",
                    sample=None,
                    reprojection_error_px=record_quality.get("reprojection_error_px"),
                    board_margin_px=record_quality.get("board_margin_px"),
                ),
            )
            if event_name in {"waypoint_capture_skipped", "waypoint_capture_disabled", "waypoint_dry_run_complete"}:
                reason = str(event.get("reason", event.get("message", "")))
                if event_name == "waypoint_capture_disabled" and not reason:
                    reason = "capture=false"
                if event_name == "waypoint_dry_run_complete" and not reason:
                    reason = "dry-run"
                record.update(
                    self._waypoint_record(
                        session_id=session_id,
                        display_index=record.get("index"),
                        name=name,
                        waypoint=record.get("waypoint", {"name": name}),
                        status="skipped",
                        result="DRY" if event_name == "waypoint_dry_run_complete" else ("SKIP" if event_name == "waypoint_capture_disabled" else "FAIL"),
                        reason=reason,
                        sample=None,
                        reprojection_error_px=event.get("reprojection_error_px", record.get("reprojection_error_px")),
                        board_margin_px=event.get("board_margin_px", record.get("board_margin_px")),
                    )
                )
            elif event_name in {"waypoint_sample_captured", "sample_captured"}:
                try:
                    event_sample_index = int(event.get("sample_index"))
                except (TypeError, ValueError):
                    event_sample_index = None
                sample = samples_by_index.get(event_sample_index) if event_sample_index is not None else None
                sample_row = sample.get("row_index") if sample else None
                residuals = residuals_by_row.get(sample_row, {}) if sample_row else {}
                record.update(
                    self._waypoint_record(
                        session_id=session_id,
                        display_index=record.get("index"),
                        name=name,
                        waypoint=record.get("waypoint", {"name": name}),
                        status="accepted",
                        result="OK",
                        reason="",
                        sample=sample,
                        event_sample_index=event_sample_index,
                        reprojection_error_px=_first_present(sample, event, "reprojection_error_px"),
                        board_margin_px=_first_present(sample, event, "board_margin_px"),
                        residuals=residuals,
                    )
                )
            elif event_name in {"waypoint_motion_started", "waypoint_reached", "waypoint_capture_started"} and record.get("status") == "pending":
                record["status"] = "running"
                record["result"] = "..."

        if not events and samples and records:
            for record, sample in zip(records.values(), samples):
                name = str(record.get("name", ""))
                if not name:
                    continue
                sample_row = sample.get("row_index") if sample else None
                residuals = residuals_by_row.get(sample_row, {}) if sample_row else {}
                record.update(
                    self._waypoint_record(
                        session_id=session_id,
                        display_index=record.get("index"),
                        name=name,
                        waypoint=record.get("waypoint", {"name": name}),
                        status="accepted",
                        result="OK",
                        reason="",
                        sample=sample,
                        event_sample_index=sample.get("row_index") or sample.get("sample_index"),
                        reprojection_error_px=sample.get("reprojection_error_px"),
                        board_margin_px=sample.get("board_margin_px"),
                        residuals=residuals,
                    )
                )

        return list(records.values())

    def read_waypoints(self) -> dict[str, Any]:
        return self._read_waypoints_from_path(self.trajectory_path)

    def sample_image_path(self, session_id: str, row_index: int) -> Path | None:
        sample = self.sample_for_row(session_id, row_index)
        if sample is None:
            return None
        image_path = sample.get("image_path")
        if image_path:
            candidate = Path(str(image_path))
            if candidate.exists():
                return candidate
        fallback = self._session_path(session_id) / "images" / f"sample_{row_index:03d}.png"
        return fallback if fallback.exists() else None

    def _read_waypoints_from_path(self, path: Path | None) -> dict[str, Any]:
        if path is None:
            return {"trajectory_path": None, "waypoints": [], "error": None}
        if not path.exists():
            return {"trajectory_path": str(path), "waypoints": [], "error": "Trajectory YAML does not exist."}
        try:
            trajectory = load_trajectory(path)
        except (TrajectoryValidationError, yaml.YAMLError, OSError, ValueError) as exc:
            return {"trajectory_path": str(path), "waypoints": [], "error": str(exc)}
        return {
            "trajectory_path": str(path),
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

    def _waypoint_record(
        self,
        *,
        session_id: str,
        display_index: int | None,
        name: str,
        waypoint: dict[str, Any],
        status: str,
        result: str,
        reason: str,
        sample: dict[str, Any] | None,
        event_sample_index: int | None = None,
        reprojection_error_px: Any = None,
        board_margin_px: Any = None,
        residuals: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        reason_info = classify_operator_message(reason) if reason else {"code": None, "message": "", "raw": "", "clean": ""}
        row_index = sample.get("row_index") if sample else event_sample_index
        image_path = sample.get("image_path") if sample else None
        has_image = bool(sample and sample.get("has_image"))
        camera_to_board_translation_m = sample.get("camera_to_board_translation_m") if sample else None
        flags = self._quality_flags(reprojection_error_px, board_margin_px)
        thumbnail_url = f"/api/sessions/{session_id}/sample-image/{row_index}.jpg?mode=overlay" if row_index and has_image else None
        return {
            "waypoint": waypoint,
            "index": display_index,
            "name": name,
            "status": status,
            "result": result,
            "capture": waypoint.get("capture", True),
            "reason": reason,
            "reason_code": reason_info.get("code"),
            "reason_display": reason_info.get("message") if reason else "",
            "sample_index": event_sample_index,
            "sample_row_index": row_index,
            "image_path": image_path,
            "has_image": has_image,
            "thumbnail_url": thumbnail_url,
            "reprojection_error_px": _to_float(reprojection_error_px),
            "board_margin_px": _to_float(board_margin_px),
            "thresholds": flags,
            "camera_to_board_translation_m": camera_to_board_translation_m,
            "camera_to_board_rotation_rpy_deg": sample.get("camera_to_board_rotation_rpy_deg") if sample else None,
            "board_angle_deg": sample.get("board_angle_deg") if sample else None,
            "tcp_pose_mmdeg": sample.get("tcp_pose_mmdeg") if sample else None,
            "image_sequence": sample.get("image_sequence") if sample else None,
            "capture_time_s": sample.get("capture_time_s") if sample else None,
            "translation_residual_mm": residuals.get("translation_residual_mm") if residuals else None,
            "rotation_residual_deg": residuals.get("rotation_residual_deg") if residuals else None,
        }

    def _counts_for_session(self, *, path: Path, report: dict[str, Any], samples: list[dict[str, Any]], events: list[dict[str, Any]]) -> dict[str, int]:
        accepted = sum(1 for event in events if event.get("event") == "waypoint_sample_captured")
        skipped = sum(1 for event in events if event.get("event") in {"waypoint_capture_skipped", "waypoint_capture_disabled", "waypoint_dry_run_complete"})
        if not accepted:
            accepted = int(report.get("sample_count", 0) or len(samples))
        trajectory = self._read_waypoints_from_path(self._trajectory_path_for_session(path))
        waypoint_count = len(self.read_session_waypoints(path.name))
        if not waypoint_count:
            waypoint_count = len(trajectory.get("waypoints", []))
        if not waypoint_count:
            waypoint_count = self._waypoint_count_from_events(events)
        pending = max(waypoint_count - accepted - skipped, 0) if waypoint_count else 0
        sample_count = int(report.get("sample_count", 0) or len(samples) or accepted)
        return {"sample_count": sample_count, "accepted": accepted, "skipped": skipped, "pending": pending, "trajectory_error": trajectory.get("error")}

    def _waypoint_count_from_events(self, events: list[dict[str, Any]]) -> int:
        names: set[str] = set()
        for event in events:
            waypoint = event.get("waypoint")
            waypoint_name = event.get("waypoint_name")
            if isinstance(waypoint, dict):
                waypoint_name = waypoint.get("name", waypoint_name)
            if waypoint_name:
                name = str(waypoint_name)
                if str(event.get("event", "")) == "waypoint_deleted":
                    names.discard(name)
                else:
                    names.add(name)
        return len(names)

    def _trajectory_path_for_session(self, session_path: Path) -> Path | None:
        used = session_path / "trajectory_used.yaml"
        return used if used.exists() else None

    def _quality_flags(self, reprojection_error_px: Any, board_margin_px: Any) -> dict[str, Any]:
        reprojection = _to_float(reprojection_error_px)
        margin = _to_float(board_margin_px)
        reprojection_limit_enabled = self.max_reprojection_error_px > 0.0
        reprojection_ok = None if reprojection is None or not reprojection_limit_enabled else reprojection <= self.max_reprojection_error_px
        margin_ok = None if margin is None else margin >= self.min_board_margin_px
        return {
            "max_reprojection_error_px": self.max_reprojection_error_px,
            "min_board_margin_px": self.min_board_margin_px,
            "reprojection_limit_enabled": reprojection_limit_enabled,
            "reprojection_ok": reprojection_ok,
            "margin_ok": margin_ok,
            "quality_ok": (reprojection_ok is not False) and (margin_ok is not False),
        }

    def _residual_comparison(self, residuals: dict[str, Any]) -> list[dict[str, Any]]:
        specs = [
            ("translation_rms_mm", "平移 RMS", 10.0, "mm"),
            ("translation_mean_mm", "平移 mean", 10.0, "mm"),
            ("translation_max_mm", "平移 max", 30.0, "mm"),
            ("rotation_rms_deg", "旋转 RMS", 2.0, "deg"),
            ("rotation_mean_deg", "旋转 mean", 2.0, "deg"),
            ("rotation_max_deg", "旋转 max", 6.0, "deg"),
        ]
        rows: list[dict[str, Any]] = []
        for key, label, target, unit in specs:
            value = _to_float(residuals.get(key))
            rows.append(
                {
                    "key": key,
                    "label": label,
                    "value": value,
                    "target": target,
                    "unit": unit,
                    "ok": None if value is None else value <= target,
                    "ratio": None if value is None else min(max(value / target, 0.0), 2.0),
                    "target_note": "参考目标，后续可配置",
                }
            )
        return rows

    def _normalize_sample_fields(self, sample: dict[str, Any]) -> dict[str, Any]:
        normalized = dict(sample)
        normalized["camera_to_board_translation_m"] = normalized.get("camera_to_board_translation_m") or _matrix_translation(normalized.get("camera_to_board_matrix"))
        normalized["camera_to_board_rotation_rpy_deg"] = normalized.get("camera_to_board_rotation_rpy_deg") or _matrix_rotation_rpy_deg(normalized.get("camera_to_board_matrix"))
        normalized["board_angle_deg"] = normalized.get("board_angle_deg") if normalized.get("board_angle_deg") is not None else _matrix_board_angle_deg(normalized.get("camera_to_board_matrix"))
        return normalized

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
        try:
            with path.open("r", encoding="utf-8") as handle:
                payload = yaml.safe_load(handle) or {}
        except (yaml.YAMLError, OSError, ValueError) as exc:
            return {"error": str(exc)}
        if not isinstance(payload, dict):
            return {"error": f"{path.name} must contain a mapping."}
        return payload

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

    def _read_samples_without_report_fallback(self, session_path: Path) -> list[dict[str, Any]]:
        return self._read_jsonl(session_path / "samples.jsonl")


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


def _matrix_rotation_rpy_deg(matrix: Any) -> list[float] | None:
    if not isinstance(matrix, list) or len(matrix) < 3:
        return None
    try:
        rotation = [[float(matrix[row][col]) for col in range(3)] for row in range(3)]
    except (TypeError, ValueError, IndexError):
        return None
    return rotation_matrix_to_rpy_deg(rotation)


def _matrix_board_angle_deg(matrix: Any) -> float | None:
    if not isinstance(matrix, list) or len(matrix) < 3:
        return None
    try:
        normal_z = float(matrix[2][2])
    except (TypeError, ValueError, IndexError):
        return None
    return float(math.degrees(math.acos(max(min(abs(normal_z), 1.0), 0.0))))


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _parse_4x4(matrix: Any) -> np.ndarray | None:
    if matrix is None:
        return None
    try:
        mat = np.array(matrix, dtype=np.float64)
        if mat.shape == (4, 4):
            return mat
        return None
    except (ValueError, TypeError):
        return None


def _build_4x4_from_transform(transform: dict[str, Any]) -> np.ndarray | None:
    """Build a 4x4 homogeneous matrix from translation_m and rotation_rpy_deg using paus_perception conventions."""
    translation = transform.get("translation_m")
    rotation_rpy = transform.get("rotation_rpy_deg")
    if not translation or not rotation_rpy:
        return None
    try:
        t = np.array(translation, dtype=np.float64).reshape(3)
        rpy_deg = np.array(rotation_rpy, dtype=np.float64)
        if rpy_deg.shape != (3,):
            return None
        R = np.array(rpy_deg_to_rotation_matrix(rpy_deg.tolist()), dtype=np.float64)
        return np.array(make_transform_matrix(t.tolist(), R.tolist()), dtype=np.float64)
    except (ValueError, TypeError, IndexError, np.linalg.LinAlgError):
        return None


def _matrix_from_dict(d: dict[str, Any]) -> np.ndarray | None:
    """Build a 4x4 matrix from {translation_m, rotation_matrix} or {translation_m, rotation_rpy_deg}."""
    translation = d.get("translation_m")
    if not translation:
        return None
    rotation_matrix = d.get("rotation_matrix")
    if rotation_matrix is not None:
        try:
            R = np.array(rotation_matrix, dtype=np.float64)
            if R.shape != (3, 3):
                return None
        except (ValueError, TypeError):
            return None
    else:
        rotation_rpy = d.get("rotation_rpy_deg")
        if not rotation_rpy:
            return None
        try:
            rpy_deg = np.array(rotation_rpy, dtype=np.float64)
            if rpy_deg.shape != (3,):
                return None
            R = np.array(rpy_deg_to_rotation_matrix(rpy_deg.tolist()), dtype=np.float64)
        except (ValueError, TypeError, IndexError):
            return None
    try:
        t = np.array(translation, dtype=np.float64).reshape(3)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t
        return T
    except (ValueError, TypeError):
        return None
