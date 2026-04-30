from __future__ import annotations

import json
from pathlib import Path

import yaml

from paus_ui.session_store import SessionStore


def _write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


def _write_trajectory(path: Path) -> None:
    path.write_text(
        yaml.safe_dump(
            {
                "version": 1,
                "tool_id": 0,
                "user_id": 0,
                "defaults": {"motion": "movej", "vel": 10.0, "acc": 10.0, "dwell_s": 0.5},
                "waypoints": [
                    {
                        "name": "waypoint_001",
                        "motion": "movej",
                        "joint_deg": [0, 0, 0, 0, 0, 0],
                        "expected_tcp_pose_mmdeg": [1, 2, 3, 4, 5, 6],
                        "capture": True,
                    },
                    {
                        "name": "waypoint_002",
                        "motion": "movej",
                        "joint_deg": [1, 1, 1, 1, 1, 1],
                        "expected_tcp_pose_mmdeg": [6, 5, 4, 3, 2, 1],
                        "capture": True,
                    },
                ],
            }
        ),
        encoding="utf-8",
    )


def test_session_store_shapes_counts_reasons_and_thresholds(tmp_path: Path) -> None:
    trajectory_path = tmp_path / "eye_to_hand_trajectory.yaml"
    _write_trajectory(trajectory_path)
    session_root = tmp_path / "calibration_sessions"
    session_path = session_root / "2026-04-29_120000"
    image_dir = session_path / "images"
    image_dir.mkdir(parents=True)
    image_path = image_dir / "sample_001.png"
    image_path.write_bytes(b"fake")
    (session_path / "report.yaml").write_text(
        yaml.safe_dump(
            {
                "sample_count": 1,
                "method": "joint_absolute",
                "residuals": {
                    "translation_rms_mm": 12.0,
                    "translation_mean_mm": 8.0,
                    "translation_max_mm": 21.0,
                    "rotation_rms_deg": 1.5,
                },
            }
        ),
        encoding="utf-8",
    )
    _write_jsonl(
        session_path / "samples.jsonl",
        [
            {
                "sample_index": 1,
                "reprojection_error_px": 4.5,
                "board_margin_px": 8.0,
                "image_path": str(image_path),
                "camera_to_board_matrix": [
                    [1, 0, 0, 0.1],
                    [0, 1, 0, 0.2],
                    [0, 0, 1, 0.3],
                    [0, 0, 0, 1],
                ],
            }
        ],
    )
    _write_jsonl(
        session_path / "run.log",
        [
            {"event": "waypoint_sample_captured", "waypoint_name": "waypoint_001", "sample_index": 1},
            {"event": "waypoint_capture_skipped", "waypoint": {"name": "waypoint_002"}, "reason": "RuntimeError('Chessboard was not detected.')"},
        ],
    )

    store = SessionStore(
        session_root_path=session_root,
        trajectory_path=trajectory_path,
        max_reprojection_error_px=4.0,
        min_board_margin_px=10.0,
    )

    session = store.list_sessions()[0]
    assert session["accepted_count"] == 1
    assert session["skipped_count"] == 1
    assert store.latest_valid_session_id() == "2026-04-29_120000"

    report = store.read_report("2026-04-29_120000")
    assert report["accepted_count"] == 1
    assert report["residual_comparison"][0]["key"] == "translation_rms_mm"

    waypoints = store.read_session_waypoints("2026-04-29_120000")
    assert waypoints[0]["status"] == "accepted"
    assert waypoints[0]["thresholds"]["reprojection_ok"] is False
    assert waypoints[0]["thresholds"]["margin_ok"] is False
    assert waypoints[0]["camera_to_board_translation_m"] == [0.1, 0.2, 0.3]
    assert waypoints[1]["reason_code"] == "chessboard_not_detected"
    assert "棋盘未检测到" in waypoints[1]["reason_display"]


def test_latest_valid_session_prefers_solved_report(tmp_path: Path) -> None:
    trajectory_path = tmp_path / "eye_to_hand_trajectory.yaml"
    _write_trajectory(trajectory_path)
    session_root = tmp_path / "calibration_sessions"
    solved = session_root / "2026-04-29_120000"
    unsolved = session_root / "2026-04-29_130000"
    solved.mkdir(parents=True)
    unsolved.mkdir(parents=True)
    (solved / "report.yaml").write_text(
        yaml.safe_dump({"sample_count": 2, "residuals": {"translation_rms_mm": 3.0}}),
        encoding="utf-8",
    )
    (unsolved / "report.yaml").write_text(yaml.safe_dump({"sample_count": 0}), encoding="utf-8")

    store = SessionStore(session_root_path=session_root, trajectory_path=trajectory_path)

    assert store.latest_valid_session_id() == "2026-04-29_120000"
    assert store.read_report("2026-04-29_130000")["has_solution"] is False
    assert "尚未完成求解" in store.read_report("2026-04-29_130000")["empty_reason"]
