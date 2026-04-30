from __future__ import annotations

import json
from pathlib import Path

import yaml

from paus_ui.session_store import SessionStore


def _write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row) + "\n")


def test_session_store_reads_report_samples_and_waypoint_events(tmp_path: Path) -> None:
    trajectory_path = tmp_path / "eye_to_hand_trajectory.yaml"
    trajectory_path.write_text(
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
                        "vel": 10.0,
                        "acc": 10.0,
                        "dwell_s": 0.5,
                        "capture": True,
                    },
                    {
                        "name": "waypoint_002",
                        "motion": "movej",
                        "joint_deg": [1, 1, 1, 1, 1, 1],
                        "expected_tcp_pose_mmdeg": [6, 5, 4, 3, 2, 1],
                        "vel": 10.0,
                        "acc": 10.0,
                        "dwell_s": 0.5,
                        "capture": True,
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    session_root = tmp_path / "calibration_sessions"
    session_path = session_root / "2026-04-29_120000"
    session_path.mkdir(parents=True)
    (session_path / "report.yaml").write_text(
        yaml.safe_dump({"sample_count": 1, "method": "joint_absolute", "residuals": {"translation_mean_mm": 12.0}}),
        encoding="utf-8",
    )
    _write_jsonl(
        session_path / "samples.jsonl",
        [
            {
                "sample_index": 1,
                "reprojection_error_px": 1.25,
                "board_margin_px": 50.0,
                "image_path": str(session_path / "images" / "sample_001.png"),
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
            {
                "event": "waypoint_sample_captured",
                "waypoint_name": "waypoint_001",
                "sample_index": 1,
                "reprojection_error_px": 1.25,
                "board_margin_px": 50.0,
            },
            {
                "event": "waypoint_capture_skipped",
                "waypoint": {"name": "waypoint_002"},
                "reason": "RuntimeError('not detected')",
            },
        ],
    )

    store = SessionStore(session_root_path=session_root, trajectory_path=trajectory_path)

    sessions = store.list_sessions()
    assert sessions[0]["id"] == "2026-04-29_120000"
    assert sessions[0]["sample_count"] == 1

    samples = store.read_samples("2026-04-29_120000")
    assert samples[0]["row_index"] == 1
    assert samples[0]["reprojection_error_px"] == 1.25
    assert samples[0]["camera_to_board_rotation_rpy_deg"] == [0.0, -0.0, 0.0]

    waypoints = store.read_session_waypoints("2026-04-29_120000")
    assert waypoints[0]["status"] == "accepted"
    assert waypoints[0]["sample_index"] == 1
    assert waypoints[0]["camera_to_board_translation_m"] == [0.1, 0.2, 0.3]
    assert waypoints[1]["status"] == "skipped"
    assert "not detected" in waypoints[1]["reason"]
