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
    (session_path / "trajectory_used.yaml").write_text(trajectory_path.read_text(encoding="utf-8"), encoding="utf-8")
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


def test_session_store_marks_capture_disabled_waypoints_terminal(tmp_path: Path) -> None:
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
                        "capture": False,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    session_root = tmp_path / "calibration_sessions"
    session_path = session_root / "2026-04-29_121000"
    session_path.mkdir(parents=True)
    (session_path / "trajectory_used.yaml").write_text(trajectory_path.read_text(encoding="utf-8"), encoding="utf-8")
    _write_jsonl(
        session_path / "run.log",
        [
            {
                "event": "waypoint_reached",
                "waypoint_name": "waypoint_001",
                "waypoint": {"name": "waypoint_001", "capture": False},
            },
            {
                "event": "waypoint_capture_disabled",
                "waypoint_name": "waypoint_001",
                "waypoint": {"name": "waypoint_001", "capture": False},
                "reason": "capture=false",
            },
        ],
    )

    store = SessionStore(session_root_path=session_root, trajectory_path=trajectory_path)

    sessions = store.list_sessions()
    assert sessions[0]["skipped_count"] == 1
    assert sessions[0]["pending_count"] == 0
    waypoints = store.read_session_waypoints("2026-04-29_121000")
    assert waypoints[0]["status"] == "skipped"
    assert waypoints[0]["result"] == "SKIP"
    assert waypoints[0]["reason"] == "capture=false"


def test_session_store_marks_dry_run_waypoints_terminal(tmp_path: Path) -> None:
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
    session_path = session_root / "2026-04-29_122000"
    session_path.mkdir(parents=True)
    (session_path / "trajectory_used.yaml").write_text(trajectory_path.read_text(encoding="utf-8"), encoding="utf-8")
    _write_jsonl(
        session_path / "run.log",
        [
            {
                "event": "waypoint_dry_run_complete",
                "waypoint_name": "waypoint_001",
                "waypoint": {"name": "waypoint_001"},
                "reason": "dry-run",
            },
            {
                "event": "waypoint_dry_run_complete",
                "waypoint_name": "waypoint_002",
                "waypoint": {"name": "waypoint_002"},
                "reason": "dry-run",
            },
        ],
    )

    store = SessionStore(session_root_path=session_root, trajectory_path=trajectory_path)

    sessions = store.list_sessions()
    assert sessions[0]["skipped_count"] == 2
    assert sessions[0]["pending_count"] == 0
    waypoints = store.read_session_waypoints("2026-04-29_122000")
    assert [waypoint["result"] for waypoint in waypoints] == ["DRY", "DRY"]
    assert all(waypoint["status"] == "skipped" for waypoint in waypoints)


def test_manual_session_without_trajectory_has_no_phantom_waypoints(tmp_path: Path) -> None:
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
                        "name": "current_waypoint_from_today",
                        "motion": "movej",
                        "joint_deg": [0, 0, 0, 0, 0, 0],
                        "expected_tcp_pose_mmdeg": [1, 2, 3, 4, 5, 6],
                        "vel": 10.0,
                        "acc": 10.0,
                        "dwell_s": 0.5,
                        "capture": True,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    session_root = tmp_path / "calibration_sessions"
    session_path = session_root / "2026-04-29_123000"
    session_path.mkdir(parents=True)
    (session_path / "report.yaml").write_text(yaml.safe_dump({"sample_count": 1, "method": "joint_absolute"}), encoding="utf-8")
    _write_jsonl(
        session_path / "samples.jsonl",
        [
            {
                "sample_index": 1,
                "reprojection_error_px": 1.25,
                "board_margin_px": 50.0,
            }
        ],
    )

    store = SessionStore(session_root_path=session_root, trajectory_path=trajectory_path)

    sessions = store.list_sessions()
    assert sessions[0]["sample_count"] == 1
    assert sessions[0]["pending_count"] == 0
    assert store.read_session_waypoints("2026-04-29_123000") == []


def test_session_store_detects_fallback_sample_images(tmp_path: Path) -> None:
    trajectory_path = tmp_path / "eye_to_hand_trajectory.yaml"
    trajectory_path.write_text(
        yaml.safe_dump({"version": 1, "tool_id": 0, "user_id": 0, "defaults": {"motion": "movej"}, "waypoints": []}),
        encoding="utf-8",
    )
    session_root = tmp_path / "calibration_sessions"
    session_path = session_root / "2026-04-29_124000"
    images_path = session_path / "images"
    images_path.mkdir(parents=True)
    fallback_image = images_path / "sample_001.png"
    fallback_image.write_bytes(b"not-a-real-png-but-present")
    _write_jsonl(
        session_path / "samples.jsonl",
        [
            {
                "sample_index": 1,
                "reprojection_error_px": 1.25,
                "board_margin_px": 50.0,
            }
        ],
    )

    store = SessionStore(session_root_path=session_root, trajectory_path=trajectory_path)

    samples = store.read_samples("2026-04-29_124000")
    assert samples[0]["has_image"] is True
    assert samples[0]["image_path"] == str(fallback_image)


def test_latest_valid_session_skips_newer_archives_without_report(tmp_path: Path) -> None:
    trajectory_path = tmp_path / "eye_to_hand_trajectory.yaml"
    trajectory_path.write_text(
        yaml.safe_dump({"version": 1, "tool_id": 0, "user_id": 0, "defaults": {"motion": "movej"}, "waypoints": []}),
        encoding="utf-8",
    )
    session_root = tmp_path / "calibration_sessions"
    older_report = session_root / "2026-04-29_120000"
    newer_empty = session_root / "2026-04-29_130000"
    older_report.mkdir(parents=True)
    newer_empty.mkdir(parents=True)
    (older_report / "report.yaml").write_text(
        yaml.safe_dump({"sample_count": 1, "method": "joint_absolute"}),
        encoding="utf-8",
    )

    store = SessionStore(session_root_path=session_root, trajectory_path=trajectory_path)

    assert store.latest_valid_session_id() == "2026-04-29_120000"


def test_latest_valid_session_returns_none_for_only_no_report_archives(tmp_path: Path) -> None:
    trajectory_path = tmp_path / "eye_to_hand_trajectory.yaml"
    trajectory_path.write_text(
        yaml.safe_dump({"version": 1, "tool_id": 0, "user_id": 0, "defaults": {"motion": "movej"}, "waypoints": []}),
        encoding="utf-8",
    )
    session_root = tmp_path / "calibration_sessions"
    (session_root / "2026-04-29_130000").mkdir(parents=True)

    store = SessionStore(session_root_path=session_root, trajectory_path=trajectory_path)

    assert store.latest_valid_session_id() is None
