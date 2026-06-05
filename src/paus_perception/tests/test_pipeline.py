from __future__ import annotations

import csv
import json
import math
import socket
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
for package_root in (PERCEPTION_PACKAGE_ROOT, MARKER_PACKAGE_ROOT, MOTION_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

from paus_marker_ros2.conversions import build_status_payload, rvec_to_quaternion
from paus_motion_ros2 import ControlDecision, build_approach_decision, point_m_to_mm
from paus_perception import (
    CameraCalibration,
    EyeToHandCalibrationSolution,
    average_transform_matrices,
    build_ax_xb_motion_pairs,
    build_approach_plan,
    build_summary_record,
    detect_marker,
    estimate_marker_pose,
    evaluate_eye_to_hand_residuals,
    estimate_tool_to_board_from_samples,
    invert_transform_matrix,
    load_camera_calibration,
    load_config,
    load_eye_to_hand_solution,
    make_transform_matrix,
    make_transform_struct,
    pack_frame_packet,
    process_image_array,
    process_image_file,
    recv_frame_packet,
    rpy_deg_to_rotation_matrix,
    save_camera_calibration,
    save_eye_to_hand_solution,
    solve_eye_to_hand_opencv_handeye,
    split_transform_matrix,
    solve_ax_xb_hand_eye_park,
    write_summary_csv,
    write_summary_json,
)


def create_synthetic_marker_scene(marker_id: int = 7, canvas_size: int = 640, marker_size: int = 160) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size)
    else:
        marker = np.zeros((marker_size, marker_size), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, marker_size, marker, 1)
    marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    canvas = np.full((canvas_size, canvas_size, 3), 255, dtype=np.uint8)
    src = np.array([[0, 0], [marker_size - 1, 0], [marker_size - 1, marker_size - 1], [0, marker_size - 1]], dtype=np.float32)
    dst = np.array([[210, 120], [390, 150], [360, 330], [180, 290]], dtype=np.float32)
    transform = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(marker_bgr, transform, (canvas_size, canvas_size))
    mask = cv2.warpPerspective(np.full((marker_size, marker_size), 255, dtype=np.uint8), transform, (canvas_size, canvas_size))
    canvas[mask > 0] = warped[mask > 0]
    return canvas


def create_test_calibration() -> CameraCalibration:
    return CameraCalibration(
        image_width=640,
        image_height=480,
        camera_matrix=[
            [800.0, 0.0, 320.0],
            [0.0, 800.0, 240.0],
            [0.0, 0.0, 1.0],
        ],
        dist_coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error=0.1,
        board_rows=6,
        board_cols=9,
        square_size_m=0.01,
    )


class MarkerPipelineTests(unittest.TestCase):
    def test_detect_marker_returns_marker_only_result(self) -> None:
        config = load_config()
        image = create_synthetic_marker_scene()
        result = detect_marker(image, config)
        self.assertEqual(result.status, "ok")
        self.assertIsNotNone(result.marker)
        assert result.marker is not None
        self.assertEqual(result.marker.marker_id, 7)
        self.assertIsNone(result.pose)
        self.assertIsNone(result.approach_plan)

    def test_process_image_array_keeps_everything_in_memory(self) -> None:
        config = load_config()
        image = create_synthetic_marker_scene()
        calibration = create_test_calibration()
        result = process_image_array(image, config, camera_calibration=calibration)
        self.assertEqual(result.status, "ok")
        self.assertIsNotNone(result.pose)
        self.assertIsNotNone(result.approach_plan)

    def test_no_marker_returns_not_found(self) -> None:
        config = load_config()
        image = np.full((480, 640, 3), 255, dtype=np.uint8)
        result = detect_marker(image, config)
        self.assertEqual(result.status, "not_found")
        self.assertIsNone(result.marker)

    def test_pose_estimation_and_plan_work_on_synthetic_projection(self) -> None:
        calibration = create_test_calibration()
        marker_length_m = 0.04
        object_points = np.array(
            [
                [-marker_length_m / 2.0, marker_length_m / 2.0, 0.0],
                [marker_length_m / 2.0, marker_length_m / 2.0, 0.0],
                [marker_length_m / 2.0, -marker_length_m / 2.0, 0.0],
                [-marker_length_m / 2.0, -marker_length_m / 2.0, 0.0],
            ],
            dtype=np.float32,
        )
        rvec = np.array([[0.0], [0.0], [0.0]], dtype=np.float64)
        tvec = np.array([[0.01], [0.02], [0.50]], dtype=np.float64)
        image_points, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            np.asarray(calibration.camera_matrix, dtype=np.float64),
            np.asarray(calibration.dist_coeffs, dtype=np.float64),
        )
        corners = image_points.reshape(4, 2)
        marker = type("SyntheticMarker", (), {})()
        marker.marker_id = 7
        marker.corners = corners.tolist()
        marker.center = corners.mean(axis=0).tolist()
        marker.angle_deg = 0.0
        marker.average_side_length = 10.0
        marker.selection_reason = "matched_target_id:7"

        pose = estimate_marker_pose(marker, calibration, marker_length_m)
        self.assertIsNotNone(pose)
        assert pose is not None
        self.assertEqual(pose.status, "ok")
        self.assertTrue(math.isclose(pose.distance_m, float(np.linalg.norm(tvec)), rel_tol=0.05))

        plan = build_approach_plan(pose, 0.05, "camera")
        self.assertIsNotNone(plan)
        assert plan is not None
        self.assertEqual(plan.target_status, "ok")
        self.assertEqual(len(plan.target_point), 3)

    def test_control_decision_accepts_valid_target(self) -> None:
        decision = build_approach_decision(
            target_position_base_m=[0.20, 0.0, 0.30],
            raw_target_orientation_rpy_deg=[0.0, 0.0, 0.0],
            frame_id="robot_base",
            current_tcp_pose_mmdeg=[100.0, 0.0, 300.0, 180.0, 0.0, -180.0],
            orientation_mode="face_marker_normal",
            flange_face_axis="z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=50.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
        )
        self.assertTrue(decision.check_passed)
        assert decision.candidate_pose_mmdeg is not None
        self.assertEqual(len(decision.candidate_pose_mmdeg), 6)

    def test_control_decision_rejects_wrong_frame(self) -> None:
        decision = build_approach_decision(
            target_position_base_m=[0.20, 0.0, 0.30],
            raw_target_orientation_rpy_deg=[0.0, 0.0, 0.0],
            frame_id="camera",
            current_tcp_pose_mmdeg=[100.0, 0.0, 300.0, 180.0, 0.0, -180.0],
            orientation_mode="face_marker_normal",
            flange_face_axis="z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=50.0,
            max_step_distance_mm=80.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=10.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
        )
        self.assertFalse(decision.check_passed)
        self.assertIn("frame_id", decision.error_message)

    def test_control_decision_limits_large_step(self) -> None:
        decision = build_approach_decision(
            target_position_base_m=[0.50, 0.0, 0.30],
            raw_target_orientation_rpy_deg=[0.0, 0.0, 0.0],
            frame_id="robot_base",
            current_tcp_pose_mmdeg=[100.0, 0.0, 300.0, 180.0, 0.0, -180.0],
            orientation_mode="face_marker_normal",
            flange_face_axis="z",
            hover_clearance_mm=30.0,
            pre_approach_distance_mm=50.0,
            max_step_distance_mm=40.0,
            min_safe_z_mm=50.0,
            min_plane_clearance_mm=5.0,
            workspace_min_mm=[-1000.0, -1000.0, 0.0],
            workspace_max_mm=[1000.0, 1000.0, 1000.0],
            enable_safe_lift_on_low_clearance=False,
        )
        self.assertTrue(decision.check_passed)
        assert decision.step_distance_mm is not None
        self.assertLessEqual(decision.step_distance_mm, 40.0)

    def test_process_image_file_writes_pose_outputs(self) -> None:
        config = load_config()
        image = create_synthetic_marker_scene()
        calibration = create_test_calibration()

        with tempfile.TemporaryDirectory() as temp_dir:
            image_path = Path(temp_dir) / "sample.png"
            output_dir = Path(temp_dir) / "result"
            cv2.imwrite(str(image_path), image)

            result = process_image_file(image_path, output_dir, config, camera_calibration=calibration)

            self.assertEqual(result.status, "ok")
            self.assertTrue((output_dir / "detection_result.json").exists())
            self.assertTrue((output_dir / "visualization.png").exists())

            payload = json.loads((output_dir / "detection_result.json").read_text(encoding="utf-8"))
            self.assertIn("pose", payload)
            self.assertIn("approach_plan", payload)

    def test_summary_files_can_be_written(self) -> None:
        config = load_config()
        image = create_synthetic_marker_scene()
        calibration = create_test_calibration()

        with tempfile.TemporaryDirectory() as temp_dir:
            base_dir = Path(temp_dir)
            image_path = base_dir / "sample.png"
            output_dir = base_dir / "result"
            cv2.imwrite(str(image_path), image)

            result = process_image_file(image_path, output_dir, config, camera_calibration=calibration)
            records = [build_summary_record(image_path, result, "sample", "camera.yaml")]

            json_path = base_dir / "summary.json"
            csv_path = base_dir / "summary.csv"
            write_summary_json(records, json_path)
            write_summary_csv(records, csv_path)

            summary_json = json.loads(json_path.read_text(encoding="utf-8"))
            self.assertIn("distance_m", summary_json[0])
            self.assertEqual(summary_json[0]["camera_yaml_used"], "camera.yaml")

            with csv_path.open("r", encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["image_name"], "sample.png")
            self.assertEqual(rows[0]["output_subdir"], "sample")

    def test_camera_yaml_round_trip(self) -> None:
        calibration = create_test_calibration()
        with tempfile.TemporaryDirectory() as temp_dir:
            yaml_path = Path(temp_dir) / "camera.yaml"
            save_camera_calibration(calibration, yaml_path)
            loaded = load_camera_calibration(yaml_path)
            self.assertEqual(loaded.image_width, calibration.image_width)
            self.assertEqual(loaded.board_rows, calibration.board_rows)
            self.assertEqual(len(loaded.dist_coeffs), len(calibration.dist_coeffs))

    def test_eye_to_hand_yaml_round_trip(self) -> None:
        transform = make_transform_struct([0.1, 0.2, 0.3], np.eye(3), "robot_base", "camera")
        estimated_tool_to_board = make_transform_struct([0.02, -0.03, 0.1], np.eye(3), "tool", "board")
        solution = EyeToHandCalibrationSolution(
            status="ok",
            success=True,
            sample_count=12,
            message="ok",
            base_to_camera=transform,
            tool_to_board_translation_m=[0.0, 0.0, 0.1],
            tool_to_board_rotation_rpy_deg=[0.0, 0.0, 0.0],
            estimated_tool_to_board=estimated_tool_to_board,
            raw_residuals=None,
            corrected_residuals=None,
            quality_status="ok",
            quality_warnings=[],
            generated_at="2026-05-20T00:00:00+00:00",
            session_dir="/tmp/session",
            trajectory_path="/tmp/trajectory.yaml",
            git_branch="codex/test",
            git_commit="abc1234",
            git_dirty=True,
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            yaml_path = Path(temp_dir) / "extrinsics.yaml"
            save_eye_to_hand_solution(solution, yaml_path)
            loaded = load_eye_to_hand_solution(yaml_path)
            self.assertTrue(loaded.success)
            assert loaded.base_to_camera is not None
            self.assertEqual(loaded.sample_count, 12)
            self.assertEqual(loaded.base_to_camera.translation_m, [0.1, 0.2, 0.3])
            assert loaded.estimated_tool_to_board is not None
            self.assertEqual(loaded.estimated_tool_to_board.translation_m, [0.02, -0.03, 0.1])
            self.assertEqual(loaded.generated_at, "2026-05-20T00:00:00+00:00")
            self.assertEqual(loaded.session_dir, "/tmp/session")
            self.assertEqual(loaded.trajectory_path, "/tmp/trajectory.yaml")
            self.assertEqual(loaded.git_branch, "codex/test")
            self.assertEqual(loaded.git_commit, "abc1234")
            self.assertTrue(loaded.git_dirty)

    def test_average_transform_matrices(self) -> None:
        first = make_transform_matrix([0.1, 0.0, 0.0], np.eye(3))
        second = make_transform_matrix([0.3, 0.0, 0.0], np.eye(3))
        averaged = average_transform_matrices([first, second])
        translation, rotation = split_transform_matrix(averaged)
        self.assertTrue(np.allclose(translation, [0.2, 0.0, 0.0]))
        self.assertTrue(np.allclose(rotation, np.eye(3)))

    def test_ax_xb_hand_eye_solver_recovers_base_to_camera(self) -> None:
        base_to_camera = make_transform_matrix([0.24, -0.18, 0.42], rpy_deg_to_rotation_matrix([175.0, -5.0, 92.0]))
        tool_to_board = make_transform_matrix([0.02, 0.01, 0.12], rpy_deg_to_rotation_matrix([0.0, 0.0, 0.0]))

        base_to_tool_samples = [
            make_transform_matrix([0.35, -0.10, 0.20], rpy_deg_to_rotation_matrix([-150.0, 10.0, -90.0])),
            make_transform_matrix([0.30, -0.05, 0.25], rpy_deg_to_rotation_matrix([-145.0, 15.0, -80.0])),
            make_transform_matrix([0.40, -0.15, 0.22], rpy_deg_to_rotation_matrix([-160.0, 5.0, -100.0])),
            make_transform_matrix([0.32, -0.20, 0.28], rpy_deg_to_rotation_matrix([-155.0, -2.0, -95.0])),
        ]
        camera_to_board_samples = [
            invert_transform_matrix(base_to_camera) @ base_to_tool @ tool_to_board
            for base_to_tool in base_to_tool_samples
        ]

        motion_pairs = build_ax_xb_motion_pairs(base_to_tool_samples, camera_to_board_samples)
        self.assertGreaterEqual(len(motion_pairs), 6)

        solved = solve_ax_xb_hand_eye_park(base_to_tool_samples, camera_to_board_samples)
        translation, rotation = split_transform_matrix(solved)
        expected_translation, expected_rotation = split_transform_matrix(base_to_camera)

        self.assertTrue(np.allclose(translation, expected_translation, atol=1e-6))
        self.assertTrue(np.allclose(rotation, expected_rotation, atol=1e-6))

    def test_opencv_eye_to_hand_solver_recovers_base_to_camera(self) -> None:
        base_to_camera = make_transform_matrix([0.24, -0.18, 0.42], rpy_deg_to_rotation_matrix([175.0, -5.0, 92.0]))
        tool_to_board = make_transform_matrix([0.02, 0.01, 0.12], rpy_deg_to_rotation_matrix([0.0, 0.0, 0.0]))

        base_to_tool_samples = [
            make_transform_matrix([0.35, -0.10, 0.20], rpy_deg_to_rotation_matrix([-150.0, 10.0, -90.0])),
            make_transform_matrix([0.30, -0.05, 0.25], rpy_deg_to_rotation_matrix([-145.0, 15.0, -80.0])),
            make_transform_matrix([0.40, -0.15, 0.22], rpy_deg_to_rotation_matrix([-160.0, 5.0, -100.0])),
            make_transform_matrix([0.32, -0.20, 0.28], rpy_deg_to_rotation_matrix([-155.0, -2.0, -95.0])),
        ]
        target_to_camera_samples = [
            invert_transform_matrix(base_to_camera) @ base_to_tool @ tool_to_board
            for base_to_tool in base_to_tool_samples
        ]

        solved = solve_eye_to_hand_opencv_handeye(base_to_tool_samples, target_to_camera_samples)
        translation, rotation = split_transform_matrix(solved)
        expected_translation, expected_rotation = split_transform_matrix(base_to_camera)

        self.assertTrue(np.allclose(translation, expected_translation, atol=1e-6))
        self.assertTrue(np.allclose(rotation, expected_rotation, atol=1e-6))

    def test_eye_to_hand_residuals_are_zero_for_consistent_samples(self) -> None:
        base_to_camera = make_transform_matrix([0.24, -0.18, 0.42], rpy_deg_to_rotation_matrix([175.0, -5.0, 92.0]))
        tool_to_board = make_transform_matrix([0.02, 0.01, 0.12], np.eye(3))
        base_to_tool_samples = [
            make_transform_matrix([0.35, -0.10, 0.20], rpy_deg_to_rotation_matrix([-150.0, 10.0, -90.0])),
            make_transform_matrix([0.30, -0.05, 0.25], rpy_deg_to_rotation_matrix([-145.0, 15.0, -80.0])),
            make_transform_matrix([0.40, -0.15, 0.22], rpy_deg_to_rotation_matrix([-160.0, 5.0, -100.0])),
        ]
        camera_to_board_samples = [
            invert_transform_matrix(base_to_camera) @ base_to_tool @ tool_to_board
            for base_to_tool in base_to_tool_samples
        ]

        residuals = evaluate_eye_to_hand_residuals(
            base_to_camera,
            base_to_tool_samples,
            camera_to_board_samples,
            tool_to_board,
        )

        self.assertLess(residuals.translation_rms_mm, 1e-6)
        self.assertLess(residuals.rotation_rms_deg, 1e-6)

    def test_estimated_tool_to_board_corrects_residuals(self) -> None:
        base_to_camera = make_transform_matrix([0.24, -0.18, 0.42], rpy_deg_to_rotation_matrix([175.0, -5.0, 92.0]))
        tool_to_board = make_transform_matrix([0.02, -0.03, 0.12], rpy_deg_to_rotation_matrix([1.0, -2.0, 3.0]))
        base_to_tool_samples = [
            make_transform_matrix([0.35, -0.10, 0.20], rpy_deg_to_rotation_matrix([-150.0, 10.0, -90.0])),
            make_transform_matrix([0.30, -0.05, 0.25], rpy_deg_to_rotation_matrix([-145.0, 15.0, -80.0])),
            make_transform_matrix([0.40, -0.15, 0.22], rpy_deg_to_rotation_matrix([-160.0, 5.0, -100.0])),
            make_transform_matrix([0.32, -0.20, 0.28], rpy_deg_to_rotation_matrix([-155.0, -2.0, -95.0])),
        ]
        camera_to_board_samples = [
            invert_transform_matrix(base_to_camera) @ base_to_tool @ tool_to_board
            for base_to_tool in base_to_tool_samples
        ]

        raw = evaluate_eye_to_hand_residuals(base_to_camera, base_to_tool_samples, camera_to_board_samples, np.eye(4))
        estimated = estimate_tool_to_board_from_samples(base_to_camera, base_to_tool_samples, camera_to_board_samples)
        corrected = evaluate_eye_to_hand_residuals(base_to_camera, base_to_tool_samples, camera_to_board_samples, estimated)

        self.assertGreater(raw.translation_rms_mm, 100.0)
        self.assertTrue(np.allclose(estimated, tool_to_board, atol=1e-6))
        self.assertLess(corrected.translation_rms_mm, 1e-6)
        self.assertLess(corrected.rotation_rms_deg, 1e-6)

    def test_transform_inverse_round_trip(self) -> None:
        rotation = rpy_deg_to_rotation_matrix([10.0, 20.0, 30.0])
        matrix = make_transform_matrix([0.1, -0.2, 0.3], rotation)
        identity = matrix @ invert_transform_matrix(matrix)
        self.assertTrue(np.allclose(identity, np.eye(4), atol=1e-6))

    def test_rvec_to_quaternion_identity(self) -> None:
        quaternion = rvec_to_quaternion([0.0, 0.0, 0.0])
        self.assertTrue(math.isclose(quaternion[0], 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(quaternion[1], 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(quaternion[2], 0.0, abs_tol=1e-6))
        self.assertTrue(math.isclose(quaternion[3], 1.0, abs_tol=1e-6))

    def test_build_status_payload_contains_expected_keys(self) -> None:
        config = load_config()
        image = create_synthetic_marker_scene()
        calibration = create_test_calibration()
        result = process_image_array(image, config, camera_calibration=calibration)
        payload = json.loads(build_status_payload(result))
        self.assertIn("status", payload)
        self.assertIn("marker_id", payload)
        self.assertIn("distance_m", payload)

    def test_bridge_protocol_round_trip(self) -> None:
        header = {
            "frame_type": "rgb",
            "encoding": "jpeg",
            "width": 640,
            "height": 480,
            "timestamp_ns": 123456789,
            "frame_id": "camera",
            "camera_info_version": "camera_yaml",
        }
        payload = b"test-image-payload"
        packet = pack_frame_packet(header, payload)
        sender, receiver = socket.socketpair()
        try:
            sender.sendall(packet)
            received_header, received_payload = recv_frame_packet(receiver)
        finally:
            sender.close()
            receiver.close()
        self.assertEqual(received_header["frame_type"], "rgb")
        self.assertEqual(received_header["payload_size"], len(payload))
        self.assertEqual(received_payload, payload)


if __name__ == "__main__":
    unittest.main()
