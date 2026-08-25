from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
if str(PERCEPTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_PACKAGE_ROOT))

from paus_perception.neck_surface import (
    REASON_INSUFFICIENT_CENTER_DEPTH_POINTS,
    REASON_INSUFFICIENT_REFINEMENT_POINTS,
    REASON_INSUFFICIENT_PATCH_POINTS,
    REASON_LOW_KEYPOINT_CONFIDENCE,
    REASON_NECK_WINDOW_OUT_OF_BOUNDS,
    REASON_PLANE_FIT_RMSE_TOO_HIGH,
    REASON_TANGENT_DIRECTION_DEGENERATE,
    STATUS_FAILED,
    STATUS_OK,
    TARGET_MODE_SHOULDER_CENTER,
    ImageWindow,
    NeckSurfaceConfig,
    PoseKeypoint,
    build_neck_windows,
    build_side_approach_rotation,
    build_surface_rotation,
    compute_target_point_from_anchor,
    depth_to_meters,
    draw_debug_overlay,
    draw_eval_overlay,
    draw_surface_overlay,
    estimate_neck_surface_pose,
    estimate_pca_normal,
    extract_local_patch,
    filter_points_by_depth_band,
    median_surface_point,
    orient_normal_toward_camera,
    project_depth_window_to_points,
    project_tangent_to_plane,
    write_logging_artifacts,
)


def _camera_matrix() -> list[list[float]]:
    return [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]


def _keypoints(*, ears: bool = False, confidence: float = 0.9) -> dict[str, PoseKeypoint]:
    points = {
        "left_shoulder": PoseKeypoint("left_shoulder", 250.0, 340.0, confidence),
        "right_shoulder": PoseKeypoint("right_shoulder", 390.0, 340.0, confidence),
        "nose": PoseKeypoint("nose", 320.0, 120.0, confidence),
    }
    if ears:
        points["left_ear"] = PoseKeypoint("left_ear", 275.0, 150.0, confidence)
        points["right_ear"] = PoseKeypoint("right_ear", 365.0, 150.0, confidence)
    return points


def _depth_image(width: int = 640, height: int = 480, z_m: float = 0.7) -> np.ndarray:
    return np.full((height, width), z_m, dtype=np.float32)


class NeckSurfaceWindowTests(unittest.TestCase):
    def test_nose_based_window_is_bounded(self) -> None:
        cfg = NeckSurfaceConfig()
        result = build_neck_windows(_keypoints(), 640, 480, cfg)
        self.assertEqual(result.status, STATUS_OK, msg=result.message)
        self.assertFalse(result.using_ears)
        assert result.search_window is not None
        assert result.center_window is not None
        self.assertGreater(result.search_window.width, result.center_window.width)
        self.assertGreater(result.search_window.height, result.center_window.height)
        self.assertGreaterEqual(result.search_window.x, 0)
        self.assertLessEqual(result.search_window.x2, 640)

    def test_ear_reference_is_used_when_confident(self) -> None:
        cfg = NeckSurfaceConfig()
        result = build_neck_windows(_keypoints(ears=True), 640, 480, cfg)
        self.assertEqual(result.status, STATUS_OK, msg=result.message)
        self.assertTrue(result.using_ears)

    def test_low_confidence_shoulders_fail(self) -> None:
        cfg = NeckSurfaceConfig(keypoint_confidence_threshold=0.8)
        points = _keypoints(confidence=0.2)
        result = build_neck_windows(points, 640, 480, cfg)
        self.assertEqual(result.status, STATUS_FAILED)
        self.assertEqual(result.reason, REASON_LOW_KEYPOINT_CONFIDENCE)

    def test_out_of_bounds_window_fails(self) -> None:
        cfg = NeckSurfaceConfig()
        points = {
            "left_shoulder": PoseKeypoint("left_shoulder", 20.0, 120.0, 0.9),
            "right_shoulder": PoseKeypoint("right_shoulder", 80.0, 120.0, 0.9),
            "nose": PoseKeypoint("nose", -180.0, 20.0, 0.9),
        }
        result = build_neck_windows(points, 640, 480, cfg)
        self.assertEqual(result.status, STATUS_FAILED)
        self.assertEqual(result.reason, REASON_NECK_WINDOW_OUT_OF_BOUNDS)


class NeckSurfaceGeometryTests(unittest.TestCase):
    def test_depth_projection_uses_camera_intrinsics(self) -> None:
        depth = _depth_image(z_m=1.0)
        points, pixels = project_depth_window_to_points(depth, ImageWindow(320, 240, 1, 1), _camera_matrix())
        self.assertEqual(points.shape, (1, 3))
        np.testing.assert_allclose(points[0], [0.0, 0.0, 1.0], atol=1e-9)
        np.testing.assert_allclose(pixels[0], [320.0, 240.0], atol=1e-9)

    def test_uint16_depth_converts_millimeters_to_meters(self) -> None:
        depth = np.array([[700]], dtype=np.uint16)
        converted = depth_to_meters(depth)
        self.assertTrue(math.isclose(float(converted[0, 0]), 0.7))

    def test_filter_points_rejects_depth_outliers(self) -> None:
        points = np.array([[0.0, 0.0, 0.70], [0.0, 0.0, 0.71], [0.0, 0.0, 1.20]], dtype=np.float64)
        filtered, ratio = filter_points_by_depth_band(points, 50.0)
        self.assertEqual(filtered.shape[0], 2)
        self.assertGreater(ratio, 0.0)
        np.testing.assert_allclose(median_surface_point(filtered), [0.0, 0.0, 0.705], atol=1e-9)

    def test_pca_normal_matches_synthetic_plane(self) -> None:
        xs, ys = np.meshgrid(np.linspace(-0.03, 0.03, 8), np.linspace(-0.03, 0.03, 8))
        points = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, 0.7)))
        normal, rmse = estimate_pca_normal(points)
        self.assertLess(rmse, 1e-6)
        self.assertTrue(abs(float(np.dot(normal, [0.0, 0.0, 1.0]))) > 0.999)
        oriented = orient_normal_toward_camera(normal, np.array([0.0, 0.0, 0.7]))
        np.testing.assert_allclose(oriented, [0.0, 0.0, -1.0], atol=1e-9)

    def test_local_patch_extracts_nearby_points(self) -> None:
        points = np.array([[0.0, 0.0, 0.7], [0.01, 0.0, 0.7], [0.1, 0.0, 0.7]], dtype=np.float64)
        patch = extract_local_patch(points, np.array([0.0, 0.0, 0.7]), 20.0)
        self.assertEqual(patch.shape[0], 2)

    def test_tangent_projection_and_rotation_are_orthonormal(self) -> None:
        normal = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        tangent = project_tangent_to_plane(np.array([1.0, 0.0, 0.2]), normal)
        self.assertTrue(math.isclose(float(np.dot(tangent, normal)), 0.0, abs_tol=1e-9))
        rotation = build_surface_rotation(tangent, normal)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
        self.assertTrue(math.isclose(float(np.linalg.det(rotation)), 1.0, rel_tol=1e-9))

    def test_side_approach_rotation_uses_patient_left_outward_as_z_axis(self) -> None:
        patient_left_outward = np.array([-1.0, 0.0, 0.0], dtype=np.float64)
        shoulder_vec = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        fallback_tangent = np.array([0.0, 1.0, 0.0], dtype=np.float64)

        rotation = build_side_approach_rotation(patient_left_outward, shoulder_vec, fallback_tangent)

        np.testing.assert_allclose(rotation[:, 2], patient_left_outward, atol=1e-9)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
        self.assertTrue(math.isclose(float(np.linalg.det(rotation)), 1.0, rel_tol=1e-9))

    def test_degenerate_tangent_raises(self) -> None:
        with self.assertRaises(ValueError):
            project_tangent_to_plane(np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0]))


class NeckSurfaceEstimatorTests(unittest.TestCase):
    def test_target_point_offsets_from_anchor_in_surface_frame(self) -> None:
        anchor = np.array([0.0, 0.0, 0.7], dtype=np.float64)
        rotation = np.eye(3, dtype=np.float64)

        left_target = compute_target_point_from_anchor(anchor, rotation, "patient_left", 30.0, 10.0)
        right_target = compute_target_point_from_anchor(anchor, rotation, "patient_right", 30.0, 10.0)
        center_target = compute_target_point_from_anchor(anchor, rotation, "center", 30.0, 10.0)

        np.testing.assert_allclose(left_target, [-0.03, 0.01, 0.7], atol=1e-9)
        np.testing.assert_allclose(right_target, [0.03, 0.01, 0.7], atol=1e-9)
        np.testing.assert_allclose(center_target, [0.0, 0.01, 0.7], atol=1e-9)

    def test_estimator_returns_surface_pose_for_valid_depth(self) -> None:
        cfg = NeckSurfaceConfig(min_center_points=10, min_patch_points=8, patch_radius_mm=80.0)
        estimate = estimate_neck_surface_pose(_keypoints(), _depth_image(), _camera_matrix(), cfg)
        self.assertEqual(estimate.status, STATUS_OK, msg=estimate.message)
        self.assertIsNotNone(estimate.surface_point_camera_m)
        self.assertIsNotNone(estimate.target_point_camera_m)
        self.assertIsNotNone(estimate.surface_normal_camera)
        self.assertIsNotNone(estimate.rotation_matrix_camera)
        rotation = np.asarray(estimate.rotation_matrix_camera, dtype=np.float64)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
        self.assertTrue(math.isclose(float(np.linalg.det(rotation)), 1.0, rel_tol=1e-9))

    def test_estimator_refines_patient_left_target_without_second_offset(self) -> None:
        cfg = NeckSurfaceConfig(
            min_center_points=10,
            min_patch_points=8,
            patch_radius_mm=35.0,
            target_region="patient_left",
            lateral_offset_mm=30.0,
            inferior_offset_mm=10.0,
        )
        estimate = estimate_neck_surface_pose(_keypoints(), _depth_image(), _camera_matrix(), cfg)
        self.assertEqual(estimate.status, STATUS_OK, msg=estimate.message)
        assert estimate.surface_point_camera_m is not None
        assert estimate.target_point_camera_m is not None
        assert estimate.coarse_surface_point_camera_m is not None
        assert estimate.coarse_target_point_camera_m is not None
        self.assertEqual(estimate.target_region, "patient_left")
        self.assertEqual(estimate.refinement_status, STATUS_OK)
        np.testing.assert_allclose(estimate.target_point_camera_m, estimate.surface_point_camera_m, atol=1e-9)
        self.assertGreater(
            float(np.linalg.norm(np.asarray(estimate.coarse_target_point_camera_m) - np.asarray(estimate.coarse_surface_point_camera_m))),
            0.025,
        )
        self.assertLess(
            float(np.linalg.norm(np.asarray(estimate.target_point_camera_m) - np.asarray(estimate.coarse_target_point_camera_m))),
            0.025,
        )
        payload = estimate.status_payload()
        self.assertEqual(payload["neck_anchor_camera_m"], estimate.surface_point_camera_m)
        self.assertEqual(payload["target_point_camera_m"], estimate.target_point_camera_m)
        self.assertEqual(payload["coarse_target_point_camera_m"], estimate.coarse_target_point_camera_m)
        self.assertEqual(payload["refinement_status"], STATUS_OK)
        assert estimate.motion_normal_camera is not None
        assert estimate.patient_left_outward_normal_camera is not None
        assert estimate.pca_surface_normal_camera is not None
        np.testing.assert_allclose(np.asarray(estimate.rotation_matrix_camera, dtype=np.float64)[:, 2], estimate.motion_normal_camera, atol=1e-9)
        np.testing.assert_allclose(estimate.pca_surface_normal_camera, estimate.surface_normal_camera, atol=1e-9)
        np.testing.assert_allclose(estimate.motion_normal_camera, estimate.pca_surface_normal_camera, atol=1e-9)
        self.assertTrue(math.isclose(float(np.linalg.norm(estimate.patient_left_outward_normal_camera)), 1.0, rel_tol=1e-9))
        self.assertEqual(payload["motion_normal_camera"], estimate.motion_normal_camera)
        self.assertEqual(payload["pca_surface_normal_camera"], estimate.pca_surface_normal_camera)

    def test_estimator_can_target_shoulder_center_without_refinement(self) -> None:
        cfg = NeckSurfaceConfig(
            min_center_points=10,
            min_patch_points=8,
            patch_radius_mm=35.0,
            target_mode=TARGET_MODE_SHOULDER_CENTER,
            target_region="patient_left",
            lateral_offset_mm=50.0,
            inferior_offset_mm=10.0,
        )

        estimate = estimate_neck_surface_pose(_keypoints(), _depth_image(), _camera_matrix(), cfg)

        self.assertEqual(estimate.status, STATUS_OK, msg=estimate.message)
        self.assertEqual(estimate.target_mode, TARGET_MODE_SHOULDER_CENTER)
        self.assertEqual(estimate.refinement_status, "skipped")
        self.assertEqual(estimate.refinement_reason, TARGET_MODE_SHOULDER_CENTER)
        assert estimate.left_shoulder_camera_m is not None
        assert estimate.right_shoulder_camera_m is not None
        assert estimate.shoulder_center_camera_m is not None
        expected_center = (
            np.asarray(estimate.left_shoulder_camera_m, dtype=np.float64)
            + np.asarray(estimate.right_shoulder_camera_m, dtype=np.float64)
        ) / 2.0
        np.testing.assert_allclose(estimate.shoulder_center_camera_m, expected_center, atol=1e-9)
        np.testing.assert_allclose(estimate.target_point_camera_m, estimate.shoulder_center_camera_m, atol=1e-9)
        assert estimate.rotation_matrix_camera is not None
        np.testing.assert_allclose(np.asarray(estimate.rotation_matrix_camera, dtype=np.float64)[:, 2], estimate.motion_normal_camera, atol=1e-9)
        np.testing.assert_allclose(estimate.motion_normal_camera, estimate.pca_surface_normal_camera, atol=1e-9)
        assert estimate.patient_left_outward_normal_camera is not None
        self.assertTrue(math.isclose(float(np.linalg.norm(estimate.patient_left_outward_normal_camera)), 1.0, rel_tol=1e-9))
        payload = estimate.status_payload()
        self.assertEqual(payload["target_mode"], TARGET_MODE_SHOULDER_CENTER)
        self.assertEqual(payload["shoulder_center_camera_m"], estimate.shoulder_center_camera_m)

    def test_estimator_fails_refinement_without_coarse_fallback_target(self) -> None:
        cfg = NeckSurfaceConfig(
            min_center_points=10,
            min_patch_points=8,
            patch_radius_mm=30.0,
            target_region="patient_left",
            lateral_offset_mm=200.0,
            inferior_offset_mm=0.0,
        )
        estimate = estimate_neck_surface_pose(_keypoints(), _depth_image(), _camera_matrix(), cfg)
        self.assertEqual(estimate.status, STATUS_FAILED)
        self.assertEqual(estimate.reason, REASON_INSUFFICIENT_PATCH_POINTS)
        self.assertEqual(estimate.refinement_status, STATUS_FAILED)
        self.assertEqual(estimate.refinement_reason, REASON_INSUFFICIENT_REFINEMENT_POINTS)
        self.assertIsNone(estimate.target_point_camera_m)
        self.assertIsNone(estimate.rotation_matrix_camera)
        self.assertIsNotNone(estimate.coarse_target_point_camera_m)

    def test_estimator_fails_when_center_depth_is_sparse(self) -> None:
        cfg = NeckSurfaceConfig(min_center_points=10)
        depth = np.zeros((480, 640), dtype=np.float32)
        estimate = estimate_neck_surface_pose(_keypoints(), depth, _camera_matrix(), cfg)
        self.assertEqual(estimate.status, STATUS_FAILED)
        self.assertEqual(estimate.reason, REASON_INSUFFICIENT_CENTER_DEPTH_POINTS)

    def test_estimator_fails_when_patch_is_too_small(self) -> None:
        cfg = NeckSurfaceConfig(min_center_points=10, min_patch_points=5000, patch_radius_mm=10.0)
        estimate = estimate_neck_surface_pose(_keypoints(), _depth_image(), _camera_matrix(), cfg)
        self.assertEqual(estimate.status, STATUS_FAILED)
        self.assertEqual(estimate.reason, REASON_INSUFFICIENT_PATCH_POINTS)

    def test_estimator_fails_for_high_plane_rmse(self) -> None:
        cfg = NeckSurfaceConfig(min_center_points=10, min_patch_points=8, patch_radius_mm=80.0, max_plane_rmse_mm=0.01)
        depth = _depth_image()
        rng = np.random.default_rng(7)
        depth[285:330, 290:350] += rng.normal(0.0, 0.02, size=(45, 60)).astype(np.float32)
        estimate = estimate_neck_surface_pose(_keypoints(), depth, _camera_matrix(), cfg)
        self.assertEqual(estimate.status, STATUS_FAILED)
        self.assertEqual(estimate.reason, REASON_PLANE_FIT_RMSE_TOO_HIGH)

    def test_debug_overlay_draws_target_point_when_camera_matrix_is_available(self) -> None:
        cfg = NeckSurfaceConfig(
            min_center_points=10,
            min_patch_points=8,
            patch_radius_mm=80.0,
            target_region="patient_left",
            lateral_offset_mm=30.0,
            inferior_offset_mm=10.0,
        )
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        estimate = estimate_neck_surface_pose(_keypoints(), _depth_image(), _camera_matrix(), cfg)
        debug = draw_debug_overlay(image, estimate, _camera_matrix())
        self.assertEqual(debug.shape, image.shape)
        self.assertGreater(int(debug[:, :, 0].sum()), 0)
        self.assertGreater(int(debug[:, :, 2].sum()), 0)

    def test_debug_overlay_and_logging_metadata(self) -> None:
        cfg = NeckSurfaceConfig(min_center_points=10, min_patch_points=8, patch_radius_mm=80.0)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = _depth_image()
        estimate = estimate_neck_surface_pose(_keypoints(), depth, _camera_matrix(), cfg)
        debug = draw_debug_overlay(image, estimate)
        debug_eval = draw_eval_overlay(
            image,
            estimate,
            _camera_matrix(),
            {
                "status": "ok",
                "marker_pose_camera_m": [0.0, 0.0, 0.7],
                "markerless_target_camera_m": estimate.target_point_camera_m,
                "error_norm_mm": 12.0,
                "normal_angle_error_deg": 5.0,
            },
        )
        debug_surface = draw_surface_overlay(image, estimate, _camera_matrix())
        self.assertEqual(debug.shape, image.shape)
        with tempfile.TemporaryDirectory() as tmp_dir:
            frame_dir = write_logging_artifacts(
                tmp_dir,
                image,
                depth,
                debug,
                estimate,
                "frame_000001",
                debug_eval_image=debug_eval,
                debug_surface_image=debug_surface,
                save_depth=False,
            )
            self.assertTrue((frame_dir / "rgb.png").exists())
            self.assertFalse((frame_dir / "depth.npy").exists())
            self.assertTrue((frame_dir / "debug.png").exists())
            self.assertTrue((frame_dir / "debug_eval.png").exists())
            self.assertTrue((frame_dir / "debug_surface.png").exists())
            metadata = json.loads((frame_dir / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["source"], "markerless_neck")
            self.assertIn("keypoints", metadata)

    def test_logging_can_save_depth_when_enabled(self) -> None:
        cfg = NeckSurfaceConfig(min_center_points=10, min_patch_points=8, patch_radius_mm=80.0)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        depth = _depth_image()
        estimate = estimate_neck_surface_pose(_keypoints(), depth, _camera_matrix(), cfg)
        debug = draw_debug_overlay(image, estimate)
        with tempfile.TemporaryDirectory() as tmp_dir:
            frame_dir = write_logging_artifacts(tmp_dir, image, depth, debug, estimate, "frame_000001", save_depth=True)
            self.assertTrue((frame_dir / "depth.npy").exists())


if __name__ == "__main__":
    unittest.main()
