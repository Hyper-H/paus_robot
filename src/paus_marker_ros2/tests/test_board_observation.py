from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
for package_root in (MARKER_PACKAGE_ROOT, PERCEPTION_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

from paus_marker_ros2.board_observation import (  # noqa: E402
    DepthObservationQualityConfig,
    build_board_object_points,
    estimate_depth_aligned_board_pose_from_corners,
)
from paus_perception import make_transform_matrix  # noqa: E402


class BoardObservationTests(unittest.TestCase):
    def test_depth_aligned_pose_recovers_fronto_parallel_board(self) -> None:
        board_rows = 3
        board_cols = 3
        square_size_m = 0.1
        camera_matrix = np.array(
            [
                [100.0, 0.0, 50.0],
                [0.0, 100.0, 50.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        dist_coeffs = np.zeros(5, dtype=np.float64)
        object_points = build_board_object_points(board_rows, board_cols, square_size_m)
        rvec = np.zeros((3, 1), dtype=np.float64)
        tvec = np.array([[0.0], [0.0], [1.0]], dtype=np.float64)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
        corners_xy = projected.reshape((board_rows, board_cols, 2)).astype(np.float32)
        depth_m = np.ones((120, 120), dtype=np.float32)

        estimate = estimate_depth_aligned_board_pose_from_corners(
            corners_xy,
            depth_m,
            board_rows=board_rows,
            board_cols=board_cols,
            square_size_m=square_size_m,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            quality_config=DepthObservationQualityConfig(
                corner_patch_size_px=3,
                corner_min_points=3,
                min_board_center_z_m=0.0,
                max_board_center_z_m=2.0,
            ),
            rgb_depth_delta_ms=1.0,
        )

        self.assertTrue(bool(estimate.quality["accepted"]))
        assert estimate.camera_to_board_matrix is not None
        expected = make_transform_matrix([0.0, 0.0, 1.0], np.eye(3))
        self.assertTrue(np.allclose(estimate.camera_to_board_matrix, expected, atol=1e-6))
        self.assertEqual(estimate.quality["status"], "accepted")
        self.assertLess(float(estimate.quality["board_model_fit_rmse_mm"]), 1e-3)
        self.assertAlmostEqual(float(estimate.quality["valid_corner_patch_ratio"]), 1.0, places=6)

    def test_depth_aligned_pose_rejects_when_depth_missing(self) -> None:
        board_rows = 3
        board_cols = 3
        square_size_m = 0.1
        camera_matrix = np.array(
            [
                [100.0, 0.0, 50.0],
                [0.0, 100.0, 50.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        dist_coeffs = np.zeros(5, dtype=np.float64)
        object_points = build_board_object_points(board_rows, board_cols, square_size_m)
        projected, _ = cv2.projectPoints(
            object_points,
            np.zeros((3, 1), dtype=np.float64),
            np.array([[0.0], [0.0], [1.0]], dtype=np.float64),
            camera_matrix,
            dist_coeffs,
        )
        corners_xy = projected.reshape((board_rows, board_cols, 2)).astype(np.float32)
        depth_m = np.full((120, 120), np.nan, dtype=np.float32)

        estimate = estimate_depth_aligned_board_pose_from_corners(
            corners_xy,
            depth_m,
            board_rows=board_rows,
            board_cols=board_cols,
            square_size_m=square_size_m,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            quality_config=DepthObservationQualityConfig(
                corner_patch_size_px=3,
                corner_min_points=3,
                min_board_center_z_m=0.0,
                max_board_center_z_m=2.0,
            ),
            rgb_depth_delta_ms=1.0,
        )

        self.assertFalse(bool(estimate.quality["accepted"]))
        self.assertEqual(estimate.quality["reject_reason"], "no_valid_corner_patches")
        self.assertIsNone(estimate.camera_to_board_matrix)


if __name__ == "__main__":
    unittest.main()
