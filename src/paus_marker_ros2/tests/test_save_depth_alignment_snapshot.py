from __future__ import annotations

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
for package_root in (PERCEPTION_PACKAGE_ROOT, MARKER_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
SCRIPT_PATH = PROJECT_ROOT / "src" / "paus_marker_ros2" / "scripts" / "save_depth_alignment_snapshot.py"
SPEC = importlib.util.spec_from_file_location("save_depth_alignment_snapshot", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
snapshot = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(snapshot)


class SaveDepthAlignmentSnapshotTests(unittest.TestCase):
    def test_estimate_board_geometry_projects_center(self) -> None:
        board_rows = 8
        board_cols = 11
        corners = np.zeros((board_rows, board_cols, 2), dtype=np.float32)
        for row in range(board_rows):
            for col in range(board_cols):
                corners[row, col] = np.array([100.0 + 20.0 * col + 3.0 * row, 50.0 + 2.0 * col + 18.0 * row], dtype=np.float32)

        geometry = snapshot._estimate_board_geometry(corners, board_rows, board_cols, (400, 400, 3))
        self.assertIsNotNone(geometry)
        assert geometry is not None
        self.assertEqual(geometry.board_mask.shape, (400, 400))
        self.assertGreater(int(np.count_nonzero(geometry.board_mask)), 0)
        expected_center = np.array([100.0 + 20.0 * 5.0 + 3.0 * 3.5, 50.0 + 2.0 * 5.0 + 18.0 * 3.5], dtype=np.float32)
        self.assertTrue(np.allclose(geometry.center_xy, expected_center, atol=1e-4))

    def test_ray_plane_intersection_m_returns_expected_point(self) -> None:
        camera_matrix = np.eye(3, dtype=np.float64)
        dist_coeffs = np.zeros(5, dtype=np.float64)
        point = snapshot._ray_plane_intersection_m(
            camera_matrix,
            dist_coeffs,
            (0.0, 0.0),
            np.array([0.0, 0.0, 2.0], dtype=np.float64),
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
        )
        self.assertIsNotNone(point)
        assert point is not None
        self.assertTrue(np.allclose(point, np.array([0.0, 0.0, 2.0], dtype=np.float64)))

    def test_marker_comparison_uses_camera_frame_position_only(self) -> None:
        marker_message = types.SimpleNamespace(
            header=types.SimpleNamespace(
                frame_id="camera",
                stamp=types.SimpleNamespace(sec=10, nanosec=0),
            ),
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.01, y=0.02, z=0.03),
            ),
        )
        result = snapshot._compute_marker_comparison(
            marker_message=marker_message,
            board_geometry_center_m=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            rgb_stamp_ns=10_000_000_000,
            max_stamp_delta_ns=100_000_000,
        )
        self.assertEqual(result["status"], "ok")
        self.assertTrue(math.isclose(float(result["position_error_mm"]), math.sqrt(0.01**2 + 0.02**2 + 0.03**2) * 1000.0, rel_tol=1e-6))
        self.assertTrue(math.isclose(float(result["delta_x_m"]), 0.01, rel_tol=1e-9))
        self.assertTrue(math.isclose(float(result["delta_y_m"]), 0.02, rel_tol=1e-9))
        self.assertTrue(math.isclose(float(result["delta_z_m"]), 0.03, rel_tol=1e-9))

    def test_marker_comparison_skips_frame_mismatch(self) -> None:
        marker_message = types.SimpleNamespace(
            header=types.SimpleNamespace(
                frame_id="base",
                stamp=types.SimpleNamespace(sec=10, nanosec=0),
            ),
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            ),
        )
        result = snapshot._compute_marker_comparison(
            marker_message=marker_message,
            board_geometry_center_m=np.array([0.0, 0.0, 0.0], dtype=np.float64),
            rgb_stamp_ns=10_000_000_000,
            max_stamp_delta_ns=100_000_000,
        )
        self.assertEqual(result["status"], "skipped")
        self.assertEqual(result["reason"], "marker_frame_mismatch")


if __name__ == "__main__":
    unittest.main()
