from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
if str(PERCEPTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_PACKAGE_ROOT))

from paus_perception import (
    CameraCalibration,
    DepthAlignmentConfig,
    build_factory_extrinsic_matrix,
    convert_pointcloud_raw_to_xyz_meters,
    project_xyz_to_rgb_grid,
    scale_xyz_to_meters,
)


def create_test_calibration(translation_mm: list[float] | None = None) -> CameraCalibration:
    return CameraCalibration(
        image_width=5,
        image_height=5,
        camera_matrix=[
            [1.0, 0.0, 2.0],
            [0.0, 1.0, 2.0],
            [0.0, 0.0, 1.0],
        ],
        dist_coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error=0.0,
        board_rows=0,
        board_cols=0,
        square_size_m=0.0,
        rotation_matrix=[
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],
        translation_vector=translation_mm or [0.0, 0.0, 0.0],
    )


class FakePhotoInfo:
    def __init__(self, pixel_width: int, pixel_height: int, payload_size: int) -> None:
        self.pixel_width = pixel_width
        self.pixel_height = pixel_height
        self.payload_size = payload_size


class FakeSdk:
    def __init__(self) -> None:
        self.last_payload_size: int | None = None

    def Convert3DPointFromCharToFloatCSharp(self, camera_obj, point_info, point_buffer, payload_size, xyz):
        self.last_payload_size = int(payload_size)
        values = np.array(
            [
                1000.0,
                0.0,
                1000.0,
                0.0,
                0.0,
                2000.0,
            ],
            dtype=np.float32,
        )
        xyz[:] = values
        return 0


class DkamDepthAlignmentTests(unittest.TestCase):
    def test_scale_xyz_to_meters_converts_mm_to_m(self) -> None:
        xyz_mm = np.array([[[1000.0, 2000.0, 3000.0]]], dtype=np.float32)
        xyz_m = scale_xyz_to_meters(xyz_mm, 0.001)
        self.assertTrue(np.allclose(xyz_m, np.array([[[1.0, 2.0, 3.0]]], dtype=np.float32)))

    def test_convert_pointcloud_raw_to_xyz_meters_scales_output(self) -> None:
        sdk = FakeSdk()
        point_info = FakePhotoInfo(pixel_width=2, pixel_height=1, payload_size=6)
        xyz_m = convert_pointcloud_raw_to_xyz_meters(
            sdk=sdk,
            camera_obj=0,
            point_info=point_info,
            point_buffer=b"123456",
            point_buffer_size=12,
            unit_scale_to_m=0.001,
        )
        self.assertEqual(xyz_m.shape, (1, 2, 3))
        self.assertEqual(sdk.last_payload_size, 12)
        self.assertTrue(np.allclose(xyz_m[0, 0], np.array([1.0, 0.0, 1.0], dtype=np.float32)))
        self.assertTrue(np.allclose(xyz_m[0, 1], np.array([0.0, 0.0, 2.0], dtype=np.float32)))

    def test_build_factory_extrinsic_matrix_inverse_translation(self) -> None:
        calibration = create_test_calibration([1000.0, 0.0, 0.0])
        matrix = build_factory_extrinsic_matrix(calibration, extrinsic_direction="factory_rt_inverse")
        self.assertTrue(np.allclose(matrix[:3, 3], np.array([-1.0, 0.0, 0.0], dtype=np.float64)))

    def test_project_xyz_to_rgb_grid_keeps_nearest_depth_and_nan_holes(self) -> None:
        calibration = create_test_calibration()
        xyz_m = np.array(
            [
                [[0.0, 0.0, 1.0], [0.0, 0.0, 2.0], [1.0, 0.0, 1.0]],
            ],
            dtype=np.float32,
        )
        result = project_xyz_to_rgb_grid(
            xyz_m,
            calibration,
            config=DepthAlignmentConfig(extrinsic_direction="factory_rt_inverse"),
        )
        self.assertEqual(result.aligned_depth_m.shape, (5, 5))
        self.assertTrue(np.isclose(result.aligned_depth_m[2, 2], 1.0))
        self.assertTrue(np.isclose(result.aligned_depth_m[2, 3], 1.0))
        self.assertTrue(np.isnan(result.aligned_depth_m[0, 0]))

    def test_project_xyz_to_rgb_grid_respects_factory_rt_inverse_direction(self) -> None:
        calibration = create_test_calibration([1000.0, 0.0, 0.0])
        xyz_m = np.array([[[1.0, 0.0, 1.0]]], dtype=np.float32)
        inverse_result = project_xyz_to_rgb_grid(
            xyz_m,
            calibration,
            config=DepthAlignmentConfig(extrinsic_direction="factory_rt_inverse"),
        )
        direct_result = project_xyz_to_rgb_grid(
            xyz_m,
            calibration,
            config=DepthAlignmentConfig(extrinsic_direction="factory_rt"),
        )
        self.assertTrue(np.isclose(inverse_result.aligned_depth_m[2, 2], 1.0))
        self.assertTrue(np.isnan(direct_result.aligned_depth_m[2, 2]))
        self.assertTrue(np.isclose(direct_result.aligned_depth_m[2, 4], 1.0))


if __name__ == "__main__":
    unittest.main()
