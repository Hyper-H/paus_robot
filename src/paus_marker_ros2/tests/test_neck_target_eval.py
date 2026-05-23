from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
if str(MARKER_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MARKER_PACKAGE_ROOT))

from paus_marker_ros2.neck_target_eval import build_eval_record, compute_error_mm, stamp_to_ns


class NeckTargetEvalTests(unittest.TestCase):
    def test_compute_error_uses_markerless_minus_marker_in_mm(self) -> None:
        error_xyz_mm, error_norm_mm = compute_error_mm([0.05, -0.10, 0.79], [0.02, -0.11, 0.78])

        self.assertEqual(len(error_xyz_mm), 3)
        self.assertTrue(math.isclose(error_xyz_mm[0], 30.0))
        self.assertTrue(math.isclose(error_xyz_mm[1], 10.0))
        self.assertTrue(math.isclose(error_xyz_mm[2], 10.0))
        self.assertTrue(math.isclose(error_norm_mm, math.sqrt(1100.0)))

    def test_build_eval_record_contains_primary_metrics(self) -> None:
        payload = {
            "status": "ok",
            "target_point_camera_m": [0.05, -0.10, 0.79],
            "neck_anchor_camera_m": [0.02, -0.09, 0.80],
            "target_region": "patient_left",
            "lateral_offset_mm": 30.0,
            "inferior_offset_mm": 10.0,
            "plane_rmse_mm": 4.2,
            "patch_points": 180,
            "rgb_depth_delta_ms": 35.0,
        }

        record = build_eval_record(
            payload,
            [0.02, -0.11, 0.78],
            neck_stamp_ns=1_000_000_000,
            marker_stamp_ns=1_028_400_000,
        )

        self.assertEqual(record["status"], "ok")
        self.assertEqual(record["pair_delta_ms"], 28.4)
        self.assertEqual(record["marker_pose_camera_m"], [0.02, -0.11, 0.78])
        self.assertEqual(record["markerless_target_camera_m"], [0.05, -0.10, 0.79])
        self.assertEqual(record["neck_anchor_camera_m"], [0.02, -0.09, 0.80])
        self.assertEqual(record["target_region"], "patient_left")
        self.assertEqual(record["lateral_offset_mm"], 30.0)
        self.assertEqual(record["inferior_offset_mm"], 10.0)
        self.assertEqual(record["plane_rmse_mm"], 4.2)
        self.assertEqual(record["patch_points"], 180)
        self.assertEqual(record["rgb_depth_delta_ms"], 35.0)
        self.assertTrue(math.isclose(record["error_norm_mm"], math.sqrt(1100.0)))

    def test_build_eval_record_rejects_stale_pairs(self) -> None:
        payload = {"status": "ok", "target_point_camera_m": [0.0, 0.0, 0.7]}

        record = build_eval_record(
            payload,
            [0.0, 0.0, 0.7],
            neck_stamp_ns=1_000_000_000,
            marker_stamp_ns=1_400_000_000,
            max_pair_delta_ms=300.0,
        )

        self.assertEqual(record["status"], "waiting")
        self.assertEqual(record["reason"], "pair_delta_exceeded")
        self.assertEqual(record["pair_delta_ms"], 400.0)

    def test_build_eval_record_waits_for_missing_markerless_target(self) -> None:
        record = build_eval_record({"status": "ok"}, [0.0, 0.0, 0.7], neck_stamp_ns=1, marker_stamp_ns=1)

        self.assertEqual(record["status"], "waiting")
        self.assertEqual(record["reason"], "missing_markerless_target")

    def test_build_eval_record_waits_when_markerless_is_not_ok(self) -> None:
        record = build_eval_record({"status": "failed", "reason": "low_keypoint_confidence"}, [0.0, 0.0, 0.7], neck_stamp_ns=1, marker_stamp_ns=1)

        self.assertEqual(record["status"], "waiting")
        self.assertEqual(record["reason"], "markerless_not_ok")
        self.assertEqual(record["markerless_reason"], "low_keypoint_confidence")

    def test_stamp_to_ns_converts_ros_stamp_shape(self) -> None:
        stamp = SimpleNamespace(sec=12, nanosec=34)

        self.assertEqual(stamp_to_ns(stamp), 12_000_000_034)


if __name__ == "__main__":
    unittest.main()