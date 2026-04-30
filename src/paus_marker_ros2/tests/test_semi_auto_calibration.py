from __future__ import annotations

from datetime import datetime
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
if str(MARKER_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MARKER_PACKAGE_ROOT))

from paus_marker_ros2.semi_auto_calibration import (
    TrajectoryValidationError,
    build_recorded_waypoint,
    create_session_dir,
    empty_trajectory,
    load_trajectory,
    save_trajectory,
)
from paus_marker_ros2.eye_to_hand_calibration_node import _wrapped_rotation_delta_norm_deg


class SemiAutoCalibrationTrajectoryTests(unittest.TestCase):
    def test_wrapped_rotation_delta_handles_boundary_crossing(self) -> None:
        delta = _wrapped_rotation_delta_norm_deg([0.0, 0.0, -179.9], [0.0, 0.0, 179.9])

        self.assertLess(delta, 0.3)

    def test_save_and_load_recorded_movej_trajectory(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            trajectory = empty_trajectory(tool_id=0, user_id=0, default_vel=10.0, default_acc=10.0, default_dwell_s=0.5)
            trajectory.waypoints.append(
                build_recorded_waypoint(
                    index=1,
                    joint_deg=[1, 2, 3, 4, 5, 6],
                    tcp_pose_mmdeg=[100, 200, 300, 10, 20, 30],
                    vel=8.0,
                    acc=9.0,
                    dwell_s=0.7,
                    record_quality={
                        "detected": True,
                        "reprojection_error_px": 3.2,
                        "board_margin_px": 42.0,
                    },
                )
            )
            path = Path(temp_dir) / "eye_to_hand_trajectory.yaml"

            save_trajectory(trajectory, path)
            loaded = load_trajectory(path)

            self.assertEqual(loaded.tool_id, 0)
            self.assertEqual(loaded.user_id, 0)
            self.assertEqual(len(loaded.waypoints), 1)
            self.assertEqual(loaded.waypoints[0].motion, "movej")
            self.assertEqual(loaded.waypoints[0].joint_deg, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
            self.assertEqual(loaded.waypoints[0].expected_tcp_pose_mmdeg, [100.0, 200.0, 300.0, 10.0, 20.0, 30.0])
            self.assertEqual(loaded.waypoints[0].record_quality["reprojection_error_px"], 3.2)

    def test_invalid_motion_mode_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "bad.yaml"
            path.write_text(
                yaml.safe_dump(
                    {
                        "version": 1,
                        "defaults": {"motion": "movel"},
                        "waypoints": [
                            {
                                "joint_deg": [1, 2, 3, 4, 5, 6],
                                "expected_tcp_pose_mmdeg": [100, 200, 300, 10, 20, 30],
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaises(TrajectoryValidationError):
                load_trajectory(path)

    def test_empty_trajectory_is_rejected_for_execution(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "empty.yaml"
            save_trajectory(empty_trajectory(tool_id=0, user_id=0, default_vel=10.0, default_acc=10.0, default_dwell_s=0.5), path)

            with self.assertRaises(TrajectoryValidationError):
                load_trajectory(path)

    def test_session_dir_uses_timestamp_and_unique_suffix(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            now = datetime(2026, 4, 28, 15, 30, 12)

            first = create_session_dir(temp_dir, now=now)
            second = create_session_dir(temp_dir, now=now)

            self.assertEqual(first.name, "2026-04-28_153012")
            self.assertEqual(second.name, "2026-04-28_153012_02")
            self.assertTrue((first / "images").is_dir())


if __name__ == "__main__":
    unittest.main()
