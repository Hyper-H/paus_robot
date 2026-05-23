from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
BRINGUP_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_bringup"
if str(BRINGUP_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRINGUP_PACKAGE_ROOT))

from paus_bringup.stop_paus_runtime import ProcessInfo, find_candidates, is_paus_runtime_command, parse_ps_line


class StopPausRuntimeTests(unittest.TestCase):
    def test_matches_paus_runtime_commands(self) -> None:
        commands = [
            "/usr/bin/python3 /opt/ros/humble/bin/ros2 launch paus_bringup online_stack.launch.py execute_motion:=false",
            "/home/chen_lab/miniconda3/envs/paus_robot/bin/python3 /mnt/data/projects/paus_robot/repo/install/paus_marker_ros2/share/paus_marker_ros2/scripts/camera_bridge.py --enable-depth",
            "/usr/bin/python3 /mnt/data/projects/paus_robot/repo/install/paus_marker_ros2/lib/paus_marker_ros2/neck_surface_pose_node --ros-args",
            "/usr/bin/python3 -m paus_ui.web_server --ros-args -r __node:=paus_ui_server",
        ]

        for command in commands:
            self.assertTrue(is_paus_runtime_command(command), command)

    def test_does_not_match_unrelated_processes(self) -> None:
        commands = [
            "/usr/bin/python3 /usr/bin/update-manager --no-update",
            "/opt/google/chrome/chrome --type=renderer",
            "python3 /tmp/random_script.py",
            "ros2 topic echo /neck_target_eval_status",
            "python3 -m pytest src/paus_bringup/tests/test_stop_paus_runtime.py",
        ]

        for command in commands:
            self.assertFalse(is_paus_runtime_command(command), command)

    def test_find_candidates_filters_process_list(self) -> None:
        processes = [
            ProcessInfo(1, 0, "Sl", 0.0, 0.1, "python3 /usr/bin/update-manager"),
            ProcessInfo(2, 1, "Sl", 12.0, 0.4, "python3 /opt/ros/humble/bin/ros2 launch paus_bringup ui.launch.py"),
            ProcessInfo(3, 2, "Rl", 80.0, 0.6, "python3 /mnt/data/projects/paus_robot/repo/install/paus_marker_ros2/share/paus_marker_ros2/scripts/camera_bridge.py --enable-depth"),
        ]

        candidates = find_candidates(processes)

        self.assertEqual([process.pid for process in candidates], [2, 3])

    def test_parse_ps_line_handles_command_with_spaces(self) -> None:
        process = parse_ps_line("  123  1 Sl  2.5  0.4 python3 /opt/ros/humble/bin/ros2 launch paus_bringup online_stack.launch.py")

        self.assertIsNotNone(process)
        assert process is not None
        self.assertEqual(process.pid, 123)
        self.assertEqual(process.ppid, 1)
        self.assertEqual(process.command, "python3 /opt/ros/humble/bin/ros2 launch paus_bringup online_stack.launch.py")


if __name__ == "__main__":
    unittest.main()
