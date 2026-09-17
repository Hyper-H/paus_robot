from __future__ import annotations

import sys
import tempfile
import unittest
import os
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
if str(PERCEPTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_PACKAGE_ROOT))
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
if str(MOTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MOTION_PACKAGE_ROOT))

from paus_perception import load_config, resolve_config_path
from paus_motion_ros2.fairino_linux_client import FairinoLinuxClient


class ConfigControlDefaultsTests(unittest.TestCase):
    def test_default_control_config_contains_fairino_sdk_fallback_settings(self) -> None:
        config = load_config()
        control = config["control"]
        self.assertEqual(control["linux_fairino_sdk_root"], "/opt/fairino_python_sdk/linux")
        self.assertNotIn("prefer_windows_exec_bridge", control)
        self.assertNotIn("prefer_remote_command_service", control)
        self.assertEqual(control["global_speed"], 100.0)

    def test_default_control_config_uses_side_approach_hover(self) -> None:
        config = load_config(PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml")
        control = config["control"]
        self.assertFalse(control["execute_motion"])
        self.assertEqual(control["hover_clearance_mm"], 50.0)
        self.assertFalse(config["admittance_1d"]["enabled"])
        self.assertNotIn("contact_hover_clearance_mm", config["admittance_1d"])
        self.assertFalse(control["prefer_positive_z_surface_normal"])
        self.assertEqual(control["max_execution_stage"], "final_hover")
        self.assertEqual(control["motion_strategy"], "staged_patient_left_final_hover")
        self.assertFalse(control["move_to_approach_ready_on_start"])
        self.assertEqual(len(control["approach_ready_joint_deg"]), 6)
        self.assertTrue(all(isinstance(value, (int, float)) for value in control["approach_ready_joint_deg"]))
        self.assertEqual(control["approach_ready_tolerance_deg"], 10.0)
        self.assertEqual(control["approach_ready_vel"], 30.0)
        self.assertEqual(control["pre_approach_vel"], 20.0)
        self.assertEqual(control["final_hover_vel"], 5.0)
        self.assertEqual(control["max_direct_final_hover_distance_mm"], 250.0)
        self.assertEqual(control["roll_policy"], "base_up_projection")
        self.assertEqual(control["roll_offset_deg"], 0.0)

    def test_user_config_can_override_fairino_sdk_paths(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "config.yaml"
            config_path.write_text(
                yaml.safe_dump(
                    {
                        "control": {
                            "linux_fairino_sdk_root": "/srv/fairino/linux_sdk",
                            "use_mock_pose": True,
                        }
                    },
                    sort_keys=False,
                    allow_unicode=True,
                ),
                encoding="utf-8",
            )

            config = load_config(config_path)
            control = config["control"]
            self.assertEqual(control["linux_fairino_sdk_root"], "/srv/fairino/linux_sdk")
            self.assertTrue(control["use_mock_pose"])

    def test_default_yaml_uses_external_calibration_artifacts(self) -> None:
        config_path = PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml"

        config = load_config(config_path)
        calibration = config["calibration"]

        self.assertEqual(calibration["output_path"], "/mnt/data/projects/paus_robot/calibration/current/extrinsics.yaml")
        self.assertEqual(calibration["session_root_path"], "/mnt/data/projects/paus_robot/calibration/sessions")
        self.assertEqual(
            calibration["example_output_path"],
            str(PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "extrinsics.yaml"),
        )
        self.assertEqual(
            calibration["trajectory_path"],
            str(PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "eye_to_hand_trajectory.example.yaml"),
        )

    def test_resolve_config_path_uses_config_location_for_relative_paths(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            project_root = Path(temp_dir) / "demo_ws"
            config_dir = project_root / "src" / "paus_bringup" / "configs"
            perception_dir = project_root / "src" / "paus_perception"
            perception_dir.mkdir(parents=True, exist_ok=True)
            config_dir.mkdir(parents=True, exist_ok=True)
            config_path = config_dir / "default.yaml"
            config_path.write_text("calibration: {}\n", encoding="utf-8")

            resolved = resolve_config_path("calibration_sessions", config_path)

            self.assertEqual(resolved, str(project_root / "calibration_sessions"))

    def test_default_yaml_paths_are_install_safe(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            runtime_root = Path(temp_dir) / "runtime"
            old_runtime_root = os.environ.get("PAUS_ROBOT_RUNTIME_DIR")
            os.environ["PAUS_ROBOT_RUNTIME_DIR"] = str(runtime_root)
            install_config_dir = Path(temp_dir) / "install" / "paus_bringup" / "share" / "paus_bringup" / "configs"
            install_config_dir.mkdir(parents=True, exist_ok=True)
            config_path = install_config_dir / "default.yaml"
            try:
                config_path.write_text(
                    yaml.safe_dump(
                        {
                            "calibration": {
                                "output_path": "extrinsics.yaml",
                                "trajectory_path": "eye_to_hand_trajectory.yaml",
                                "session_root_path": "calibration_sessions",
                            }
                        },
                        sort_keys=False,
                    ),
                    encoding="utf-8",
                )

                config = load_config(config_path)
                calibration = config["calibration"]

                self.assertEqual(calibration["output_path"], str(runtime_root / "configs" / "extrinsics.yaml"))
                self.assertEqual(calibration["trajectory_path"], str(runtime_root / "configs" / "eye_to_hand_trajectory.yaml"))
                self.assertEqual(calibration["session_root_path"], str(runtime_root / "calibration_sessions"))
            finally:
                if old_runtime_root is None:
                    os.environ.pop("PAUS_ROBOT_RUNTIME_DIR", None)
                else:
                    os.environ["PAUS_ROBOT_RUNTIME_DIR"] = old_runtime_root


class FairinoLinuxClientTests(unittest.TestCase):
    def test_prepare_motion_waits_for_automatic_mode_confirmation(self) -> None:
        class State:
            main_code = 0
            sub_code = 0
            robot_state = 1
            program_state = 1
            rbtEnableState = 1
            EmergencyStop = 0
            safety_stop0_state = 0
            safety_stop1_state = 0
            motion_done = 1
            mc_queue_len = 0
            collisionState = 0

        class RobotStub:
            def __init__(self) -> None:
                self.calls: list[tuple[str, int | None]] = []
                self.states = [1, 0]

            def ResetAllError(self):
                self.calls.append(("ResetAllError", None))
                return 0

            def RobotEnable(self, state):
                self.calls.append(("RobotEnable", state))
                return 0

            def Mode(self, state):
                self.calls.append(("Mode", state))
                return 0

            def GetRobotRealTimeState(self):
                state = State()
                state.robot_mode = self.states.pop(0)
                return 0, state

        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")
        client.robot = RobotStub()

        result = client.prepare_motion(mode_timeout_s=1.0, poll_interval_s=0.0)

        self.assertEqual(result["Mode"], 0)
        self.assertEqual(result["ModeConfirm"], 0)
        self.assertEqual(result["RobotMode"], 0)
        self.assertEqual(
            client.robot.calls,
            [("ResetAllError", None), ("RobotEnable", 1), ("Mode", 0)],
        )

    def test_prepare_motion_sets_global_speed_after_mode_confirmation(self) -> None:
        class State:
            main_code = 0
            sub_code = 0
            robot_mode = 0
            robot_state = 1
            program_state = 1
            rbtEnableState = 1
            EmergencyStop = 0
            safety_stop0_state = 0
            safety_stop1_state = 0
            motion_done = 1
            mc_queue_len = 0
            collisionState = 0

        class RobotStub:
            def __init__(self) -> None:
                self.calls: list[tuple[str, int | float | None]] = []

            def ResetAllError(self):
                self.calls.append(("ResetAllError", None))
                return 0

            def RobotEnable(self, state):
                self.calls.append(("RobotEnable", state))
                return 0

            def Mode(self, state):
                self.calls.append(("Mode", state))
                return 0

            def GetRobotRealTimeState(self):
                return 0, State()

            def SetSpeed(self, speed):
                self.calls.append(("SetSpeed", speed))
                return 0

        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")
        client.robot = RobotStub()

        result = client.prepare_motion(global_speed=100.0, mode_timeout_s=0.0, poll_interval_s=0.0)

        self.assertEqual(result["GlobalSpeed"], 0)
        self.assertEqual(
            client.robot.calls,
            [("ResetAllError", None), ("RobotEnable", 1), ("Mode", 0), ("SetSpeed", 100)],
        )

    def test_set_speed_rejects_values_outside_controller_range(self) -> None:
        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")

        with self.assertRaisesRegex(ValueError, r"\[0, 100\]"):
            client.set_speed(100.1)

    def test_prepare_motion_refuses_to_continue_when_mode_stays_manual(self) -> None:
        class State:
            main_code = 0
            sub_code = 0
            robot_mode = 1
            robot_state = 1
            program_state = 1
            rbtEnableState = 1
            EmergencyStop = 0
            safety_stop0_state = 0
            safety_stop1_state = 0
            motion_done = 1
            mc_queue_len = 0
            collisionState = 0

        class RobotStub:
            def ResetAllError(self):
                return 0

            def RobotEnable(self, state):
                del state
                return 0

            def Mode(self, state):
                del state
                return 0

            def GetRobotRealTimeState(self):
                return 0, State()

        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")
        client.robot = RobotStub()

        result = client.prepare_motion(mode_timeout_s=0.0, poll_interval_s=0.0)

        self.assertEqual(result["Mode"], 0)
        self.assertEqual(result["ModeConfirm"], 1)
        self.assertEqual(result["RobotMode"], 1)
        self.assertEqual(result["ModeAttempts"], 3)

    def test_get_motion_diagnostics_contains_controller_state(self) -> None:
        class State:
            main_code = 14
            sub_code = 154
            robot_mode = 1
            robot_state = 1
            program_state = 1
            rbtEnableState = 1
            EmergencyStop = 0
            safety_stop0_state = 0
            safety_stop1_state = 0
            motion_done = 1
            mc_queue_len = 0
            collisionState = 0

        class RobotStub:
            def GetRobotRealTimeState(self):
                return 0, State()

        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")
        client.robot = RobotStub()

        diagnostics = client.get_motion_diagnostics()

        self.assertEqual(diagnostics["realtime_state_error"], 0)
        self.assertEqual(diagnostics["main_code"], 14)
        self.assertEqual(diagnostics["sub_code"], 154)
        self.assertEqual(diagnostics["robot_mode"], 1)

    def test_realtime_state_waits_for_async_feedback_instance(self) -> None:
        class State:
            main_code = 0
            sub_code = 0
            robot_mode = 1
            robot_state = 1
            program_state = 1
            rbtEnableState = 1
            EmergencyStop = 0
            safety_stop0_state = 0
            safety_stop1_state = 0
            motion_done = 1
            mc_queue_len = 0
            collisionState = 0

        class RobotStub:
            def __init__(self) -> None:
                self.calls = 0

            def GetRobotRealTimeState(self):
                self.calls += 1
                return 0, State if self.calls == 1 else State()

        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")
        client.robot = RobotStub()

        error, state = client.get_robot_realtime_state()

        self.assertEqual(error, 0)
        self.assertEqual(state["robot_mode"], 1)
        self.assertEqual(client.robot.calls, 2)

    def test_client_wraps_forward_kinematics_and_joint_soft_limits(self) -> None:
        class RobotStub:
            def GetForwardKin(self, joint_pos):
                del joint_pos
                return 0, 100, 200, 300, 1, 2, 3

            def GetJointSoftLimitDeg(self):
                return 0, -180, 180, -120, 120, -120, 120, -180, 180, -120, 120, -360, 360

        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")
        client.robot = RobotStub()

        fk_error, pose = client.get_forward_kin([1, 2, 3, 4, 5, 6])
        limit_error, limits = client.get_joint_soft_limit_deg()

        self.assertEqual(fk_error, 0)
        self.assertEqual(pose, [100.0, 200.0, 300.0, 1.0, 2.0, 3.0])
        self.assertEqual(limit_error, 0)
        self.assertEqual(len(limits), 12)

    def test_client_wraps_robot_error_code(self) -> None:
        class RobotStub:
            def GetRobotErrorCode(self):
                return 0, [14, 154]

        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")
        client.robot = RobotStub()

        error, codes = client.get_robot_error_code()

        self.assertEqual(error, 0)
        self.assertEqual(codes, [14, 154])

    def test_move_j_forwards_acceleration_when_present(self) -> None:
        class RobotStub:
            def __init__(self) -> None:
                self.kwargs = None

            def MoveJ(self, joint_pos, **kwargs):
                del joint_pos
                self.kwargs = kwargs
                return 0

        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")
        client.robot = RobotStub()

        error = client.move_j([1, 2, 3, 4, 5, 6], tool_id=0, user_id=0, vel=10.0, acc=7.5)

        self.assertEqual(error, 0)
        self.assertEqual(client.robot.kwargs["acc"], 7.5)

    def test_normalize_pose_result_accepts_sdk_list_shape(self) -> None:
        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")

        error, pose = client._normalize_pose_result((0, [1, 2, 3, 4, 5, 6]), "GetActualTCPPose")

        self.assertEqual(error, 0)
        self.assertEqual(pose, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_normalize_pose_result_accepts_xmlrpc_flat_shape(self) -> None:
        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")

        error, pose = client._normalize_pose_result((0, 1, 2, 3, 4, 5, 6), "GetActualTCPPose")

        self.assertEqual(error, 0)
        self.assertEqual(pose, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_normalize_pose_result_accepts_joint_position_shape(self) -> None:
        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")

        error, joints = client._normalize_pose_result((0, [10, 20, 30, 40, 50, 60]), "GetActualJointPosDegree")

        self.assertEqual(error, 0)
        self.assertEqual(joints, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])


if __name__ == "__main__":
    unittest.main()
