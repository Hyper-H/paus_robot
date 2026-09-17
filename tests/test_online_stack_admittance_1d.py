from __future__ import annotations

import importlib.util
from pathlib import Path
import threading
import unittest

from paus_motion_ros2.admittance_1d import FzAdmittanceConfig, FzAdmittanceController
from paus_motion_ros2.admittance_session import (
    AdmittanceSession,
    AdmittanceSessionConfig,
    AdmittanceSessionState,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, duration: float) -> None:
        self.now += duration


class FakeClient:
    def __init__(self, samples: list[list[float]]) -> None:
        self.samples = list(samples)
        self.servo_targets: list[list[float]] = []
        self.servo_modes: list[int] = []
        self.calls: list[str] = []

    def ft_activate(self, state: bool, **kwargs) -> int:
        del kwargs
        self.calls.append(f"ft_activate:{state}")
        return 0

    def ft_set_zero(self, state: bool = True) -> int:
        self.calls.append(f"ft_set_zero:{state}")
        return 0

    def ft_set_rcs(self, ref: int = 0, **kwargs) -> int:
        del kwargs
        self.calls.append(f"ft_set_rcs:{ref}")
        return 0

    def ft_get_force_torque_rcs(self) -> tuple[int, list[float]]:
        if not self.samples:
            return 0, [0.0, 0.0, -0.1, 0.0, 0.0, 0.0]
        return 0, list(self.samples.pop(0))

    def servo_move_start(self, **kwargs) -> int:
        del kwargs
        self.calls.append("servo_move_start")
        return 0

    def servo_move_end(self, **kwargs) -> int:
        del kwargs
        self.calls.append("servo_move_end")
        return 0

    def servo_cart_tool_delta(self, pose: list[float], **kwargs) -> int:
        self.servo_modes.append(int(kwargs.get("mode", -1)))
        self.servo_targets.append(list(pose))
        return 0

    def stop_motion(self, **kwargs) -> int:
        del kwargs
        self.calls.append("stop_motion")
        return 0


class FailingLateralClient(FakeClient):
    def __init__(self, wrench: list[float]) -> None:
        super().__init__([wrench])


class OnlineStackAdmittanceTests(unittest.TestCase):
    def test_controller_limits_and_integrates_actual_dt(self) -> None:
        controller = FzAdmittanceController(
            FzAdmittanceConfig(
                target_force_n=2.0,
                deadband_n=0.1,
                gain_mm_s_n=10.0,
                max_speed_mm_s=0.5,
                max_press_mm=0.2,
                force_filter_s=0.0,
            )
        )
        step = controller.update(0.0, 0.1)
        self.assertEqual(step.state.command_speed_mm_s, 0.5)
        self.assertAlmostEqual(step.state.press_travel_mm, 0.05)
        self.assertAlmostEqual(step.delta_z_mm, -0.05)
        step = controller.update(0.0, 1.0)
        self.assertAlmostEqual(step.state.press_travel_mm, 0.2)
        self.assertTrue(step.state.at_press_limit)

    def test_session_contact_settle_hold_release_return(self) -> None:
        clock = FakeClock()
        client = FakeClient(
            [
                [0.0, 0.0, -0.5, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, -2.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, -2.0, 0.0, 0.0, 0.0],
            ]
        )
        returned: list[bool] = []
        session = AdmittanceSession(
            client,
            config=AdmittanceSessionConfig(
                hold_s=0.0,
                force_stable_s=0.0,
                loop_hz=10.0,
                force_filter_s=0.0,
                contact_timeout_s=1.0,
                settling_timeout_s=1.0,
            ),
            start_pose_mmdeg=[0.0] * 6,
            return_callback=lambda: returned.append(True),
            clock=clock,
            sleep=clock.sleep,
        )
        result = session.run()
        self.assertTrue(result.success)
        self.assertEqual(result.state, AdmittanceSessionState.DONE)
        self.assertTrue(returned)
        self.assertEqual(client.calls.count("servo_move_end"), 1)
        self.assertIn("ft_activate:False", client.calls)
        self.assertTrue(any(target[2] < 0.0 for target in client.servo_targets))
        self.assertTrue(client.servo_modes)
        self.assertTrue(all(mode == 2 for mode in client.servo_modes))

    def test_session_runs_configured_strokes_and_returns_to_origin(self) -> None:
        clock = FakeClock()
        client = FakeClient(
            [
                [0.0, 0.0, -0.5, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
                *[[0.0, 0.0, -2.0, 0.0, 0.0, 0.0] for _ in range(14)],
                *[[0.0, 0.0, -0.1, 0.0, 0.0, 0.0] for _ in range(40)],
            ]
        )
        trace: list[dict[str, object]] = []
        session = AdmittanceSession(
            client,
            config=AdmittanceSessionConfig(
                hold_s=0.0,
                force_stable_s=0.0,
                loop_hz=10.0,
                force_filter_s=0.0,
                contact_timeout_s=1.0,
                settling_timeout_s=1.0,
                release_timeout_s=2.0,
                stroke_enabled=True,
                stroke_axes=("x", "y", "xy"),
                stroke_distances_mm=(0.2, -0.2, 0.2),
                stroke_speed_mm_s=1.0,
                stroke_return_to_origin=True,
            ),
            start_pose_mmdeg=[0.0] * 6,
            trace_callback=trace.append,
            clock=clock,
            sleep=clock.sleep,
        )

        result = session.run()

        self.assertTrue(result.success)
        self.assertEqual(result.state, AdmittanceSessionState.DONE)
        self.assertEqual(result.stroke_completed_segments, 3)
        self.assertAlmostEqual(result.stroke_total_distance_mm, 1.2)
        self.assertTrue(any(event["event"] == "stroke_sequence_completed" for event in result.events))
        stroke_samples = [
            event
            for event in trace
            if event.get("stroke_axis") is not None and event.get("stroke_phase") is not None
        ]
        self.assertTrue(stroke_samples)
        self.assertTrue(any(sample["stroke_phase"] == "OUTBOUND" for sample in stroke_samples))
        self.assertTrue(any(sample["stroke_phase"] == "RETURN" for sample in stroke_samples))
        self.assertTrue(any(abs(target[0]) > 0.0 for target in client.servo_targets))
        self.assertTrue(any(abs(target[1]) > 0.0 for target in client.servo_targets))

    def test_session_contact_stage_releases_at_detection_force_without_settling(self) -> None:
        clock = FakeClock()
        client = FakeClient(
            [
                [0.0, 0.0, -0.5, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, -0.2, 0.0, 0.0, 0.0],
            ]
        )
        session = AdmittanceSession(
            client,
            config=AdmittanceSessionConfig(
                target_force_n=2.0,
                stop_after_contact=True,
                loop_hz=10.0,
                force_filter_s=0.0,
                contact_timeout_s=1.0,
                release_timeout_s=1.0,
            ),
            start_pose_mmdeg=[0.0] * 6,
            clock=clock,
            sleep=clock.sleep,
        )
        result = session.run()
        self.assertTrue(result.success)
        self.assertEqual(result.state, AdmittanceSessionState.DONE)
        self.assertTrue(any(event["event"] == "contact_target_reached" for event in result.events))
        self.assertFalse(any(event["event"] == "force_settled" for event in result.events))

    def test_session_contact_stops_at_max_press_limit(self) -> None:
        clock = FakeClock()
        client = FakeClient([])
        session = AdmittanceSession(
            client,
            config=AdmittanceSessionConfig(
                loop_hz=10.0,
                contact_speed_mm_s=1.0,
                max_press_mm=0.2,
                contact_timeout_s=2.0,
            ),
            start_pose_mmdeg=[0.0] * 6,
            clock=clock,
            sleep=clock.sleep,
        )

        result = session.run()

        self.assertFalse(result.success)
        self.assertEqual(result.reason, "max_press_limit_without_force")
        self.assertAlmostEqual(result.contact_travel_mm, 0.2)
        self.assertAlmostEqual(sum(abs(target[2]) for target in client.servo_targets), 0.2)

    def test_lateral_quality_gate_and_hard_limit_fail_closed(self) -> None:
        for fx, expected in ((1.1, "lateral_force_quality_gate"), (2.1, "lateral_force_hard_limit")):
            client = FailingLateralClient([fx, 0.0, -1.0, 0.0, 0.0, 0.0])
            session = AdmittanceSession(
                client,
                config=AdmittanceSessionConfig(
                    hold_s=0.0,
                    force_stable_s=0.0,
                    contact_timeout_s=1.0,
                    settling_timeout_s=1.0,
                ),
                start_pose_mmdeg=[0.0] * 6,
                clock=FakeClock(),
                sleep=lambda _: None,
            )
            result = session.run()
            self.assertFalse(result.success)
            self.assertIn(expected, result.reason)
            self.assertIn("stop_motion", client.calls)

    def test_stop_event_cancels_and_cleanup_is_once(self) -> None:
        client = FakeClient([[0.0, 0.0, -0.2, 0.0, 0.0, 0.0]])
        stop = threading.Event()
        stop.set()
        session = AdmittanceSession(
            client,
            config=AdmittanceSessionConfig(contact_timeout_s=1.0),
            start_pose_mmdeg=[0.0] * 6,
            stop_event=stop,
            clock=FakeClock(),
            sleep=lambda _: None,
        )
        result = session.run()
        self.assertEqual(result.state, AdmittanceSessionState.CANCELLED)
        self.assertEqual(client.calls.count("servo_move_end"), 1)
        self.assertEqual(client.calls.count("ft_activate:False"), 1)

    def test_stop_after_contact_releases_before_cancel_cleanup(self) -> None:
        class StopAfterContactEvent:
            def __init__(self) -> None:
                self.checks = 0

            def is_set(self) -> bool:
                self.checks += 1
                return self.checks >= 2

        stop = StopAfterContactEvent()

        client = FakeClient(
            [
                [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, -0.2, 0.0, 0.0, 0.0],
            ]
        )
        session = AdmittanceSession(
            client,
            config=AdmittanceSessionConfig(
                contact_timeout_s=1.0,
                release_timeout_s=1.0,
                force_filter_s=0.0,
            ),
            start_pose_mmdeg=[0.0] * 6,
            stop_event=stop,
            return_callback=lambda: client.calls.append("return_to_start"),
            clock=FakeClock(),
            sleep=lambda _: None,
        )

        result = session.run()

        self.assertEqual(result.state, AdmittanceSessionState.CANCELLED)
        self.assertEqual(result.reason, "stop_requested_released")
        self.assertIn("return_to_start", client.calls)
        self.assertEqual(client.calls.count("servo_move_end"), 1)
        self.assertEqual(client.calls.count("ft_activate:False"), 1)

    def test_online_node_dry_run_gate_and_single_owner(self) -> None:
        source = (
            Path(__file__).resolve().parents[1]
            / "src"
            / "paus_motion_ros2"
            / "paus_motion_ros2"
            / "fairino_control_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("if self.execute_motion and not self.use_mock_pose:", source)
        self.assertIn('"/force_control/stop"', source)
        self.assertIn("AdmittanceSession(", source)
        self.assertNotIn("FT_Control(", source)
        self.assertNotIn("ImpedanceControlStartStop(", source)
        self.assertNotIn("FT_ComplianceStart(", source)
        self.assertNotIn("FT_ComplianceStop(", source)


if __name__ == "__main__":
    unittest.main()
