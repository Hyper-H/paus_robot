from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from paus_perception import load_config, resolve_config_artifact_path


STATUS_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _resolve_default_config_path() -> Path:
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    workspace_root = bringup_share.parents[3]
    source_config = workspace_root / "src" / "paus_bringup" / "configs" / "default.yaml"
    return source_config if source_config.exists() else bringup_share / "configs" / "default.yaml"


def _resolve_default_trajectory_path() -> str:
    default_config = _resolve_default_config_path()
    config = load_config(default_config)
    return str(config["calibration"].get("trajectory_path", default_config.parent / "eye_to_hand_trajectory.yaml"))


def _format_status(payload: dict[str, object]) -> str:
    status = str(payload.get("status", "status"))
    message = str(payload.get("message", ""))
    prefix_parts = [status]
    if "waypoint_index" in payload and "waypoint_count" in payload:
        prefix_parts.append(f"{payload['waypoint_index']}/{payload['waypoint_count']}")
    if "sample_count" in payload:
        prefix_parts.append(f"samples={payload['sample_count']}")
    if "captured_count" in payload or "skipped_count" in payload:
        prefix_parts.append(f"accepted={payload.get('captured_count', '-')}")
        prefix_parts.append(f"skipped={payload.get('skipped_count', '-')}")
    if "reprojection_error_px" in payload:
        prefix_parts.append(f"reproj={float(payload['reprojection_error_px']):.3f}px")
    if "board_margin_px" in payload:
        prefix_parts.append(f"margin={float(payload['board_margin_px']):.1f}px")
    if status == "solved" and isinstance(payload.get("residuals"), dict):
        residuals = payload["residuals"]
        prefix_parts.append(f"rms={float(residuals.get('translation_rms_mm', 0.0)):.2f}mm")
        prefix_parts.append(f"rot={float(residuals.get('rotation_rms_deg', 0.0)):.2f}deg")
    return f"[STATUS] {' '.join(prefix_parts)} | {message}"


def _print_status_message(message: String) -> None:
    try:
        payload = json.loads(message.data)
    except json.JSONDecodeError:
        print(f"[STATUS] {message.data}", flush=True)
        return
    if isinstance(payload, dict):
        print(_format_status(payload), flush=True)
    else:
        print(f"[STATUS] {message.data}", flush=True)


def _call_trigger(
    node: Node,
    service_name: str,
    timeout_s: float,
    *,
    echo_status: bool = False,
    status_topic: str = "/eye_to_hand/status",
) -> Trigger.Response:
    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service(timeout_sec=min(timeout_s, 30.0)):
        raise RuntimeError(f"Service is not available: {service_name}")
    subscription = None
    if echo_status:
        subscription = node.create_subscription(String, status_topic, _print_status_message, STATUS_QOS)
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + timeout_s
    try:
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Service timed out: {service_name}")
            rclpy.spin_once(node, timeout_sec=0.2)
        response = future.result()
        if response is None:
            raise RuntimeError(f"Service returned no response: {service_name}")
        return response
    finally:
        if subscription is not None:
            node.destroy_subscription(subscription)


def _print_response(response: Trigger.Response) -> None:
    status = "OK" if response.success else "FAIL"
    print(f"[{status}] {response.message}")


def _wait_for_backend_status(node: Node, timeout_s: float, status_topic: str) -> dict[str, object] | None:
    latest: dict[str, object] | None = None

    def _capture_status(message: String) -> None:
        nonlocal latest
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            latest = payload

    subscription = node.create_subscription(String, status_topic, _capture_status, STATUS_QOS)
    deadline = time.monotonic() + max(0.0, timeout_s)
    try:
        while rclpy.ok() and latest is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return latest
    finally:
        node.destroy_subscription(subscription)


def _interactive_record(node: Node, timeout_s: float) -> bool:
    print("Trajectory YAML is missing or record mode was requested.")
    print("Move the robot to a safe calibration pose, then press:")
    print("  r = record current waypoint")
    print("  d = delete last waypoint")
    print("  f = finish recording")
    print("  q = quit without running")
    while True:
        command = input("eye-to-hand> ").strip().lower()
        if command == "r":
            _print_response(_call_trigger(node, "/eye_to_hand/record_waypoint", timeout_s))
        elif command == "d":
            _print_response(_call_trigger(node, "/eye_to_hand/delete_last_waypoint", timeout_s))
        elif command == "f":
            response = _call_trigger(node, "/eye_to_hand/save_trajectory", timeout_s)
            _print_response(response)
            return response.success
        elif command == "q":
            print("Recording cancelled.")
            return False
        else:
            print("Unknown command. Use r, d, f, or q.")


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive helper for semi-automatic eye-to-hand calibration.")
    parser.add_argument("--trajectory-path", default="", help="Require the running calibration node to use this trajectory YAML path.")
    parser.add_argument("--record", action="store_true", help="Enter waypoint recording mode before running.")
    parser.add_argument("--record-only", action="store_true", help="Record/save trajectory and do not run calibration.")
    parser.add_argument("--run-only", action="store_true", help="Do not enter recorder even if the trajectory file is missing.")
    parser.add_argument("--timeout-s", type=float, default=3600.0, help="Service call timeout in seconds.")
    parser.add_argument("--status-topic", default="/eye_to_hand/status", help="Status topic used by the running calibration node.")
    parser.add_argument("--quiet-status", action="store_true", help="Do not print status topic messages while running calibration.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    default_config = _resolve_default_config_path()
    if args.trajectory_path:
        trajectory_path = Path(resolve_config_artifact_path(args.trajectory_path, default_config))
    else:
        trajectory_path = Path(_resolve_default_trajectory_path())

    rclpy.init(args=[])
    node = rclpy.create_node("eye_to_hand_semi_auto_cli")
    try:
        backend_status = _wait_for_backend_status(node, min(args.timeout_s, 10.0), args.status_topic)
        backend_trajectory_value = backend_status.get("trajectory_path") if isinstance(backend_status, dict) else None
        if backend_trajectory_value:
            backend_trajectory_path = Path(str(backend_trajectory_value)).expanduser().resolve()
            if args.trajectory_path and backend_trajectory_path != trajectory_path.expanduser().resolve():
                print(
                    (
                        "--trajectory-path does not match the running calibration node. "
                        f"requested={trajectory_path} backend={backend_trajectory_path}. "
                        "Restart the calibration node with trajectory_path:=... before using this CLI option."
                    ),
                    file=sys.stderr,
                )
                return 2
            trajectory_path = backend_trajectory_path
        elif args.trajectory_path:
            print(
                f"Cannot verify --trajectory-path because no {args.status_topic} message with trajectory_path was received.",
                file=sys.stderr,
            )
            return 2
        should_record = args.record or args.record_only or (not args.run_only and not trajectory_path.exists())
        if should_record:
            recorded = _interactive_record(node, args.timeout_s)
            if not recorded:
                return 1
        if args.record_only:
            return 0
        response = _call_trigger(
            node,
            "/eye_to_hand/run_semi_auto_calibration",
            args.timeout_s,
            echo_status=not args.quiet_status,
            status_topic=args.status_topic,
        )
        _print_response(response)
        return 0 if response.success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
