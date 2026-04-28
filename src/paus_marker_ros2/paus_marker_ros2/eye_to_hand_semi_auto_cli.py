from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from paus_perception import load_config


def _resolve_default_trajectory_path() -> str:
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    default_config = bringup_share / "configs" / "default.yaml"
    config = load_config(default_config)
    return str(config["calibration"].get("trajectory_path", bringup_share / "configs" / "eye_to_hand_trajectory.yaml"))


def _call_trigger(node: Node, service_name: str, timeout_s: float) -> Trigger.Response:
    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError(f"Service is not available: {service_name}")
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise RuntimeError(f"Service timed out: {service_name}")
    response = future.result()
    if response is None:
        raise RuntimeError(f"Service returned no response: {service_name}")
    return response


def _print_response(response: Trigger.Response) -> None:
    status = "OK" if response.success else "FAIL"
    print(f"[{status}] {response.message}")


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
    parser.add_argument("--trajectory-path", default="", help="Trajectory YAML path used by the calibration node.")
    parser.add_argument("--record", action="store_true", help="Enter waypoint recording mode before running.")
    parser.add_argument("--record-only", action="store_true", help="Record/save trajectory and do not run calibration.")
    parser.add_argument("--run-only", action="store_true", help="Do not enter recorder even if the trajectory file is missing.")
    parser.add_argument("--timeout-s", type=float, default=30.0, help="Service wait/call timeout in seconds.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    trajectory_path = Path(args.trajectory_path or _resolve_default_trajectory_path())

    rclpy.init(args=[])
    node = rclpy.create_node("eye_to_hand_semi_auto_cli")
    try:
        should_record = args.record or (not args.run_only and not trajectory_path.exists())
        if should_record:
            recorded = _interactive_record(node, args.timeout_s)
            if not recorded:
                return 1
        if args.record_only:
            return 0
        response = _call_trigger(node, "/eye_to_hand/run_semi_auto_calibration", args.timeout_s)
        _print_response(response)
        return 0 if response.success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
