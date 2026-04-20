from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ag_repro import send_execution_bridge_request


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a test command to the Windows FAIRINO execution bridge.")
    parser.add_argument("--host", default="127.0.0.1", help="Bridge host.")
    parser.add_argument("--port", type=int, default=5002, help="Bridge port.")
    parser.add_argument(
        "--command",
        required=True,
        choices=("ping", "set_speed", "move_j", "move_l"),
        help="Command to send.",
    )
    parser.add_argument("--speed", type=float, default=5.0, help="Speed value for set_speed.")
    parser.add_argument("--tool-id", type=int, default=0, help="Tool id for move commands.")
    parser.add_argument("--user-id", type=int, default=0, help="User id for move commands.")
    parser.add_argument("--vel", type=float, default=5.0, help="Velocity for move commands.")
    parser.add_argument("--joint-pos", nargs=6, type=float, help="Joint pose for move_j.")
    parser.add_argument("--pose-mmdeg", nargs=6, type=float, help="Cartesian pose for move_l.")
    parser.add_argument("--timeout", type=float, default=5.0, help="Request timeout in seconds.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    request: dict[str, object] = {"command": args.command}

    if args.command == "set_speed":
        request["speed"] = float(args.speed)
    elif args.command == "move_j":
        if args.joint_pos is None:
            raise SystemExit("--joint-pos is required for move_j")
        request["joint_pos"] = [float(value) for value in args.joint_pos]
        request["tool_id"] = int(args.tool_id)
        request["user_id"] = int(args.user_id)
        request["vel"] = float(args.vel)
    elif args.command == "move_l":
        if args.pose_mmdeg is None:
            raise SystemExit("--pose-mmdeg is required for move_l")
        request["pose_mmdeg"] = [float(value) for value in args.pose_mmdeg]
        request["tool_id"] = int(args.tool_id)
        request["user_id"] = int(args.user_id)
        request["vel"] = float(args.vel)

    response = send_execution_bridge_request(
        host=args.host,
        port=args.port,
        request=request,
        timeout_s=float(args.timeout),
    )
    print(json.dumps(response, ensure_ascii=False))
    return 0 if bool(response.get("ok")) else 1


if __name__ == "__main__":
    raise SystemExit(main())
