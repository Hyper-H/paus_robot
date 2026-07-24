from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
import signal
import subprocess
import sys
import time
from typing import Iterable

PAUS_RUNTIME_PATTERNS = (
    "ros2 launch paus_bringup",
    "camera_bridge.py",
    "image_receiver_node",
    "marker_pose_node",
    "neck_surface_pose_node",
    "neck_target_eval_node",
    "target_transform_node",
    "fairino_control_node",
    "paus_ui_server",
    "eye_to_hand_calibration_node",
)

EXCLUDE_PATTERNS = (
    "stop_paus_runtime",
    "grep -E",
)


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    ppid: int
    stat: str
    pcpu: float
    pmem: float
    command: str


def is_paus_runtime_command(command: str) -> bool:
    if any(pattern in command for pattern in EXCLUDE_PATTERNS):
        return False
    return any(pattern in command for pattern in PAUS_RUNTIME_PATTERNS)


def parse_ps_line(line: str) -> ProcessInfo | None:
    parts = line.strip().split(None, 5)
    if len(parts) < 6:
        return None
    try:
        return ProcessInfo(
            pid=int(parts[0]),
            ppid=int(parts[1]),
            stat=parts[2],
            pcpu=float(parts[3]),
            pmem=float(parts[4]),
            command=parts[5],
        )
    except ValueError:
        return None


def collect_processes() -> list[ProcessInfo]:
    output = subprocess.check_output(
        ["ps", "-eo", "pid=,ppid=,stat=,pcpu=,pmem=,args="],
        text=True,
    )
    current_pid = os.getpid()
    parent_pid = os.getppid()
    processes: list[ProcessInfo] = []
    for line in output.splitlines():
        process = parse_ps_line(line)
        if process is None:
            continue
        if process.pid in {current_pid, parent_pid}:
            continue
        processes.append(process)
    return processes


def find_candidates(processes: Iterable[ProcessInfo]) -> list[ProcessInfo]:
    return [process for process in processes if is_paus_runtime_command(process.command)]


def _pid_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _signal_candidates(candidates: list[ProcessInfo], sig: signal.Signals) -> None:
    for process in candidates:
        try:
            os.kill(process.pid, sig)
            print(f"sent {sig.name} pid={process.pid} cmd={process.command}")
        except ProcessLookupError:
            continue
        except PermissionError as exc:
            print(f"warning: permission denied pid={process.pid}: {exc}", file=sys.stderr)


def _wait_for_exit(candidates: list[ProcessInfo], timeout_s: float) -> list[ProcessInfo]:
    deadline = time.monotonic() + timeout_s
    remaining = candidates
    while time.monotonic() < deadline:
        remaining = [process for process in remaining if _pid_exists(process.pid)]
        if not remaining:
            return []
        time.sleep(0.1)
    return [process for process in remaining if _pid_exists(process.pid)]


def format_process(process: ProcessInfo) -> str:
    return f"pid={process.pid} ppid={process.ppid} stat={process.stat} cpu={process.pcpu:.1f} mem={process.pmem:.1f} cmd={process.command}"


def print_candidates(label: str, candidates: list[ProcessInfo]) -> None:
    print(f"{label}: {len(candidates)} PAUS runtime process(es)")
    for process in candidates:
        print(format_process(process))


def stop_candidates(candidates: list[ProcessInfo], *, grace_s: float, force: bool) -> list[ProcessInfo]:
    # Children first reduces orphaned camera bridges when a launch parent is also present.
    ordered = sorted(candidates, key=lambda process: process.ppid, reverse=True)
    _signal_candidates(ordered, signal.SIGINT)
    remaining = _wait_for_exit(ordered, grace_s)
    if remaining:
        _signal_candidates(remaining, signal.SIGTERM)
        remaining = _wait_for_exit(remaining, grace_s)
    if remaining and force:
        _signal_candidates(remaining, signal.SIGKILL)
        remaining = _wait_for_exit(remaining, 1.0)
    return remaining


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="List or stop PAUS ROS runtime processes.")
    parser.add_argument("--kill", action="store_true", help="Send signals to matching PAUS runtime processes. Without this flag, only list candidates.")
    parser.add_argument("--force", action="store_true", help="Send SIGKILL after SIGINT/SIGTERM if processes remain.")
    parser.add_argument("--grace-s", type=float, default=3.0, help="Seconds to wait after SIGINT and SIGTERM.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    candidates = find_candidates(collect_processes())
    print_candidates("before", candidates)
    if not args.kill:
        print("dry-run only; re-run with --kill to stop these processes.")
        return 0
    remaining = stop_candidates(candidates, grace_s=float(args.grace_s), force=bool(args.force))
    print_candidates("after", find_candidates(collect_processes()))
    if remaining:
        print("warning: some processes are still alive; use --force if you want SIGKILL.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
