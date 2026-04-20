from __future__ import annotations

import json
import socket
import time
from typing import Any

from .bridge_protocol import pack_frame_packet, recv_frame_packet


def send_execution_bridge_request(
    host: str,
    port: int,
    request: dict[str, Any],
    timeout_s: float = 5.0,
) -> dict[str, Any]:
    request_payload = dict(request)
    request_payload.setdefault("request_id", f"req-{time.time_ns()}")

    with socket.create_connection((host, int(port)), timeout=float(timeout_s)) as connection:
        connection.sendall(
            pack_frame_packet(
                {
                    "message_type": "exec_request",
                    "timestamp_ns": time.time_ns(),
                },
                json.dumps(request_payload, ensure_ascii=False).encode("utf-8"),
            )
        )
        _, payload = recv_frame_packet(connection)
        return json.loads(payload.decode("utf-8"))
