from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from . import protocol_v2


@dataclass(frozen=True)
class NormalizedFrame:
    header: dict
    pixels: bytes
    received_ns: int
    received_monotonic_s: float


def normalize_packet(packet: protocol_v2.Packet) -> NormalizedFrame:
    header = dict(packet.header)
    if header["type"] != "frame":
        raise protocol_v2.ProtocolError("expected a frame, not an ACK")
    protocol_v2.validate_header(header, len(packet.payload))
    for field in ("width", "height", "sequence_id", "sender_dropped"):
        header[field] = protocol_v2.integer(header[field], field)

    dtype = np.uint8 if header["encoding"] == "mono8" else np.dtype("<u2")
    height, width = header["height"], header["width"]
    order = "F" if header["storage_order"] == "column_major" else "C"
    plane = np.frombuffer(packet.payload, dtype=dtype).reshape((height, width), order=order)
    return NormalizedFrame(
        header=header,
        pixels=np.ascontiguousarray(plane).tobytes(),
        received_ns=time.time_ns(),
        received_monotonic_s=time.monotonic(),
    )
