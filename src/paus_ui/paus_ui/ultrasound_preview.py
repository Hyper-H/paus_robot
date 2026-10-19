from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import cv2
import numpy as np


class UltrasoundPreview:
    """Independent US display cache; never participates in motion authorization."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._metadata: dict[str, Any] = {}
        self._received_at: float | None = None
        self._receiver_stamp_s: float | None = None
        self._sequence = 0
        self._jpeg_sequence = -1
        self._jpeg: bytes | None = None
        self._receiver: dict[str, Any] = {}
        self._status_at: float | None = None

    def receive_frame(self, message: Any) -> None:
        image = message.image
        if image.encoding not in {"mono8", "mono16"}:
            raise ValueError("ultrasound preview expects mono8 or mono16")
        bpp = 1 if image.encoding == "mono8" else 2
        if image.width <= 0 or image.height <= 0 or image.step < image.width * bpp:
            raise ValueError("invalid ultrasound image dimensions")
        if len(image.data) != image.step * image.height:
            raise ValueError("invalid ultrasound image data length")
        metadata = json.loads(message.metadata_json)
        if not isinstance(metadata, dict):
            raise ValueError("ultrasound metadata must be an object")
        if not message.session_id or metadata.get("session_id") != message.session_id:
            raise ValueError("ultrasound frame session differs from metadata")
        stamp = image.header.stamp
        receiver_stamp = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if not math.isfinite(receiver_stamp) or receiver_stamp <= 0:
            raise ValueError("ultrasound preview requires the Linux receive timestamp")
        dtype = np.uint8 if bpp == 1 else np.dtype(">u2" if image.is_bigendian else "<u2")
        plane = np.ndarray(
            (image.height, image.width), dtype=dtype,
            buffer=bytes(image.data), strides=(image.step, bpp),
        )
        # Fixed 16-to-8-bit display mapping, never per-frame contrast stretching.
        preview = plane.copy() if bpp == 1 else (plane >> 8).astype(np.uint8)
        with self._lock:
            self._frame = preview
            self._metadata = {**metadata, "width": image.width, "height": image.height, "encoding": image.encoding}
            self._received_at = time.monotonic()
            self._receiver_stamp_s = receiver_stamp
            self._sequence += 1

    def receive_status(self, text: str) -> None:
        payload = json.loads(text)
        if not isinstance(payload, dict):
            raise ValueError("ultrasound status must be an object")
        with self._lock:
            self._receiver = payload
            self._status_at = time.monotonic()
            if payload.get("state") != "STREAMING":
                self._frame = None
                self._jpeg = None

    def _snapshot_locked(self, now: float) -> dict[str, Any]:
        status_age = None if self._status_at is None else max(0.0, now - self._status_at)
        image_age = None if self._received_at is None else max(0.0, now - self._received_at)
        stamped_age = None if self._receiver_stamp_s is None else time.time() - self._receiver_stamp_s
        timeout = _positive_number(self._receiver.get("stale_timeout_s")) or 1.0
        receiver_recent = status_age is not None and status_age < max(1.0, timeout)
        state = str(self._receiver.get("state", "WAITING")) if receiver_recent else "WAITING"
        receiver_metadata = self._receiver.get("frame_metadata")
        receiver_metadata = receiver_metadata if isinstance(receiver_metadata, dict) else {}
        session_matches = (
            bool(self._metadata.get("session_id"))
            and self._metadata.get("session_id") == receiver_metadata.get("session_id")
        )
        receiver_age = _nonnegative_number(self._receiver.get("latest_age_s"))
        estimated_receiver_age = None if receiver_age is None or status_age is None else receiver_age + status_age
        ages = (image_age, stamped_age, estimated_receiver_age)
        fresh = all(age is not None and -0.1 <= age < timeout for age in ages)
        if state == "STREAMING" and (not fresh or not session_matches):
            state = "STALE"
        stream_available = state == "STREAMING" and self._frame is not None
        scale = self._metadata.get("display_scale_mm_per_pixel")
        scale = scale if isinstance(scale, dict) else {}
        return {
            "state": state,
            "stream_available": stream_available,
            "timed_out": state == "STALE",
            "latest_age_s": None if stamped_age is None else max(0.0, stamped_age),
            "callback_age_s": image_age,
            "receiver_frame_age_s": estimated_receiver_age,
            "receiver_status_age_s": status_age,
            "session_id": self._metadata.get("session_id"),
            "session_matches": session_matches,
            "received": self._receiver.get("received", 0),
            "acknowledged": self._receiver.get("acknowledged", 0),
            "fps_2s": self._receiver.get("fps_2s", 0.0),
            "last_error": self._receiver.get("last_error", ""),
            "roi_confirmed": self._metadata.get("roi_confirmed") is True,
            "profile_revision": self._metadata.get("profile_revision"),
            "imaging_context": self._metadata.get("imaging_context"),
            "capture_size": self._metadata.get("capture_size"),
            "roi_pixels": self._metadata.get("roi_pixels"),
            "width": self._metadata.get("width"),
            "height": self._metadata.get("height"),
            "encoding": self._metadata.get("encoding"),
            "display_scale_mm_per_pixel": {axis: _positive_number(scale.get(axis)) for axis in ("horizontal", "vertical")},
            "scale_source": "display_ruler" if any(_positive_number(scale.get(axis)) for axis in ("horizontal", "vertical")) else None,
            "geometry_valid": self._metadata.get("geometry_valid") is True,
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_locked(time.monotonic())

    def jpeg(self) -> bytes | None:
        with self._lock:
            if not self._snapshot_locked(time.monotonic())["stream_available"]:
                return None
            if self._jpeg_sequence == self._sequence:
                return self._jpeg
            frame, sequence = self._frame, self._sequence
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return None
        with self._lock:
            if not self._snapshot_locked(time.monotonic())["stream_available"]:
                return None
            if self._sequence != sequence:
                return None
            jpeg = encoded.tobytes()
            self._jpeg, self._jpeg_sequence = jpeg, sequence
            return jpeg


def _positive_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) and value > 0 else None


def _nonnegative_number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value) if math.isfinite(value) and value >= 0 else None
