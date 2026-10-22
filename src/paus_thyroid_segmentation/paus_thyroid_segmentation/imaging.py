"""ROS-independent image operations.

The OpenCV letterbox helpers specify one explicit convention. Production uses
the training adapter; reuse these helpers only after upstream parity is proven.
"""

from dataclasses import dataclass

import cv2
import numpy as np


SUPPORTED_ENCODINGS = {"mono8", "mono16", "bgr8", "rgb8"}


@dataclass(frozen=True)
class Mono16Policy:
    mode: str = "reject"
    low: float = 0.0
    high: float = 65535.0

    def __post_init__(self):
        if self.mode not in {"reject", "fixed_window"}:
            raise ValueError("mono16_mode must be reject or fixed_window")
        if not np.isfinite([self.low, self.high]).all() or not 0 <= self.low < self.high <= 65535:
            raise ValueError("mono16 window must satisfy 0 <= low < high <= 65535")


def convert_image(pixels, encoding, policy=Mono16Policy()):
    """Return gray uint8 and BGR display; never normalize by per-frame extrema."""
    if encoding not in SUPPORTED_ENCODINGS:
        raise ValueError(f"unsupported encoding {encoding!r}")
    pixels = np.asarray(pixels)
    if pixels.size == 0:
        raise ValueError("empty image")
    if encoding in {"mono8", "mono16"}:
        expected = np.uint8 if encoding == "mono8" else np.uint16
        if pixels.ndim != 2 or pixels.dtype != expected:
            raise ValueError(f"{encoding} requires a 2D {expected} array")
        if encoding == "mono16":
            if policy.mode == "reject":
                raise ValueError("mono16 rejected: confirm driver range and set a fixed_window")
            scaled = (pixels.astype(np.float32) - policy.low) * (255.0 / (policy.high - policy.low))
            gray = np.floor(np.clip(scaled, 0, 255) + 0.5).astype(np.uint8)
        else:
            gray = pixels.copy()
        return gray, cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if pixels.dtype != np.uint8 or pixels.ndim != 3 or pixels.shape[2] != 3:
        raise ValueError(f"{encoding} requires an HxWx3 uint8 array")
    bgr = pixels.copy() if encoding == "bgr8" else cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), bgr


@dataclass(frozen=True)
class Letterbox:
    height: int
    width: int
    resized_height: int
    resized_width: int
    top: int
    left: int

    def crop(self, image):
        return image[self.top:self.top + self.resized_height,
                     self.left:self.left + self.resized_width]


def letterbox(gray):
    if gray.dtype != np.uint8 or gray.ndim != 2 or gray.size == 0:
        raise ValueError("letterbox requires nonempty 2D uint8 grayscale")
    height, width = gray.shape
    scale = min(224 / height, 224 / width)
    rh, rw = max(1, round(height * scale)), max(1, round(width * scale))
    geometry = Letterbox(height, width, rh, rw, (224 - rh) // 2, (224 - rw) // 2)
    canvas = np.zeros((224, 224), dtype=np.uint8)
    canvas[geometry.top:geometry.top + rh, geometry.left:geometry.left + rw] = cv2.resize(
        gray, (rw, rh), interpolation=cv2.INTER_LINEAR)
    return canvas.astype(np.float32)[None, None] / 255.0, geometry


def restore_mask(binary, geometry):
    if binary.shape != (224, 224) or binary.dtype != np.uint8:
        raise ValueError("restore requires a 224x224 uint8 binary mask")
    return cv2.resize(geometry.crop(binary), (geometry.width, geometry.height),
                      interpolation=cv2.INTER_NEAREST)


def draw_overlay(bgr, mask):
    overlay = bgr.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, (0, 255, 0), 1)
    return overlay
