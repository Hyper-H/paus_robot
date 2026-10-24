from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import json
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

try:
    from paus_interfaces.msg import ThyroidSegmentationResult, UltrasoundFrame
except ImportError:  # The image-only unit tests can run before the interface overlay is built.
    ThyroidSegmentationResult = None
    UltrasoundFrame = None

from .imaging import Mono16Policy, SUPPORTED_ENCODINGS, convert_image, draw_overlay
from .inference import Segmenter, validate_options
from .latest import LatestFrame


IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE)
DEFAULTS = {
    "checkpoint": "", "model_bundle": "", "device": "cpu", "cpu_fallback": False,
    "input_mode": "frame", "frame_topic": "/ultrasound/us/frame",
    "input_topic": "/ultrasound/us/image_raw",
    "mask_topic": "/thyroid/gland/mask", "overlay_topic": "/thyroid/gland/overlay",
    "result_topic": "/thyroid/gland/result", "status_topic": "/thyroid/gland/status",
    "model_version": "tg3k_resnet18_unet_v1", "model_sha256": "",
    "threshold": 0.5, "min_area": 1, "warmup_frames": 5, "cpu_threads": 1,
    "mono16_mode": "reject", "mono16_low": 0.0, "mono16_high": 65535.0,
    "publish_overlay": True, "publish_result": True, "stale_timeout_sec": 2.0,
}


@dataclass(frozen=True)
class FrameContext:
    image: Image
    received: float
    generation: int
    stream_id: str = ""
    session_id: str = ""
    sequence_id: int = -1
    metadata_json: str = ""


def output_images(bridge, source, prediction, bgr, publish_overlay=True):
    mask = bridge.cv2_to_imgmsg(prediction.mask, encoding="mono8")
    mask.header = deepcopy(source.header)
    overlay = None
    if publish_overlay:
        overlay = bridge.cv2_to_imgmsg(draw_overlay(bgr, prediction.mask), encoding="bgr8")
        overlay.header = deepcopy(source.header)
    return mask, overlay


def decode_image(bridge, message, policy):
    if message.encoding not in SUPPORTED_ENCODINGS:
        raise ValueError(f"unsupported encoding {message.encoding!r}; frame discarded")
    channels = 3 if message.encoding in {"bgr8", "rgb8"} else 1
    pixel_bytes = 2 if message.encoding == "mono16" else channels
    if (message.height <= 0 or message.width <= 0
            or message.step < message.width * pixel_bytes
            or len(message.data) != message.height * message.step):
        raise ValueError("invalid Image dimensions, step or payload length")
    compact = message
    row_bytes = message.width * pixel_bytes
    if message.step != row_bytes:
        compact = Image()
        compact.header = message.header
        compact.height, compact.width = message.height, message.width
        compact.encoding, compact.is_bigendian = message.encoding, message.is_bigendian
        compact.step = row_bytes
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        compact.data = rows[:, :row_bytes].copy().tobytes()
    pixels = bridge.imgmsg_to_cv2(compact, desired_encoding="passthrough")
    if pixels.shape[:2] != (message.height, message.width):
        raise ValueError("cv_bridge output shape mismatch")
    return convert_image(pixels, message.encoding, policy)


def _metadata(text):
    if not text:
        return {}
    try:
        value = json.loads(text)
    except (TypeError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


class SegmentationNode(Node):
    def __init__(self, *, engine=None, **kwargs):
        super().__init__("thyroid_segmentation", **kwargs)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.options = {name: self.get_parameter(name).value for name in DEFAULTS}
        p = self.options
        if p["input_mode"] not in {"frame", "image"}:
            raise ValueError("input_mode must be frame or image")
        if p["input_mode"] == "frame" and UltrasoundFrame is None:
            raise RuntimeError("paus_interfaces/msg/UltrasoundFrame is unavailable; build the interface overlay")
        if p["publish_result"] and ThyroidSegmentationResult is None:
            raise RuntimeError("paus_interfaces/msg/ThyroidSegmentationResult is unavailable; build the interface overlay")
        validate_options(p["threshold"], p["min_area"])
        self.policy = Mono16Policy(p["mono16_mode"], p["mono16_low"], p["mono16_high"])
        if not np.isfinite(p["stale_timeout_sec"]) or p["stale_timeout_sec"] <= 0:
            raise ValueError("stale_timeout_sec must be finite and positive")
        topics = [p[key] for key in ("mask_topic", "overlay_topic", "status_topic")]
        if p["publish_result"]:
            topics.append(p["result_topic"])
        topics.append(p["frame_topic"] if p["input_mode"] == "frame" else p["input_topic"])
        resolved_topics = [self.resolve_topic_name(topic) for topic in topics]
        if any(not topic for topic in topics) or len(set(resolved_topics)) != len(resolved_topics):
            raise ValueError("input/output topics must be nonempty and distinct after remapping")
        self.engine = engine if engine is not None else Segmenter(
            p["checkpoint"], p["model_bundle"], device=p["device"],
            cpu_fallback=p["cpu_fallback"], threshold=p["threshold"],
            min_area=p["min_area"], warmup_frames=p["warmup_frames"],
            cpu_threads=p["cpu_threads"], log=self.get_logger().warning)
        self.bridge = CvBridge()
        self.mask_publisher = self.create_publisher(Image, p["mask_topic"], IMAGE_QOS)
        self.overlay_publisher = self.create_publisher(Image, p["overlay_topic"], IMAGE_QOS)
        self.result_publisher = (self.create_publisher(ThyroidSegmentationResult, p["result_topic"], IMAGE_QOS)
                                 if p["publish_result"] else None)
        self.status_publisher = self.create_publisher(DiagnosticArray, p["status_topic"], STATUS_QOS)
        self.mailbox = LatestFrame()
        self._state_lock = threading.RLock()
        self._processed = self._errors = self._out_of_order = self._dropped_generation = 0
        self._started = None
        self._last_receive = None
        self._last_completion = time.perf_counter()
        self._last_header = None
        self._last_source = None
        self._last_bgr = None
        self._outputs_published = False
        self._stream_key = None
        self._last_sequence = None
        self._generation = 0
        self._latencies = deque(maxlen=256)
        self._worker = threading.Thread(target=self._run, name="thyroid-inference", daemon=False)
        if p["input_mode"] == "frame":
            self.create_subscription(UltrasoundFrame, p["frame_topic"], self._receive_frame, IMAGE_QOS)
        else:
            self.create_subscription(Image, p["input_topic"], self._receive_image, IMAGE_QOS)
        self.create_timer(1.0, self._watchdog)
        self._worker.start()
        self.get_logger().warning(
            "Research-only visible thyroid-region segmentation; not nodules or clinical assurance. "
            "No robot control. Parameters are startup-only.")

    def _identity(self, image, stream_id, session_id, sequence_id, metadata_json):
        metadata = _metadata(metadata_json)
        def frozen(value):
            if isinstance(value, list):
                return tuple(frozen(item) for item in value)
            if isinstance(value, dict):
                return tuple(sorted((str(k), frozen(v)) for k, v in value.items()))
            return value
        return (str(stream_id), str(session_id), frozen(metadata.get("roi_pixels")),
                frozen(metadata.get("capture_size")), metadata.get("profile_revision"),
                image.width, image.height)

    def _accept(self, image, received, stream_id, session_id, sequence_id, metadata_json):
        with self._state_lock:
            key = self._identity(image, stream_id, session_id, sequence_id, metadata_json)
            if self._stream_key != key:
                if self._outputs_published:
                    self._clear_outputs_locked("source identity changed")
                self._stream_key = key
                self._generation += 1
                self._last_sequence = None
                self._outputs_published = False
            if (sequence_id > 0 and self._last_sequence is not None
                    and sequence_id <= self._last_sequence):
                self._out_of_order += 1
                return None
            if sequence_id > 0:
                self._last_sequence = sequence_id
            if self._started is None:
                self._started = received
            self._last_receive = received
            return FrameContext(image, received, self._generation, str(stream_id),
                                str(session_id), int(sequence_id), str(metadata_json))

    def _receive_frame(self, message):
        context = self._accept(message.image, time.perf_counter(), message.stream_id,
                                message.session_id, int(message.sequence_id), message.metadata_json)
        if context is not None:
            self.mailbox.put(context)

    def _receive_image(self, message):
        context = self._accept(message, time.perf_counter(), "image_topic", "", -1, "")
        if context is not None:
            self.mailbox.put(context)

    def _run(self):
        while True:
            context = self.mailbox.take()
            if context is None:
                return
            try:
                start = time.perf_counter()
                gray, bgr = decode_image(self.bridge, context.image, self.policy)
                converted = time.perf_counter()
                prediction = self.engine.predict(gray)
                post_start = time.perf_counter()
                mask, overlay = output_images(
                    self.bridge, context.image, prediction, bgr, self.options["publish_overlay"])
                finished = time.perf_counter()
                e2e = (finished - context.received) * 1000
                with self._state_lock:
                    if context.generation != self._generation:
                        self._dropped_generation += 1
                        continue
                    self.mask_publisher.publish(mask)
                    if overlay is not None:
                        self.overlay_publisher.publish(overlay)
                    self._publish_result_locked(context, mask, overlay, prediction, cleared=False)
                    self._processed += 1
                    self._last_completion = finished
                    self._last_header = deepcopy(context.image.header)
                    self._last_source = context
                    self._last_bgr = bgr
                    self._outputs_published = True
                    self._latencies.append(e2e)
                    metrics = {
                        "valid": prediction.valid, "mask_area_px": prediction.area,
                        "mask_area_fraction": prediction.area / (context.image.width * context.image.height),
                        "confidence": prediction.confidence, "confidence_calibrated": False,
                        "width": context.image.width, "height": context.image.height,
                        "input_encoding": context.image.encoding, "gray_min": int(gray.min()),
                        "gray_max": int(gray.max()), "queue_ms": (start - context.received) * 1000,
                        "preprocess_ms": prediction.preprocess_ms + (converted - start) * 1000,
                        "model_ms": prediction.model_ms,
                        "postprocess_ms": prediction.postprocess_ms + (finished - post_start) * 1000,
                        "end_to_end_ms": e2e, "end_to_end_mean_ms": float(np.mean(self._latencies)),
                        "end_to_end_p95_ms": float(np.percentile(self._latencies, 95)),
                        "processing_fps": self._processed / max(finished - self._started, 1e-9),
                        "stream_id": context.stream_id, "session_id": context.session_id,
                        "sequence_id": context.sequence_id, "generation": self._generation,
                        **self.engine.memory_metrics(),
                    }
                    self._status(context.image.header, metrics,
                                 DiagnosticStatus.OK if prediction.valid else DiagnosticStatus.WARN,
                                 "ok" if prediction.valid else "empty mask or area below min_area")
            except Exception as exc:
                with self._state_lock:
                    self._errors += 1
                    self._status(context.image.header, {"valid": False,
                                                         "stream_id": context.stream_id,
                                                         "session_id": context.session_id,
                                                         "sequence_id": context.sequence_id},
                                 DiagnosticStatus.ERROR, str(exc))
                self.get_logger().warning(f"Frame discarded: {exc}", throttle_duration_sec=2.0)

    def _publish_result_locked(self, context, mask, overlay, prediction, cleared):
        if self.result_publisher is None:
            return
        result = ThyroidSegmentationResult()
        result.header = deepcopy(context.image.header)
        result.stream_id = context.stream_id
        result.session_id = context.session_id
        result.sequence_id = max(0, context.sequence_id)
        result.metadata_json = context.metadata_json
        result.mask = mask
        if overlay is not None:
            result.overlay = overlay
        result.model_version = str(self.options["model_version"])
        result.model_sha256 = str(self.options["model_sha256"])
        result.threshold = float(self.options["threshold"])
        result.valid = bool(prediction.valid)
        result.cleared = bool(cleared)
        result.mask_area_px = int(prediction.area)
        result.mask_area_fraction = float(prediction.area / max(context.image.width * context.image.height, 1))
        result.confidence = float(prediction.confidence)
        self.result_publisher.publish(result)

    def _clear_outputs_locked(self, reason):
        if not self._outputs_published or self._last_source is None or self._last_bgr is None:
            return
        blank = np.zeros((self._last_source.image.height, self._last_source.image.width), np.uint8)
        prediction = type("ClearedPrediction", (), {"mask": blank, "area": 0,
            "valid": False, "confidence": 0.0})()
        mask, overlay = output_images(self.bridge, self._last_source.image, prediction,
                                      self._last_bgr, self.options["publish_overlay"])
        self.mask_publisher.publish(mask)
        if overlay is not None:
            self.overlay_publisher.publish(overlay)
        self._publish_result_locked(self._last_source, mask, overlay, prediction, cleared=True)
        self._outputs_published = False
        self._status(self._last_source.image.header, {"valid": False, "cleared": True,
                                                       "clear_reason": reason},
                     DiagnosticStatus.STALE, reason)

    def _status(self, header, metrics, level, text):
        stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec if header else 0
        age_ms = (self.get_clock().now().nanoseconds - stamp_ns) / 1e6 if stamp_ns else -1.0
        values = {
            **self.mailbox.counts(), "processed": self._processed, "errors": self._errors,
            "out_of_order": self._out_of_order, "dropped_generation": self._dropped_generation,
            "device": str(self.engine.device), "source_age_ms": age_ms if age_ms >= 0 else -1.0,
            "source_age_valid": bool(stamp_ns and age_ms >= 0), "valid": False,
            "mask_area_px": -1, "mask_area_fraction": -1.0, "confidence": -1.0,
            "confidence_calibrated": False, "queue_ms": -1.0, "preprocess_ms": -1.0,
            "model_ms": -1.0, "postprocess_ms": -1.0, "end_to_end_ms": -1.0,
            **metrics,
        }
        status = DiagnosticStatus(
            level=level, name="thyroid_gland_segmentation", hardware_id="research_only",
            message=text,
            values=[KeyValue(key=k, value=str(v).lower() if isinstance(v, bool) else str(v))
                    for k, v in values.items()])
        array = DiagnosticArray(status=[status])
        if header is not None:
            array.header = deepcopy(header)
        self.status_publisher.publish(array)

    def _watchdog(self):
        with self._state_lock:
            now = time.perf_counter()
            if self._last_receive is None:
                return
            if now - self._last_receive > self.options["stale_timeout_sec"]:
                self._clear_outputs_locked("input stream stale")
                self._status(self._last_header, {"valid": False}, DiagnosticStatus.STALE,
                             "input stream stale")
            elif now - self._last_completion > self.options["stale_timeout_sec"]:
                self._clear_outputs_locked("no completed inference within stale_timeout_sec")
                self._status(self._last_header, {"valid": False}, DiagnosticStatus.STALE,
                             "no completed inference within stale_timeout_sec")

    def destroy_node(self):
        if hasattr(self, "mailbox"):
            self.mailbox.close()
            self._worker.join()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SegmentationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
