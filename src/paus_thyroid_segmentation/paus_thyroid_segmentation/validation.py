"""Bounded validation commands. Never starts robot nodes."""

import argparse
import json
from pathlib import Path
import time
import uuid

import cv2
import numpy as np


def compare(args):
    from .inference import Segmenter

    engine = Segmenter(args.checkpoint, args.model_bundle, device=args.device,
                       threshold=args.threshold)
    with np.load(args.reference, allow_pickle=False) as reference:
        gray, expected = reference["gray"], reference["mask"]
        batch, _ = engine.adapter.prepare_image(gray)
        if "input" not in reference:
            raise ValueError("reference must contain gray, input, mask exported by training infer.py")
        np.testing.assert_array_equal(batch, reference["input"])
        actual = engine.predict(gray).mask
        np.testing.assert_array_equal(actual, expected)
    print(json.dumps({"input_equal": True, "mask_equal": True, "pixels": int(actual.size)}))


def compare_ros(args):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    from .node import IMAGE_QOS

    with np.load(args.reference, allow_pickle=False) as reference:
        gray, expected = reference["gray"], reference["mask"]
    if gray.dtype != np.uint8 or gray.ndim != 2 or not gray.size:
        raise ValueError("reference gray must be HxW uint8")
    if expected.dtype != np.uint8 or expected.shape != gray.shape or not np.isin(expected, [0, 255]).all():
        raise ValueError("reference mask must be same-size uint8 0/255")
    rclpy.init()
    node = Node("thyroid_reference_comparison")
    bridge = CvBridge()
    image = bridge.cv2_to_imgmsg(gray, encoding="mono8")
    image.header.frame_id = "reference_" + uuid.uuid4().hex
    publisher = node.create_publisher(Image, args.input_topic, IMAGE_QOS)
    received = []

    def on_mask(message):
        if message.header.frame_id == image.header.frame_id:
            received.append(message)

    node.create_subscription(Image, args.mask_topic, on_mask, IMAGE_QOS)
    try:
        deadline = time.perf_counter() + args.timeout
        while publisher.get_subscription_count() == 0 and time.perf_counter() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("no segmentation subscriber discovered")
        image.header.stamp = node.get_clock().now().to_msg()
        # Retry the same fixed reference to tolerate initial best-effort discovery loss.
        next_publish = 0.0
        while not received and time.perf_counter() < deadline:
            if time.perf_counter() >= next_publish:
                publisher.publish(image)
                next_publish = time.perf_counter() + 0.5
            rclpy.spin_once(node, timeout_sec=0.05)
        if not received:
            raise RuntimeError("no matching ROS mask received")
        actual = received[0]
        if actual.header != image.header or actual.encoding != "mono8":
            raise AssertionError("ROS header or mask encoding differs from the input contract")
        if (actual.height, actual.width) != gray.shape:
            raise AssertionError("ROS mask dimensions differ from the input")
        np.testing.assert_array_equal(bridge.imgmsg_to_cv2(actual, "passthrough"), expected)
        print(json.dumps({"ros_mask_equal": True, "header_equal": True, "pixels": int(gray.size)}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def inspect_bag(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from cv_bridge import CvBridge

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""))
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if args.topic not in topics:
        raise ValueError(f"topic not in bag: {args.topic}; topics={topics}")
    message_type = get_message(topics[args.topic])
    bridge = CvBridge()
    count, shapes, first, last = 0, set(), None, None
    minimum, maximum = 65535, 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic != args.topic:
            continue
        message = deserialize_message(data, message_type)
        image = message.image if hasattr(message, "image") else message
        pixels = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        minimum, maximum = min(minimum, int(pixels.min())), max(maximum, int(pixels.max()))
        shapes.add((image.width, image.height, image.encoding, image.header.frame_id))
        count += 1
        first = stamp if first is None else first
        last = stamp
    print(json.dumps({
        "topic": args.topic, "type": topics[args.topic], "count": count,
        "shapes_width_height_encoding_frame": sorted(shapes),
        "range": [minimum, maximum] if count else None,
        "first_to_last_fps": (count - 1) * 1e9 / (last - first) if count > 1 and last > first else None,
    }, indent=2))


def synthetic_pixels(width, height, encoding):
    gray = np.zeros((height, width), np.uint8)
    cv2.ellipse(gray, (width // 2, height // 2),
                (max(1, width // 4), max(1, height // 4)), 0, 0, 360, 180, -1)
    if encoding == "mono16":
        return gray.astype(np.uint16) * 257
    if encoding in {"bgr8", "rgb8"}:
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return gray


def ros_validation(args):
    import rclpy
    from cv_bridge import CvBridge
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    from .node import IMAGE_QOS, STATUS_QOS

    rclpy.init()
    node = Node("thyroid_validation")
    observations = {"input": 0, "mask": 0}
    delays = {key: [] for key in ("preprocess_ms", "model_ms", "postprocess_ms", "end_to_end_ms")}
    arrivals = []
    latest = {}
    seen = set()

    def on_input(message):
        observations["input"] += 1

    def on_mask(message):
        observations["mask"] += 1
        arrivals.append(time.perf_counter())

    def on_status(message):
        for status in message.status:
            values = {pair.key: pair.value for pair in status.values}
            latest.clear()
            latest.update(values)
            key = values.get("processed", "0")
            if key not in seen and float(values.get("end_to_end_ms", -1)) >= 0:
                seen.add(key)
                for name in delays:
                    delays[name].append(float(values[name]))

    node.create_subscription(Image, args.input_topic, on_input, IMAGE_QOS)
    node.create_subscription(Image, args.mask_topic, on_mask, IMAGE_QOS)
    node.create_subscription(DiagnosticArray, args.status_topic, on_status, STATUS_QOS)
    if args.command == "synthetic":
        publisher = node.create_publisher(Image, args.input_topic, IMAGE_QOS)
        bridge = CvBridge()
        pixels = synthetic_pixels(args.width, args.height, args.encoding)

        def publish():
            image = bridge.cv2_to_imgmsg(pixels, encoding=args.encoding)
            image.header.stamp = node.get_clock().now().to_msg()
            image.header.frame_id = "synthetic_thyroid_test"
            publisher.publish(image)

        node.create_timer(1.0 / args.fps, publish)
    try:
        stop = time.perf_counter() + args.seconds
        while rclpy.ok() and time.perf_counter() < stop:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result = {
        "kind": "synthetic_transport_check" if args.command == "synthetic" else "observed_ros_stream",
        "observations": observations,
        "observed_mask_fps": (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else 0,
        "latencies": {name: {"samples": len(values), "mean_ms": float(np.mean(values)),
                              "p95_ms": float(np.percentile(values, 95))}
                      for name, values in delays.items() if values},
        "latest_status": latest,
        "note": "Observer counts include DDS loss; overwritten counts only node pending-slot replacements.",
    }
    print(json.dumps(result, indent=2))
    if not arrivals or not any(delays.values()):
        raise SystemExit("No completed inference observed; this is NOT a successful model validation")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    parity = sub.add_parser("compare")
    parity.add_argument("--checkpoint", required=True)
    parity.add_argument("--model-bundle", required=True)
    parity.add_argument("--reference", type=Path, required=True)
    parity.add_argument("--device", default="cpu")
    parity.add_argument("--threshold", type=float, default=0.5)
    parity_ros = sub.add_parser("compare-ros")
    parity_ros.add_argument("--reference", type=Path, required=True)
    parity_ros.add_argument("--input-topic", default="/ultrasound/us/image_raw")
    parity_ros.add_argument("--mask-topic", default="/thyroid/gland/mask")
    parity_ros.add_argument("--timeout", type=float, default=10)
    bag = sub.add_parser("inspect-bag")
    bag.add_argument("bag")
    bag.add_argument("--topic", default="/ultrasound/us/frame")
    for name in ("synthetic", "benchmark"):
        command = sub.add_parser(name)
        command.add_argument("--seconds", type=float, default=20)
        command.add_argument("--input-topic", default="/ultrasound/us/image_raw")
        command.add_argument("--mask-topic", default="/thyroid/gland/mask")
        command.add_argument("--status-topic", default="/thyroid/gland/status")
        if name == "synthetic":
            command.add_argument("--fps", type=float, default=45)
            command.add_argument("--width", type=int, default=179)
            command.add_argument("--height", type=int, default=1129)
            command.add_argument("--encoding", choices=["mono8", "mono16", "bgr8", "rgb8"], default="mono8")
    args = parser.parse_args(argv)
    if args.command in {"synthetic", "benchmark"}:
        if not np.isfinite(args.seconds) or not 0 < args.seconds <= 3600:
            parser.error("seconds must be in (0,3600]")
        if args.command == "synthetic" and (
            not np.isfinite(args.fps) or not 0 < args.fps <= 1000
            or not 0 < args.width <= 4096 or not 0 < args.height <= 4096
        ):
            parser.error("fps must be in (0,1000]; dimensions must be in [1,4096]")
        ros_validation(args)
    elif args.command == "compare":
        compare(args)
    elif args.command == "compare-ros":
        if not np.isfinite(args.timeout) or not 0 < args.timeout <= 300:
            parser.error("timeout must be in (0,300]")
        compare_ros(args)
    else:
        inspect_bag(args)
