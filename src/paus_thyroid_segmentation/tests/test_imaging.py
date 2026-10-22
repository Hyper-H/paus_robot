import numpy as np
import pytest

from paus_thyroid_segmentation.imaging import (
    Mono16Policy, convert_image, draw_overlay, letterbox, restore_mask,
)
from paus_thyroid_segmentation.inference import validate_options
from paus_thyroid_segmentation.latest import LatestFrame


@pytest.mark.parametrize("shape", [(1129, 179), (179, 1129), (223, 137), (1, 5000), (5000, 1), (224, 224)])
def test_letterbox_restore_shape_and_padding(shape):
    gray = np.full(shape, 255, np.uint8)
    batch, geometry = letterbox(gray)
    assert batch.shape == (1, 1, 224, 224)
    assert batch.dtype == np.float32
    assert np.all(geometry.crop(batch[0, 0]) == 1)
    assert np.count_nonzero(batch) == geometry.resized_width * geometry.resized_height
    mask = restore_mask((batch[0, 0] >= 0.5).astype(np.uint8) * 255, geometry)
    assert mask.shape == shape
    assert np.all(mask == 255)
    padding_only = (batch[0, 0] == 0).astype(np.uint8) * 255
    assert not restore_mask(padding_only, geometry).any()


def test_normalization_and_nearest():
    gray = np.arange(224 * 224, dtype=np.uint8).reshape(224, 224)
    batch, _ = letterbox(gray)
    np.testing.assert_array_equal(batch[0, 0], gray.astype(np.float32) / 255)
    _, geometry = letterbox(np.zeros((11, 17), np.uint8))
    binary = np.zeros((224, 224), np.uint8)
    binary[:, 112:] = 255
    mask = restore_mask(binary, geometry)
    assert set(np.unique(mask)) == {0, 255}
    assert not mask[:, :8].any()
    assert mask[:, 9:].all()


def test_color_order_and_overlay_preservation():
    rgb = np.array([[[255, 0, 0], [0, 0, 255], [0, 255, 0]]], np.uint8)
    gray, bgr = convert_image(rgb, "rgb8")
    np.testing.assert_array_equal(gray, [[76, 29, 150]])
    np.testing.assert_array_equal(gray, convert_image(bgr, "bgr8")[0])
    np.testing.assert_array_equal(draw_overlay(bgr, np.zeros((1, 3), np.uint8)), bgr)
    np.testing.assert_array_equal(rgb, [[[255, 0, 0], [0, 0, 255], [0, 255, 0]]])


def test_mono16_fixed_window_clips_and_rounds_without_wrapping():
    pixels = np.array([[0, 99, 100, 1100, 2100, 4095, 65535]], np.uint16)
    gray, _ = convert_image(pixels, "mono16", Mono16Policy("fixed_window", 100, 2100))
    np.testing.assert_array_equal(gray, [[0, 0, 0, 128, 255, 255, 255]])
    full, _ = convert_image(np.array([[0, 32768, 65535]], np.uint16), "mono16",
                            Mono16Policy("fixed_window"))
    np.testing.assert_array_equal(full, [[0, 128, 255]])
    with pytest.raises(ValueError, match="mono16 rejected"):
        convert_image(pixels, "mono16")


@pytest.mark.parametrize("mode,low,high", [("auto", 0, 100), ("fixed_window", 0, 0),
                                          ("fixed_window", -1, 65535), ("reject", 0, float("nan"))])
def test_bad_mono16_policy(mode, low, high):
    with pytest.raises(ValueError):
        Mono16Policy(mode, low, high)


@pytest.mark.parametrize("array,encoding", [
    (np.zeros((2, 2), np.uint8), "8UC1"),
    (np.zeros((2, 2), np.float32), "mono8"),
    (np.zeros((2, 2, 4), np.uint8), "rgb8"),
    (np.zeros((0, 2), np.uint8), "mono8"),
])
def test_bad_encoding_or_array(array, encoding):
    with pytest.raises(ValueError):
        convert_image(array, encoding)


@pytest.mark.parametrize("threshold,area", [(0, 0), (1, 0), (float("nan"), 1), (0.5, -1), (0.5, 2.5)])
def test_invalid_inference_options(threshold, area):
    with pytest.raises(ValueError):
        validate_options(threshold, area)


def test_latest_slot_overwrites_and_shutdown():
    mailbox = LatestFrame()
    mailbox.put("old")
    mailbox.put("new")
    assert mailbox.take() == "new"
    assert mailbox.counts() == {"received": 2, "overwritten": 1, "pending": 0}
    mailbox.close()
    mailbox.put("ignored")
    assert mailbox.take() is None
