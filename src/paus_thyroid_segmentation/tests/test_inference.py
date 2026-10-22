import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest

from paus_thyroid_segmentation.imaging import letterbox, restore_mask
from paus_thyroid_segmentation.inference import Segmenter, gpu_preflight, load_adapter


def test_bundle_missing_fails_closed(tmp_path):
    with pytest.raises(ValueError, match="checkpoint not found"):
        load_adapter(tmp_path, tmp_path / "missing.pth")
    checkpoint = tmp_path / "checkpoint.pth"
    checkpoint.write_bytes(b"not a pickle")
    with pytest.raises(ValueError, match="missing local training bundle"):
        load_adapter(tmp_path, checkpoint)


def test_bundle_hash_and_parity_gate(tmp_path):
    checkpoint = tmp_path / "checkpoint.pth"
    checkpoint.write_bytes(b"not a pickle")
    manifest = {
        "task": "whole_thyroid_gland", "model": "resnet18_unet",
        "input_shape": [1, 1, 224, 224], "resize_mode": "letterbox",
        "normalization": "float32_div_255", "reference_parity_verified": False,
        "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "files": {"adapter.py": "wrong"},
    }
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="parity"):
        load_adapter(tmp_path, checkpoint)
    manifest["reference_parity_verified"] = True
    path.write_text(json.dumps(manifest))
    (tmp_path / "adapter.py").write_text("raise AssertionError('must not execute')")
    with pytest.raises(ValueError, match="SHA256 mismatch"):
        load_adapter(tmp_path, checkpoint)


def test_gpu_preflight_unavailable_and_busy(monkeypatch):
    def unavailable(*args, **kwargs):
        raise OSError("no nvidia-smi")
    monkeypatch.setattr("paus_thyroid_segmentation.inference.subprocess.run", unavailable)
    with pytest.raises(RuntimeError, match="unverified"):
        gpu_preflight()
    monkeypatch.setattr("paus_thyroid_segmentation.inference.subprocess.run",
                        lambda *a, **k: SimpleNamespace(stdout="95, 4000, 8000"))
    with pytest.raises(RuntimeError, match="busy"):
        gpu_preflight()


def test_gpu_preflight_idle_and_compute_occupancy(monkeypatch):
    def idle(command, **kwargs):
        return SimpleNamespace(stdout="0, 100, 8000" if "--query-gpu" in command[1] else "")
    monkeypatch.setattr("paus_thyroid_segmentation.inference.subprocess.run", idle)
    assert gpu_preflight() == "0, 100, 8000"
    def occupied(command, **kwargs):
        return SimpleNamespace(stdout="0, 100, 8000" if "--query-gpu" in command[1] else "123, python")
    monkeypatch.setattr("paus_thyroid_segmentation.inference.subprocess.run", occupied)
    with pytest.raises(RuntimeError, match="compute processes"):
        gpu_preflight()


@pytest.mark.parametrize("output", ["", "N/A, 0, 8000", "0, 0, 0", "nan, 10, 8000"])
def test_gpu_preflight_rejects_unverifiable_output(monkeypatch, output):
    monkeypatch.setattr("paus_thyroid_segmentation.inference.subprocess.run",
                        lambda *a, **k: SimpleNamespace(stdout=output))
    with pytest.raises(RuntimeError, match="unverified"):
        gpu_preflight()


def make_torch_engine(tmp_path, monkeypatch, device="cpu", fallback=False, min_area=1):
    torch = pytest.importorskip("torch", reason="PyTorch not installed; lifecycle not verified")
    events = {"loads": 0, "eval": 0, "calls": 0}

    class TinyModel(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.bias = torch.nn.Parameter(torch.tensor(-0.5))

        def eval(self):
            events["eval"] += 1
            return super().eval()

        def forward(self, x):
            assert not self.training
            assert torch.is_inference_mode_enabled()
            events["calls"] += 1
            return (x + self.bias) * 10

    checkpoint = tmp_path / "test_only.pth"
    torch.save({"config": {"test_only": True}, "model_state": TinyModel().state_dict()}, checkpoint)
    original_load = torch.load

    def tracked_load(*args, **kwargs):
        assert kwargs["weights_only"] is True
        events["loads"] += 1
        return original_load(*args, **kwargs)

    monkeypatch.setattr(torch, "load", tracked_load)

    def validate(config):
        assert config == {"test_only": True}

    adapter = SimpleNamespace(build_model=lambda config: TinyModel(), validate_config=validate,
                              prepare_image=letterbox, restore_mask=restore_mask)
    logs = []
    engine = Segmenter(checkpoint, "", device=device, cpu_fallback=fallback,
                       min_area=min_area, log=logs.append, adapter=adapter)
    return engine, events, logs


def test_torch_resident_eval_inference_and_empty_confidence(tmp_path, monkeypatch):
    engine, events, _ = make_torch_engine(tmp_path, monkeypatch)
    for _ in range(3):
        result = engine.predict(np.full((101, 179), 255, np.uint8))
        assert result.valid and result.area == 101 * 179
        assert 0.99 < result.confidence < 1
    empty = engine.predict(np.zeros((101, 179), np.uint8))
    assert not empty.valid and empty.confidence == 0 and empty.area == 0
    assert events == {"loads": 1, "eval": 1, "calls": 4}
    assert engine.memory_metrics()["gpu_allocated_mb"] == 0


def test_explicit_cpu_fallback(tmp_path, monkeypatch):
    def unavailable():
        raise RuntimeError("test GPU unavailable")
    monkeypatch.setattr("paus_thyroid_segmentation.inference.gpu_preflight", unavailable)
    with pytest.raises(RuntimeError, match="GPU unavailable"):
        make_torch_engine(tmp_path, monkeypatch, "cuda:0")
    engine, _, logs = make_torch_engine(tmp_path, monkeypatch, "cuda:0", True)
    assert str(engine.device) == "cpu"
    assert any("EXPLICIT CPU FALLBACK" in message for message in logs)


def test_torch_min_area_does_not_erase_prediction(tmp_path, monkeypatch):
    engine, _, _ = make_torch_engine(tmp_path, monkeypatch, min_area=100000)
    result = engine.predict(np.full((51, 31), 255, np.uint8))
    assert not result.valid and result.area == 51 * 31
    assert result.mask.all()
