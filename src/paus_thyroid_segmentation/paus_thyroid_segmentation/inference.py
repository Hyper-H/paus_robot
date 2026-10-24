"""Persistent PyTorch inference, without any ROS imports."""

from dataclasses import dataclass
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import threading
import time

import numpy as np

from .imaging import Letterbox


@dataclass
class Prediction:
    mask: np.ndarray
    valid: bool
    area: int
    confidence: float
    preprocess_ms: float
    model_ms: float
    postprocess_ms: float


def validate_options(threshold, min_area):
    if not np.isfinite(threshold) or not 0 < threshold < 1:
        raise ValueError("threshold must be finite and strictly between 0 and 1")
    if type(min_area) is not int or min_area < 0:
        raise ValueError("min_area must be a nonnegative integer (original-image pixels)")


def load_adapter(bundle, checkpoint):
    """Verify a trusted local deployment bundle before importing its Python code."""
    if not str(bundle) or not str(checkpoint):
        raise ValueError("checkpoint and model_bundle must be explicit local paths")
    root = Path(bundle).expanduser().resolve()
    path = Path(checkpoint).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"local checkpoint not found: {path}")
    manifest_path = root / "manifest.json"
    if not manifest_path.is_file():
        raise ValueError(f"missing local training bundle: {manifest_path}; see README")
    manifest = json.loads(manifest_path.read_text())
    # Accept the versioned deployment manifest emitted by the training bundle
    # builder. Older hand-written manifests used the same values at top level.
    preprocessing = manifest.get("preprocessing", {})
    model_name = manifest.get("model")
    if model_name is None:
        model_name = "resnet18_unet" if manifest.get("task") == "whole_thyroid_gland" else None
    model_size = preprocessing.get("model_size", [224, 224])
    normalization = preprocessing.get("normalization", manifest.get("normalization"))
    resize_mode = preprocessing.get("resize", manifest.get("resize_mode"))
    resize_ok = resize_mode in {"letterbox", "Pillow BILINEAR aspect-preserving letterbox"}
    normalization_ok = normalization in {"float32_div_255", "float32 pixel / 255.0"}
    if (manifest.get("task") != "whole_thyroid_gland" or model_name != "resnet18_unet"
            or model_size != [224, 224] or not resize_ok or not normalization_ok):
        raise ValueError("bundle does not declare the required whole-gland input contract")
    if manifest.get("reference_parity_verified") is not True:
        raise ValueError("training preprocessing/reference parity has not been verified")
    checkpoint_record = files_checkpoint = manifest.get("files", {}).get(
        str(path.relative_to(root)) if path.is_relative_to(root) else "", {})
    expected_checkpoint = manifest.get("checkpoint_sha256")
    if not expected_checkpoint and isinstance(checkpoint_record, dict):
        expected_checkpoint = checkpoint_record.get("sha256")
    if not expected_checkpoint or hashlib.sha256(path.read_bytes()).hexdigest() != expected_checkpoint:
        raise ValueError("checkpoint SHA256 mismatch")
    files = manifest.get("files", {})
    if not isinstance(files, dict) or "adapter.py" not in files:
        raise ValueError("manifest must hash adapter.py and all vendored runtime sources")
    for name, record in files.items():
        digest = record.get("sha256") if isinstance(record, dict) else record
        source = (root / name).resolve()
        if not source.is_relative_to(root) or not source.is_file():
            raise ValueError(f"invalid bundle source: {name}")
        if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
            raise ValueError(f"bundle SHA256 mismatch: {name}")
    unlisted = {str(source.relative_to(root)) for source in root.rglob("*.py")} - files.keys()
    if unlisted:
        raise ValueError(f"unhashed Python files in bundle: {sorted(unlisted)}")
    # Bundled training modules may import each other. This path is local only.
    sys.path.insert(0, str(root))
    name = "paus_thyroid_training_adapter_" + hashlib.sha256(str(root).encode()).hexdigest()[:16]
    spec = importlib.util.spec_from_file_location(name, root / "adapter.py")
    adapter = importlib.util.module_from_spec(spec)
    sys.modules[name] = adapter
    try:
        spec.loader.exec_module(adapter)
    except Exception:
        sys.modules.pop(name, None)
        raise
    for name in ("build_model", "validate_config", "prepare_image", "restore_mask"):
        if not callable(getattr(adapter, name, None)):
            raise ValueError(f"training adapter missing callable {name}")
    return adapter


def gpu_preflight():
    """Refuse unavailable/busy GPUs before creating CUDA contexts."""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, check=True, timeout=10)
        rows = [tuple(float(value.strip()) for value in row.split(","))
                for row in result.stdout.strip().splitlines()]
        if not rows or any(len(row) != 3 or not np.isfinite(row).all()
                           or row[2] <= 0 or min(row) < 0 for row in rows):
            raise ValueError("unexpected nvidia-smi output")
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        raise RuntimeError("nvidia-smi unavailable/failed; GPU availability is unverified") from exc
    # Conservative across visible/physical index remapping; never interrupt other jobs.
    if any(util > 20 or used / total > 0.8 for util, used, total in rows):
        raise RuntimeError(f"GPU preflight reports busy devices: {rows}")
    try:
        processes = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid,process_name",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, check=True, timeout=10)
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError("GPU compute-process occupancy is unverified") from exc
    if processes.stdout.strip():
        raise RuntimeError("GPU preflight reports active compute processes; refusing competing inference")
    return result.stdout.strip()


class Segmenter:
    def __init__(self, checkpoint, model_bundle, device="cuda:0", cpu_fallback=False,
                 threshold=0.5, min_area=1, warmup_frames=5, cpu_threads=1, log=None,
                 *, adapter=None):
        validate_options(threshold, min_area)
        if type(warmup_frames) is not int or warmup_frames < 1:
            raise ValueError("warmup_frames must be a positive integer")
        if type(cpu_threads) is not int or cpu_threads < 1:
            raise ValueError("cpu_threads must be a positive integer")
        self._log = log or (lambda text: None)
        self.threshold, self.min_area = threshold, min_area
        self._lock = threading.Lock()
        try:
            import torch
        except ImportError as exc:
            raise RuntimeError("PyTorch is not installed in the active ROS Python environment") from exc
        self.torch = torch
        target = torch.device(device)
        if target.type not in {"cuda", "cpu"}:
            raise ValueError("device must be cpu or cuda[:index]")
        if target.type == "cuda":
            try:
                self._log("GPU preflight: " + gpu_preflight())
                if not torch.cuda.is_available():
                    raise RuntimeError("PyTorch CUDA is unavailable")
                index = target.index if target.index is not None else 0
                if index >= torch.cuda.device_count():
                    raise RuntimeError(f"CUDA device index {index} is unavailable")
            except RuntimeError as exc:
                if not cpu_fallback:
                    raise
                self._log(f"EXPLICIT CPU FALLBACK: {exc}")
                target = torch.device("cpu")
        self.device = target
        adapter = adapter if adapter is not None else load_adapter(model_bundle, checkpoint)
        torch.set_num_threads(cpu_threads)
        # Never silently fall back to unsafe pickle loading.
        payload = torch.load(str(Path(checkpoint).expanduser()), map_location="cpu", weights_only=True)
        if not isinstance(payload, dict) or not {"config", "model_state"} <= payload.keys():
            raise ValueError("checkpoint must contain config and model_state")
        adapter.validate_config(payload["config"])
        self.adapter = adapter
        self.model = adapter.build_model(payload["config"])
        self.model.load_state_dict(payload["model_state"], strict=True)
        self.model.to(target)
        self.model.eval()
        if target.type == "cuda":
            with torch.inference_mode():
                for _ in range(warmup_frames):
                    self._check_logits(self.model(torch.zeros((1, 1, 224, 224), device=target)))
            torch.cuda.synchronize(target)
            torch.cuda.reset_peak_memory_stats(target)
        self._log(f"Loaded whole-gland model once on {target}; confidence is uncalibrated")

    def _check_logits(self, logits):
        if not self.torch.is_tensor(logits) or tuple(logits.shape) != (1, 1, 224, 224):
            raise ValueError("model must return a single [1,1,224,224] logits tensor")
        if not self.torch.isfinite(logits).all().item():
            raise ValueError("nonfinite logits")
        return logits

    @staticmethod
    def _geometry_crop(geometry, gray_shape):
        """Return the model-content crop for either runtime Letterbox schema."""
        if isinstance(geometry, Letterbox):
            if (geometry.height, geometry.width) != gray_shape:
                raise ValueError("adapter letterbox geometry does not match the source image")
            if not (0 <= geometry.top < 224 and 0 <= geometry.left < 224
                    and 1 <= geometry.resized_height <= 224 - geometry.top
                    and 1 <= geometry.resized_width <= 224 - geometry.left):
                raise ValueError("invalid adapter letterbox geometry")
            return geometry.crop
        original = getattr(geometry, "original_size", None)
        content_box = getattr(geometry, "content_box", None)
        model_size = getattr(geometry, "model_size", None)
        if (original is None or content_box is None or model_size != 224
                or tuple(reversed(original)) != tuple(gray_shape)):
            raise ValueError("invalid adapter letterbox geometry")
        left, top, right, bottom = content_box
        if not (0 <= left < right <= 224 and 0 <= top < bottom <= 224):
            raise ValueError("invalid adapter letterbox content box")
        return lambda image: image[top:bottom, left:right]

    def predict(self, gray):
        with self._lock, self.torch.inference_mode():
            start = time.perf_counter()
            if not isinstance(gray, np.ndarray) or gray.dtype != np.uint8 or gray.ndim != 2 or not gray.size:
                raise ValueError("predict requires nonempty HxW uint8 grayscale")
            batch, geometry = self.adapter.prepare_image(gray)
            batch = np.asarray(batch)
            if batch.shape != (1, 1, 224, 224) or batch.dtype != np.float32:
                raise ValueError("adapter must return float32 [1,1,224,224]")
            if not np.isfinite(batch).all() or batch.min() < 0 or batch.max() > 1:
                raise ValueError("adapter input must be finite and normalized to [0,1]")
            crop_geometry = self._geometry_crop(geometry, gray.shape)
            tensor = self.torch.from_numpy(np.ascontiguousarray(batch)).to(self.device)
            if self.device.type == "cuda":
                self.torch.cuda.synchronize(self.device)
            prepared = time.perf_counter()
            logits = self._check_logits(self.model(tensor))
            if self.device.type == "cuda":
                self.torch.cuda.synchronize(self.device)
            inferred = time.perf_counter()
            probability = self.torch.sigmoid(logits)[0, 0].cpu().numpy()
            binary = (probability >= self.threshold).astype(np.uint8) * 255
            mask = np.asarray(self.adapter.restore_mask(binary, geometry))
            if mask.shape != gray.shape or mask.dtype != np.uint8 or not np.isin(mask, [0, 255]).all():
                raise ValueError("restored mask must be original-size uint8 with values 0/255")
            foreground = crop_geometry(probability)
            selected = foreground[foreground >= self.threshold]
            area = int(np.count_nonzero(mask))
            confidence = float(selected.mean()) if selected.size and area else 0.0
            finished = time.perf_counter()
            return Prediction(
                mask, area > 0 and area >= self.min_area, area, confidence,
                (prepared - start) * 1000, (inferred - prepared) * 1000,
                (finished - inferred) * 1000)

    def memory_metrics(self):
        if self.device.type != "cuda":
            return {"gpu_allocated_mb": 0.0, "gpu_peak_allocated_mb": 0.0}
        return {
            "gpu_allocated_mb": self.torch.cuda.memory_allocated(self.device) / 2**20,
            "gpu_peak_allocated_mb": self.torch.cuda.max_memory_allocated(self.device) / 2**20,
        }
