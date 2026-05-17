"""Camera calibration loading and undistortion helpers."""

import json
import logging
from pathlib import Path

import cv2
import numpy as np

try:
    import yaml
except ImportError:  # YAML loading is optional until pyyaml is installed.
    yaml = None

LOG = logging.getLogger(__name__)


class CameraProfile:
    """Small container for OpenCV camera calibration data."""

    def __init__(self, camera_matrix, dist_coeffs, image_size=None, model="pinhole",
                 new_camera_matrix=None, source_path=None, metadata=None):
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)
        self.image_size = _normalise_image_size(image_size)
        self.model = str(model or "pinhole").lower()
        self.new_camera_matrix = None
        if new_camera_matrix is not None:
            self.new_camera_matrix = np.asarray(new_camera_matrix, dtype=np.float64)
        self.source_path = str(source_path) if source_path else None
        self.metadata = metadata or {}
        self._validate()

    def _validate(self):
        if self.camera_matrix.shape != (3, 3):
            raise ValueError("camera_matrix/K must be a 3x3 matrix")
        if self.dist_coeffs.size == 0:
            raise ValueError("dist_coeffs/D must not be empty")
        if self.model not in ("pinhole", "fisheye"):
            LOG.warning("Unknown camera model %r; using pinhole undistortion", self.model)
            self.model = "pinhole"

    def undistort(self, frame):
        """Return an undistorted copy of frame using the stored calibration."""
        if self.model == "fisheye":
            return self._undistort_fisheye(frame)
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None,
                             self.new_camera_matrix)

    def _undistort_fisheye(self, frame):
        if not hasattr(cv2, "fisheye"):
            raise RuntimeError("OpenCV fisheye module is not available; cannot undistort fisheye calibration")
        d = self.dist_coeffs.reshape(-1, 1)
        if d.size < 4:
            raise RuntimeError("Fisheye calibration needs at least four distortion coefficients")
        new_k = self.new_camera_matrix if self.new_camera_matrix is not None else self.camera_matrix
        return cv2.fisheye.undistortImage(frame, self.camera_matrix, d[:4], Knew=new_k)


def load_camera_profile(path):
    """Load a CameraProfile from .npz, .json, .yaml, or .yml."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(str(path))
    suffix = path.suffix.lower()
    if suffix == ".npz":
        data = _load_npz(path)
    elif suffix == ".json":
        data = _load_json(path)
    elif suffix in (".yaml", ".yml"):
        data = _load_yaml(path)
    else:
        raise ValueError("Unsupported camera calibration format: {}".format(path.suffix))
    return _profile_from_mapping(data, path)


def find_default_camera_profile(base_dir="data/calibration/cameras"):
    """Return the first likely overhead calibration file, or None."""
    base = Path(base_dir)
    candidates = [
        base / "overhead_calibration.npz",
        base / "overhead_calibration.json",
        base / "overhead_calibration.yaml",
        base / "overhead_calibration.yml",
        base / "overhead.npz",
        base / "overhead.json",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    for pattern in ("*overhead*.npz", "*overhead*.json", "*overhead*.yaml", "*overhead*.yml"):
        matches = sorted(base.glob(pattern)) if base.exists() else []
        if matches:
            return matches[0]
    return None


def _load_npz(path):
    wanted = {
        "camera_matrix", "K",
        "dist_coeffs", "D",
        "image_size",
        "model",
        "new_camera_matrix",
    }
    data = {}
    with np.load(str(path), allow_pickle=True) as npz:
        for key in npz.files:
            if key not in wanted:
                continue
            try:
                data[key] = _np_value(npz[key])
            except Exception as exc:
                LOG.warning("Skipping optional calibration key %s in %s: %s", key, path, exc)
    return data


def _load_json(path):
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _load_yaml(path):
    if yaml is None:
        raise RuntimeError("pyyaml is required to load YAML camera calibration files")
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _profile_from_mapping(data, path):
    camera_matrix = _first_present(data, ("camera_matrix", "K"))
    dist_coeffs = _first_present(data, ("dist_coeffs", "D"))
    if camera_matrix is None or dist_coeffs is None:
        raise ValueError("{} does not contain camera_matrix/K and dist_coeffs/D".format(path))
    image_size = _first_present(data, ("image_size",))
    if image_size is None and "image_width" in data and "image_height" in data:
        image_size = [data["image_width"], data["image_height"]]
    return CameraProfile(
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=image_size,
        model=data.get("model", "pinhole"),
        new_camera_matrix=data.get("new_camera_matrix"),
        source_path=path,
        metadata=data.get("metadata") if isinstance(data.get("metadata"), dict) else data,
    )


def _first_present(mapping, names):
    for name in names:
        if name in mapping:
            return mapping[name]
    return None


def _normalise_image_size(value):
    if value is None:
        return None
    arr = np.asarray(value).astype(int).reshape(-1)
    if arr.size < 2:
        return None
    return int(arr[0]), int(arr[1])


def _np_value(value):
    if isinstance(value, np.ndarray):
        if value.shape == ():
            return value.item()
        return value
    return value
