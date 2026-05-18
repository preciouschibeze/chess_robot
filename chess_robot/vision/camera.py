"""Overhead camera capture boundary."""

from pathlib import Path
import shutil
import subprocess

import cv2

from chess_robot.calibration.camera_profile import load_camera_profile

try:
    import yaml
except ImportError:
    yaml = None

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CAMERA_CONFIG_PATH = PROJECT_ROOT / "configs" / "cameras.yaml"
OVERHEAD_CAMERA_NAME = "overhead"


class CameraCaptureError(RuntimeError):
    pass


def load_camera_config(config_path=DEFAULT_CAMERA_CONFIG_PATH):
    """Load camera configuration from configs/cameras.yaml."""
    if yaml is None:
        raise CameraCaptureError("pyyaml is required to read camera configuration")
    path = Path(config_path)
    if not path.exists():
        raise CameraCaptureError("Camera configuration not found: {}".format(path))
    with path.open("r") as handle:
        data = yaml.safe_load(handle) or {}
    return data


def get_camera_config(name=OVERHEAD_CAMERA_NAME, config_path=DEFAULT_CAMERA_CONFIG_PATH):
    data = load_camera_config(config_path)
    if name in data:
        section = data[name] or {}
    else:
        section = (data.get("cameras") or {}).get(name) or {}
    if not section:
        raise CameraCaptureError("Camera configuration missing section: {}".format(name))
    return section


def is_overhead_camera(camera_index, camera_config=None):
    """Return True when camera_index addresses the configured overhead camera."""
    config = camera_config if camera_config is not None else get_camera_config()
    configured_index = config.get("camera_index", config.get("index"))
    configured_device = config.get("device_path")
    value = str(camera_index)
    if configured_device and value == str(configured_device):
        return True
    return configured_index is not None and value == str(configured_index)


def apply_v4l2_controls(camera_config):
    """Apply configured V4L2 controls with v4l2-ctl and fail clearly on errors."""
    device_path = camera_config.get("device_path")
    controls = camera_config.get("v4l2_controls") or {}
    if not device_path:
        raise CameraCaptureError("Camera configuration is missing device_path")
    if not controls:
        raise CameraCaptureError("Camera configuration is missing v4l2_controls")

    tool = shutil.which("v4l2-ctl")
    if tool is None:
        raise CameraCaptureError("v4l2-ctl is required to apply camera settings but was not found")

    applied = []
    for name, value in controls.items():
        control = "{}={}".format(name, value)
        command = [tool, "-d", str(device_path), "--set-ctrl={}".format(control)]
        try:
            subprocess.check_call(command)
        except subprocess.CalledProcessError as exc:
            raise CameraCaptureError(
                "Failed to apply V4L2 control {} to {}: exit {}".format(
                    control, device_path, exc.returncode
                )
            )
        applied.append(control)
    return applied


def apply_overhead_camera_settings(config_path=DEFAULT_CAMERA_CONFIG_PATH):
    """Apply the configured overhead camera V4L2 controls."""
    return apply_v4l2_controls(get_camera_config(OVERHEAD_CAMERA_NAME, config_path))


def capture_frame(camera_index=0, width=None, height=None):
    """Capture one frame from an OpenCV camera index or device path."""
    device = camera_index
    if isinstance(camera_index, str) and camera_index.isdigit():
        device = int(camera_index)
    cap = cv2.VideoCapture(device)
    try:
        if not cap.isOpened():
            raise CameraCaptureError("Could not open camera {}".format(camera_index))
        if width is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height is not None:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        ok, frame = cap.read()
        if not ok or frame is None:
            raise CameraCaptureError("Could not read frame from camera {}".format(camera_index))
        return frame
    finally:
        cap.release()


def save_image(path, frame):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), frame):
        raise CameraCaptureError("Could not write image {}".format(path))
    return path


def undistort_frame(frame, calibration_path):
    profile = load_camera_profile(calibration_path)
    return profile.undistort(frame), profile
