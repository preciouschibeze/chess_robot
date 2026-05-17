"""Overhead camera capture boundary."""

from pathlib import Path

import cv2

from chess_robot.calibration.camera_profile import load_camera_profile


class CameraCaptureError(RuntimeError):
    pass


def capture_frame(camera_index=0):
    """Capture one frame from an OpenCV camera index or device path."""
    device = camera_index
    if isinstance(camera_index, str) and camera_index.isdigit():
        device = int(camera_index)
    cap = cv2.VideoCapture(device)
    try:
        if not cap.isOpened():
            raise CameraCaptureError("Could not open camera {}".format(camera_index))
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
