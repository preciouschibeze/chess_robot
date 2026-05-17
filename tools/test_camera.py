#!/usr/bin/env python3
"""Capture one overhead camera frame and optionally undistort it."""

import argparse
import logging
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from chess_robot.calibration.camera_profile import find_default_camera_profile, load_camera_profile
from chess_robot.vision.camera import capture_frame, save_image


def setup_logging():
    log_dir = ROOT / "data" / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[
            logging.FileHandler(str(log_dir / "vision.log")),
            logging.StreamHandler(sys.stdout),
        ],
    )


def build_parser():
    parser = argparse.ArgumentParser(description="Capture one overhead camera frame.")
    parser.add_argument("--camera-index", default="0", help="OpenCV camera index or device path. Default: 0")
    parser.add_argument("--calib", default=None, help="Optional calibration file path (.npz/.json/.yaml/.yml)")
    parser.add_argument("--output-dir", default="data/snapshots", help="Directory for latest_raw.png and latest_undistorted.png")
    return parser


def main():
    args = build_parser().parse_args()
    setup_logging()
    log = logging.getLogger("test_camera")

    output_dir = Path(args.output_dir)
    if not output_dir.is_absolute():
        output_dir = ROOT / output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    camera_index = int(args.camera_index) if str(args.camera_index).isdigit() else args.camera_index
    frame = capture_frame(camera_index)
    height, width = frame.shape[:2]

    raw_path = save_image(output_dir / "latest_raw.png", frame)
    print("Captured frame: {}x{}".format(width, height))
    print("Raw image: {}".format(raw_path))
    log.info("Saved raw camera frame to %s", raw_path)

    calib_path = Path(args.calib) if args.calib else find_default_camera_profile(ROOT / "data" / "calibration" / "cameras")
    if calib_path is None:
        message = "WARNING: no calibration file provided or found; saved raw frame only"
        print(message)
        log.warning(message)
        return
    if not calib_path.is_absolute():
        calib_path = ROOT / calib_path

    try:
        profile = load_camera_profile(calib_path)
        undistorted = profile.undistort(frame)
    except Exception as exc:
        message = "WARNING: calibration failed for {}: {}; saved raw frame only".format(calib_path, exc)
        print(message)
        log.warning(message)
        return

    undistorted_path = save_image(output_dir / "latest_undistorted.png", undistorted)
    print("Calibration: {} ({})".format(calib_path, profile.model))
    print("Undistorted image: {}".format(undistorted_path))
    log.info("Saved undistorted camera frame to %s", undistorted_path)


if __name__ == "__main__":
    main()
