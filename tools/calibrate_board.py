#!/usr/bin/env python3
"""Seam-aware board calibration tool for a manual 9x9 grid model."""

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from chess_robot.calibration.board_profile import BOARD_PROFILE_VERSION
from chess_robot.vision.board_calibration import (
    DEFAULT_GRID_MODEL_FOUR_CORNER,
    DEFAULT_GRID_MODEL_MANUAL_9X9,
    build_board_profile,
    build_corner_labels,
    build_ignored_seam_region,
    grid_points_from_four_corners,
    grid_points_from_manual_points,
)
from chess_robot.vision.board_renderer import save_debug_overlays

WINDOW_NAME = "board_calibration"


class PointCollector(object):
    def __init__(self, image, target_count, mode, title, instructions, hint_func=None,
                 allow_finish_key=False, display_scale=1.0):
        self.image = image
        self.target_count = target_count
        self.mode = mode
        self.title = title
        self.instructions = list(instructions)
        self.hint_func = hint_func
        self.allow_finish_key = allow_finish_key
        self.display_scale = float(display_scale)
        self.points = []
        self.finished = False
        self.aborted = False

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.finished:
            return
        image_x = float(x) / self.display_scale
        image_y = float(y) / self.display_scale
        image_x = max(0.0, min(float(self.image.shape[1] - 1), image_x))
        image_y = max(0.0, min(float(self.image.shape[0] - 1), image_y))
        self.points.append([image_x, image_y])
        if len(self.points) >= self.target_count:
            self.finished = True

    def handle_key(self, key):
        if key in (27, ord("q"), ord("Q")):
            self.aborted = True
            return
        if key in (ord("r"), ord("R")):
            self.points = []
            self.finished = False
            return
        if key in (8, 127, ord("u"), ord("U")):
            if self.points:
                self.points.pop()
            self.finished = False
            return
        if self.allow_finish_key and key in (13, 10):
            if len(self.points) >= 3:
                self.finished = True

    def render(self):
        canvas = self.image.copy()
        _draw_instruction_panel(canvas, self.instructions, self.mode, len(self.points),
                               self.target_count, self.hint_func(self.points) if self.hint_func else None)
        _draw_points(canvas, self.points)
        return _resize_for_display(canvas, self.display_scale)


def build_parser():
    parser = argparse.ArgumentParser(description="Calibrate the chess board geometry from a fixed overhead image.")
    parser.add_argument(
        "--image",
        required=True,
        help="Path to the overhead image to calibrate, preferably data/snapshots/latest_undistorted.png",
    )
    parser.add_argument(
        "--output",
        default="data/calibration/board/board_profile.yaml",
        help="Output board profile YAML path. Default: data/calibration/board/board_profile.yaml",
    )
    parser.add_argument(
        "--crop-fraction",
        type=float,
        default=0.60,
        help="Central crop fraction used for occupancy polygons. Default: 0.60",
    )
    parser.add_argument(
        "--mode",
        choices=("four-corner", "manual-9x9"),
        default="manual-9x9",
        help="Calibration mode: four-corner or manual-9x9",
    )
    parser.add_argument(
        "--seam",
        action="store_true",
        help="After board selection, allow an optional seam polygon to be clicked and ignored.",
    )
    parser.add_argument(
        "--display-scale",
        type=float,
        default=1.0,
        help="Scale the OpenCV display before clicking, e.g. 0.75, 1.0, 1.5, or 2.0. Default: 1.0",
    )
    parser.add_argument(
        "--debug-dir",
        default="data/debug",
        help="Directory for generated board calibration overlays. Default: data/debug",
    )
    return parser


def main():
    args = build_parser().parse_args()
    image_path = _resolve_path(args.image)
    output_path = _resolve_path(args.output)
    debug_dir = _resolve_path(args.debug_dir)
    display_scale = _validate_display_scale(args.display_scale)

    image = cv2.imread(str(image_path))
    if image is None:
        raise SystemExit("Could not read image {}".format(image_path))
    height, width = image.shape[:2]

    print_board_instructions(args.mode, args.seam)

    try:
        if args.mode == "four-corner":
            corners = collect_four_corners(image, display_scale)
            grid_points = grid_points_from_four_corners(corners)
            grid_model = DEFAULT_GRID_MODEL_FOUR_CORNER
            seam_points = []
        else:
            points = collect_manual_grid_points(image, display_scale)
            grid_points = grid_points_from_manual_points(points)
            grid_model = DEFAULT_GRID_MODEL_MANUAL_9X9
            seam_points = []

        if args.seam:
            seam_points = collect_seam_polygon(image, display_scale)

        ignored_regions = []
        seam_region = build_ignored_seam_region(seam_points)
        if seam_region is not None:
            ignored_regions.append(seam_region)

        created_at = datetime.utcnow().isoformat() + "Z"
        board_profile = build_board_profile(
            source_image_path=str(args.image),
            image_size=(width, height),
            grid_points=grid_points,
            crop_fraction=args.crop_fraction,
            grid_model=grid_model,
            ignored_regions=ignored_regions,
            version=BOARD_PROFILE_VERSION,
            created_at=created_at,
            corner_labels=build_corner_labels(),
        )

        output_path.parent.mkdir(parents=True, exist_ok=True)
        board_profile.save(output_path)

        overlays = save_debug_overlays(image, board_profile, debug_dir)

        print("Saved board profile: {}".format(output_path))
        print("Saved debug overlay: {}".format(overlays["grid"]))
        print("Saved debug overlay: {}".format(overlays["labels"]))
        print("Saved debug overlay: {}".format(overlays["occupancy"]))
        print("Calibration mode: {}".format(args.mode))
        print("Grid model: {}".format(grid_model))
        print("Display scale: {:.2f}".format(display_scale))
        print("Crop fraction: {:.2f}".format(args.crop_fraction))
        if seam_region is not None:
            print("Ignored seam polygon: yes")
        else:
            print("Ignored seam polygon: no")
    finally:
        cv2.destroyAllWindows()


def collect_four_corners(image, display_scale=1.0):
    instructions = [
        "Click the 4 playable-board corners in this order:",
        "1) top-left (h1)",
        "2) top-right (a1)",
        "3) bottom-right (a8)",
        "4) bottom-left (h8)",
        "Keys: r reset, u/backspace undo, q or Esc quit",
    ]
    collector = PointCollector(
        image=image,
        target_count=4,
        mode="four-corner",
        title="four-corner calibration",
        instructions=instructions,
        hint_func=_corner_hint,
        display_scale=display_scale,
    )
    return _run_point_selector(collector)


def collect_manual_grid_points(image, display_scale=1.0):
    instructions = [
        "Click all 81 grid intersections in row-major order:",
        "left to right across the top row, then the next row down",
        "The first row is h1..a1 and the last row is h8..a8",
        "Keys: r reset, u/backspace undo, q or Esc quit",
    ]
    collector = PointCollector(
        image=image,
        target_count=81,
        mode="manual-9x9",
        title="manual 9x9 calibration",
        instructions=instructions,
        hint_func=_manual_hint,
        display_scale=display_scale,
    )
    return _run_point_selector(collector)


def collect_seam_polygon(image, display_scale=1.0):
    instructions = [
        "Optional seam polygon selection.",
        "Click around the central board seam to mark an ignored region.",
        "Press Enter to finish when you have at least 3 points.",
        "Keys: r reset, u/backspace undo, s skip seam selection, q or Esc quit",
    ]
    collector = PointCollector(
        image=image,
        target_count=9999,
        mode="seam",
        title="seam selection",
        instructions=instructions,
        hint_func=_seam_hint,
        allow_finish_key=True,
        display_scale=display_scale,
    )
    try:
        return _run_point_selector(collector)
    except SkipSelection:
        return []


def _run_point_selector(collector):
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    display_width = max(1, int(round(collector.image.shape[1] * collector.display_scale)))
    display_height = max(1, int(round(collector.image.shape[0] * collector.display_scale)))
    cv2.resizeWindow(WINDOW_NAME, display_width, display_height)
    cv2.setMouseCallback(WINDOW_NAME, collector.on_mouse)
    while True:
        cv2.imshow(WINDOW_NAME, collector.render())
        key = cv2.waitKey(20) & 0xFF
        if key in (ord("s"), ord("S")) and collector.mode == "seam":
            raise SkipSelection()
        collector.handle_key(key)
        if collector.aborted:
            raise SystemExit("Calibration aborted by user")
        if collector.finished:
            points = list(collector.points)
            if collector.mode == "seam" and len(points) < 3:
                continue
            return points


class SkipSelection(Exception):
    pass


def print_board_instructions(mode, seam_enabled):
    print("Board calibration starting.")
    print("Mode: {}".format(mode))
    print("Board orientation: robot_black_side (top-left h1, top-right a1, bottom-right a8, bottom-left h8)")
    if mode == "four-corner":
        print("Click 4 corners in this order: top-left, top-right, bottom-right, bottom-left.")
        print("The tool will bilinearly interpolate the 9x9 grid from those corners.")
    else:
        print("Click 81 grid intersections in row-major order: left-to-right, top-to-bottom.")
        print("The tool will save the exact 9x9 grid points.")
    if seam_enabled:
        print("After the board points are captured, you may click an optional seam polygon.")
        print("Press Enter to finish seam selection, or s to skip it.")
    print("Controls: r reset, u/backspace undo last point, q or Esc quit.")
    print("Close the OpenCV window or press q/Esc to exit.")


def _draw_instruction_panel(canvas, instructions, mode, count, target_count, hint):
    overlay = canvas.copy()
    panel_height = min(canvas.shape[0], 154)
    cv2.rectangle(overlay, (0, 0), (canvas.shape[1], panel_height), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0.0, canvas)
    y = 20
    for line in instructions:
        cv2.putText(canvas, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        y += 18
    next_index = count + 1
    if next_index > target_count:
        next_index = target_count
    status = "{}: captured {}/{} | current click {}/{}".format(
        mode, count, target_count, next_index, target_count
    )
    if hint:
        status = status + " | expected: {}".format(hint)
    cv2.putText(canvas, status, (12, min(canvas.shape[0] - 12, 140)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)


def _draw_points(canvas, points):
    for index, point in enumerate(points):
        x = int(round(point[0]))
        y = int(round(point[1]))
        cv2.circle(canvas, (x, y), 5, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.putText(canvas, str(index + 1), (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (255, 255, 0), 1, cv2.LINE_AA)


def _corner_hint(points):
    hints = [
        "top-left (h1)",
        "top-right (a1)",
        "bottom-right (a8)",
        "bottom-left (h8)",
    ]
    if len(points) >= len(hints):
        return None
    return hints[len(points)]


def _manual_hint(points):
    index = len(points)
    if index >= 81:
        return None
    row = index // 9
    col = index % 9
    return "row {}, col {}".format(row, col)


def _seam_hint(points):
    if not points:
        return "click around the seam"
    return "{} seam points captured".format(len(points))


def _resolve_path(raw_path):
    path = Path(raw_path)
    if path.is_absolute():
        return path
    return ROOT / path


def _validate_display_scale(display_scale):
    try:
        value = float(display_scale)
    except (TypeError, ValueError):
        raise SystemExit("--display-scale must be a number")
    if value <= 0.0:
        raise SystemExit("--display-scale must be greater than 0")
    return value


def _resize_for_display(image, display_scale):
    if abs(float(display_scale) - 1.0) < 0.0001:
        return image
    width = max(1, int(round(image.shape[1] * float(display_scale))))
    height = max(1, int(round(image.shape[0] * float(display_scale))))
    interpolation = cv2.INTER_AREA if display_scale < 1.0 else cv2.INTER_LINEAR
    return cv2.resize(image, (width, height), interpolation=interpolation)


if __name__ == "__main__":
    main()
