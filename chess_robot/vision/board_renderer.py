"""Board and occupancy-grid rendering helpers for debug images."""

from pathlib import Path

import cv2
import numpy as np

from chess_robot.calibration.board_profile import square_names

GRID_LINE_COLOR = (0, 215, 255)
LABEL_COLOR = (255, 255, 255)
LABEL_SHADOW_COLOR = (0, 0, 0)
SQUARE_COLOR = (0, 180, 255)
CROP_COLOR = (0, 220, 0)
IGNORED_REGION_COLOR = (0, 0, 255)
POINT_COLOR = (255, 255, 255)


def render_board_grid_overlay(image, board_profile):
    """Render the 9x9 grid lines and ignored regions."""
    canvas = image.copy()
    _draw_grid_lines(canvas, board_profile.grid_points, GRID_LINE_COLOR, 2)
    _draw_ignored_regions(canvas, board_profile.ignored_regions)
    return canvas


def render_board_labels_overlay(image, board_profile):
    """Render the grid plus square labels at square centres."""
    canvas = image.copy()
    _draw_grid_lines(canvas, board_profile.grid_points, GRID_LINE_COLOR, 2)
    _draw_square_labels(canvas, board_profile)
    _draw_ignored_regions(canvas, board_profile.ignored_regions)
    return canvas


def render_occupancy_crop_overlay(image, board_profile):
    """Render square polygons and central crop polygons."""
    canvas = image.copy()
    _draw_grid_lines(canvas, board_profile.grid_points, (80, 80, 80), 1)
    for name in square_names():
        square_polygon = board_profile.square_polygon(name)
        crop_polygon = board_profile.square_crop_polygon(name)
        _draw_polygon(canvas, square_polygon, SQUARE_COLOR, 1)
        _draw_polygon(canvas, crop_polygon, CROP_COLOR, 2)
    _draw_ignored_regions(canvas, board_profile.ignored_regions)
    return canvas


def save_image(path, image):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise IOError("Could not write image {}".format(path))
    return path


def save_debug_overlays(image, board_profile, output_dir):
    """Write all required board-calibration debug overlays."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    grid_path = output_dir / "board_grid_overlay.png"
    labels_path = output_dir / "board_labels_overlay.png"
    crop_path = output_dir / "occupancy_crop_overlay.png"
    save_image(grid_path, render_board_grid_overlay(image, board_profile))
    save_image(labels_path, render_board_labels_overlay(image, board_profile))
    save_image(crop_path, render_occupancy_crop_overlay(image, board_profile))
    return {
        "grid": grid_path,
        "labels": labels_path,
        "occupancy": crop_path,
    }


def _draw_grid_lines(canvas, grid_points, color, thickness):
    for col in range(len(grid_points[0])):
        points = [grid_points[row][col] for row in range(len(grid_points))]
        _draw_polyline(canvas, points, color, thickness)
    for row in range(len(grid_points)):
        points = list(grid_points[row])
        _draw_polyline(canvas, points, color, thickness)


def _draw_square_labels(canvas, board_profile):
    for name in square_names():
        centre = board_profile.square_centre(name)
        x = int(round(centre[0]))
        y = int(round(centre[1]))
        _draw_text(canvas, name, (x, y))
        cv2.circle(canvas, (x, y), 2, POINT_COLOR, -1)


def _draw_ignored_regions(canvas, ignored_regions):
    for region in ignored_regions or []:
        polygon = _region_polygon(region)
        if polygon:
            _draw_polygon(canvas, polygon, IGNORED_REGION_COLOR, 2)


def _region_polygon(region):
    if isinstance(region, dict):
        if "points" in region:
            return region["points"]
        if "polygon" in region:
            return region["polygon"]
    elif isinstance(region, (list, tuple)):
        return region
    return None


def _draw_polygon(canvas, polygon, color, thickness):
    if not polygon:
        return
    points = np.asarray([[int(round(point[0])), int(round(point[1]))] for point in polygon], dtype=np.int32)
    cv2.polylines(canvas, [points], True, color, thickness, cv2.LINE_AA)


def _draw_polyline(canvas, points, color, thickness):
    if len(points) < 2:
        return
    array = np.asarray([[int(round(point[0])), int(round(point[1]))] for point in points], dtype=np.int32)
    cv2.polylines(canvas, [array], False, color, thickness, cv2.LINE_AA)


def _draw_text(canvas, text, origin):
    x, y = origin
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.45
    thickness = 1
    shadow_offset = 1
    cv2.putText(canvas, text, (x + shadow_offset, y + shadow_offset), font, scale,
                LABEL_SHADOW_COLOR, thickness + 1, cv2.LINE_AA)
    cv2.putText(canvas, text, (x, y), font, scale, LABEL_COLOR, thickness, cv2.LINE_AA)
