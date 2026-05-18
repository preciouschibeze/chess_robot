"""Central square crop extraction and conservative occupancy evidence."""

import json
import math
import os

import cv2
import numpy as np

from chess_robot.calibration.board_profile import BoardProfile, square_names

DEFAULT_PROFILE_PATH = "data/calibration/board/board_profile.yaml"
DEFAULT_OUTPUT_DIR = "data/debug"
DEFAULT_OCCUPIED_THRESHOLD = 0.12
DEFAULT_UNCERTAIN_THRESHOLD = 0.06
DEFAULT_UNCERTAIN_MARGIN = None

STATE_UNKNOWN = "unknown"
STATE_OCCUPIED = "occupied"
STATE_EMPTY = "empty"
STATE_UNCERTAIN = "uncertain"

_STATE_SYMBOLS = {
    STATE_OCCUPIED: "O",
    STATE_EMPTY: ".",
    STATE_UNCERTAIN: "?",
    STATE_UNKNOWN: "U",
}

_STATE_COLOURS = {
    STATE_OCCUPIED: (60, 70, 230),
    STATE_EMPTY: (70, 180, 90),
    STATE_UNCERTAIN: (45, 180, 230),
    STATE_UNKNOWN: (160, 160, 160),
}


def load_image(path):
    """Load an image from disk with OpenCV and fail loudly if missing."""
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("could not read image: {}".format(path))
    return image


def load_profile(path=DEFAULT_PROFILE_PATH):
    """Load the calibrated board profile."""
    return BoardProfile.load(path)


def analyse_image(image_path, profile_path=DEFAULT_PROFILE_PATH,
                  empty_reference_path=None, occupied_threshold=DEFAULT_OCCUPIED_THRESHOLD,
                  uncertain_margin=DEFAULT_UNCERTAIN_MARGIN,
                  uncertain_threshold=DEFAULT_UNCERTAIN_THRESHOLD):
    """Analyse all 64 central square crop polygons for occupancy evidence."""
    image = load_image(image_path)
    profile = load_profile(profile_path)
    reference = load_image(empty_reference_path) if empty_reference_path else None
    if reference is not None and reference.shape[:2] != image.shape[:2]:
        raise ValueError(
            "empty reference image size {} does not match input image size {}".format(
                reference.shape[:2], image.shape[:2]
            )
        )
    return analyse_board_image(
        image,
        profile,
        image_path=image_path,
        profile_path=profile_path,
        reference_image=reference,
        empty_reference_path=empty_reference_path,
        occupied_threshold=occupied_threshold,
        uncertain_threshold=uncertain_threshold,
        uncertain_margin=uncertain_margin,
    )


def analyse_board_image(image, profile, image_path=None, profile_path=None,
                        reference_image=None, empty_reference_path=None,
                        occupied_threshold=DEFAULT_OCCUPIED_THRESHOLD,
                        uncertain_margin=DEFAULT_UNCERTAIN_MARGIN,
                        uncertain_threshold=DEFAULT_UNCERTAIN_THRESHOLD):
    """Return a JSON-friendly occupancy result dictionary for an image."""
    effective_uncertain_threshold = resolve_uncertain_threshold(
        occupied_threshold,
        uncertain_threshold,
        uncertain_margin,
    )
    result = {
        "image_path": str(image_path) if image_path is not None else None,
        "profile_path": str(profile_path) if profile_path is not None else None,
        "empty_reference_path": str(empty_reference_path) if empty_reference_path is not None else None,
        "board_orientation": profile.board_orientation,
        "occupied_threshold": float(occupied_threshold),
        "uncertain_threshold": float(effective_uncertain_threshold),
        "uncertain_margin": float(uncertain_margin) if uncertain_margin is not None else None,
        "squares": {},
    }

    for square in square_names():
        polygon = profile.square_crop_polygon(square)
        diagnostics = compute_crop_diagnostics(image, polygon)
        entry = {
            "state": STATE_UNKNOWN,
            "confidence": 0.0,
            "score": 0.0,
            "diagnostics": diagnostics,
        }
        if reference_image is not None:
            reference_diagnostics = compute_crop_diagnostics(reference_image, polygon)
            score, comparison = compute_reference_difference(
                image,
                reference_image,
                polygon,
                diagnostics,
                reference_diagnostics,
            )
            state, confidence = classify_score(
                score,
                occupied_threshold,
                uncertain_threshold=effective_uncertain_threshold,
            )
            entry["state"] = state
            entry["confidence"] = confidence
            entry["score"] = score
            entry["diagnostics"].update(comparison)
        result["squares"][square] = entry
    return result


def compute_crop_diagnostics(image, polygon):
    """Compute diagnostics using only pixels inside the crop polygon."""
    mask = polygon_mask(image.shape[:2], polygon)
    inside = mask > 0
    crop_area = int(np.count_nonzero(inside))
    if crop_area <= 0:
        return {
            "mean_brightness": 0.0,
            "std_brightness": 0.0,
            "laplacian_variance": 0.0,
            "crop_area": 0,
            "mean_colour": [0.0, 0.0, 0.0],
        }

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    brightness_values = gray[inside].astype(np.float32)
    colour_values = image[inside].astype(np.float32)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    laplacian_values = laplacian[inside]
    mean_bgr = np.mean(colour_values, axis=0)

    return {
        "mean_brightness": _round_float(np.mean(brightness_values)),
        "std_brightness": _round_float(np.std(brightness_values)),
        "laplacian_variance": _round_float(np.var(laplacian_values)),
        "crop_area": crop_area,
        "mean_colour": [
            _round_float(mean_bgr[2]),
            _round_float(mean_bgr[1]),
            _round_float(mean_bgr[0]),
        ],
    }


def compute_reference_difference(image, reference_image, polygon, diagnostics, reference_diagnostics):
    """Compare a current square crop against an empty-board reference crop."""
    mask = polygon_mask(image.shape[:2], polygon)
    inside = mask > 0
    if not np.any(inside):
        return 0.0, {
            "reference_mean_brightness": reference_diagnostics["mean_brightness"],
            "reference_std_brightness": reference_diagnostics["std_brightness"],
            "reference_laplacian_variance": reference_diagnostics["laplacian_variance"],
            "reference_mean_colour": reference_diagnostics["mean_colour"],
            "pixel_mae": 0.0,
            "brightness_difference": 0.0,
            "colour_difference": 0.0,
            "laplacian_difference": 0.0,
        }

    current_pixels = image[inside].astype(np.float32)
    reference_pixels = reference_image[inside].astype(np.float32)
    pixel_mae = float(np.mean(np.abs(current_pixels - reference_pixels)))
    brightness_difference = abs(diagnostics["mean_brightness"] - reference_diagnostics["mean_brightness"])
    current_colour = np.array(diagnostics["mean_colour"], dtype=np.float32)
    reference_colour = np.array(reference_diagnostics["mean_colour"], dtype=np.float32)
    colour_difference = float(np.linalg.norm(current_colour - reference_colour))
    laplacian_difference = abs(
        diagnostics["laplacian_variance"] - reference_diagnostics["laplacian_variance"]
    )

    pixel_score = _clamp(pixel_mae / 255.0, 0.0, 1.0)
    brightness_score = _clamp(brightness_difference / 255.0, 0.0, 1.0)
    colour_score = _clamp(colour_difference / (math.sqrt(3.0) * 255.0), 0.0, 1.0)
    edge_score = _clamp(laplacian_difference / 1000.0, 0.0, 1.0)
    score = (
        0.65 * pixel_score +
        0.15 * brightness_score +
        0.10 * colour_score +
        0.10 * edge_score
    )

    comparison = {
        "reference_mean_brightness": reference_diagnostics["mean_brightness"],
        "reference_std_brightness": reference_diagnostics["std_brightness"],
        "reference_laplacian_variance": reference_diagnostics["laplacian_variance"],
        "reference_mean_colour": reference_diagnostics["mean_colour"],
        "pixel_mae": _round_float(pixel_mae),
        "brightness_difference": _round_float(brightness_difference),
        "colour_difference": _round_float(colour_difference),
        "laplacian_difference": _round_float(laplacian_difference),
    }
    return _round_float(score), comparison


def resolve_uncertain_threshold(occupied_threshold=DEFAULT_OCCUPIED_THRESHOLD,
                                uncertain_threshold=DEFAULT_UNCERTAIN_THRESHOLD,
                                uncertain_margin=DEFAULT_UNCERTAIN_MARGIN):
    """Resolve the lower uncertainty boundary, preserving old margin callers."""
    threshold = float(occupied_threshold)
    if uncertain_margin is not None:
        return _clamp(threshold - max(0.0, float(uncertain_margin)), 0.0, threshold)
    return _clamp(float(uncertain_threshold), 0.0, threshold)


def classify_score(score, occupied_threshold=DEFAULT_OCCUPIED_THRESHOLD,
                   uncertain_margin=DEFAULT_UNCERTAIN_MARGIN,
                   uncertain_threshold=DEFAULT_UNCERTAIN_THRESHOLD):
    """Classify occupied, uncertain, or empty with explicit decision bands."""
    threshold = float(occupied_threshold)
    uncertain = resolve_uncertain_threshold(threshold, uncertain_threshold, uncertain_margin)
    if score >= threshold:
        confidence = (score - threshold) / max(1.0 - threshold, 1e-6)
        return STATE_OCCUPIED, _round_float(_clamp(confidence, 0.0, 1.0))
    if score < uncertain:
        confidence = (uncertain - score) / max(uncertain, 1e-6)
        return STATE_EMPTY, _round_float(_clamp(confidence, 0.0, 1.0))
    return STATE_UNCERTAIN, 0.0


def polygon_mask(image_shape, polygon):
    """Return a uint8 mask for a polygon in an image with shape (height, width)."""
    height, width = image_shape
    mask = np.zeros((height, width), dtype=np.uint8)
    points = _polygon_points(polygon)
    cv2.fillPoly(mask, [points], 255)
    return mask


def masked_crop(image, polygon, background=(245, 245, 245)):
    """Return the polygon crop bounding rectangle with outside-polygon pixels hidden."""
    points = _polygon_points(polygon)
    x, y, width, height = cv2.boundingRect(points)
    x = max(0, x)
    y = max(0, y)
    width = max(1, min(width, image.shape[1] - x))
    height = max(1, min(height, image.shape[0] - y))
    roi = image[y:y + height, x:x + width].copy()
    local_points = points.copy()
    local_points[:, 0] -= x
    local_points[:, 1] -= y
    local_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(local_mask, [local_points], 255)
    fill = np.zeros_like(roi)
    fill[:, :] = background
    fill[local_mask > 0] = roi[local_mask > 0]
    return fill


def save_result_json(result, output_path):
    parent = os.path.dirname(output_path)
    if parent:
        ensure_dir(parent)
    with open(output_path, "w") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
        handle.write("\n")


def save_debug_outputs(image_path, profile_path, result, output_dir):
    """Save overlay, contact sheet, and robot-black occupancy grid images."""
    ensure_dir(output_dir)
    image = load_image(image_path)
    profile = load_profile(profile_path)
    overlay_path = os.path.join(output_dir, "occupancy_crop_overlay.png")
    contact_path = os.path.join(output_dir, "square_crops_contact_sheet.png")
    grid_path = os.path.join(output_dir, "occupancy_grid.png")
    save_crop_overlay(image, profile, result, overlay_path)
    save_contact_sheet(image, profile, result, contact_path)
    save_occupancy_grid(result, grid_path)
    return {
        "occupancy_crop_overlay": overlay_path,
        "square_crops_contact_sheet": contact_path,
        "occupancy_grid": grid_path,
    }


def save_crop_overlay(image, profile, result, output_path):
    canvas = image.copy()
    for square in square_names():
        entry = result["squares"].get(square, {})
        state = entry.get("state", STATE_UNKNOWN)
        colour = _STATE_COLOURS.get(state, _STATE_COLOURS[STATE_UNKNOWN])
        points = _polygon_points(profile.square_crop_polygon(square))
        cv2.polylines(canvas, [points], True, colour, 2, cv2.LINE_AA)
        centre = profile.square_center(square)
        label = square
        cv2.putText(
            canvas,
            label,
            (int(round(centre[0])) - 12, int(round(centre[1])) + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            label,
            (int(round(centre[0])) - 12, int(round(centre[1])) + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )
    cv2.imwrite(output_path, canvas)


def save_contact_sheet(image, profile, result, output_path):
    cell_size = 112
    label_height = 24
    sheet = np.zeros((8 * (cell_size + label_height), 8 * cell_size, 3), dtype=np.uint8)
    sheet[:, :] = (245, 245, 245)

    for index, square in enumerate(square_names()):
        row = index // 8
        col = index % 8
        x0 = col * cell_size
        y0 = row * (cell_size + label_height)
        crop = masked_crop(image, profile.square_crop_polygon(square))
        resized = _resize_to_fit(crop, cell_size, cell_size)
        y_pad = y0 + label_height + (cell_size - resized.shape[0]) // 2
        x_pad = x0 + (cell_size - resized.shape[1]) // 2
        sheet[y_pad:y_pad + resized.shape[0], x_pad:x_pad + resized.shape[1]] = resized
        entry = result["squares"].get(square, {})
        symbol = _STATE_SYMBOLS.get(entry.get("state", STATE_UNKNOWN), "U")
        label = "{} {}".format(square, symbol)
        cv2.putText(sheet, label, (x0 + 6, y0 + 17), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, (30, 30, 30), 1, cv2.LINE_AA)
        cv2.rectangle(sheet, (x0, y0 + label_height),
                      (x0 + cell_size - 1, y0 + label_height + cell_size - 1),
                      (180, 180, 180), 1)
    cv2.imwrite(output_path, sheet)


def save_occupancy_grid(result, output_path):
    cell = 96
    header = 34
    width = cell * 8
    height = header + cell * 8
    grid = np.zeros((height, width, 3), dtype=np.uint8)
    grid[:, :] = (250, 250, 250)
    cv2.putText(grid, "Robot-black occupancy: top h1 -> a1, bottom h8 -> a8",
                (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)

    for index, square in enumerate(square_names()):
        row = index // 8
        col = index % 8
        x0 = col * cell
        y0 = header + row * cell
        entry = result["squares"].get(square, {})
        state = entry.get("state", STATE_UNKNOWN)
        colour = _STATE_COLOURS.get(state, _STATE_COLOURS[STATE_UNKNOWN])
        score = float(entry.get("score", 0.0))
        symbol = _STATE_SYMBOLS.get(state, "U")
        cv2.rectangle(grid, (x0, y0), (x0 + cell - 1, y0 + cell - 1), (225, 225, 225), -1)
        cv2.rectangle(grid, (x0 + 3, y0 + 3), (x0 + cell - 4, y0 + cell - 4), colour, 2)
        cv2.putText(grid, square, (x0 + 8, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (40, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(grid, symbol, (x0 + 34, y0 + 60), cv2.FONT_HERSHEY_SIMPLEX,
                    1.2, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(grid, "{:.2f}".format(score), (x0 + 23, y0 + 86), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (70, 70, 70), 1, cv2.LINE_AA)
    cv2.imwrite(output_path, grid)


def ensure_dir(path):
    if path and not os.path.isdir(path):
        os.makedirs(path)


def _polygon_points(polygon):
    return np.array([[int(round(point[0])), int(round(point[1]))] for point in polygon], dtype=np.int32)


def _resize_to_fit(image, max_width, max_height):
    height, width = image.shape[:2]
    scale = min(float(max_width) / float(width), float(max_height) / float(height))
    new_width = max(1, int(round(width * scale)))
    new_height = max(1, int(round(height * scale)))
    return cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)


def _round_float(value):
    return round(float(value), 6)


def _clamp(value, low, high):
    return max(low, min(high, float(value)))
