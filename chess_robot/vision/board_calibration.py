"""Board calibration helpers for seam-aware manual grid calibration."""

from chess_robot.calibration.board_profile import (
    BOARD_ORIENTATION,
    BOARD_PROFILE_VERSION,
    BOARD_SIZE,
    DEFAULT_CORNER_LABELS,
    GRID_SIZE,
    BoardProfile,
    grid_to_square,
)

DEFAULT_GRID_MODEL_FOUR_CORNER = "four_corner_bilinear"
DEFAULT_GRID_MODEL_MANUAL_9X9 = "manual_9x9"
DEFAULT_OCCUPANCY_CROP_FRACTION = 0.60
DEFAULT_SEAM_REGION_NAME = "centre_seam"


def build_board_profile(source_image_path, image_size, grid_points, crop_fraction,
                        grid_model, ignored_regions=None, version=None,
                        created_at=None, corner_labels=None):
    """Build a BoardProfile from calibrated grid geometry."""
    squares = build_square_entries(grid_points, crop_fraction)
    occupancy = {
        "crop_fraction": float(crop_fraction),
        "ignore_square_borders": True,
    }
    if ignored_regions:
        occupancy["ignore_regions"] = _region_names(ignored_regions)
    source_image = {
        "path": str(source_image_path),
        "size": [int(image_size[0]), int(image_size[1])],
    }
    return BoardProfile(
        board_orientation=BOARD_ORIENTATION,
        grid_points=grid_points,
        squares=squares,
        ignored_regions=ignored_regions or [],
        version=version if version is not None else BOARD_PROFILE_VERSION,
        created_at=created_at,
        source_image=source_image,
        corner_labels=corner_labels or dict(DEFAULT_CORNER_LABELS),
        grid_model=grid_model,
        occupancy=occupancy,
    )


def build_square_entries(grid_points, crop_fraction):
    """Return the 64 square definitions for a 9x9 grid."""
    crop_fraction = float(crop_fraction)
    if crop_fraction <= 0.0 or crop_fraction > 1.0:
        raise ValueError("crop_fraction must be within 0 and 1")
    squares = {}
    for row in range(BOARD_SIZE):
        for col in range(BOARD_SIZE):
            square_name = grid_to_square(row, col)
            polygon = square_polygon_from_grid(grid_points, row, col)
            centre = polygon_centre(polygon)
            crop_polygon = scale_polygon_towards_centre(polygon, crop_fraction)
            squares[square_name] = {
                "polygon": polygon,
                "centre": centre,
                "crop_polygon": crop_polygon,
            }
    return squares


def grid_points_from_manual_points(points):
    """Convert 81 row-major clicks into a 9x9 grid point matrix."""
    if len(points) != GRID_SIZE * GRID_SIZE:
        raise ValueError("manual 9x9 calibration needs exactly 81 points")
    grid_points = []
    index = 0
    for row in range(GRID_SIZE):
        row_points = []
        for col in range(GRID_SIZE):
            row_points.append(normalise_point(points[index]))
            index += 1
        grid_points.append(row_points)
    return grid_points


def grid_points_from_four_corners(corners):
    """Interpolate a 9x9 grid from four playable-board corners."""
    if len(corners) != 4:
        raise ValueError("four-corner calibration needs exactly 4 points")
    top_left = normalise_point(corners[0])
    top_right = normalise_point(corners[1])
    bottom_right = normalise_point(corners[2])
    bottom_left = normalise_point(corners[3])

    grid_points = []
    for row in range(GRID_SIZE):
        t = row / float(BOARD_SIZE)
        left = lerp_point(top_left, bottom_left, t)
        right = lerp_point(top_right, bottom_right, t)
        row_points = []
        for col in range(GRID_SIZE):
            s = col / float(BOARD_SIZE)
            row_points.append(lerp_point(left, right, s))
        grid_points.append(row_points)
    return grid_points


def square_polygon_from_grid(grid_points, row, col):
    """Return the four corners of one square in image order."""
    return [
        list(grid_points[row][col]),
        list(grid_points[row][col + 1]),
        list(grid_points[row + 1][col + 1]),
        list(grid_points[row + 1][col]),
    ]


def polygon_centre(polygon):
    """Return the centroid of a polygon represented as point lists."""
    total_x = 0.0
    total_y = 0.0
    for point in polygon:
        total_x += float(point[0])
        total_y += float(point[1])
    count = float(len(polygon))
    return [total_x / count, total_y / count]


def scale_polygon_towards_centre(polygon, scale):
    """Scale polygon vertices toward the centroid."""
    scale = float(scale)
    if scale <= 0.0 or scale > 1.0:
        raise ValueError("scale must be within 0 and 1")
    centre = polygon_centre(polygon)
    return [
        [centre[0] + (float(point[0]) - centre[0]) * scale,
         centre[1] + (float(point[1]) - centre[1]) * scale]
        for point in polygon
    ]


def lerp_point(a, b, t):
    """Linear interpolation between two points."""
    t = float(t)
    return [
        float(a[0]) + (float(b[0]) - float(a[0])) * t,
        float(a[1]) + (float(b[1]) - float(a[1])) * t,
    ]


def normalise_point(point):
    """Return a simple float [x, y] pair."""
    if isinstance(point, dict):
        if "x" not in point or "y" not in point:
            raise ValueError("point dict must contain x and y")
        point = [point["x"], point["y"]]
    if not isinstance(point, (list, tuple)) or len(point) != 2:
        raise ValueError("point must contain exactly two coordinates")
    try:
        return [float(point[0]), float(point[1])]
    except (TypeError, ValueError):
        raise ValueError("point must contain numeric coordinates")


def build_corner_labels():
    """Return the required robot-black corner labels."""
    return dict(DEFAULT_CORNER_LABELS)


def build_ignored_seam_region(points, name=DEFAULT_SEAM_REGION_NAME):
    """Wrap seam polygon points in the stored ignored-region format."""
    if not points:
        return None
    return {
        "name": name,
        "type": "polygon",
        "points": [normalise_point(point) for point in points],
    }


def _region_names(ignored_regions):
    names = []
    for region in ignored_regions:
        if isinstance(region, dict) and region.get("name"):
            names.append(str(region["name"]))
    return names
