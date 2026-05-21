"""Board calibration profile helpers.

The calibrated image grid is stored from the robot black side:
row 0 is the top image row, col 0 is the left image column.
"""

from pathlib import Path

try:
    import yaml
except ImportError:  # pragma: no cover - dependency note is raised on load/save.
    yaml = None

BOARD_ORIENTATION = "robot_black_side"
BOARD_PROFILE_VERSION = 1
DEFAULT_BOARD_PROFILE_PATH = Path("data/calibration/board/board_profile.yaml")
FILES_ROBOT_BLACK = "hgfedcba"
BOARD_SIZE = 8
GRID_SIZE = BOARD_SIZE + 1
DEFAULT_CORNER_LABELS = {
    "top_left": "h1",
    "top_right": "a1",
    "bottom_left": "h8",
    "bottom_right": "a8",
}


def grid_to_square(row, col):
    """Return the algebraic square for a robot-black image grid cell."""
    _validate_grid_index(row, "row")
    _validate_grid_index(col, "col")
    return "{}{}".format(FILES_ROBOT_BLACK[col], row + 1)


def square_names():
    """Return all 64 square names in image grid order."""
    return [grid_to_square(row, col) for row in range(BOARD_SIZE) for col in range(BOARD_SIZE)]


def square_to_grid(square_name):
    """Return (row, col) for a robot-black algebraic square name."""
    if not isinstance(square_name, str) or len(square_name) != 2:
        raise ValueError("square_name must be an algebraic square such as h1 or a8")
    file_name = square_name[0]
    rank_name = square_name[1]
    if file_name not in FILES_ROBOT_BLACK or rank_name not in "12345678":
        raise ValueError("square_name must be within a1 through h8")
    return int(rank_name) - 1, FILES_ROBOT_BLACK.index(file_name)


class BoardProfile:
    """YAML-backed board calibration profile for the fixed overhead board."""

    def __init__(self, board_orientation, grid_points, squares,
                 ignored_regions=None, source_path=None, metadata=None,
                 version=BOARD_PROFILE_VERSION, created_at=None,
                 source_image=None, corner_labels=None, grid_model=None,
                 occupancy=None):
        self.board_orientation = board_orientation
        self.grid_points = _normalise_grid_points(grid_points)
        self.squares = _normalise_squares(squares, self.grid_points)
        self.ignored_regions = _normalise_ignored_regions(ignored_regions)
        self.source_path = str(source_path) if source_path else None
        self.metadata = metadata or {}
        self.version = version if version is not None else BOARD_PROFILE_VERSION
        self.created_at = created_at
        self.source_image = _normalise_source_image(source_image)
        self.corner_labels = _normalise_corner_labels(corner_labels)
        self.grid_model = str(grid_model) if grid_model is not None else None
        self.occupancy = _normalise_occupancy(occupancy)
        self.validate()

    @classmethod
    def load(cls, path=DEFAULT_BOARD_PROFILE_PATH):
        """Load a board profile YAML file."""
        _require_yaml()
        path = Path(path)
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise ValueError("board profile YAML must contain a mapping")
        return cls(
            board_orientation=data.get("board_orientation"),
            grid_points=data.get("grid_points"),
            squares=data.get("squares"),
            ignored_regions=data.get("ignored_regions", []),
            source_path=path,
            metadata=data.get("metadata") if isinstance(data.get("metadata"), dict) else {},
            version=data.get("version", BOARD_PROFILE_VERSION),
            created_at=data.get("created_at"),
            source_image=data.get("source_image"),
            corner_labels=data.get("corner_labels"),
            grid_model=data.get("grid_model"),
            occupancy=data.get("occupancy"),
        )

    def save(self, path=DEFAULT_BOARD_PROFILE_PATH):
        """Validate and save this board profile as YAML."""
        _require_yaml()
        self.validate()
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(self.to_dict(), handle, default_flow_style=False)
        self.source_path = str(path)
        return path

    def validate(self):
        if self.board_orientation != BOARD_ORIENTATION:
            raise ValueError("board_orientation must be {!r}".format(BOARD_ORIENTATION))
        _normalise_grid_points(self.grid_points)
        _normalise_squares(self.squares, self.grid_points)
        _normalise_ignored_regions(self.ignored_regions)
        if self.corner_labels is not None:
            _normalise_corner_labels(self.corner_labels)
        if self.source_image is not None:
            _normalise_source_image(self.source_image)
        if self.occupancy is not None:
            _normalise_occupancy(self.occupancy)

    def to_dict(self):
        data = {
            "version": self.version if self.version is not None else BOARD_PROFILE_VERSION,
            "board_orientation": self.board_orientation,
            "grid_points": _grid_points_to_dict(self.grid_points),
            "squares": _copy_nested(self.squares),
            "ignored_regions": _copy_nested(self.ignored_regions),
        }
        if self.created_at is not None:
            data["created_at"] = self.created_at
        if self.source_image is not None:
            data["source_image"] = _copy_nested(self.source_image)
        if self.corner_labels is not None:
            data["corner_labels"] = _copy_nested(self.corner_labels)
        if self.grid_model is not None:
            data["grid_model"] = self.grid_model
        if self.occupancy is not None:
            data["occupancy"] = _copy_nested(self.occupancy)
        if self.metadata:
            data["metadata"] = _copy_nested(self.metadata)
        return data

    def square_polygon(self, square_name):
        return _copy_nested(self._square_entry(square_name)["polygon"])

    def square_centre(self, square_name):
        return list(self._square_entry(square_name)["centre"])

    def square_center(self, square_name):
        return self.square_centre(square_name)

    def centre(self, square_name):
        return self.square_centre(square_name)

    def center(self, square_name):
        return self.square_centre(square_name)

    def square_crop_polygon(self, square_name):
        return _copy_nested(self._square_entry(square_name)["crop_polygon"])

    def crop_polygon(self, square_name):
        return self.square_crop_polygon(square_name)

    def _square_entry(self, square_name):
        if square_name not in self.squares:
            raise ValueError("unknown square {!r}".format(square_name))
        return self.squares[square_name]


def _validate_grid_index(value, name):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} must be an integer from 0 to 7".format(name))
    if value < 0 or value >= BOARD_SIZE:
        raise ValueError("{} must be an integer from 0 to 7".format(name))


def _normalise_grid_points(grid_points):
    points = _extract_grid_point_rows(grid_points)
    if not isinstance(points, (list, tuple)) or len(points) != GRID_SIZE:
        raise ValueError("grid_points must contain 9 rows")
    normalised = []
    for row_index, row in enumerate(points):
        if not isinstance(row, (list, tuple)) or len(row) != GRID_SIZE:
            raise ValueError("grid_points row {} must contain 9 points".format(row_index))
        normalised.append([
            _normalise_point(point, "grid_points[{}][{}]".format(row_index, col_index))
            for col_index, point in enumerate(row)
        ])
    return normalised


def _extract_grid_point_rows(grid_points):
    if isinstance(grid_points, dict):
        if "points" not in grid_points:
            raise ValueError("grid_points mapping must contain points")
        return grid_points["points"]
    return grid_points


def _grid_points_to_dict(grid_points):
    return {
        "rows": GRID_SIZE,
        "cols": GRID_SIZE,
        "points": _copy_nested(grid_points),
    }


def _normalise_squares(squares, grid_points):
    if not isinstance(squares, dict):
        raise ValueError("squares must contain 64 square entries")
    expected = set(square_names())
    actual = set(squares.keys())
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        detail = []
        if missing:
            detail.append("missing {}".format(", ".join(missing)))
        if extra:
            detail.append("extra {}".format(", ".join(extra)))
        raise ValueError("squares must contain exactly 64 entries ({})".format("; ".join(detail)))

    normalised = {}
    for name in square_names():
        entry = squares[name]
        if entry is None:
            entry = {}
        if not isinstance(entry, dict):
            raise ValueError("squares[{!r}] must be a mapping".format(name))
        default_polygon = _polygon_from_grid(name, grid_points)
        polygon = _normalise_polygon(entry.get("polygon", default_polygon), "squares[{}].polygon".format(name))
        centre = _normalise_point(entry.get("centre", _polygon_centre(polygon)), "squares[{}].centre".format(name))
        crop_polygon = _normalise_polygon(
            entry.get("crop_polygon", polygon),
            "squares[{}].crop_polygon".format(name),
        )
        normalised[name] = {
            "polygon": polygon,
            "centre": centre,
            "crop_polygon": crop_polygon,
        }
    return normalised


def _normalise_ignored_regions(ignored_regions):
    if ignored_regions is None:
        return []
    if not isinstance(ignored_regions, (list, tuple)):
        raise ValueError("ignored_regions must be a list")
    normalised = []
    for index, region in enumerate(ignored_regions):
        if isinstance(region, dict):
            item = dict(region)
            polygon_key = None
            if "polygon" in item:
                polygon_key = "polygon"
            elif "points" in item:
                polygon_key = "points"
            if polygon_key is None:
                raise ValueError("ignored_regions[{}] must contain polygon or points".format(index))
            item[polygon_key] = _normalise_polygon(
                item[polygon_key],
                "ignored_regions[{}].{}".format(index, polygon_key),
            )
            normalised.append(item)
        else:
            normalised.append(_normalise_polygon(region, "ignored_regions[{}]".format(index)))
    return normalised


def _normalise_source_image(source_image):
    if source_image is None:
        return None
    if isinstance(source_image, (str, Path)):
        return {"path": str(source_image)}
    if not isinstance(source_image, dict):
        raise ValueError("source_image must be a mapping or path")
    if "path" not in source_image:
        raise ValueError("source_image must contain path")
    item = dict(source_image)
    item["path"] = str(item["path"])
    if "size" in item and item["size"] is not None:
        item["size"] = _normalise_image_size(item["size"], "source_image.size")
    return item


def _normalise_corner_labels(corner_labels):
    if corner_labels is None:
        return None
    if not isinstance(corner_labels, dict):
        raise ValueError("corner_labels must be a mapping")
    normalised = {}
    for key in ("top_left", "top_right", "bottom_left", "bottom_right"):
        if key not in corner_labels:
            raise ValueError("corner_labels must contain {}".format(key))
        value = corner_labels[key]
        if not isinstance(value, str) or not value:
            raise ValueError("corner_labels[{}] must be a non-empty string".format(key))
        normalised[key] = value
    return normalised


def _normalise_occupancy(occupancy):
    if occupancy is None:
        return None
    if not isinstance(occupancy, dict):
        raise ValueError("occupancy must be a mapping")
    item = dict(occupancy)
    if "crop_fraction" in item and item["crop_fraction"] is not None:
        try:
            crop_fraction = float(item["crop_fraction"])
        except (TypeError, ValueError):
            raise ValueError("occupancy.crop_fraction must be numeric")
        if crop_fraction <= 0.0 or crop_fraction > 1.0:
            raise ValueError("occupancy.crop_fraction must be within 0 and 1")
        item["crop_fraction"] = crop_fraction
    if "ignore_square_borders" in item and item["ignore_square_borders"] is not None:
        item["ignore_square_borders"] = bool(item["ignore_square_borders"])
    if "ignore_regions" in item and item["ignore_regions"] is not None:
        if not isinstance(item["ignore_regions"], (list, tuple)):
            raise ValueError("occupancy.ignore_regions must be a list")
        item["ignore_regions"] = [str(region) for region in item["ignore_regions"]]
    return item


def _polygon_from_grid(square_name, grid_points):
    row, col = square_to_grid(square_name)
    return [
        list(grid_points[row][col]),
        list(grid_points[row][col + 1]),
        list(grid_points[row + 1][col + 1]),
        list(grid_points[row + 1][col]),
    ]


def _polygon_centre(polygon):
    x_values = [point[0] for point in polygon]
    y_values = [point[1] for point in polygon]
    return [sum(x_values) / float(len(x_values)), sum(y_values) / float(len(y_values))]


def _normalise_polygon(value, field_name):
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        raise ValueError("{} must contain at least 3 points".format(field_name))
    return [_normalise_point(point, "{}[{}]".format(field_name, index)) for index, point in enumerate(value)]


def _normalise_point(value, field_name):
    if isinstance(value, dict):
        if "x" not in value or "y" not in value:
            raise ValueError("{} must contain x and y".format(field_name))
        value = [value["x"], value["y"]]
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("{} must be a 2D point".format(field_name))
    try:
        return [float(value[0]), float(value[1])]
    except (TypeError, ValueError):
        raise ValueError("{} must contain numeric coordinates".format(field_name))


def _normalise_image_size(value, field_name):
    if isinstance(value, dict):
        if "width" in value and "height" in value:
            value = [value["width"], value["height"]]
        else:
            raise ValueError("{} must contain width and height".format(field_name))
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("{} must be a 2D size".format(field_name))
    try:
        return [int(value[0]), int(value[1])]
    except (TypeError, ValueError):
        raise ValueError("{} must contain numeric dimensions".format(field_name))


def _copy_nested(value):
    if isinstance(value, dict):
        return {key: _copy_nested(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_copy_nested(item) for item in value]
    return value


def _require_yaml():
    if yaml is None:
        raise RuntimeError("PyYAML is required to load or save board_profile.yaml")
