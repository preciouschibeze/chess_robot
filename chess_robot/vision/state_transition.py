import re

VALID_STATES = set(["occupied", "empty", "uncertain"])
VALID_TRANSITION_TYPES = set([
    "no_change",
    "add",
    "remove",
    "move",
    "multi_change",
    "uncertain",
    "invalid",
])

BOARD_ORIENTATION = "robot_black_side"
_FILES_BY_COL = "hgfedcba"
_SQUARE_RE = re.compile(r"^[a-h][1-8]$")
_VALID_SQUARES = ["{}{}".format(file_name, rank) for rank in range(1, 9) for file_name in "abcdefgh"]
_VALID_SQUARES_SET = set(_VALID_SQUARES)


def grid_to_square(row, col):
    if row < 0 or row > 7 or col < 0 or col > 7:
        raise ValueError("grid row/col out of range: ({}, {})".format(row, col))
    file_name = _FILES_BY_COL[col]
    rank = row + 1
    return "{}{}".format(file_name, rank)


def _square_to_grid(square_name):
    if not isinstance(square_name, str) or not _SQUARE_RE.match(square_name):
        return None, None
    file_name = square_name[0]
    rank = int(square_name[1])
    col = _FILES_BY_COL.index(file_name)
    row = rank - 1
    return row, col


def _base_result():
    return {
        "schema_version": 1,
        "type": "occupancy_transition",
        "board_orientation": BOARD_ORIENTATION,
        "summary": {
            "changed_count": 0,
            "uncertain_count": 0,
            "transition_type": "no_change",
            "status": "clean",
        },
        "changed_squares": [],
        "removed_squares": [],
        "added_squares": [],
        "uncertain_squares": [],
        "notes": [],
    }


def _invalid_result(notes):
    result = _base_result()
    result["summary"]["transition_type"] = "invalid"
    result["summary"]["status"] = "invalid"
    result["notes"] = list(notes)
    return result


def _to_float_or_none(value):
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _extract_square_entry(square_name, entry):
    if isinstance(entry, str):
        state = entry
        confidence = None
        score = None
        row = None
        col = None
    elif isinstance(entry, dict):
        state = entry.get("state")
        confidence = _to_float_or_none(entry.get("confidence"))
        score = _to_float_or_none(entry.get("score"))
        row = entry.get("row")
        col = entry.get("col")
    else:
        return None, "square {} entry must be string or dict".format(square_name)

    if state not in VALID_STATES:
        return None, "square {} has invalid state '{}'".format(square_name, state)

    if row is not None:
        try:
            row = int(row)
        except (TypeError, ValueError):
            return None, "square {} has non-integer row".format(square_name)
    if col is not None:
        try:
            col = int(col)
        except (TypeError, ValueError):
            return None, "square {} has non-integer col".format(square_name)

    return {
        "state": state,
        "confidence": confidence,
        "score": score,
        "row": row,
        "col": col,
    }, None


def normalise_occupancy_snapshot(raw):
    if not isinstance(raw, dict):
        raise ValueError("snapshot must be a dict")

    notes = []
    orientation = raw.get("board_orientation")
    if orientation is None:
        orientation = BOARD_ORIENTATION
        notes.append("missing board_orientation; defaulted to '{}'".format(BOARD_ORIENTATION))
    elif orientation != BOARD_ORIENTATION:
        raise ValueError("board_orientation must be '{}'".format(BOARD_ORIENTATION))

    source = {
        "format": "normalized" if raw.get("type") == "occupancy_snapshot" else "occupancy_diagnostics",
    }
    for key in ["image_path", "profile_path", "empty_reference_path", "occupied_threshold", "uncertain_threshold", "uncertain_margin"]:
        if key in raw:
            source[key] = raw.get(key)

    squares = raw.get("squares")
    if not isinstance(squares, dict):
        raise ValueError("snapshot missing 'squares' dict")

    invalid_square_labels = sorted([square_name for square_name in squares.keys() if square_name not in _VALID_SQUARES_SET])
    if invalid_square_labels:
        raise ValueError("snapshot has invalid square labels: {}".format(invalid_square_labels))

    normalized_squares = {}
    for square_name in _VALID_SQUARES:
        if square_name not in squares:
            raise ValueError("snapshot missing square '{}'".format(square_name))
        parsed, error = _extract_square_entry(square_name, squares.get(square_name))
        if error:
            raise ValueError(error)

        row, col = _square_to_grid(square_name)
        normalized_squares[square_name] = {
            "row": parsed["row"] if parsed["row"] is not None else row,
            "col": parsed["col"] if parsed["col"] is not None else col,
            "state": parsed["state"],
            "score": parsed["score"],
            "confidence": parsed["confidence"],
        }

    snapshot = {
        "schema_version": 1,
        "type": "occupancy_snapshot",
        "board_orientation": orientation,
        "squares": normalized_squares,
        "source": source,
    }
    if notes:
        snapshot["notes"] = notes
    return snapshot


def _validate_snapshot(snapshot, label):
    errors = []

    try:
        normalized = normalise_occupancy_snapshot(snapshot)
    except Exception as exc:
        return None, ["{} snapshot invalid: {}".format(label, exc)]

    squares = normalized.get("squares", {})
    keys = set(squares.keys())
    missing = sorted(_VALID_SQUARES_SET - keys)
    extra = sorted(keys - _VALID_SQUARES_SET)
    if missing:
        errors.append("{} snapshot missing squares: {}".format(label, missing))
    if extra:
        errors.append("{} snapshot has extra squares: {}".format(label, extra))

    if errors:
        return None, errors
    return squares, []


def _classify_change(previous_state, current_state):
    if previous_state == current_state:
        return None
    if "uncertain" in [previous_state, current_state]:
        return "uncertain"
    if previous_state == "empty" and current_state == "occupied":
        return "added"
    if previous_state == "occupied" and current_state == "empty":
        return "removed"
    return "changed"


def _resolve_transition_type(added_count, removed_count, uncertain_count):
    if uncertain_count > 0:
        return "uncertain", "uncertain"

    clean_change_count = added_count + removed_count
    if clean_change_count == 0:
        return "no_change", "clean"
    if added_count == 1 and removed_count == 0:
        return "add", "clean"
    if added_count == 0 and removed_count == 1:
        return "remove", "clean"
    if added_count == 1 and removed_count == 1:
        return "move", "clean"
    return "multi_change", "uncertain"


def compare_occupancy_snapshots(previous, current):
    """Compare occupancy snapshots and return a deterministic transition result."""
    previous_squares, previous_errors = _validate_snapshot(previous, "previous")
    current_squares, current_errors = _validate_snapshot(current, "current")
    if previous_errors or current_errors:
        return _invalid_result(previous_errors + current_errors)

    result = _base_result()
    changed_entries = []
    added_squares = []
    removed_squares = []
    uncertain_squares = []

    for square_name in sorted(_VALID_SQUARES):
        previous_entry = previous_squares[square_name]
        current_entry = current_squares[square_name]
        change_type = _classify_change(previous_entry["state"], current_entry["state"])
        if change_type is None:
            continue

        row, col = _square_to_grid(square_name)
        entry = {
            "square": square_name,
            "row": row,
            "col": col,
            "previous_state": previous_entry["state"],
            "current_state": current_entry["state"],
            "previous_score": previous_entry.get("score"),
            "current_score": current_entry.get("score"),
            "change_type": change_type,
            "confidence": current_entry["confidence"],
        }
        changed_entries.append(entry)

        if change_type == "added":
            added_squares.append(square_name)
        elif change_type == "removed":
            removed_squares.append(square_name)
        elif change_type == "uncertain":
            uncertain_squares.append(square_name)

    transition_type, status = _resolve_transition_type(
        len(added_squares),
        len(removed_squares),
        len(uncertain_squares),
    )

    if transition_type == "multi_change":
        result["notes"].append(
            "multiple clean occupancy changes detected (added={}, removed={})".format(
                len(added_squares), len(removed_squares)
            )
        )
    if transition_type == "uncertain":
        result["notes"].append("one or more changed squares are uncertain")

    result["changed_squares"] = changed_entries
    result["removed_squares"] = sorted(removed_squares)
    result["added_squares"] = sorted(added_squares)
    result["uncertain_squares"] = sorted(uncertain_squares)
    result["summary"] = {
        "changed_count": len(changed_entries),
        "uncertain_count": len(uncertain_squares),
        "transition_type": transition_type,
        "status": status,
    }

    if result["summary"]["transition_type"] not in VALID_TRANSITION_TYPES:
        return _invalid_result(["computed invalid transition type"])

    return result
