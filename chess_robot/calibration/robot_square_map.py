from __future__ import absolute_import

import copy
import datetime
import math
import os
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - depends on runtime image
    yaml = None

BOARD_SIZE = 8
BOARD_ORIENTATION = "robot_black_side"
POSE_TYPE = "joint_ticks"
TARGETS_VERSION = 1
FILES_ROBOT_BLACK = "hgfedcba"
DEFAULT_TARGETS_PATH = os.path.join("data", "calibration", "robot", "square_targets.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join("data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_SERVO_MAP_PATH = os.path.join("data", "calibration", "robot", "servo_map.yaml")
DEFAULT_JOINT_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
RECOMMENDED_ANCHORS = [
    "a8", "c8", "f8", "h8",
    "a6", "c6", "f6", "h6",
    "a3", "c3", "f3", "h3",
    "a1", "c1", "f1", "h1",
]
MIN_MANUAL_ANCHORS_FOR_WRITE = 9
DEFAULT_GENERATED_NOTE = "requires manual validation before playback"
ALLOWED_POSE_NAMES = ["above_pose", "pick_pose", "place_pose"]


class SquareTargetError(ValueError):
    """Raised when square-target calibration data is invalid."""


def _utc_timestamp() -> str:
    return datetime.datetime.utcnow().replace(microsecond=0).isoformat() + "Z"


def _require_yaml() -> None:
    if yaml is None:
        raise RuntimeError("PyYAML is required to load or save square target calibration files.")


def _deepcopy(value: Any) -> Any:
    return copy.deepcopy(value)


def _validate_grid_index(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} must be an integer from 0 to 7".format(name))
    if value < 0 or value >= BOARD_SIZE:
        raise ValueError("{} must be an integer from 0 to 7".format(name))
    return value


def normalise_square_name(square: str) -> str:
    if not isinstance(square, str):
        raise ValueError("square must be a string like e4")
    value = square.strip().lower()
    if len(value) != 2:
        raise ValueError("square must be within a1 through h8")
    file_name = value[0]
    rank_name = value[1]
    if file_name not in FILES_ROBOT_BLACK or rank_name not in "12345678":
        raise ValueError("square must be within a1 through h8")
    return value


def grid_to_square(row: int, col: int) -> str:
    _validate_grid_index(row, "row")
    _validate_grid_index(col, "col")
    return "{}{}".format(FILES_ROBOT_BLACK[col], row + 1)


def square_to_grid(square: str) -> Tuple[int, int]:
    value = normalise_square_name(square)
    return int(value[1]) - 1, FILES_ROBOT_BLACK.index(value[0])


def square_names() -> List[str]:
    names = []
    for row in range(BOARD_SIZE):
        for col in range(BOARD_SIZE):
            names.append(grid_to_square(row, col))
    return names


def default_square_targets() -> Dict[str, Any]:
    return {
        "version": TARGETS_VERSION,
        "board_orientation": BOARD_ORIENTATION,
        "pose_type": POSE_TYPE,
        "joint_order": list(DEFAULT_JOINT_ORDER),
        "recommended_anchors": list(RECOMMENDED_ANCHORS),
        "metadata": {
            "created_by": "robot_square_calibration",
            "notes": [
                "above_pose only",
                "generated poses are joint-space interpolations and require manual validation",
            ],
        },
        "squares": {},
    }


def _normalise_joint_order(value: Any) -> List[str]:
    if not isinstance(value, list) or not value:
        return list(DEFAULT_JOINT_ORDER)
    result = []
    for entry in value:
        if isinstance(entry, str) and entry and entry not in result:
            result.append(entry)
    return result or list(DEFAULT_JOINT_ORDER)


def normalise_square_targets_document(data: Any) -> Dict[str, Any]:
    document = default_square_targets()
    if isinstance(data, dict):
        document.update(_deepcopy(data))
    document["version"] = int(document.get("version") or TARGETS_VERSION)
    document["board_orientation"] = document.get("board_orientation") or BOARD_ORIENTATION
    document["pose_type"] = document.get("pose_type") or POSE_TYPE
    document["joint_order"] = _normalise_joint_order(document.get("joint_order"))
    recommended = document.get("recommended_anchors")
    if not isinstance(recommended, list) or not recommended:
        document["recommended_anchors"] = list(RECOMMENDED_ANCHORS)
    else:
        cleaned = []
        for square in recommended:
            try:
                value = normalise_square_name(square)
            except ValueError:
                continue
            if value not in cleaned:
                cleaned.append(value)
        document["recommended_anchors"] = cleaned or list(RECOMMENDED_ANCHORS)
    metadata = document.get("metadata")
    if not isinstance(metadata, dict):
        metadata = {}
    notes = metadata.get("notes")
    if not isinstance(notes, list):
        notes = list(default_square_targets()["metadata"]["notes"])
    metadata["created_by"] = metadata.get("created_by") or "robot_square_calibration"
    metadata["notes"] = notes
    document["metadata"] = metadata
    squares = document.get("squares")
    document["squares"] = squares if isinstance(squares, dict) else {}
    return document


def load_yaml_file(path: str, default: Optional[Any] = None) -> Any:
    _require_yaml()
    if not path or not os.path.exists(path):
        return _deepcopy(default)
    with open(path, "r") as handle:
        data = yaml.safe_load(handle)
    if data is None:
        return _deepcopy(default)
    return data


def save_yaml_file(path: str, data: Any) -> str:
    _require_yaml()
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w") as handle:
        yaml.safe_dump(data, handle, default_flow_style=False)
    return path


def load_square_targets(path: str = DEFAULT_TARGETS_PATH) -> Dict[str, Any]:
    data = load_yaml_file(path, default_square_targets())
    return normalise_square_targets_document(data)


def load_joint_limits(path: str = DEFAULT_JOINT_LIMITS_PATH) -> Dict[str, Dict[str, int]]:
    data = load_yaml_file(path, {})
    limits = data.get("limits") if isinstance(data, dict) else None
    if not isinstance(limits, dict):
        raise SquareTargetError("joint_limits.yaml must contain a 'limits' mapping.")
    return limits


def load_servo_map(path: str = DEFAULT_SERVO_MAP_PATH) -> Dict[str, Any]:
    data = load_yaml_file(path, {})
    joints = data.get("joints") if isinstance(data, dict) else None
    if not isinstance(joints, dict):
        raise SquareTargetError("servo_map.yaml must contain a 'joints' mapping.")
    aliases = data.get("aliases")
    if aliases is not None and not isinstance(aliases, dict):
        raise SquareTargetError("servo_map.yaml 'aliases' must be a mapping when present.")
    return data


def _joint_limit_bounds(limit_entry: Dict[str, Any], joint_name: str) -> Tuple[int, int]:
    if not isinstance(limit_entry, dict):
        raise SquareTargetError("joint_limits entry for {} must be a mapping.".format(joint_name))
    lower = limit_entry.get("provisional_min")
    upper = limit_entry.get("provisional_max")
    if isinstance(lower, bool) or not isinstance(lower, int):
        raise SquareTargetError("joint_limits entry for {} is missing provisional_min.".format(joint_name))
    if isinstance(upper, bool) or not isinstance(upper, int):
        raise SquareTargetError("joint_limits entry for {} is missing provisional_max.".format(joint_name))
    return lower, upper


def validate_pose_joints(joints: Any, joint_limits: Dict[str, Dict[str, int]],
                         joint_order: Optional[List[str]] = None) -> List[str]:
    order = list(joint_order or DEFAULT_JOINT_ORDER)
    issues = []
    if not isinstance(joints, dict):
        return ["joints must be a mapping"]
    for joint_name in order:
        if joint_name not in joints:
            issues.append("missing joint {}".format(joint_name))
    for joint_name in sorted(joints.keys()):
        if joint_name not in order:
            issues.append("unknown joint {}".format(joint_name))
            continue
        value = joints.get(joint_name)
        if isinstance(value, bool) or not isinstance(value, int):
            issues.append("non-integer value for {}".format(joint_name))
            continue
        limit_entry = joint_limits.get(joint_name)
        if limit_entry is None:
            issues.append("missing joint_limits entry for {}".format(joint_name))
            continue
        try:
            lower, upper = _joint_limit_bounds(limit_entry, joint_name)
        except SquareTargetError as exc:
            issues.append(str(exc))
            continue
        if value < lower:
            issues.append("{} below software min {}".format(joint_name, lower))
        if value > upper:
            issues.append("{} above software max {}".format(joint_name, upper))
    return issues


def validate_square_targets_document(document: Dict[str, Any],
                                     joint_limits: Dict[str, Dict[str, int]]) -> Dict[str, List[str]]:
    doc = normalise_square_targets_document(document)
    issues_by_square = {}
    joint_order = doc.get("joint_order") or list(DEFAULT_JOINT_ORDER)
    for square_name, square_info in sorted(doc.get("squares", {}).items()):
        if not isinstance(square_info, dict):
            issues_by_square[square_name] = ["square entry must be a mapping"]
            continue
        above_pose = square_info.get("above_pose")
        if above_pose is None:
            continue
        if not isinstance(above_pose, dict):
            issues_by_square[square_name] = ["above_pose must be a mapping"]
            continue
        pose_issues = validate_pose_joints(above_pose.get("joints"), joint_limits, joint_order)
        if pose_issues:
            issues_by_square[square_name] = pose_issues
    return issues_by_square


def count_pose_sources(document: Dict[str, Any]) -> Dict[str, int]:
    counts = {"manual": 0, "generated": 0, "other": 0}
    doc = normalise_square_targets_document(document)
    for square_info in doc.get("squares", {}).values():
        if not isinstance(square_info, dict):
            continue
        above_pose = square_info.get("above_pose")
        if not isinstance(above_pose, dict):
            continue
        source = above_pose.get("source")
        if source in counts:
            counts[source] += 1
        else:
            counts["other"] += 1
    return counts


def validate_pose_name(pose_name: str) -> str:
    if pose_name not in ALLOWED_POSE_NAMES:
        raise SquareTargetError(
            "pose_name must be one of: {}".format(", ".join(ALLOWED_POSE_NAMES))
        )
    return pose_name


def _normalise_notes(notes: Optional[Any] = None, note: Optional[str] = None) -> List[str]:
    result = []
    if notes is not None:
        if isinstance(notes, list):
            result.extend([str(item) for item in notes])
        else:
            result.append(str(notes))
    if note:
        result.append(str(note))
    return result


def build_manual_pose_entry(joints: Dict[str, int], notes: Optional[Any] = None,
                            timestamp: Optional[str] = None,
                            note: Optional[str] = None) -> Dict[str, Any]:
    return {
        "source": "manual",
        "confidence": "taught",
        "joints": dict(joints),
        "recorded_at": timestamp or _utc_timestamp(),
        "notes": _normalise_notes(notes=notes, note=note),
    }


def build_manual_above_pose(joints: Dict[str, int], note: Optional[str] = None,
                            recorded_at: Optional[str] = None) -> Dict[str, Any]:
    return build_manual_pose_entry(joints, note=note, timestamp=recorded_at)


def upsert_manual_pose(document: Dict[str, Any], square: str, pose_name: str,
                       joints: Dict[str, int], notes: Optional[Any] = None,
                       force: bool = False,
                       recorded_at: Optional[str] = None,
                       note: Optional[str] = None) -> Dict[str, Any]:
    pose_key = validate_pose_name(pose_name)
    doc = normalise_square_targets_document(document)
    square_name = normalise_square_name(square)
    square_info = doc.get("squares", {}).get(square_name)
    if square_info is None:
        square_info = {}
        doc["squares"][square_name] = square_info
    elif not isinstance(square_info, dict):
        raise SquareTargetError("square entry for {} must be a mapping".format(square_name))
    existing = square_info.get(pose_key)
    if isinstance(existing, dict) and existing.get("source") == "manual" and not force:
        raise SquareTargetError(
            "square {} already has a manual {}; use --force to replace it.".format(square_name, pose_key)
        )
    square_info[pose_key] = build_manual_pose_entry(
        joints,
        notes=notes,
        note=note,
        timestamp=recorded_at,
    )
    return doc


def upsert_manual_above_pose(document: Dict[str, Any], square: str, joints: Dict[str, int],
                             note: Optional[str] = None, force: bool = False,
                             recorded_at: Optional[str] = None) -> Dict[str, Any]:
    return upsert_manual_pose(
        document,
        square,
        "above_pose",
        joints,
        note=note,
        force=force,
        recorded_at=recorded_at,
    )


def _coerce_anchor_joints(joints: Any, joint_order: List[str]) -> Optional[Dict[str, int]]:
    if not isinstance(joints, dict):
        return None
    result = {}
    for joint_name in joint_order:
        value = joints.get(joint_name)
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        result[joint_name] = int(value)
    return result


def collect_manual_anchor_squares(document: Dict[str, Any],
                                  joint_order: Optional[List[str]] = None) -> List[str]:
    doc = normalise_square_targets_document(document)
    order = list(joint_order or doc.get("joint_order") or DEFAULT_JOINT_ORDER)
    anchors = []
    for square_name in square_names():
        square_info = doc.get("squares", {}).get(square_name, {})
        if not isinstance(square_info, dict):
            continue
        above_pose = square_info.get("above_pose")
        if not isinstance(above_pose, dict) or above_pose.get("source") != "manual":
            continue
        if _coerce_anchor_joints(above_pose.get("joints"), order) is None:
            continue
        anchors.append(square_name)
    return anchors


def missing_recommended_anchors(document: Dict[str, Any]) -> List[str]:
    doc = normalise_square_targets_document(document)
    anchors = set(collect_manual_anchor_squares(doc, doc.get("joint_order")))
    return [square for square in doc.get("recommended_anchors", RECOMMENDED_ANCHORS) if square not in anchors]


def _collect_anchor_payloads(document: Dict[str, Any], joint_order: List[str],
                             allow_generated_anchors: bool = False) -> Tuple[Dict[str, Dict[str, int]], List[str]]:
    doc = normalise_square_targets_document(document)
    payloads = {}
    skipped = []
    for square_name, square_info in doc.get("squares", {}).items():
        if not isinstance(square_info, dict):
            continue
        above_pose = square_info.get("above_pose")
        if not isinstance(above_pose, dict):
            continue
        source = above_pose.get("source")
        if source != "manual" and not (allow_generated_anchors and source == "generated"):
            continue
        joints = _coerce_anchor_joints(above_pose.get("joints"), joint_order)
        if joints is None:
            skipped.append(square_name)
            continue
        payloads[normalise_square_name(square_name)] = joints
    return payloads, sorted(skipped)


def _distance(row_a: int, col_a: int, row_b: int, col_b: int) -> float:
    return math.sqrt(float((row_a - row_b) ** 2 + (col_a - col_b) ** 2))


def interpolate_square_pose(square: str, anchor_payloads: Dict[str, Dict[str, int]],
                            joint_order: List[str], method: str = "idw") -> Tuple[Dict[str, int], List[str]]:
    if method != "idw":
        raise SquareTargetError("Unsupported interpolation method: {}".format(method))
    square_name = normalise_square_name(square)
    target_row, target_col = square_to_grid(square_name)
    if square_name in anchor_payloads:
        return dict(anchor_payloads[square_name]), [square_name]
    distances = []
    for anchor_square, joints in anchor_payloads.items():
        row, col = square_to_grid(anchor_square)
        distances.append((_distance(target_row, target_col, row, col), anchor_square, joints))
    if not distances:
        raise SquareTargetError("No anchor poses available for interpolation.")
    distances.sort(key=lambda item: (item[0], item[1]))
    selected = distances[:4]
    interpolated = {}
    for joint_name in joint_order:
        numerator = 0.0
        denominator = 0.0
        for distance_value, _, joints in selected:
            if distance_value == 0.0:
                interpolated[joint_name] = int(joints[joint_name])
                denominator = 1.0
                numerator = float(joints[joint_name])
                break
            weight = 1.0 / (distance_value ** 2)
            numerator += float(joints[joint_name]) * weight
            denominator += weight
        if denominator == 0.0:
            raise SquareTargetError("Interpolation denominator was zero for {}.".format(square_name))
        interpolated[joint_name] = int(round(numerator / denominator))
    return interpolated, [item[1] for item in selected]


def _confidence_label(manual_anchor_count: int, missing_recommended_count: int) -> str:
    if manual_anchor_count >= len(RECOMMENDED_ANCHORS) and missing_recommended_count == 0:
        return "high"
    if manual_anchor_count >= MIN_MANUAL_ANCHORS_FOR_WRITE:
        return "medium"
    return "low"


def generate_square_targets(document: Dict[str, Any], joint_limits: Dict[str, Dict[str, int]],
                            method: str = "idw", allow_generated_anchors: bool = False,
                            generated_at: Optional[str] = None) -> Dict[str, Any]:
    doc = normalise_square_targets_document(document)
    joint_order = list(doc.get("joint_order") or DEFAULT_JOINT_ORDER)
    generated_timestamp = generated_at or _utc_timestamp()
    anchor_payloads, skipped_invalid_anchors = _collect_anchor_payloads(
        doc,
        joint_order,
        allow_generated_anchors=allow_generated_anchors,
    )
    manual_anchor_squares = []
    for square_name in sorted(anchor_payloads.keys()):
        square_info = doc.get("squares", {}).get(square_name, {})
        above_pose = square_info.get("above_pose") if isinstance(square_info, dict) else {}
        if isinstance(above_pose, dict) and above_pose.get("source") == "manual":
            manual_anchor_squares.append(square_name)
    missing_recommended = [
        square for square in doc.get("recommended_anchors", RECOMMENDED_ANCHORS)
        if square not in manual_anchor_squares
    ]
    warnings = []
    if missing_recommended:
        warnings.append(
            "Missing recommended anchors: {}".format(", ".join(missing_recommended))
        )
    if len(manual_anchor_squares) < len(doc.get("recommended_anchors", RECOMMENDED_ANCHORS)):
        warnings.append("Interpolation confidence is lower because fewer than 16 recommended anchors are available.")
    if len(manual_anchor_squares) < MIN_MANUAL_ANCHORS_FOR_WRITE:
        warnings.append("Fewer than 9 manual anchors are available; --write must be refused.")
    if skipped_invalid_anchors:
        warnings.append(
            "Skipped invalid anchor poses for interpolation: {}".format(", ".join(skipped_invalid_anchors))
        )

    updated = _deepcopy(doc)
    generated_validation_errors = {}
    all_validation_errors = {}
    generated_squares = []
    skipped_manual_count = 0
    confidence = _confidence_label(len(manual_anchor_squares), len(missing_recommended))

    for square_name in square_names():
        square_info = updated.get("squares", {}).get(square_name)
        if square_info is None:
            square_info = {}
            updated["squares"][square_name] = square_info
        elif not isinstance(square_info, dict):
            square_info = {}
            updated["squares"][square_name] = square_info
        existing_pose = square_info.get("above_pose")
        if isinstance(existing_pose, dict) and existing_pose.get("source") == "manual":
            skipped_manual_count += 1
            pose_issues = validate_pose_joints(existing_pose.get("joints"), joint_limits, joint_order)
            if pose_issues:
                all_validation_errors[square_name] = pose_issues
            continue
        if not anchor_payloads:
            continue
        joints, anchors_used = interpolate_square_pose(square_name, anchor_payloads, joint_order, method=method)
        generated_pose = {
            "source": "generated",
            "confidence": confidence,
            "interpolation": {
                "method": "joint_space_idw",
                "anchors_used": anchors_used,
            },
            "joints": joints,
            "generated_at": generated_timestamp,
            "notes": [DEFAULT_GENERATED_NOTE],
        }
        square_info["above_pose"] = generated_pose
        generated_squares.append(square_name)
        pose_issues = validate_pose_joints(joints, joint_limits, joint_order)
        if pose_issues:
            generated_validation_errors[square_name] = pose_issues
            all_validation_errors[square_name] = pose_issues

    write_ready = len(manual_anchor_squares) >= MIN_MANUAL_ANCHORS_FOR_WRITE and not generated_validation_errors
    return {
        "data": updated,
        "manual_anchor_squares": manual_anchor_squares,
        "manual_anchor_count": len(manual_anchor_squares),
        "missing_recommended_anchors": missing_recommended,
        "generated_squares": generated_squares,
        "generated_count": len(generated_squares),
        "skipped_manual_count": skipped_manual_count,
        "skipped_invalid_anchor_squares": skipped_invalid_anchors,
        "generated_validation_errors": generated_validation_errors,
        "all_validation_errors": all_validation_errors,
        "warnings": warnings,
        "write_ready": write_ready,
        "confidence": confidence,
    }


def square_status_rows(document: Dict[str, Any], joint_limits: Dict[str, Dict[str, int]]) -> List[Dict[str, Any]]:
    doc = normalise_square_targets_document(document)
    joint_order = list(doc.get("joint_order") or DEFAULT_JOINT_ORDER)
    validation = validate_square_targets_document(doc, joint_limits)
    rows = []
    for square_name in square_names():
        square_info = doc.get("squares", {}).get(square_name, {})
        source = None
        if isinstance(square_info, dict):
            above_pose = square_info.get("above_pose")
            if isinstance(above_pose, dict):
                source = above_pose.get("source") or "unknown"
        rows.append({
            "square": square_name,
            "source": source,
            "issues": validation.get(square_name, []),
            "joint_order": joint_order,
        })
    return rows
