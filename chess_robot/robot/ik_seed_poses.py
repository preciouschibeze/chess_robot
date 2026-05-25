from __future__ import absolute_import

import copy
import os

import yaml

from chess_robot.robot.ik_validation import ARM_JOINTS
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians


IK_SEED_POSES_VERSION = 1
DEFAULT_SEED_SOURCE = "current_or_home"
DEFAULT_DOCUMENT_NOTES = [
    "Square-specific IK seed poses for difficult board squares.",
    "Seed poses are starting guesses for numerical IK, not final commanded targets.",
    "All final targets still require IK, joint safety checks, and path validation.",
]
DEFAULT_SQUARE_ENTRIES = {
    "a1": "Placeholder. Replace with manually taught extended posture.",
    "b1": "Placeholder. Replace with manually taught extended posture.",
    "h1": "Placeholder. Replace with manually taught extended posture.",
}


class IKSeedPoseError(ValueError):
    pass


def default_ik_seed_poses_document():
    squares = {}
    for square_name in sorted(DEFAULT_SQUARE_ENTRIES.keys()):
        squares[square_name] = {
            "notes": DEFAULT_SQUARE_ENTRIES[square_name],
            "seed_ticks": {},
        }
    return {
        "ik_seed_poses": {
            "version": IK_SEED_POSES_VERSION,
            "notes": list(DEFAULT_DOCUMENT_NOTES),
            "default": {
                "seed_source": DEFAULT_SEED_SOURCE,
            },
            "squares": squares,
        }
    }


def load_ik_seed_poses(path):
    if not path:
        raise IKSeedPoseError("IK seed pose path is required.")
    if not os.path.exists(path):
        raise IKSeedPoseError("IK seed pose file was not found: %s" % path)

    try:
        with open(path, "r") as handle:
            document = yaml.safe_load(handle) or {}
    except yaml.YAMLError as exc:
        raise IKSeedPoseError("Failed to parse IK seed poses %s: %s" % (path, exc))
    except IOError as exc:
        raise IKSeedPoseError("Failed to read IK seed poses %s: %s" % (path, exc))

    if not isinstance(document, dict):
        raise IKSeedPoseError("IK seed pose file must contain a top-level mapping.")

    root = document.get("ik_seed_poses", document)
    if not isinstance(root, dict):
        raise IKSeedPoseError("IK seed pose file must contain an 'ik_seed_poses' mapping.")

    version = root.get("version")
    if int(version) != IK_SEED_POSES_VERSION:
        raise IKSeedPoseError(
            "IK seed pose version must be %d, got %r."
            % (IK_SEED_POSES_VERSION, version)
        )

    notes = _normalize_notes_list(root.get("notes"), "ik_seed_poses notes")
    default_entry = _normalize_default_entry(root.get("default"))

    squares_root = root.get("squares") or {}
    if not isinstance(squares_root, dict):
        raise IKSeedPoseError("IK seed poses 'squares' entry must be a mapping.")

    squares = {}
    for square_name, raw_entry in squares_root.items():
        normalized_square = str(square_name).lower()
        squares[normalized_square] = _normalize_square_entry(
            raw_entry,
            "IK seed pose square %s" % normalized_square,
        )

    return {
        "path": path,
        "version": IK_SEED_POSES_VERSION,
        "notes": notes,
        "default": default_entry,
        "squares": squares,
    }


def resolve_square_ik_seed(document, square):
    normalized_square = None if square is None else str(square).lower()
    square_entry = None
    if normalized_square is not None:
        square_entry = (document.get("squares") or {}).get(normalized_square)
    if square_entry is None:
        square_entry = {
            "notes": None,
            "seed_ticks": {},
        }

    seed_ticks = dict(
        (joint_name, int(square_entry["seed_ticks"][joint_name]))
        for joint_name in sorted(square_entry.get("seed_ticks", {}).keys())
    )
    notes = _normalize_notes_list(square_entry.get("notes"), "IK seed pose notes for %s" % normalized_square)
    return {
        "path": document.get("path"),
        "square": normalized_square,
        "seed_source": str((document.get("default") or {}).get("seed_source") or DEFAULT_SEED_SOURCE),
        "seed_applied": bool(seed_ticks),
        "notes": notes,
        "seed_ticks": seed_ticks,
    }


def prepare_square_ik_seed(
        document,
        square,
        calibration,
        joint_safety_limits,
        locked_joint_positions_rad,
        locked_joint_ticks):
    resolved = resolve_square_ik_seed(document, square)
    prepared = dict(resolved)
    prepared["seed_ticks_used"] = {}
    prepared["seed_positions_rad_used"] = {}
    prepared["seed_joints_used"] = []

    if not resolved["seed_applied"]:
        return prepared

    validate_seed_ticks(resolved["square"], resolved["seed_ticks"], joint_safety_limits)
    seed_positions_rad = convert_pose_ticks_to_urdf_radians(resolved["seed_ticks"], calibration)
    missing = [joint_name for joint_name in sorted(resolved["seed_ticks"].keys()) if joint_name not in seed_positions_rad]
    if missing:
        raise IKSeedPoseError(
            "IK seed pose for %s could not be converted to radians for joints: %s"
            % (resolved["square"], ", ".join(missing))
        )

    seed_ticks_used, seed_positions_rad_used = apply_locked_joint_overrides(
        resolved["seed_ticks"],
        seed_positions_rad,
        locked_joint_positions_rad,
        locked_joint_ticks,
    )
    prepared["seed_ticks_used"] = seed_ticks_used
    prepared["seed_positions_rad_used"] = seed_positions_rad_used
    prepared["seed_joints_used"] = [joint_name for joint_name in ARM_JOINTS if joint_name in seed_ticks_used]
    return prepared


def validate_seed_ticks(square, seed_ticks, joint_safety_limits):
    if not isinstance(seed_ticks, dict):
        raise IKSeedPoseError("IK seed ticks for %s must be a mapping." % square)

    unknown = [joint_name for joint_name in sorted(seed_ticks.keys()) if joint_name not in ARM_JOINTS]
    if unknown:
        raise IKSeedPoseError(
            "IK seed pose for %s contains unknown joints: %s"
            % (square, ", ".join(unknown))
        )

    joints = (joint_safety_limits or {}).get("joints") or {}
    for joint_name in sorted(seed_ticks.keys()):
        tick_value = int(seed_ticks[joint_name])
        joint_limits = joints.get(joint_name)
        if not isinstance(joint_limits, dict):
            raise IKSeedPoseError("Joint safety limits are missing %s for IK seed validation." % joint_name)
        minimum = joint_limits.get("min_tick")
        maximum = joint_limits.get("max_tick")
        if minimum is None or maximum is None:
            raise IKSeedPoseError("Joint safety limits for %s are incomplete." % joint_name)
        minimum = int(minimum)
        maximum = int(maximum)
        if tick_value < minimum or tick_value > maximum:
            raise IKSeedPoseError(
                "IK seed pose for %s joint %s tick %d is outside joint safety limits %d..%d."
                % (square, joint_name, tick_value, minimum, maximum)
            )


def apply_locked_joint_overrides(seed_ticks, seed_positions_rad, locked_joint_positions_rad, locked_joint_ticks):
    seed_ticks_used = {}
    for joint_name in ARM_JOINTS:
        if joint_name in seed_ticks:
            seed_ticks_used[joint_name] = int(seed_ticks[joint_name])
    seed_positions_rad_used = {}
    for joint_name in ARM_JOINTS:
        if joint_name in seed_positions_rad:
            seed_positions_rad_used[joint_name] = float(seed_positions_rad[joint_name])

    for joint_name in ARM_JOINTS:
        if joint_name in (locked_joint_ticks or {}):
            seed_ticks_used[joint_name] = int(locked_joint_ticks[joint_name])
        if joint_name in (locked_joint_positions_rad or {}):
            seed_positions_rad_used[joint_name] = float(locked_joint_positions_rad[joint_name])
    return seed_ticks_used, seed_positions_rad_used


def upsert_square_seed_entry(document, square, seed_ticks, notes=None):
    normalized_square = str(square).lower()
    validate_seed_ticks(normalized_square, seed_ticks, {
        "joints": dict((joint_name, {"min_tick": -2147483648, "max_tick": 2147483647}) for joint_name in ARM_JOINTS)
    })

    if document is None:
        document = default_ik_seed_poses_document()
    else:
        document = copy.deepcopy(document)

    root = document.get("ik_seed_poses")
    if not isinstance(root, dict):
        raise IKSeedPoseError("IK seed pose document must contain an 'ik_seed_poses' mapping.")

    squares = root.get("squares")
    if not isinstance(squares, dict):
        squares = {}
        root["squares"] = squares

    existing_entry = squares.get(normalized_square)
    if not isinstance(existing_entry, dict):
        existing_entry = {
            "notes": DEFAULT_SQUARE_ENTRIES.get(normalized_square),
            "seed_ticks": {},
        }

    updated_entry = dict(existing_entry)
    if notes is not None:
        updated_entry["notes"] = str(notes)
    updated_entry["seed_ticks"] = dict(
        (joint_name, int(seed_ticks[joint_name]))
        for joint_name in ARM_JOINTS
        if joint_name in seed_ticks
    )
    squares[normalized_square] = updated_entry
    return document


def save_ik_seed_poses(path, document):
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w") as handle:
        yaml.safe_dump(document, handle, default_flow_style=False)


def load_or_default_ik_seed_poses(path):
    if path and os.path.exists(path):
        return load_ik_seed_poses(path)
    default_document = default_ik_seed_poses_document()
    root = default_document["ik_seed_poses"]
    return {
        "path": path,
        "version": IK_SEED_POSES_VERSION,
        "notes": list(root["notes"]),
        "default": dict(root["default"]),
        "squares": dict((square_name, dict(entry)) for square_name, entry in root["squares"].items()),
    }


def _normalize_default_entry(raw_entry):
    if raw_entry is None:
        return {"seed_source": DEFAULT_SEED_SOURCE}
    if not isinstance(raw_entry, dict):
        raise IKSeedPoseError("IK seed poses default entry must be a mapping.")
    seed_source = str(raw_entry.get("seed_source") or DEFAULT_SEED_SOURCE)
    return {"seed_source": seed_source}


def _normalize_square_entry(raw_entry, label):
    if not isinstance(raw_entry, dict):
        raise IKSeedPoseError("%s must be a mapping." % label)
    seed_ticks = raw_entry.get("seed_ticks")
    if seed_ticks is None:
        seed_ticks = {}
    if not isinstance(seed_ticks, dict):
        raise IKSeedPoseError("%s seed_ticks must be a mapping." % label)

    normalized_ticks = {}
    for joint_name, raw_value in seed_ticks.items():
        if isinstance(raw_value, bool):
            raise IKSeedPoseError("%s joint %s tick must be an integer." % (label, joint_name))
        try:
            normalized_ticks[str(joint_name)] = int(raw_value)
        except (TypeError, ValueError):
            raise IKSeedPoseError("%s joint %s tick must be an integer." % (label, joint_name))

    return {
        "notes": raw_entry.get("notes"),
        "seed_ticks": normalized_ticks,
    }


def _normalize_notes_list(raw_value, label):
    if raw_value is None:
        return []
    if isinstance(raw_value, list):
        return [str(item) for item in raw_value]
    if isinstance(raw_value, tuple):
        return [str(item) for item in raw_value]
    if isinstance(raw_value, str):
        return [raw_value]
    raise IKSeedPoseError("%s must be a string or list of strings." % label)
