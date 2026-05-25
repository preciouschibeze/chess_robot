from __future__ import absolute_import

import os

import yaml


APPROACH_POLICY_VERSION = 1
APPROACH_AXIS_NAMES = (
    "plus_x",
    "minus_x",
    "plus_y",
    "minus_y",
    "plus_z",
    "minus_z",
)
APPROACH_POLICY_FIELDS = (
    "prefer_vertical_approach",
    "enforce_approach_angle",
    "approach_axis_name",
    "approach_weight",
    "max_approach_tilt_deg",
    "max_edge_approach_tilt_deg",
    "normal_above_offset_m",
    "high_above_offset_m",
    "transit_clearance_m",
    "board_clearance_m",
    "lock_wrist_roll_home",
)

_BOOLEAN_FIELDS = (
    "prefer_vertical_approach",
    "enforce_approach_angle",
    "lock_wrist_roll_home",
)
_FLOAT_FIELDS = (
    "approach_weight",
    "max_approach_tilt_deg",
    "max_edge_approach_tilt_deg",
    "normal_above_offset_m",
    "high_above_offset_m",
    "transit_clearance_m",
    "board_clearance_m",
)


class ApproachPolicyError(ValueError):
    pass


def load_approach_policy(path):
    if not path:
        raise ApproachPolicyError("Approach policy path is required.")
    if not os.path.exists(path):
        raise ApproachPolicyError("Approach policy file was not found: %s" % path)

    try:
        with open(path, "r") as handle:
            document = yaml.safe_load(handle) or {}
    except yaml.YAMLError as exc:
        raise ApproachPolicyError("Failed to parse approach policy %s: %s" % (path, exc))
    except IOError as exc:
        raise ApproachPolicyError("Failed to read approach policy %s: %s" % (path, exc))

    if not isinstance(document, dict):
        raise ApproachPolicyError("Approach policy file must contain a top-level mapping.")

    root = document.get("approach_policy", document)
    if not isinstance(root, dict):
        raise ApproachPolicyError("Approach policy file must contain an 'approach_policy' mapping.")

    version = root.get("version")
    if int(version) != APPROACH_POLICY_VERSION:
        raise ApproachPolicyError(
            "Approach policy version must be %d, got %r."
            % (APPROACH_POLICY_VERSION, version)
        )

    default_policy = _normalize_policy_entry(
        root.get("default"),
        "approach policy default",
        require_all_fields=True,
    )

    overrides_root = root.get("overrides") or {}
    if not isinstance(overrides_root, dict):
        raise ApproachPolicyError("Approach policy overrides must be a mapping.")

    overrides = {}
    for square_name, raw_override in overrides_root.items():
        normalized_square = str(square_name).lower()
        overrides[normalized_square] = _normalize_policy_entry(
            raw_override,
            "approach policy override for %s" % normalized_square,
            require_all_fields=False,
        )

    return {
        "path": path,
        "version": APPROACH_POLICY_VERSION,
        "default": default_policy,
        "overrides": overrides,
    }


def resolve_approach_policy(policy_document, square):
    resolved_policy = dict(policy_document["default"])
    normalized_square = None
    override_applied = False

    if square is not None:
        normalized_square = str(square).lower()
        override = (policy_document.get("overrides") or {}).get(normalized_square)
        if override is not None:
            override_applied = True
            for field_name in APPROACH_POLICY_FIELDS:
                if field_name in override:
                    resolved_policy[field_name] = override[field_name]

    return {
        "path": policy_document["path"],
        "version": policy_document["version"],
        "square": normalized_square,
        "resolved_policy": resolved_policy,
        "policy_override_applied": override_applied,
    }


def apply_approach_policy(args, explicit_dests=None):
    explicit_dests = set(explicit_dests or [])
    policy_path = getattr(args, "approach_policy", None)
    if not policy_path:
        args.approach_policy_path = None
        args.approach_policy_square = None
        args.resolved_policy = None
        args.policy_override_applied = False
        return args

    policy_document = load_approach_policy(policy_path)
    policy_info = resolve_approach_policy(policy_document, getattr(args, "square", None))

    merged_policy = dict(policy_info["resolved_policy"])
    for field_name in APPROACH_POLICY_FIELDS:
        if field_name not in explicit_dests:
            setattr(args, field_name, merged_policy[field_name])

    args.approach_policy_path = policy_path
    args.approach_policy_square = policy_info["square"]
    args.resolved_policy = {}
    for field_name in APPROACH_POLICY_FIELDS:
        args.resolved_policy[field_name] = getattr(args, field_name)
    args.policy_override_applied = bool(policy_info["policy_override_applied"])
    return args


def _normalize_policy_entry(raw_entry, label, require_all_fields):
    if not isinstance(raw_entry, dict):
        raise ApproachPolicyError("%s must be a mapping." % label)

    normalized = {}
    for field_name in APPROACH_POLICY_FIELDS:
        if field_name not in raw_entry:
            if require_all_fields:
                raise ApproachPolicyError("%s is missing %s." % (label, field_name))
            continue
        value = raw_entry[field_name]
        if field_name in _BOOLEAN_FIELDS:
            if not isinstance(value, bool):
                raise ApproachPolicyError("%s field %s must be true or false." % (label, field_name))
            normalized[field_name] = bool(value)
        elif field_name in _FLOAT_FIELDS:
            if isinstance(value, bool):
                raise ApproachPolicyError("%s field %s must be numeric." % (label, field_name))
            normalized[field_name] = float(value)
        elif field_name == "approach_axis_name":
            axis_name = str(value)
            if axis_name not in APPROACH_AXIS_NAMES:
                raise ApproachPolicyError(
                    "%s field %s must be one of %s."
                    % (label, field_name, ", ".join(APPROACH_AXIS_NAMES))
                )
            normalized[field_name] = axis_name
    return normalized
