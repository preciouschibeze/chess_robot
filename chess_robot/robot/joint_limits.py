from __future__ import absolute_import

import math

import yaml

LEGACY_HARD_LIMITS_WARNING = "WARNING: using legacy joint_limits.yaml as hard safety limits."
UNCALIBRATED_SAFETY_LIMITS_WARNING = "WARNING: joint_safety_limits.yaml is marked uncalibrated."
PROFILE_KIND_LEGACY = "legacy"
PROFILE_KIND_SAFETY = "safety"


def load_joint_limits(path):
    return load_legacy_joint_limits(path)


def load_legacy_joint_limits(path):
    data = _load_yaml_mapping(path, "joint limits")
    limits = data.get("limits") or data.get("joints") or {}
    if not isinstance(limits, dict):
        raise ValueError("Joint limits file must contain a 'limits' mapping.")
    return limits


def load_joint_safety_limits(path):
    data = _load_yaml_mapping(path, "joint safety limits")
    root = data.get("safety_limits", data)
    if not isinstance(root, dict):
        raise ValueError("Joint safety limits file must contain a 'safety_limits' mapping.")

    joints = root.get("joints") or {}
    if not isinstance(joints, dict):
        raise ValueError("Joint safety limits must contain a 'joints' mapping.")

    normalized_joints = {}
    for joint_name, raw_entry in joints.items():
        if not isinstance(raw_entry, dict):
            raise ValueError("Joint safety limits entry for %s must be a mapping." % joint_name)
        normalized_joints[str(joint_name)] = _normalize_safety_limit_entry(raw_entry, joint_name)

    warnings = []
    if not bool(root.get("calibrated", False)):
        warnings.append(UNCALIBRATED_SAFETY_LIMITS_WARNING)

    return {
        "profile_kind": PROFILE_KIND_SAFETY,
        "path": path,
        "source": root.get("source"),
        "calibrated": bool(root.get("calibrated", False)),
        "notes": root.get("notes"),
        "joints": normalized_joints,
        "warnings": warnings,
    }


def load_joint_preferences(path):
    data = _load_yaml_mapping(path, "joint preferences")
    root = data.get("joint_preferences", data)
    if not isinstance(root, dict):
        raise ValueError("Joint preferences file must contain a 'joint_preferences' mapping.")

    joints = root.get("joints") or {}
    if not isinstance(joints, dict):
        raise ValueError("Joint preferences must contain a 'joints' mapping.")

    normalized_joints = {}
    for joint_name, raw_entry in joints.items():
        if not isinstance(raw_entry, dict):
            raise ValueError("Joint preferences entry for %s must be a mapping." % joint_name)
        normalized_joints[str(joint_name)] = _normalize_preference_entry(raw_entry, joint_name)

    return {
        "path": path,
        "source": root.get("source"),
        "notes": root.get("notes"),
        "joints": normalized_joints,
        "warnings": [],
    }


def convert_limits_ticks_to_angle_limits(joint_limits, calibration):
    profile_kind, joints = _resolve_limit_profile(joint_limits)
    converted = {}
    for user_joint in calibration["joint_order"]:
        calibration_entry = calibration["joints"][user_joint]
        limit_entry = _lookup_joint_entry(joints, user_joint, calibration_entry["urdf_joint"])
        if not isinstance(limit_entry, dict):
            continue

        min_tick = _extract_named_int(limit_entry, ("min_tick", "provisional_min", "min", "lower"))
        max_tick = _extract_named_int(limit_entry, ("max_tick", "provisional_max", "max", "upper"))
        neutral_tick = _extract_named_int(limit_entry, ("neutral_tick", "neutral"))
        margin_ticks = _extract_named_int(limit_entry, ("margin_ticks",))
        status = limit_entry.get("status")
        notes = limit_entry.get("notes")

        converted_entry = {
            "profile_kind": profile_kind,
            "user_joint": user_joint,
            "urdf_joint": calibration_entry["urdf_joint"],
            "direction_sign": calibration_entry["direction_sign"],
            "zero_tick": calibration_entry["zero_tick"],
            "min_tick": min_tick,
            "max_tick": max_tick,
            "neutral_tick": neutral_tick,
            "margin_ticks": margin_ticks,
            "status": status,
            "notes": notes,
            "provisional_min_tick": min_tick,
            "provisional_max_tick": max_tick,
        }

        _attach_angle_fields(
            converted_entry,
            calibration_entry,
            calibration,
            minimum_tick=min_tick,
            maximum_tick=max_tick,
            neutral_tick=neutral_tick,
        )
        converted[calibration_entry["urdf_joint"]] = converted_entry
    return converted


def convert_joint_safety_limits_to_angle_limits(joint_safety_limits, calibration):
    return convert_limits_ticks_to_angle_limits(joint_safety_limits, calibration)


def convert_joint_preferences_to_urdf_radians(joint_preferences, calibration):
    joints = joint_preferences.get("joints") or {}
    converted = {}
    for user_joint in calibration["joint_order"]:
        calibration_entry = calibration["joints"][user_joint]
        preference_entry = _lookup_joint_entry(joints, user_joint, calibration_entry["urdf_joint"])
        if not isinstance(preference_entry, dict):
            continue

        preferred_tick = _extract_named_int(preference_entry, ("preferred_tick", "neutral_tick", "neutral"))
        preferred_range_ticks = preference_entry.get("preferred_range_ticks")
        converted_entry = {
            "user_joint": user_joint,
            "urdf_joint": calibration_entry["urdf_joint"],
            "weight": float(preference_entry.get("weight", 0.0)),
            "preferred_tick": preferred_tick,
            "preferred_range_ticks": None,
        }

        if preferred_tick is not None:
            preferred_deg = _tick_to_angle_deg(calibration_entry, preferred_tick, calibration)
            converted_entry["preferred_deg"] = preferred_deg
            converted_entry["preferred_rad"] = math.radians(preferred_deg)
        else:
            converted_entry["preferred_deg"] = None
            converted_entry["preferred_rad"] = None

        if preferred_range_ticks is not None:
            lower_tick = int(preferred_range_ticks[0])
            upper_tick = int(preferred_range_ticks[1])
            lower_deg = _tick_to_angle_deg(calibration_entry, lower_tick, calibration)
            upper_deg = _tick_to_angle_deg(calibration_entry, upper_tick, calibration)
            lower_rad = math.radians(lower_deg)
            upper_rad = math.radians(upper_deg)
            converted_entry["preferred_range_ticks"] = [lower_tick, upper_tick]
            converted_entry["preferred_range_deg"] = [min(lower_deg, upper_deg), max(lower_deg, upper_deg)]
            converted_entry["preferred_range_rad"] = [min(lower_rad, upper_rad), max(lower_rad, upper_rad)]
        else:
            converted_entry["preferred_range_deg"] = None
            converted_entry["preferred_range_rad"] = None

        converted[calibration_entry["urdf_joint"]] = converted_entry
    return converted


def resolve_hard_limit_profile(joint_limits=None, joint_safety_limits=None):
    if joint_safety_limits is not None:
        return joint_safety_limits, PROFILE_KIND_SAFETY, list(joint_safety_limits.get("warnings", []))
    if joint_limits is None:
        raise ValueError("Hard software limit source requires joint limits or joint safety limits.")
    return joint_limits, PROFILE_KIND_LEGACY, [LEGACY_HARD_LIMITS_WARNING]


def _normalize_safety_limit_entry(raw_entry, joint_name):
    min_tick = _extract_named_int(raw_entry, ("min_tick",))
    max_tick = _extract_named_int(raw_entry, ("max_tick",))
    if min_tick is None:
        raise ValueError("Joint safety limits entry for %s is missing min_tick." % joint_name)
    if max_tick is None:
        raise ValueError("Joint safety limits entry for %s is missing max_tick." % joint_name)

    normalized_entry = {
        "min_tick": min_tick,
        "max_tick": max_tick,
        "neutral_tick": _extract_named_int(raw_entry, ("neutral_tick", "neutral")),
        "margin_ticks": _extract_named_int(raw_entry, ("margin_ticks",)),
        "status": raw_entry.get("status"),
        "notes": raw_entry.get("notes"),
    }
    return normalized_entry


def _normalize_preference_entry(raw_entry, joint_name):
    normalized_entry = {
        "preferred_tick": _extract_named_int(raw_entry, ("preferred_tick",)),
        "weight": float(raw_entry.get("weight", 0.0)),
    }
    preferred_range_ticks = raw_entry.get("preferred_range_ticks")
    if preferred_range_ticks is not None:
        if not isinstance(preferred_range_ticks, (list, tuple)) or len(preferred_range_ticks) != 2:
            raise ValueError(
                "Joint preferences entry for %s must use a two-value preferred_range_ticks sequence."
                % joint_name
            )
        normalized_entry["preferred_range_ticks"] = [
            int(preferred_range_ticks[0]),
            int(preferred_range_ticks[1]),
        ]
    else:
        normalized_entry["preferred_range_ticks"] = None
    return normalized_entry


def _resolve_limit_profile(joint_limits):
    if isinstance(joint_limits, dict) and isinstance(joint_limits.get("joints"), dict):
        profile_kind = str(joint_limits.get("profile_kind") or PROFILE_KIND_SAFETY)
        return profile_kind, joint_limits["joints"]
    return PROFILE_KIND_LEGACY, joint_limits


def _attach_angle_fields(
    converted_entry,
    calibration_entry,
    calibration,
    minimum_tick,
    maximum_tick,
    neutral_tick,
):
    if minimum_tick is not None:
        minimum_deg = _tick_to_angle_deg(calibration_entry, minimum_tick, calibration)
        minimum_rad = math.radians(minimum_deg)
    else:
        minimum_deg = None
        minimum_rad = None

    if maximum_tick is not None:
        maximum_deg = _tick_to_angle_deg(calibration_entry, maximum_tick, calibration)
        maximum_rad = math.radians(maximum_deg)
    else:
        maximum_deg = None
        maximum_rad = None

    if neutral_tick is not None:
        neutral_deg = _tick_to_angle_deg(calibration_entry, neutral_tick, calibration)
        neutral_rad = math.radians(neutral_deg)
    else:
        neutral_deg = None
        neutral_rad = None

    converted_entry["min_deg"] = minimum_deg
    converted_entry["max_deg"] = maximum_deg
    converted_entry["neutral_deg"] = neutral_deg
    converted_entry["min_rad"] = minimum_rad
    converted_entry["max_rad"] = maximum_rad
    converted_entry["neutral_rad"] = neutral_rad
    converted_entry["provisional_min_deg"] = minimum_deg
    converted_entry["provisional_max_deg"] = maximum_deg
    converted_entry["provisional_min_rad"] = minimum_rad
    converted_entry["provisional_max_rad"] = maximum_rad

    if minimum_deg is not None and maximum_deg is not None:
        converted_entry["lower_deg"] = min(minimum_deg, maximum_deg)
        converted_entry["upper_deg"] = max(minimum_deg, maximum_deg)
        converted_entry["lower_rad"] = min(minimum_rad, maximum_rad)
        converted_entry["upper_rad"] = max(minimum_rad, maximum_rad)
    else:
        converted_entry["lower_deg"] = None
        converted_entry["upper_deg"] = None
        converted_entry["lower_rad"] = None
        converted_entry["upper_rad"] = None


def _tick_to_angle_deg(calibration_entry, tick, calibration):
    return (
        float(calibration_entry["direction_sign"])
        * (float(tick) - float(calibration_entry["zero_tick"]))
        * 360.0
        / float(calibration["ticks_per_rev"])
    )


def _load_yaml_mapping(path, label):
    with open(path, "r") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("%s file must contain a YAML mapping: %s" % (label, path))
    return data


def _lookup_joint_entry(mapping, primary_name, secondary_name):
    if primary_name in mapping:
        return mapping.get(primary_name)
    if secondary_name in mapping:
        return mapping.get(secondary_name)
    return None


def _extract_named_int(mapping, names):
    for name in names:
        if mapping.get(name) is not None:
            return int(mapping.get(name))
    return None
