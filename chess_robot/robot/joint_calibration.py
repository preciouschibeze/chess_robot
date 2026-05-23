from __future__ import absolute_import

import math

import yaml


def load_joint_calibration(path):
    data = _load_yaml_mapping(path, "joint calibration")
    root = data.get("joint_calibration", data)
    if not isinstance(root, dict):
        raise ValueError("Joint calibration file must contain a mapping.")

    ticks_per_rev = int(root.get("ticks_per_rev", 4096))
    provisional = bool(root.get("provisional", False))
    joints_data = root.get("joints") or {}
    if not isinstance(joints_data, dict):
        raise ValueError("Joint calibration must contain a 'joints' mapping.")

    joints = {}
    urdf_to_user = {}
    joint_order = []
    for user_joint, raw_entry in joints_data.items():
        if not isinstance(raw_entry, dict):
            raise ValueError("Joint calibration entry for %s must be a mapping." % user_joint)
        urdf_joint = str(raw_entry.get("urdf_joint", user_joint))
        direction_sign = int(raw_entry.get("direction_sign", 1))
        zero_tick = raw_entry.get("zero_tick")
        if zero_tick is None:
            raise ValueError("Joint calibration entry for %s is missing zero_tick." % user_joint)
        zero_tick = int(zero_tick)
        if direction_sign not in (-1, 1):
            raise ValueError("direction_sign for %s must be 1 or -1." % user_joint)
        if urdf_joint in urdf_to_user:
            raise ValueError("Duplicate URDF joint mapping for %s." % urdf_joint)

        entry = {
            "user_joint": str(user_joint),
            "urdf_joint": urdf_joint,
            "direction_sign": direction_sign,
            "zero_tick": zero_tick,
        }
        joints[str(user_joint)] = entry
        urdf_to_user[urdf_joint] = str(user_joint)
        joint_order.append(str(user_joint))

    warnings = []
    if provisional:
        warnings.append("WARNING: joint calibration is marked provisional.")

    return {
        "ticks_per_rev": ticks_per_rev,
        "provisional": provisional,
        "joints": joints,
        "urdf_to_user": urdf_to_user,
        "joint_order": joint_order,
        "warnings": warnings,
    }


def load_pose_ticks(path):
    data = _load_yaml_mapping(path, "home pose")
    joints = data.get("joints") or data.get("pose_ticks") or {}
    if not isinstance(joints, dict):
        raise ValueError("Home pose file must contain a 'joints' mapping.")

    pose_ticks = {}
    for joint_name, raw_entry in joints.items():
        tick = _extract_tick_value(raw_entry)
        if tick is not None:
            pose_ticks[str(joint_name)] = tick
    return pose_ticks


def load_joint_limits(path):
    data = _load_yaml_mapping(path, "joint limits")
    limits = data.get("limits") or data.get("joints") or {}
    if not isinstance(limits, dict):
        raise ValueError("Joint limits file must contain a 'limits' mapping.")
    return limits


def tick_to_angle_deg(joint_name, tick, calibration):
    entry = get_calibration_entry(joint_name, calibration)
    angle_deg = (
        entry["direction_sign"]
        * (float(tick) - float(entry["zero_tick"]))
        * 360.0
        / float(calibration["ticks_per_rev"])
    )
    return angle_deg


def tick_to_angle_rad(joint_name, tick, calibration):
    return math.radians(tick_to_angle_deg(joint_name, tick, calibration))


def angle_rad_to_tick(joint_name, angle_rad, calibration):
    entry = get_calibration_entry(joint_name, calibration)
    angle_deg = math.degrees(float(angle_rad))
    tick = (
        float(entry["zero_tick"])
        + (float(entry["direction_sign"]) * angle_deg * float(calibration["ticks_per_rev"]) / 360.0)
    )
    return int(round(tick))


def convert_pose_ticks_to_urdf_radians(pose_ticks, calibration):
    converted = {}
    for user_joint in calibration["joint_order"]:
        entry = calibration["joints"][user_joint]
        tick = _lookup_tick_value(pose_ticks, user_joint, entry["urdf_joint"])
        if tick is None:
            continue
        converted[entry["urdf_joint"]] = tick_to_angle_rad(user_joint, tick, calibration)
    return converted


def convert_limits_ticks_to_angle_limits(joint_limits, calibration):
    converted = {}
    for user_joint in calibration["joint_order"]:
        entry = calibration["joints"][user_joint]
        limit_entry = _lookup_joint_entry(joint_limits, user_joint, entry["urdf_joint"])
        if not isinstance(limit_entry, dict):
            continue

        min_tick = _extract_named_int(limit_entry, ("provisional_min", "min", "lower"))
        max_tick = _extract_named_int(limit_entry, ("provisional_max", "max", "upper"))
        neutral_tick = _extract_named_int(limit_entry, ("neutral",))
        converted_entry = {
            "user_joint": user_joint,
            "urdf_joint": entry["urdf_joint"],
            "direction_sign": entry["direction_sign"],
            "zero_tick": entry["zero_tick"],
            "provisional_min_tick": min_tick,
            "provisional_max_tick": max_tick,
            "neutral_tick": neutral_tick,
        }

        if min_tick is not None:
            converted_entry["provisional_min_deg"] = tick_to_angle_deg(user_joint, min_tick, calibration)
            converted_entry["provisional_min_rad"] = tick_to_angle_rad(user_joint, min_tick, calibration)
        else:
            converted_entry["provisional_min_deg"] = None
            converted_entry["provisional_min_rad"] = None

        if max_tick is not None:
            converted_entry["provisional_max_deg"] = tick_to_angle_deg(user_joint, max_tick, calibration)
            converted_entry["provisional_max_rad"] = tick_to_angle_rad(user_joint, max_tick, calibration)
        else:
            converted_entry["provisional_max_deg"] = None
            converted_entry["provisional_max_rad"] = None

        if neutral_tick is not None:
            converted_entry["neutral_deg"] = tick_to_angle_deg(user_joint, neutral_tick, calibration)
            converted_entry["neutral_rad"] = tick_to_angle_rad(user_joint, neutral_tick, calibration)
        else:
            converted_entry["neutral_deg"] = None
            converted_entry["neutral_rad"] = None

        if min_tick is not None and max_tick is not None:
            lower_deg = min(converted_entry["provisional_min_deg"], converted_entry["provisional_max_deg"])
            upper_deg = max(converted_entry["provisional_min_deg"], converted_entry["provisional_max_deg"])
            lower_rad = min(converted_entry["provisional_min_rad"], converted_entry["provisional_max_rad"])
            upper_rad = max(converted_entry["provisional_min_rad"], converted_entry["provisional_max_rad"])
        else:
            lower_deg = None
            upper_deg = None
            lower_rad = None
            upper_rad = None

        converted_entry["lower_deg"] = lower_deg
        converted_entry["upper_deg"] = upper_deg
        converted_entry["lower_rad"] = lower_rad
        converted_entry["upper_rad"] = upper_rad
        converted[entry["urdf_joint"]] = converted_entry
    return converted


def get_calibration_entry(joint_name, calibration):
    if joint_name in calibration["joints"]:
        return calibration["joints"][joint_name]
    if joint_name in calibration["urdf_to_user"]:
        return calibration["joints"][calibration["urdf_to_user"][joint_name]]
    raise KeyError("Unknown calibrated joint: %s" % joint_name)


def _load_yaml_mapping(path, label):
    with open(path, "r") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("%s file must contain a YAML mapping: %s" % (label, path))
    return data


def _extract_tick_value(raw_entry):
    if isinstance(raw_entry, dict):
        for key in ("position", "tick", "ticks", "value"):
            if raw_entry.get(key) is not None:
                return int(raw_entry.get(key))
        return None
    if raw_entry is None:
        return None
    return int(raw_entry)


def _lookup_tick_value(mapping, primary_name, secondary_name):
    if primary_name in mapping:
        return _extract_tick_value(mapping.get(primary_name))
    if secondary_name in mapping:
        return _extract_tick_value(mapping.get(secondary_name))
    return None


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
