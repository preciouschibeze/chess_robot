from __future__ import absolute_import

import argparse
import copy
import os
import shutil
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_limits import convert_joint_safety_limits_to_angle_limits
from chess_robot.robot.joint_limits import load_joint_safety_limits

CALIBRATION_METHOD = "midpoint_of_joint_safety_limits"
ZERO_TICK_SOURCE = "midpoint_of_joint_safety_limits"
ZERO_TICK_NOTE = "Provisional zero tick estimated from midpoint of broad safety limits."
DEFAULT_JOINT_CALIBRATION_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
DEFAULT_JOINT_SAFETY_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Recompute provisional joint_calibration zero_tick values from joint_safety_limits midpoints."
    )
    parser.add_argument(
        "--joint-calibration",
        default=DEFAULT_JOINT_CALIBRATION_PATH,
        help="Source joint_calibration.yaml path.",
    )
    parser.add_argument(
        "--joint-safety-limits",
        default=DEFAULT_JOINT_SAFETY_LIMITS_PATH,
        help="Source joint_safety_limits.yaml path.",
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_JOINT_CALIBRATION_PATH,
        help="Output joint_calibration.yaml path.",
    )
    parser.add_argument(
        "--backup",
        required=True,
        help="Backup path for the original joint_calibration.yaml.",
    )
    return parser


def compute_midpoint_zero_tick(min_tick, max_tick):
    return int(round((int(min_tick) + int(max_tick)) / 2.0))


def build_midpoint_zero_calibration_document(joint_calibration_path, joint_safety_limits_path):
    calibration_document = _load_yaml_mapping(joint_calibration_path, "joint calibration")
    calibration_root = calibration_document.get("joint_calibration", calibration_document)
    if not isinstance(calibration_root, dict):
        raise ValueError("Joint calibration file must contain a 'joint_calibration' mapping.")

    joints = calibration_root.get("joints") or {}
    if not isinstance(joints, dict):
        raise ValueError("Joint calibration file must contain a 'joints' mapping.")

    joint_safety_limits = load_joint_safety_limits(joint_safety_limits_path)
    safety_joints = joint_safety_limits.get("joints") or {}

    updated_root = copy.deepcopy(calibration_root)
    updated_root["provisional"] = True
    updated_root["calibration_method"] = CALIBRATION_METHOD
    updated_joints = updated_root.get("joints") or {}

    changes = []
    for user_joint, raw_joint_entry in joints.items():
        if not isinstance(raw_joint_entry, dict):
            raise ValueError("Joint calibration entry for %s must be a mapping." % user_joint)

        urdf_joint = str(raw_joint_entry.get("urdf_joint", user_joint))
        safety_entry = safety_joints.get(urdf_joint)
        if not isinstance(safety_entry, dict):
            raise ValueError(
                "Missing joint_safety_limits entry for calibrated URDF joint %s." % urdf_joint
            )

        old_zero_tick = raw_joint_entry.get("zero_tick")
        if old_zero_tick is None:
            raise ValueError("Joint calibration entry for %s is missing zero_tick." % user_joint)

        midpoint_zero_tick = compute_midpoint_zero_tick(
            safety_entry["min_tick"],
            safety_entry["max_tick"],
        )
        updated_entry = copy.deepcopy(raw_joint_entry)
        updated_entry["urdf_joint"] = urdf_joint
        updated_entry["direction_sign"] = int(updated_entry.get("direction_sign", 1))
        updated_entry["zero_tick"] = int(midpoint_zero_tick)
        updated_entry["zero_tick_source"] = ZERO_TICK_SOURCE
        updated_entry["zero_tick_note"] = ZERO_TICK_NOTE
        updated_joints[str(user_joint)] = updated_entry

        changes.append(
            {
                "user_joint": str(user_joint),
                "urdf_joint": urdf_joint,
                "old_zero_tick": int(old_zero_tick),
                "new_zero_tick": int(midpoint_zero_tick),
                "delta_ticks": int(midpoint_zero_tick) - int(old_zero_tick),
                "min_tick": int(safety_entry["min_tick"]),
                "max_tick": int(safety_entry["max_tick"]),
            }
        )

    updated_root["joints"] = updated_joints
    return {"joint_calibration": updated_root}, changes


def apply_midpoint_zero_calibration(
    joint_calibration_path,
    joint_safety_limits_path,
    output_path,
    backup_path,
):
    source_path = os.path.abspath(joint_calibration_path)
    output_path = os.path.abspath(output_path)
    backup_path = os.path.abspath(backup_path)
    if backup_path == source_path or backup_path == output_path:
        raise ValueError("Backup path must be distinct from the source and output paths.")

    updated_document, changes = build_midpoint_zero_calibration_document(
        joint_calibration_path,
        joint_safety_limits_path,
    )
    _ensure_parent_dir(backup_path)
    shutil.copyfile(source_path, backup_path)

    _ensure_parent_dir(output_path)
    _write_yaml_mapping(output_path, updated_document)

    updated_calibration = load_joint_calibration(output_path)
    joint_safety_limits = load_joint_safety_limits(joint_safety_limits_path)
    converted_safety_limits = convert_joint_safety_limits_to_angle_limits(
        joint_safety_limits,
        updated_calibration,
    )
    return updated_document, changes, converted_safety_limits


def main(argv=None):
    args = build_parser().parse_args(argv)
    _, changes, converted_safety_limits = apply_midpoint_zero_calibration(
        joint_calibration_path=args.joint_calibration,
        joint_safety_limits_path=args.joint_safety_limits,
        output_path=args.output,
        backup_path=args.backup,
    )

    print("Calibration method: %s" % CALIBRATION_METHOD)
    print("Backup written: %s" % os.path.abspath(args.backup))
    for change in changes:
        converted_entry = converted_safety_limits.get(change["urdf_joint"])
        print(
            "%s (%s): old zero_tick=%d new zero_tick=%d delta_ticks=%+d safety lower/upper deg: %s / %s"
            % (
                change["user_joint"],
                change["urdf_joint"],
                change["old_zero_tick"],
                change["new_zero_tick"],
                change["delta_ticks"],
                _format_deg(_lookup(converted_entry, "lower_deg")),
                _format_deg(_lookup(converted_entry, "upper_deg")),
            )
        )
    return 0


def _load_yaml_mapping(path, label):
    with open(path, "r") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("%s file must contain a YAML mapping: %s" % (label, path))
    return data


def _write_yaml_mapping(path, data):
    with open(path, "w") as handle:
        try:
            yaml.safe_dump(data, handle, default_flow_style=False, sort_keys=False)
        except TypeError:
            yaml.safe_dump(data, handle, default_flow_style=False)


def _ensure_parent_dir(path):
    directory = os.path.dirname(os.path.abspath(path))
    if directory and not os.path.exists(directory):
        os.makedirs(directory)


def _lookup(mapping, key):
    if not isinstance(mapping, dict):
        return None
    return mapping.get(key)


def _format_deg(value):
    if value is None:
        return "missing"
    return "%.2f" % float(value)


if __name__ == "__main__":
    raise SystemExit(main())
