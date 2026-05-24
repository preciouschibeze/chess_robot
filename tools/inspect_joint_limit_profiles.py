from __future__ import absolute_import

import argparse
import math
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_joint_limits
from chess_robot.robot.joint_limits import convert_joint_preferences_to_urdf_radians
from chess_robot.robot.joint_limits import convert_joint_safety_limits_to_angle_limits
from chess_robot.robot.joint_limits import load_joint_preferences
from chess_robot.robot.joint_limits import load_joint_safety_limits
from chess_robot.robot.urdf_model import load_urdf_model

DEFAULT_URDF_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
DEFAULT_JOINT_CALIBRATION_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
DEFAULT_LEGACY_JOINT_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_JOINT_SAFETY_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
DEFAULT_JOINT_PREFERENCES_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_preferences.yaml")
LARGE_NARROWING_THRESHOLD_DEG = 10.0


def build_parser():
    parser = argparse.ArgumentParser(description="Inspect URDF, legacy, safety, and preference joint limit profiles.")
    parser.add_argument("--urdf", default=DEFAULT_URDF_PATH, help="URDF model path.")
    parser.add_argument("--joint-calibration", default=DEFAULT_JOINT_CALIBRATION_PATH, help="Joint calibration YAML path.")
    parser.add_argument("--legacy-joint-limits", default=DEFAULT_LEGACY_JOINT_LIMITS_PATH, help="Legacy joint_limits.yaml path.")
    parser.add_argument("--joint-safety-limits", default=DEFAULT_JOINT_SAFETY_LIMITS_PATH, help="Joint safety limits YAML path.")
    parser.add_argument("--joint-preferences", default=DEFAULT_JOINT_PREFERENCES_PATH, help="Joint preference YAML path.")
    return parser


def main():
    args = build_parser().parse_args()
    model = load_urdf_model(args.urdf)
    calibration = load_joint_calibration(args.joint_calibration)
    legacy_joint_limits = load_joint_limits(args.legacy_joint_limits)
    joint_safety_limits = load_joint_safety_limits(args.joint_safety_limits)
    joint_preferences = load_joint_preferences(args.joint_preferences)

    legacy_limits_rad = convert_joint_safety_limits_to_angle_limits(legacy_joint_limits, calibration)
    safety_limits_rad = convert_joint_safety_limits_to_angle_limits(joint_safety_limits, calibration)
    preferences_rad = convert_joint_preferences_to_urdf_radians(joint_preferences, calibration)

    print("Joint safety limits source: %s" % joint_safety_limits.get("source"))
    print("Joint safety limits calibrated: %s" % joint_safety_limits.get("calibrated"))
    notes = joint_safety_limits.get("notes")
    if notes:
        print("Joint safety limits notes: %s" % notes)
    for warning in joint_safety_limits.get("warnings", []):
        print(warning)
    print("")

    for joint in model.get_arm_chain():
        legacy_entry = legacy_limits_rad.get(joint.name)
        safety_entry = safety_limits_rad.get(joint.name)
        preference_entry = preferences_rad.get(joint.name)
        print("Joint: %s" % joint.name)
        print(
            "  URDF lower/upper deg: %s / %s"
            % (_format_deg(math.degrees(joint.limit.lower)), _format_deg(math.degrees(joint.limit.upper)))
        )
        print("  legacy software lower/upper deg: %s / %s" % _format_bounds_deg(legacy_entry))
        print("  safety lower/upper deg: %s / %s" % _format_bounds_deg(safety_entry))
        print("  preference preferred deg: %s" % _format_preference_deg(preference_entry))
        print("  preference range deg: %s" % _format_preference_range_deg(preference_entry))

        warnings = _build_joint_warnings(joint, safety_entry)
        for warning in warnings:
            print("  WARNING: %s" % warning)
        print("")


def _build_joint_warnings(joint, safety_entry):
    warnings = []
    if not isinstance(safety_entry, dict):
        warnings.append("missing safety limit entry")
        return warnings

    status = safety_entry.get("status")
    if status:
        warnings.append("safety status is %s" % status)

    urdf_lower_deg = math.degrees(joint.limit.lower)
    urdf_upper_deg = math.degrees(joint.limit.upper)
    safety_lower_deg = safety_entry.get("lower_deg")
    safety_upper_deg = safety_entry.get("upper_deg")
    if safety_lower_deg is not None:
        lower_narrowing_deg = safety_lower_deg - urdf_lower_deg
        if lower_narrowing_deg > LARGE_NARROWING_THRESHOLD_DEG:
            warnings.append(
                "safety lower bound is %.2f deg inside the URDF lower bound" % lower_narrowing_deg
            )
    if safety_upper_deg is not None:
        upper_narrowing_deg = urdf_upper_deg - safety_upper_deg
        if upper_narrowing_deg > LARGE_NARROWING_THRESHOLD_DEG:
            warnings.append(
                "safety upper bound is %.2f deg inside the URDF upper bound" % upper_narrowing_deg
            )
    return warnings


def _format_bounds_deg(entry):
    if not isinstance(entry, dict):
        return ("missing", "missing")
    return (_format_deg(entry.get("lower_deg")), _format_deg(entry.get("upper_deg")))


def _format_preference_deg(entry):
    if not isinstance(entry, dict):
        return "missing"
    return _format_deg(entry.get("preferred_deg"))


def _format_preference_range_deg(entry):
    if not isinstance(entry, dict):
        return "missing"
    preferred_range_deg = entry.get("preferred_range_deg")
    if preferred_range_deg is None:
        return "none"
    return "%s / %s" % (_format_deg(preferred_range_deg[0]), _format_deg(preferred_range_deg[1]))


def _format_deg(value):
    if value is None:
        return "missing"
    return "%.2f" % float(value)


if __name__ == "__main__":
    main()
