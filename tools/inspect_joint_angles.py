from __future__ import absolute_import

import argparse
import math
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.joint_calibration import convert_limits_ticks_to_angle_limits
from chess_robot.robot.joint_calibration import get_calibration_entry
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_joint_limits
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.joint_calibration import tick_to_angle_deg
from chess_robot.robot.joint_calibration import tick_to_angle_rad
from chess_robot.robot.urdf_model import load_urdf_model

DEFAULT_URDF = os.path.join(REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--home-pose", required=True, help="Saved servo home pose YAML.")
    parser.add_argument("--joint-limits", required=True, help="Software joint limits YAML.")
    parser.add_argument("--joint-calibration", required=True, help="Joint calibration YAML.")
    parser.add_argument("--urdf", default=DEFAULT_URDF, help="URDF path for limit comparisons.")
    return parser


def main():
    args = build_parser().parse_args()
    calibration = load_joint_calibration(args.joint_calibration)
    pose_ticks = load_pose_ticks(args.home_pose)
    joint_limits = load_joint_limits(args.joint_limits)
    converted_limits = convert_limits_ticks_to_angle_limits(joint_limits, calibration)
    model = load_urdf_model(args.urdf)
    urdf_limits = dict((joint.name, joint.limit) for joint in model.get_arm_chain())

    for warning in calibration.get("warnings", []):
        print(warning)

    warnings = []
    for user_joint in calibration["joint_order"]:
        entry = calibration["joints"][user_joint]
        urdf_joint = entry["urdf_joint"]
        home_tick = _lookup_tick(pose_ticks, user_joint, urdf_joint)
        limit_entry = converted_limits.get(urdf_joint, {})

        print("Joint: %s" % user_joint)
        print("  urdf_joint: %s" % urdf_joint)
        print("  home_tick: %s" % _string_or_dash(home_tick))
        if home_tick is None:
            print("  home_angle_deg: -")
            print("  home_angle_rad: -")
        else:
            print("  home_angle_deg: %.6f" % tick_to_angle_deg(user_joint, home_tick, calibration))
            print("  home_angle_rad: %.6f" % tick_to_angle_rad(user_joint, home_tick, calibration))
        print("  software_min_tick: %s" % _string_or_dash(limit_entry.get("provisional_min_tick")))
        print("  software_max_tick: %s" % _string_or_dash(limit_entry.get("provisional_max_tick")))
        print("  software_min_deg: %s" % _format_optional_float(limit_entry.get("provisional_min_deg")))
        print("  software_max_deg: %s" % _format_optional_float(limit_entry.get("provisional_max_deg")))
        print("  software_min_rad: %s" % _format_optional_float(limit_entry.get("provisional_min_rad")))
        print("  software_max_rad: %s" % _format_optional_float(limit_entry.get("provisional_max_rad")))
        print("  direction_sign: %d" % entry["direction_sign"])
        print("  zero_tick: %d" % entry["zero_tick"])

        urdf_limit = urdf_limits.get(urdf_joint)
        if urdf_limit is not None and limit_entry:
            lower_rad = limit_entry.get("lower_rad")
            upper_rad = limit_entry.get("upper_rad")
            if lower_rad is not None and urdf_limit.lower is not None and lower_rad < float(urdf_limit.lower) - 1e-6:
                warnings.append(
                    "WARNING: %s converted software lower limit %.6f rad is below URDF lower limit %.6f rad."
                    % (urdf_joint, lower_rad, float(urdf_limit.lower))
                )
            if upper_rad is not None and urdf_limit.upper is not None and upper_rad > float(urdf_limit.upper) + 1e-6:
                warnings.append(
                    "WARNING: %s converted software upper limit %.6f rad exceeds URDF upper limit %.6f rad."
                    % (urdf_joint, upper_rad, float(urdf_limit.upper))
                )

    if warnings:
        print("Warnings:")
        for warning in warnings:
            print("  %s" % warning)


def _lookup_tick(pose_ticks, user_joint, urdf_joint):
    if user_joint in pose_ticks:
        return int(pose_ticks[user_joint])
    if urdf_joint in pose_ticks:
        return int(pose_ticks[urdf_joint])
    return None


def _string_or_dash(value):
    if value is None:
        return "-"
    return str(value)


def _format_optional_float(value):
    if value is None:
        return "-"
    return "%.6f" % float(value)


if __name__ == "__main__":
    main()
