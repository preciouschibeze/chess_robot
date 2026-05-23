#!/usr/bin/env python3
"""Inspect the SO101 URDF chain without touching hardware."""

from __future__ import absolute_import

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import load_urdf_model


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, help="Path to the URDF file to inspect.")
    parser.add_argument(
        "--end-link",
        default=DEFAULT_END_LINK,
        help="End link used to define the serial arm chain (default: %(default)s).",
    )
    return parser


def main():
    args = build_parser().parse_args()
    model = load_urdf_model(args.urdf)
    arm_chain_names = [joint.name for joint in model.get_arm_chain(end_link=args.end_link)]
    arm_joint_names = set(arm_chain_names)
    serial_chain_names = [joint.name for joint in model.get_chain(end_link=args.end_link)]

    print("URDF:", args.urdf)
    print("Robot:", model.robot_name)
    print("Root link:", model.root_link)
    print("End link:", args.end_link)
    print("Serial chain joints:", ", ".join(serial_chain_names))
    print("Arm chain joints:", ", ".join(arm_chain_names))
    print("")

    for joint in model.get_movable_joints():
        print("joint:", joint.name)
        print("  type:", joint.joint_type)
        print("  parent link:", joint.parent)
        print("  child link:", joint.child)
        print("  origin xyz:", format_vector(joint.origin_xyz))
        print("  origin rpy:", format_vector(joint.origin_rpy))
        print("  axis:", format_vector(joint.axis))
        if joint.limit is None or (joint.limit.lower is None and joint.limit.upper is None):
            print("  limits:", "none")
        else:
            print("  limits:", "%s to %s" % (joint.limit.lower, joint.limit.upper))
        print("  included in arm chain:", "yes" if joint.name in arm_joint_names else "no")
        print("")


def format_vector(values):
    return " ".join("%.6f" % value for value in values)


if __name__ == "__main__":
    main()
