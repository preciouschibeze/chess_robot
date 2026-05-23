#!/usr/bin/env python3
"""Plot a simple 3D SO101 forward-kinematics chain from the URDF model."""

from __future__ import absolute_import

import argparse
import math
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np

from chess_robot.robot.fk import compute_fk
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import load_urdf_model


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, help="Path to the URDF file to load.")
    parser.add_argument("--end-link", default=DEFAULT_END_LINK, help="TCP link (default: %(default)s).")
    parser.add_argument("--home", action="store_true", help="Use the all-zero arm configuration.")
    parser.add_argument(
        "--joint-deg",
        nargs="*",
        default=None,
        metavar="JOINT=DEGREES",
        help="Manual arm joint angles in degrees, for example shoulder_lift=20.",
    )
    parser.add_argument("--output", required=True, help="PNG path for the rendered 3D plot.")
    return parser


def main():
    args = build_parser().parse_args()
    model = load_urdf_model(args.urdf)
    arm_chain = model.get_arm_chain(end_link=args.end_link)
    arm_joint_names = [joint.name for joint in arm_chain]

    if not args.home and args.joint_deg is None:
        args.home = True

    joint_positions = dict((name, 0.0) for name in arm_joint_names)
    if args.joint_deg:
        joint_positions.update(parse_joint_degrees(args.joint_deg, arm_joint_names))

    end_transform, details = compute_fk(
        model,
        joint_positions,
        end_link=args.end_link,
        return_details=True,
    )
    plot_fk(details, joint_positions, args.output)

    print("Saved FK plot to:", args.output)
    print("Arm joints:", ", ".join(arm_joint_names))
    print("Joint positions (rad):")
    for joint_name in arm_joint_names:
        print("  %s = %.6f" % (joint_name, joint_positions[joint_name]))
    print("TCP translation:", format_vector(end_transform[:3, 3]))


def parse_joint_degrees(entries, arm_joint_names):
    allowed = set(arm_joint_names)
    parsed = {}
    for entry in entries:
        if "=" not in entry:
            raise ValueError("Expected JOINT=DEGREES, got %r" % entry)
        name, value = entry.split("=", 1)
        if name not in allowed:
            raise ValueError("Unknown arm joint %r. Allowed: %s" % (name, ", ".join(arm_joint_names)))
        parsed[name] = math.radians(float(value))
    return parsed


def plot_fk(details, joint_positions, output_path):
    root_link = details["root_link"]
    chain = details["chain"]

    labels = [root_link]
    points = [details["link_transforms"][root_link][:3, 3]]
    for entry in chain:
        labels.append(entry["child_link"])
        points.append(entry["child_transform"][:3, 3])

    points = np.asarray(points, dtype=float)

    figure = plt.figure(figsize=(8.0, 6.0))
    axes = figure.add_subplot(111, projection="3d")
    axes.plot(points[:, 0], points[:, 1], points[:, 2], "-o", color="#1f77b4", linewidth=2.0)

    for label, point in zip(labels, points):
        axes.text(point[0], point[1], point[2], label, fontsize=8)

    for entry in chain:
        joint_point = entry["joint_transform"][:3, 3]
        joint_label = "%s (%.1f deg)" % (
            entry["joint_name"],
            math.degrees(joint_positions.get(entry["joint_name"], 0.0)),
        )
        axes.scatter([joint_point[0]], [joint_point[1]], [joint_point[2]], color="#ff7f0e", s=20)
        axes.text(joint_point[0], joint_point[1], joint_point[2], joint_label, fontsize=8)

    tcp_transform = chain[-1]["child_transform"]
    draw_frame(axes, tcp_transform, axis_length=0.04)
    tcp_point = tcp_transform[:3, 3]
    axes.scatter([tcp_point[0]], [tcp_point[1]], [tcp_point[2]], color="red", s=35)
    axes.text(tcp_point[0], tcp_point[1], tcp_point[2], "TCP", fontsize=9)

    axes.set_title("SO101 FK Chain")
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Y (m)")
    axes.set_zlabel("Z (m)")
    set_axes_equal(axes, points)
    axes.view_init(elev=24, azim=-56)

    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def draw_frame(axes, transform, axis_length):
    origin = transform[:3, 3]
    rotation = transform[:3, :3]
    colors = ("red", "green", "blue")
    for index, color in enumerate(colors):
        direction = rotation[:, index] * axis_length
        axes.plot(
            [origin[0], origin[0] + direction[0]],
            [origin[1], origin[1] + direction[1]],
            [origin[2], origin[2] + direction[2]],
            color=color,
            linewidth=2.0,
        )


def set_axes_equal(axes, points):
    minima = points.min(axis=0)
    maxima = points.max(axis=0)
    center = (minima + maxima) / 2.0
    span = np.max(maxima - minima)
    if span < 0.05:
        span = 0.05
    radius = span / 2.0

    axes.set_xlim(center[0] - radius, center[0] + radius)
    axes.set_ylim(center[1] - radius, center[1] + radius)
    axes.set_zlim(center[2] - radius, center[2] + radius)


def format_vector(values):
    return " ".join("%.6f" % value for value in values)


if __name__ == "__main__":
    main()
