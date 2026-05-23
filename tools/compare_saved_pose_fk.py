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
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import load_urdf_model


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, help="Path to the URDF file to load.")
    parser.add_argument("--home-pose", required=True, help="Saved servo home pose YAML.")
    parser.add_argument("--joint-calibration", required=True, help="Joint calibration YAML.")
    parser.add_argument("--end-link", default=DEFAULT_END_LINK, help="TCP link to evaluate.")
    parser.add_argument("--output", help="Optional PNG path for the rendered 3D plot.")
    return parser


def main():
    args = build_parser().parse_args()
    model = load_urdf_model(args.urdf)
    calibration = load_joint_calibration(args.joint_calibration)
    pose_ticks = load_pose_ticks(args.home_pose)
    joint_positions = convert_pose_ticks_to_urdf_radians(pose_ticks, calibration)

    for warning in calibration.get("warnings", []):
        print(warning)

    end_transform, details = compute_fk(model, joint_positions, end_link=args.end_link, return_details=True)
    print("Converted saved home joint angles:")
    for joint in model.get_arm_chain(end_link=args.end_link):
        angle_rad = float(joint_positions.get(joint.name, 0.0))
        print("  %s: %.6f rad (%.6f deg)" % (joint.name, angle_rad, math.degrees(angle_rad)))

    print("TCP transform:")
    for row in end_transform:
        print("  %s" % " ".join("% .6f" % value for value in row))
    print("TCP XYZ in robot-base frame: %s" % format_vector(end_transform[:3, 3]))

    if args.output:
        plot_fk(details, joint_positions, args.output)
        print("Saved FK plot to: %s" % args.output)


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
        joint_label = "%s (%.1f deg)" % (entry["joint_name"], math.degrees(joint_positions.get(entry["joint_name"], 0.0)))
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
        axes.plot([origin[0], origin[0] + direction[0]], [origin[1], origin[1] + direction[1]], [origin[2], origin[2] + direction[2]], color=color, linewidth=2.0)


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
