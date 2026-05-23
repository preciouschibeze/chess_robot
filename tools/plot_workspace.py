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

from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import load_urdf_model
from chess_robot.robot.workspace import compute_home_tcp_point
from chess_robot.robot.workspace import compute_workspace_points
from chess_robot.robot.workspace import get_scene_markers
from chess_robot.robot.workspace import get_scene_overlays
from chess_robot.robot.workspace import load_scene_geometry
from chess_robot.robot.workspace import sample_joint_positions
from chess_robot.robot.workspace import transform_points_to_world
from chess_robot.robot.workspace import workspace_bounds


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, help="Path to the arm URDF.")
    parser.add_argument("--scene", required=True, help="Path to scene geometry YAML.")
    parser.add_argument("--end-link", default=DEFAULT_END_LINK, help="TCP/end link to evaluate.")
    parser.add_argument("--samples", type=int, default=5000, help="Number of random joint samples.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for reproducible sampling.")
    parser.add_argument("--limit-scale", type=float, default=1.0, help="Scale the URDF joint range around each joint midpoint.")
    parser.add_argument("--output-prefix", required=True, help="Output prefix for saved plot files, without suffix.")
    parser.add_argument("--overlay-board", action="store_true", help="Overlay the chessboard rectangle.")
    parser.add_argument("--overlay-capture-zone", action="store_true", help="Overlay the capture zone rectangle.")
    parser.add_argument("--save-points", action="store_true", help="Save the world-frame point cloud as a .npy file.")
    parser.add_argument("--home-joints-rad", nargs="*", help="Explicit home joint angles in radians, e.g. shoulder_pan=-1.57")
    parser.add_argument("--home-joints-deg", nargs="*", help="Explicit home joint angles in degrees, e.g. shoulder_pan=-90")
    parser.add_argument("--home-pose", help="Saved servo home pose YAML.")
    parser.add_argument("--joint-calibration", help="Joint calibration YAML for tick-to-angle conversion.")
    return parser


def main():
    args = build_parser().parse_args()
    ensure_output_directory(args.output_prefix)

    model = load_urdf_model(args.urdf)
    arm_joint_names = [joint.name for joint in model.get_arm_chain(end_link=args.end_link)]
    scene_geometry = load_scene_geometry(args.scene)
    for warning in scene_geometry.get("warnings", []):
        print(warning)

    home_joint_positions, home_source, home_warnings = select_home_joint_positions(args, arm_joint_names)
    for warning in home_warnings:
        print(warning)
    print("Home joint source: %s" % home_source)

    joint_samples = sample_joint_positions(model, samples=args.samples, seed=args.seed, limit_scale=args.limit_scale)
    tcp_points_base = compute_workspace_points(model, joint_samples, end_link=args.end_link)
    tcp_points_world = transform_points_to_world(tcp_points_base, scene_geometry)
    home_point_base = compute_home_tcp_point(model, joint_positions=home_joint_positions, end_link=args.end_link)
    home_point_world = transform_points_to_world(home_point_base[np.newaxis, :], scene_geometry)[0]
    scene_markers = get_scene_markers(scene_geometry)
    overlays = get_scene_overlays(scene_geometry, include_board=args.overlay_board, include_capture_zone=args.overlay_capture_zone)

    bounds = workspace_bounds(tcp_points_world)
    print_workspace_bounds(bounds, len(tcp_points_world))
    print_home_joint_positions(home_joint_positions, arm_joint_names)

    output_paths = {
        "three_d": args.output_prefix + "_3d.png",
        "xy": args.output_prefix + "_xy.png",
        "xz": args.output_prefix + "_xz.png",
    }
    plot_workspace_3d(tcp_points_world, overlays, scene_markers, home_point_world, output_paths["three_d"])
    plot_workspace_xy(tcp_points_world, overlays, scene_markers, home_point_world, output_paths["xy"])
    plot_workspace_xz(tcp_points_world, overlays, scene_markers, home_point_world, output_paths["xz"])

    if args.save_points:
        points_path = args.output_prefix + "_points.npy"
        np.save(points_path, tcp_points_world)
        print("Saved points: %s" % points_path)


def select_home_joint_positions(args, arm_joint_names):
    joint_positions = dict((joint_name, 0.0) for joint_name in arm_joint_names)
    warnings = []

    if args.home_joints_rad:
        joint_positions.update(parse_joint_assignments(args.home_joints_rad, arm_joint_names, units="rad"))
        if args.home_joints_deg:
            warnings.append("WARNING: ignoring --home-joints-deg because --home-joints-rad was supplied.")
        if args.home_pose or args.joint_calibration:
            warnings.append("WARNING: ignoring saved home pose because explicit home joint radians were supplied.")
        return joint_positions, "explicit-rad", warnings

    if args.home_joints_deg:
        joint_positions.update(parse_joint_assignments(args.home_joints_deg, arm_joint_names, units="deg"))
        if args.home_pose or args.joint_calibration:
            warnings.append("WARNING: ignoring saved home pose because explicit home joint degrees were supplied.")
        return joint_positions, "explicit-deg", warnings

    if args.home_pose or args.joint_calibration:
        if not args.home_pose or not args.joint_calibration:
            raise ValueError("Both --home-pose and --joint-calibration are required together.")
        calibration = load_joint_calibration(args.joint_calibration)
        pose_ticks = load_pose_ticks(args.home_pose)
        joint_positions.update(convert_pose_ticks_to_urdf_radians(pose_ticks, calibration))
        warnings.extend(calibration.get("warnings", []))
        return joint_positions, "saved-home", warnings

    warnings.append("WARNING: no home joint input supplied; using all-zero URDF pose.")
    return joint_positions, "zero-fallback", warnings


def parse_joint_assignments(entries, allowed_joint_names, units):
    allowed = set(allowed_joint_names)
    parsed = {}
    for entry in entries or []:
        if "=" not in entry:
            raise ValueError("Expected JOINT=VALUE, got %r" % entry)
        name, value = entry.split("=", 1)
        name = name.strip()
        if name not in allowed:
            raise ValueError("Unknown arm joint %r. Allowed: %s" % (name, ", ".join(allowed_joint_names)))
        numeric_value = float(value)
        if units == "deg":
            numeric_value = math.radians(numeric_value)
        parsed[name] = numeric_value
    return parsed


def print_home_joint_positions(joint_positions, arm_joint_names):
    print("Home joint positions (rad):")
    for joint_name in arm_joint_names:
        print("  %s = %.6f" % (joint_name, float(joint_positions.get(joint_name, 0.0))))


def ensure_output_directory(output_prefix):
    directory = os.path.dirname(output_prefix)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)


def plot_workspace_3d(points_world, overlays, scene_markers, home_point_world, output_path):
    figure = plt.figure(figsize=(9.0, 7.0))
    axes = figure.add_subplot(111, projection="3d")
    axes.scatter(points_world[:, 0], points_world[:, 1], points_world[:, 2], s=3, alpha=0.18, color="#1f77b4", linewidths=0.0)
    plot_scene_markers_3d(axes, scene_markers, home_point_world)
    plot_scene_overlays_3d(axes, overlays)
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Y (m)")
    axes.set_zlabel("Z (m)")
    axes.set_title("SO101 Workspace in World Frame")
    set_axes_equal_3d(axes, points_world, scene_markers, home_point_world, overlays)
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_workspace_xy(points_world, overlays, scene_markers, home_point_world, output_path):
    figure, axes = plt.subplots(figsize=(8.0, 7.0))
    axes.scatter(points_world[:, 0], points_world[:, 1], s=3, alpha=0.18, color="#1f77b4", linewidths=0.0)
    axes.scatter([scene_markers["robot_base"][0]], [scene_markers["robot_base"][1]], color="#d62728", marker="o", s=60, label="Base origin")
    axes.scatter([home_point_world[0]], [home_point_world[1]], color="#2ca02c", marker="x", s=80, label="TCP home")
    axes.scatter([scene_markers["overhead_camera"][0]], [scene_markers["overhead_camera"][1]], color="#9467bd", marker="^", s=70, label="Overhead camera")
    for name, overlay in sorted(overlays.items()):
        path = overlay["xy"]
        axes.plot(path[:, 0], path[:, 1], linewidth=2.0, label=overlay_label(name))
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Y (m)")
    axes.set_title("SO101 Workspace Top View (X-Y)")
    axes.set_aspect("equal")
    axes.grid(True, alpha=0.3)
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_workspace_xz(points_world, overlays, scene_markers, home_point_world, output_path):
    figure, axes = plt.subplots(figsize=(8.0, 6.0))
    axes.scatter(points_world[:, 0], points_world[:, 2], s=3, alpha=0.18, color="#1f77b4", linewidths=0.0)
    axes.scatter([scene_markers["robot_base"][0]], [scene_markers["robot_base"][2]], color="#d62728", marker="o", s=60, label="Base origin")
    axes.scatter([home_point_world[0]], [home_point_world[2]], color="#2ca02c", marker="x", s=80, label="TCP home")
    axes.scatter([scene_markers["overhead_camera"][0]], [scene_markers["overhead_camera"][2]], color="#9467bd", marker="^", s=70, label="Overhead camera")
    for name, overlay in sorted(overlays.items()):
        path = overlay["xz"]
        axes.plot(path[:, 0], path[:, 1], linewidth=2.0, label=overlay_label(name))
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Z (m)")
    axes.set_title("SO101 Workspace Side View (X-Z)")
    axes.grid(True, alpha=0.3)
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_scene_markers_3d(axes, scene_markers, home_point_world):
    axes.scatter([scene_markers["robot_base"][0]], [scene_markers["robot_base"][1]], [scene_markers["robot_base"][2]], color="#d62728", marker="o", s=50, label="Base origin")
    axes.scatter([home_point_world[0]], [home_point_world[1]], [home_point_world[2]], color="#2ca02c", marker="x", s=70, label="TCP home")
    axes.scatter([scene_markers["overhead_camera"][0]], [scene_markers["overhead_camera"][1]], [scene_markers["overhead_camera"][2]], color="#9467bd", marker="^", s=55, label="Overhead camera")


def plot_scene_overlays_3d(axes, overlays):
    color_map = {"capture_zone": "#ff7f0e", "chessboard": "#8c564b"}
    for name, overlay in sorted(overlays.items()):
        path = overlay["xy"]
        axes.plot(path[:, 0], path[:, 1], path[:, 2], color=color_map.get(name, "#333333"), linewidth=2.0, label=overlay_label(name))


def set_axes_equal_3d(axes, points_world, scene_markers, home_point_world, overlays):
    reference_points = [points_world, np.asarray(list(scene_markers.values()), dtype=float), home_point_world[np.newaxis, :]]
    for overlay in overlays.values():
        reference_points.append(overlay["xy"])
    stacked = np.vstack(reference_points)
    minima = stacked.min(axis=0)
    maxima = stacked.max(axis=0)
    center = (minima + maxima) / 2.0
    span = np.max(maxima - minima)
    if span < 0.05:
        span = 0.05
    radius = span / 2.0
    axes.set_xlim(center[0] - radius, center[0] + radius)
    axes.set_ylim(center[1] - radius, center[1] + radius)
    axes.set_zlim(center[2] - radius, center[2] + radius)


def overlay_label(name):
    if name == "capture_zone":
        return "Capture zone"
    if name == "chessboard":
        return "Chessboard"
    return name.replace("_", " ").title()


def print_workspace_bounds(bounds, sample_count):
    print("Workspace bounds (m) from %d samples:" % sample_count)
    print("  x: %.6f .. %.6f" % (bounds["min"][0], bounds["max"][0]))
    print("  y: %.6f .. %.6f" % (bounds["min"][1], bounds["max"][1]))
    print("  z: %.6f .. %.6f" % (bounds["min"][2], bounds["max"][2]))


if __name__ == "__main__":
    main()
