from __future__ import absolute_import

import argparse
import csv
import json
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_joint_limits
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.reachability import REPORT_FIELDNAMES
from chess_robot.robot.reachability import board_square_size_m
from chess_robot.robot.reachability import board_square_size_xy_m
from chess_robot.robot.reachability import count_statuses
from chess_robot.robot.reachability import filter_rows_by_target_type
from chess_robot.robot.reachability import generate_targets
from chess_robot.robot.reachability import resolve_joint_limit_bounds
from chess_robot.robot.reachability import sample_workspace
from chess_robot.robot.reachability import worst_rows
from chess_robot.robot.reachability import analyse_target_reachability
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import load_urdf_model
from chess_robot.robot.workspace import compute_home_tcp_point
from chess_robot.robot.workspace import get_scene_markers
from chess_robot.robot.workspace import get_scene_overlays
from chess_robot.robot.workspace import load_scene_geometry
from chess_robot.robot.workspace import transform_point_to_world

DEFAULT_URDF_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
DEFAULT_SCENE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
DEFAULT_JOINT_CALIBRATION_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_HOME_POSE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "home_pose.yaml")
POSITION_ONLY_WARNING = (
    "WARNING: sampled position-only reachability does not prove IK solvability, "
    "wrist orientation, collision-free descent, or gripper feasibility."
)
STATUS_COLORS = {
    "reachable": "#2ca02c",
    "marginal": "#ff7f0e",
    "unreachable": "#d62728",
}
TARGET_MARKERS = {
    "square_surface": "o",
    "square_above": "x",
    "capture_surface": "s",
    "capture_above": "^",
}


def build_parser():
    parser = argparse.ArgumentParser(description="Analyse board-square reachability against sampled FK workspace.")
    parser.add_argument("--urdf", default=DEFAULT_URDF_PATH, help="URDF model path.")
    parser.add_argument("--scene", default=DEFAULT_SCENE_PATH, help="Scene geometry YAML path.")
    parser.add_argument("--joint-calibration", default=DEFAULT_JOINT_CALIBRATION_PATH, help="Joint calibration YAML path.")
    parser.add_argument("--joint-limits", default=DEFAULT_JOINT_LIMITS_PATH, help="Joint software limits YAML path.")
    parser.add_argument("--home-pose", default=DEFAULT_HOME_POSE_PATH, help="Saved servo home pose YAML path.")
    parser.add_argument("--samples", type=int, default=20000, help="Number of workspace samples.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for sampling.")
    parser.add_argument("--output-prefix", required=True, help="Output prefix without suffix.")
    parser.add_argument("--overlay-workspace", action="store_true", help="Show the sampled workspace cloud behind the targets.")
    parser.add_argument("--above-board-offset-m", type=float, default=0.080, help="TCP height above the board top for above-square targets.")
    parser.add_argument("--pick-offset-m", type=float, default=0.030, help="Approximate TCP height above the board or capture-zone base for surface targets.")
    parser.add_argument("--capture-above-offset-m", type=float, default=0.080, help="TCP height above the capture-zone base for above-capture targets.")
    parser.add_argument("--reachable-threshold-m", type=float, default=0.020, help="Distance threshold for reachable status.")
    parser.add_argument("--marginal-threshold-m", type=float, default=0.050, help="Distance threshold for marginal status.")
    parser.add_argument(
        "--limit-source",
        choices=("urdf", "software", "intersection"),
        default="intersection",
        help="Joint-limit source for workspace sampling.",
    )
    parser.add_argument("--end-link", default=DEFAULT_END_LINK, help="TCP/end link to analyse.")
    return parser


def main():
    args = build_parser().parse_args()
    ensure_output_directory(args.output_prefix)

    model = load_urdf_model(args.urdf)
    scene_geometry = load_scene_geometry(args.scene)
    calibration = load_joint_calibration(args.joint_calibration)
    joint_limits = load_joint_limits(args.joint_limits)

    for warning in scene_geometry.get("warnings", []):
        print(warning)
    for warning in calibration.get("warnings", []):
        print(warning)

    joint_limit_bounds = resolve_joint_limit_bounds(
        model,
        limit_source=args.limit_source,
        joint_limits=joint_limits,
        calibration=calibration,
        end_link=args.end_link,
    )
    workspace_samples = sample_workspace(
        model,
        scene_geometry,
        joint_limit_bounds,
        samples=args.samples,
        seed=args.seed,
        end_link=args.end_link,
    )
    targets = generate_targets(
        scene_geometry,
        above_board_offset_m=args.above_board_offset_m,
        pick_offset_m=args.pick_offset_m,
        capture_above_offset_m=args.capture_above_offset_m,
    )
    rows = analyse_target_reachability(
        targets,
        workspace_samples,
        reachable_threshold_m=args.reachable_threshold_m,
        marginal_threshold_m=args.marginal_threshold_m,
    )

    home_point_world, home_source = load_home_point_world(
        model,
        scene_geometry,
        calibration,
        args.home_pose,
        end_link=args.end_link,
    )
    board = scene_geometry["chessboard"]
    overlays = get_scene_overlays(scene_geometry, include_board=True, include_capture_zone=True)
    scene_markers = get_scene_markers(scene_geometry)
    summary = build_summary(rows)
    output_paths = {
        "csv": args.output_prefix + ".csv",
        "json": args.output_prefix + ".json",
        "xy": args.output_prefix + "_xy.png",
        "xz": args.output_prefix + "_xz.png",
    }

    write_csv(output_paths["csv"], rows)
    write_json(
        output_paths["json"],
        {
            "metadata": {
                "sample_count": int(workspace_samples["sample_count"]),
                "seed": args.seed,
                "limit_source": joint_limit_bounds["source"],
                "end_link": args.end_link,
                "urdf_path": args.urdf,
                "scene_path": args.scene,
                "joint_calibration_path": args.joint_calibration,
                "joint_limits_path": args.joint_limits,
                "home_pose_path": args.home_pose,
                "home_pose_source": home_source,
                "reachable_threshold_m": float(args.reachable_threshold_m),
                "marginal_threshold_m": float(args.marginal_threshold_m),
                "above_board_offset_m": float(args.above_board_offset_m),
                "pick_offset_m": float(args.pick_offset_m),
                "capture_above_offset_m": float(args.capture_above_offset_m),
                "workspace_bounds_m": bounds_to_lists(workspace_samples["bounds_world"]),
                "board_center_m": list(np.asarray(board["xyz_m"], dtype=float)),
                "board_size_xy_m": list(np.asarray(board["size_xy_m"], dtype=float)),
                "square_size_xy_m": list(board_square_size_xy_m(board)),
                "position_only_warning": POSITION_ONLY_WARNING,
            },
            "summary": summary,
            "targets": rows,
        },
    )
    plot_xy(
        rows,
        workspace_samples["tcp_points_world_m"],
        overlays,
        scene_markers,
        home_point_world,
        output_paths["xy"],
        overlay_workspace=args.overlay_workspace,
    )
    plot_xz(
        rows,
        workspace_samples["tcp_points_world_m"],
        overlays,
        scene_markers,
        home_point_world,
        output_paths["xz"],
        overlay_workspace=args.overlay_workspace,
    )

    print("Workspace samples: %d" % workspace_samples["sample_count"])
    print("Limit source: %s" % joint_limit_bounds["source"])
    print_workspace_bounds(workspace_samples["bounds_world"])
    print(
        "Board centre (m): x=%.6f y=%.6f z=%.6f"
        % (board["xyz_m"][0], board["xyz_m"][1], board["xyz_m"][2])
    )
    print(
        "Board size (m): x=%.6f y=%.6f"
        % (board["size_xy_m"][0], board["size_xy_m"][1])
    )
    square_size_xy = board_square_size_xy_m(board)
    print(
        "Square size (m): x=%.6f y=%.6f mean=%.6f"
        % (square_size_xy[0], square_size_xy[1], board_square_size_m(board))
    )
    print_status_summary("Square surface", summary["square_surface"])
    print_status_summary("Square above", summary["square_above"])
    print_capture_summary(rows)
    print("Worst 10 targets by nearest distance:")
    for row in worst_rows(rows, limit=10):
        print(
            "  %s: %s, %.2f mm"
            % (row["target_name"], row["status"], row["nearest_distance_mm"])
        )
    print(POSITION_ONLY_WARNING)
    print("Saved CSV report: %s" % output_paths["csv"])
    print("Saved JSON report: %s" % output_paths["json"])
    print("Saved XY plot: %s" % output_paths["xy"])
    print("Saved XZ plot: %s" % output_paths["xz"])


def load_home_point_world(model, scene_geometry, calibration, home_pose_path, end_link):
    if not home_pose_path:
        return None, None
    if not os.path.isfile(home_pose_path):
        print("WARNING: home pose file not found: %s" % home_pose_path)
        return None, None
    pose_ticks = load_pose_ticks(home_pose_path)
    joint_positions = convert_pose_ticks_to_urdf_radians(pose_ticks, calibration)
    home_point_base = compute_home_tcp_point(model, joint_positions=joint_positions, end_link=end_link)
    return transform_point_to_world(home_point_base, scene_geometry), "saved-home-pose"


def build_summary(rows):
    return {
        "square_surface": count_statuses(filter_rows_by_target_type(rows, "square_surface")),
        "square_above": count_statuses(filter_rows_by_target_type(rows, "square_above")),
        "capture_surface": count_statuses(filter_rows_by_target_type(rows, "capture_surface")),
        "capture_above": count_statuses(filter_rows_by_target_type(rows, "capture_above")),
    }


def ensure_output_directory(output_prefix):
    directory = os.path.dirname(output_prefix)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)


def write_csv(path, rows):
    with open(path, "w") as handle:
        writer = csv.DictWriter(handle, fieldnames=REPORT_FIELDNAMES)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path, payload):
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)


def bounds_to_lists(bounds):
    return {
        "min": list(np.asarray(bounds["min"], dtype=float)),
        "max": list(np.asarray(bounds["max"], dtype=float)),
    }


def print_workspace_bounds(bounds):
    print("Workspace bounds (m):")
    print("  x: %.6f .. %.6f" % (bounds["min"][0], bounds["max"][0]))
    print("  y: %.6f .. %.6f" % (bounds["min"][1], bounds["max"][1]))
    print("  z: %.6f .. %.6f" % (bounds["min"][2], bounds["max"][2]))


def print_status_summary(label, counts):
    print(
        "%s targets: %d reachable / %d marginal / %d unreachable"
        % (
            label,
            counts.get("reachable", 0),
            counts.get("marginal", 0),
            counts.get("unreachable", 0),
        )
    )


def print_capture_summary(rows):
    capture_surface = _find_row(rows, "capture_surface")
    capture_above = _find_row(rows, "capture_above")
    if capture_surface is not None:
        print(
            "Capture surface: %s, %.2f mm"
            % (capture_surface["status"], capture_surface["nearest_distance_mm"])
        )
    if capture_above is not None:
        print(
            "Capture above: %s, %.2f mm"
            % (capture_above["status"], capture_above["nearest_distance_mm"])
        )


def plot_xy(rows, workspace_points_world_m, overlays, scene_markers, home_point_world, output_path, overlay_workspace):
    figure, axes = plt.subplots(figsize=(9.0, 8.0))
    if overlay_workspace:
        axes.scatter(
            workspace_points_world_m[:, 0],
            workspace_points_world_m[:, 1],
            s=3,
            alpha=0.12,
            color="#1f77b4",
            linewidths=0.0,
            label="Workspace samples",
        )
    plot_overlays_xy(axes, overlays)
    plot_scene_points_xy(axes, scene_markers, home_point_world)
    plot_targets_xy(axes, rows)
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Y (m)")
    axes.set_title("Square Reachability Top View (X-Y)")
    axes.set_aspect("equal")
    axes.grid(True, alpha=0.3)
    add_custom_legend(axes, overlay_workspace, home_point_world)
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_xz(rows, workspace_points_world_m, overlays, scene_markers, home_point_world, output_path, overlay_workspace):
    figure, axes = plt.subplots(figsize=(9.0, 7.0))
    if overlay_workspace:
        axes.scatter(
            workspace_points_world_m[:, 0],
            workspace_points_world_m[:, 2],
            s=3,
            alpha=0.12,
            color="#1f77b4",
            linewidths=0.0,
            label="Workspace samples",
        )
    plot_overlays_xz(axes, overlays)
    plot_scene_points_xz(axes, scene_markers, home_point_world)
    plot_targets_xz(axes, rows)
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Z (m)")
    axes.set_title("Square Reachability Side View (X-Z)")
    axes.grid(True, alpha=0.3)
    add_custom_legend(axes, overlay_workspace, home_point_world)
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_overlays_xy(axes, overlays):
    for name, overlay in sorted(overlays.items()):
        path = overlay["xy"]
        axes.plot(path[:, 0], path[:, 1], linewidth=2.0, label=overlay_label(name))


def plot_overlays_xz(axes, overlays):
    for name, overlay in sorted(overlays.items()):
        path = overlay["xz"]
        axes.plot(path[:, 0], path[:, 1], linewidth=2.0, label=overlay_label(name))


def plot_scene_points_xy(axes, scene_markers, home_point_world):
    axes.scatter([scene_markers["robot_base"][0]], [scene_markers["robot_base"][1]], color="#d62728", marker="o", s=60, label="Base origin")
    if home_point_world is not None:
        axes.scatter([home_point_world[0]], [home_point_world[1]], color="#2ca02c", marker="x", s=80, label="TCP home")


def plot_scene_points_xz(axes, scene_markers, home_point_world):
    axes.scatter([scene_markers["robot_base"][0]], [scene_markers["robot_base"][2]], color="#d62728", marker="o", s=60, label="Base origin")
    if home_point_world is not None:
        axes.scatter([home_point_world[0]], [home_point_world[2]], color="#2ca02c", marker="x", s=80, label="TCP home")


def plot_targets_xy(axes, rows):
    label_square_centers = set()
    for row in rows:
        status = row["status"]
        marker = TARGET_MARKERS[row["target_type"]]
        color = STATUS_COLORS[status]
        size = 36 if "capture" in row["target_type"] else 18
        facecolors = color
        if row["target_type"] == "square_above":
            facecolors = "none"
        axes.scatter(
            [row["x_m"]],
            [row["y_m"]],
            marker=marker,
            s=size,
            facecolors=facecolors,
            edgecolors=color,
            linewidths=0.8,
            alpha=0.85,
            label="%s %s" % (label_target_type(row["target_type"]), status),
        )
        if row["target_type"] == "square_surface" and row["square"] not in label_square_centers:
            axes.text(
                row["x_m"],
                row["y_m"],
                row["square"],
                fontsize=5,
                color="#444444",
                alpha=0.45,
                ha="center",
                va="center",
            )
            label_square_centers.add(row["square"])


def plot_targets_xz(axes, rows):
    for row in rows:
        status = row["status"]
        marker = TARGET_MARKERS[row["target_type"]]
        color = STATUS_COLORS[status]
        size = 36 if "capture" in row["target_type"] else 18
        facecolors = color
        if row["target_type"] == "square_above":
            facecolors = "none"
        axes.scatter(
            [row["x_m"]],
            [row["z_m"]],
            marker=marker,
            s=size,
            facecolors=facecolors,
            edgecolors=color,
            linewidths=0.8,
            alpha=0.85,
            label="%s %s" % (label_target_type(row["target_type"]), status),
        )


def add_custom_legend(axes, overlay_workspace, home_point_world):
    handles = []
    if overlay_workspace:
        handles.append(
            Line2D(
                [0],
                [0],
                marker="o",
                linestyle="None",
                markerfacecolor="#1f77b4",
                markeredgecolor="#1f77b4",
                markersize=5,
                alpha=0.5,
                label="Workspace samples",
            )
        )
    handles.append(
        Line2D(
            [0],
            [0],
            marker="o",
            linestyle="None",
            markerfacecolor="#d62728",
            markeredgecolor="#d62728",
            markersize=6,
            label="Base origin",
        )
    )
    if home_point_world is not None:
        handles.append(
            Line2D(
                [0],
                [0],
                marker="x",
                linestyle="None",
                color="#2ca02c",
                markersize=7,
                label="TCP home",
            )
        )
    for status in ("reachable", "marginal", "unreachable"):
        color = STATUS_COLORS[status]
        handles.append(
            Line2D(
                [0],
                [0],
                marker="o",
                linestyle="None",
                markerfacecolor=color,
                markeredgecolor=color,
                markersize=6,
                label=status,
            )
        )
    handles.append(Line2D([0], [0], marker="o", linestyle="None", color="#333333", markersize=5, label="square surface"))
    handles.append(Line2D([0], [0], marker="x", linestyle="None", color="#333333", markersize=6, label="square above"))
    handles.append(Line2D([0], [0], marker="s", linestyle="None", color="#333333", markersize=6, label="capture surface"))
    handles.append(Line2D([0], [0], marker="^", linestyle="None", color="#333333", markersize=6, label="capture above"))
    axes.legend(handles=handles, loc="best", fontsize=8)


def overlay_label(name):
    if name == "capture_zone":
        return "Capture zone"
    if name == "chessboard":
        return "Chessboard"
    return name.replace("_", " ").title()


def label_target_type(target_type):
    if target_type == "square_surface":
        return "square surface"
    if target_type == "square_above":
        return "square above"
    if target_type == "capture_surface":
        return "capture surface"
    if target_type == "capture_above":
        return "capture above"
    return target_type.replace("_", " ")


def _find_row(rows, target_name):
    for row in rows:
        if row["target_name"] == target_name:
            return row
    return None


if __name__ == "__main__":
    main()
