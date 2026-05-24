from __future__ import absolute_import

import argparse
import csv
import json
import math
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

from chess_robot.robot.ik import find_nearest_workspace_seed
from chess_robot.robot.ik import robot_base_point_to_world
from chess_robot.robot.ik import sample_position_workspace
from chess_robot.robot.ik import solve_position_ik_multi_seed
from chess_robot.robot.ik import world_point_to_robot_base
from chess_robot.robot.joint_calibration import angle_rad_to_tick
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_joint_limits
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.joint_limits import convert_joint_preferences_to_urdf_radians
from chess_robot.robot.joint_limits import load_joint_preferences
from chess_robot.robot.joint_limits import load_joint_safety_limits
from chess_robot.robot.reachability import LIMIT_SOURCE_INTERSECTION
from chess_robot.robot.reachability import LIMIT_SOURCE_SOFTWARE
from chess_robot.robot.reachability import LIMIT_SOURCE_URDF
from chess_robot.robot.reachability import generate_targets
from chess_robot.robot.reachability import grid_to_square
from chess_robot.robot.reachability import resolve_joint_limit_bounds
from chess_robot.robot.tool_frames import describe_tool_frame
from chess_robot.robot.tool_frames import get_tool_frame
from chess_robot.robot.tool_frames import load_tool_frames
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import EXPECTED_ARM_JOINT_NAMES
from chess_robot.robot.urdf_model import load_urdf_model
from chess_robot.robot.workspace import get_scene_overlays
from chess_robot.robot.workspace import load_scene_geometry

DEFAULT_URDF_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
DEFAULT_SCENE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
DEFAULT_JOINT_CALIBRATION_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_JOINT_SAFETY_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
DEFAULT_JOINT_PREFERENCES_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_preferences.yaml")
DEFAULT_HOME_POSE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "home_pose.yaml")
NO_TOOL_FRAME_WARNING = "No tool frame file provided; solving IK for gripper_frame_link origin."
POSITION_ONLY_WARNING = "WARNING: position-only IK does not prove orientation, collision, or vertical descent feasibility."
ASYMMETRIC_GRIPPER_WARNING = "WARNING: zero-offset gripper_frame is provisional for asymmetric gripper handling."
STATUS_SUCCESS = "success"
STATUS_MARGINAL = "marginal"
STATUS_FAILED = "failed"
CSV_FIELDNAMES = [
    "target_name",
    "target_type",
    "square",
    "tcp_frame",
    "tool_offset_x_m",
    "tool_offset_y_m",
    "tool_offset_z_m",
    "target_x_world_m",
    "target_y_world_m",
    "target_z_world_m",
    "final_x_world_m",
    "final_y_world_m",
    "final_z_world_m",
    "error_m",
    "error_mm",
    "success",
    "status",
    "iterations",
    "shoulder_pan_rad",
    "shoulder_lift_rad",
    "elbow_flex_rad",
    "wrist_flex_rad",
    "wrist_roll_rad",
    "shoulder_pan_deg",
    "shoulder_lift_deg",
    "elbow_flex_deg",
    "wrist_flex_deg",
    "wrist_roll_deg",
    "shoulder_pan_tick",
    "shoulder_lift_tick",
    "elbow_flex_tick",
    "wrist_flex_tick",
    "wrist_roll_tick",
    "min_limit_margin_deg",
    "hit_limit_joints",
]


def build_parser():
    parser = argparse.ArgumentParser(description="Solve square and capture IK targets for a selected TCP.")
    parser.add_argument("--urdf", default=DEFAULT_URDF_PATH, help="URDF model path.")
    parser.add_argument("--scene", default=DEFAULT_SCENE_PATH, help="Scene geometry YAML path.")
    parser.add_argument("--joint-calibration", default=DEFAULT_JOINT_CALIBRATION_PATH, help="Joint calibration YAML path.")
    parser.add_argument("--joint-limits", default=DEFAULT_JOINT_LIMITS_PATH, help="Joint software limits YAML path.")
    parser.add_argument(
        "--joint-safety-limits",
        default=None,
        help="Joint safety limits YAML path. When omitted, legacy joint_limits.yaml is used as the hard software limit source.",
    )
    parser.add_argument(
        "--joint-preferences",
        default=None,
        help="Joint soft preference YAML path. Loaded for inspection and future posture guidance only.",
    )
    parser.add_argument("--home-pose", default=DEFAULT_HOME_POSE_PATH, help="Saved servo home pose YAML path.")
    parser.add_argument("--tool-frames", default=None, help="Tool frame YAML path.")
    parser.add_argument("--tcp-frame", default=None, help="Requested TCP frame name from the tool frame YAML.")
    parser.add_argument("--output-prefix", required=True, help="Output prefix without suffix.")
    parser.add_argument("--above-board-offset-m", type=float, default=0.080, help="Height above the board top for above-square targets.")
    parser.add_argument("--pick-offset-m", type=float, default=0.030, help="Proxy surface/pick target offset above the board or capture base.")
    parser.add_argument("--capture-above-offset-m", type=float, default=None, help="Height above the capture base for capture_above. Defaults to --above-board-offset-m.")
    parser.add_argument("--end-link", default=DEFAULT_END_LINK, help="URDF end link used before any tool-frame offset.")
    parser.add_argument(
        "--limit-source",
        choices=(LIMIT_SOURCE_URDF, LIMIT_SOURCE_SOFTWARE, LIMIT_SOURCE_INTERSECTION),
        default=LIMIT_SOURCE_INTERSECTION,
        help="Joint limit source selection.",
    )
    parser.add_argument("--random-seeds", type=int, default=20, help="Number of random IK seeds per target.")
    parser.add_argument("--workspace-seed-samples", type=int, default=1000, help="Number of sampled TCP seeds used to find nearby initial guesses.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed.")
    parser.add_argument("--max-iters", type=int, default=200, help="Maximum iterations per IK solve.")
    parser.add_argument("--tolerance-m", type=float, default=0.005, help="Success tolerance in metres.")
    parser.add_argument("--damping", type=float, default=0.05, help="Damped least-squares lambda.")
    parser.add_argument("--step-scale", type=float, default=1.0, help="Scale factor applied to each IK step.")
    return parser


def generate_ik_targets(scene_geometry, above_board_offset_m, pick_offset_m, capture_above_offset_m=None):
    if capture_above_offset_m is None:
        capture_above_offset_m = above_board_offset_m
    return generate_targets(
        scene_geometry,
        above_board_offset_m=above_board_offset_m,
        pick_offset_m=pick_offset_m,
        capture_above_offset_m=capture_above_offset_m,
    )


def main():
    args = build_parser().parse_args()
    model = load_urdf_model(args.urdf)
    scene_geometry = load_scene_geometry(args.scene)
    calibration = load_joint_calibration(args.joint_calibration) if args.joint_calibration else None
    joint_limits = load_joint_limits(args.joint_limits) if args.joint_limits else None
    joint_safety_limits = load_joint_safety_limits(args.joint_safety_limits) if args.joint_safety_limits else None
    joint_preferences = load_joint_preferences(args.joint_preferences) if args.joint_preferences else None

    for warning in scene_geometry.get("warnings", []):
        print(warning)
    if calibration is not None:
        for warning in calibration.get("warnings", []):
            print(warning)
        if joint_preferences is not None:
            convert_joint_preferences_to_urdf_radians(joint_preferences, calibration)

    tool_frame, tool_frame_warning = resolve_tool_frame(args.tool_frames, args.tcp_frame, args.end_link)
    if tool_frame_warning:
        print(tool_frame_warning)
    tool_frame_description = describe_tool_frame(tool_frame, fallback_name=args.end_link)

    joint_limit_bounds = resolve_joint_limit_bounds(
        model,
        limit_source=args.limit_source,
        joint_limits=joint_limits,
        joint_safety_limits=joint_safety_limits,
        calibration=calibration,
        end_link=args.end_link,
    )
    for warning in joint_limit_bounds.get("warnings", []):
        print(warning)
    home_seed = load_home_seed(args.home_pose, calibration)
    workspace_samples = None
    if int(args.workspace_seed_samples) > 0:
        workspace_samples = sample_position_workspace(
            model,
            joint_limit_bounds,
            sample_count=args.workspace_seed_samples,
            seed=args.seed,
            end_link=args.end_link,
            tool_frame=tool_frame,
        )

    capture_above_offset_m = args.capture_above_offset_m
    if capture_above_offset_m is None:
        capture_above_offset_m = args.above_board_offset_m
    targets = generate_ik_targets(
        scene_geometry,
        above_board_offset_m=args.above_board_offset_m,
        pick_offset_m=args.pick_offset_m,
        capture_above_offset_m=capture_above_offset_m,
    )

    rows = []
    json_rows = []
    previous_success_seed = None
    for target in targets:
        target_world = np.asarray((target["x_m"], target["y_m"], target["z_m"]), dtype=float)
        target_robot = world_point_to_robot_base(target_world, scene_geometry)
        workspace_seed = None
        if workspace_samples is not None:
            workspace_seed = find_nearest_workspace_seed(target_robot, workspace_samples)["joint_positions_rad"]
        extra_seeds = []
        if previous_success_seed is not None:
            extra_seeds.append({"source": "previous_success", "joint_positions_rad": previous_success_seed})
        result = solve_position_ik_multi_seed(
            model,
            target_robot,
            joint_limit_bounds,
            end_link=args.end_link,
            tool_frame=tool_frame,
            seeds=extra_seeds,
            home_joint_positions_rad=home_seed,
            workspace_seed_joint_positions_rad=workspace_seed,
            random_seeds=args.random_seeds,
            seed=args.seed,
            max_iters=args.max_iters,
            tolerance_m=args.tolerance_m,
            damping=args.damping,
            step_scale=args.step_scale,
        )
        if result.success:
            previous_success_seed = result.joint_positions_rad
        final_world = robot_base_point_to_world(result.final_xyz_robot, scene_geometry)
        row = build_csv_row(
            target,
            target_world,
            final_world,
            result,
            calibration,
            tolerance_m=args.tolerance_m,
        )
        rows.append(row)
        json_row = dict(row)
        json_row["target_robot_m"] = [float(value) for value in target_robot]
        json_row["final_robot_m"] = [float(value) for value in result.final_xyz_robot]
        json_row["solver_status"] = result.status
        json_row["seed_source"] = result.seed_source
        json_rows.append(json_row)

    summary = build_summary(rows)
    worst = worst_rows(rows, limit=10)
    limit_hits = [row for row in rows if row["hit_limit_joints"]]

    output_paths = {
        "csv": args.output_prefix + ".csv",
        "json": args.output_prefix + ".json",
        "xy": args.output_prefix + "_xy.png",
        "heatmap": args.output_prefix + "_error_heatmap.png",
    }
    ensure_output_directory(args.output_prefix)
    write_csv(output_paths["csv"], rows)
    write_json(
        output_paths["json"],
        {
            "inputs": {
                "urdf_path": args.urdf,
                "scene_path": args.scene,
                "joint_calibration_path": args.joint_calibration,
                "joint_limits_path": args.joint_limits,
                "joint_safety_limits_path": args.joint_safety_limits,
                "joint_preferences_path": args.joint_preferences,
                "home_pose_path": args.home_pose,
                "tool_frames_path": args.tool_frames,
                "requested_tcp_frame": args.tcp_frame,
                "selected_tcp_frame": tool_frame_description["tcp_frame"],
                "selected_tool_offset_xyz_m": tool_frame_description["tool_offset_xyz_m"],
                "selected_tool_offset_rpy_deg": tool_frame_description["tool_offset_rpy_deg"],
                "limit_source": joint_limit_bounds["source"],
                "joint_limit_profile_kind": joint_limit_bounds.get("software_profile_kind"),
                "above_board_offset_m": float(args.above_board_offset_m),
                "pick_offset_m": float(args.pick_offset_m),
                "capture_above_offset_m": float(capture_above_offset_m),
                "random_seeds": int(args.random_seeds),
                "workspace_seed_samples": int(args.workspace_seed_samples),
                "seed": args.seed,
                "max_iters": int(args.max_iters),
                "tolerance_m": float(args.tolerance_m),
                "damping": float(args.damping),
                "step_scale": float(args.step_scale),
            },
            "summary": summary,
            "worst_targets": worst,
            "targets_hitting_limits": limit_hits,
            "targets": json_rows,
            "position_only_warning": POSITION_ONLY_WARNING,
            "asymmetric_gripper_warning": ASYMMETRIC_GRIPPER_WARNING,
        },
    )
    plot_xy(rows, scene_geometry, output_paths["xy"])
    plot_error_heatmap(rows, output_paths["heatmap"])

    print("Selected TCP frame: %s" % tool_frame_description["tcp_frame"])
    print(
        "Selected tool offset (m): x=%.6f y=%.6f z=%.6f"
        % tuple(tool_frame_description["tool_offset_xyz_m"])
    )
    print("Targets solved: %d" % len(rows))
    print_status_summary("Square above", summary["square_above"])
    print_status_summary("Square surface", summary["square_surface"])
    print_capture_summary(rows)
    print("Worst 10 targets by IK error:")
    for row in worst:
        print("  %s: %s, %.2f mm" % (row["target_name"], row["status"], row["error_mm"]))
    if limit_hits:
        print("Targets hitting joint limits:")
        for row in limit_hits:
            print("  %s: %s" % (row["target_name"], row["hit_limit_joints"]))
    else:
        print("Targets hitting joint limits: none")
    print(POSITION_ONLY_WARNING)
    print(ASYMMETRIC_GRIPPER_WARNING)
    print("Saved CSV report: %s" % output_paths["csv"])
    print("Saved JSON report: %s" % output_paths["json"])
    print("Saved XY plot: %s" % output_paths["xy"])
    print("Saved heatmap: %s" % output_paths["heatmap"])


def resolve_tool_frame(tool_frames_path, tcp_frame_name, end_link):
    if not tool_frames_path:
        if tcp_frame_name:
            raise ValueError("Cannot use --tcp-frame without --tool-frames.")
        return None, NO_TOOL_FRAME_WARNING
    tool_frames = load_tool_frames(tool_frames_path)
    return get_tool_frame(tool_frames, tcp_frame_name), None


def load_home_seed(home_pose_path, calibration):
    if not home_pose_path or calibration is None:
        return None
    pose_ticks = load_pose_ticks(home_pose_path)
    return convert_pose_ticks_to_urdf_radians(pose_ticks, calibration)


def classify_result(error_m, tolerance_m):
    if float(error_m) <= float(tolerance_m):
        return STATUS_SUCCESS
    if float(error_m) <= 0.015:
        return STATUS_MARGINAL
    return STATUS_FAILED


def build_csv_row(target, target_world, final_world, result, calibration, tolerance_m):
    status = classify_result(result.error_m, tolerance_m)
    joint_ticks = build_joint_ticks(result.joint_positions_rad, calibration)
    min_limit_margin_deg = math.degrees(min(result.limit_margin_rad.values()))
    row = {
        "target_name": target["target_name"],
        "target_type": target["target_type"],
        "square": target.get("square") or "",
        "tcp_frame": result.tcp_frame,
        "tool_offset_x_m": float(result.tool_offset_xyz_m[0]),
        "tool_offset_y_m": float(result.tool_offset_xyz_m[1]),
        "tool_offset_z_m": float(result.tool_offset_xyz_m[2]),
        "target_x_world_m": float(target_world[0]),
        "target_y_world_m": float(target_world[1]),
        "target_z_world_m": float(target_world[2]),
        "final_x_world_m": float(final_world[0]),
        "final_y_world_m": float(final_world[1]),
        "final_z_world_m": float(final_world[2]),
        "error_m": float(result.error_m),
        "error_mm": float(result.error_m * 1000.0),
        "success": bool(status == STATUS_SUCCESS),
        "status": status,
        "iterations": int(result.iterations),
        "shoulder_pan_rad": float(result.joint_positions_rad["shoulder_pan"]),
        "shoulder_lift_rad": float(result.joint_positions_rad["shoulder_lift"]),
        "elbow_flex_rad": float(result.joint_positions_rad["elbow_flex"]),
        "wrist_flex_rad": float(result.joint_positions_rad["wrist_flex"]),
        "wrist_roll_rad": float(result.joint_positions_rad["wrist_roll"]),
        "shoulder_pan_deg": float(result.joint_positions_deg["shoulder_pan"]),
        "shoulder_lift_deg": float(result.joint_positions_deg["shoulder_lift"]),
        "elbow_flex_deg": float(result.joint_positions_deg["elbow_flex"]),
        "wrist_flex_deg": float(result.joint_positions_deg["wrist_flex"]),
        "wrist_roll_deg": float(result.joint_positions_deg["wrist_roll"]),
        "shoulder_pan_tick": joint_ticks.get("shoulder_pan"),
        "shoulder_lift_tick": joint_ticks.get("shoulder_lift"),
        "elbow_flex_tick": joint_ticks.get("elbow_flex"),
        "wrist_flex_tick": joint_ticks.get("wrist_flex"),
        "wrist_roll_tick": joint_ticks.get("wrist_roll"),
        "min_limit_margin_deg": float(min_limit_margin_deg),
        "hit_limit_joints": ",".join(result.hit_limit_joints),
    }
    return row


def build_joint_ticks(joint_positions_rad, calibration):
    if calibration is None:
        return {}
    return dict(
        (joint_name, int(angle_rad_to_tick(joint_name, joint_positions_rad[joint_name], calibration)))
        for joint_name in joint_positions_rad
    )


def build_summary(rows):
    return {
        "square_above": count_statuses(filter_rows(rows, "square_above")),
        "square_surface": count_statuses(filter_rows(rows, "square_surface")),
        "capture_above": count_statuses(filter_rows(rows, "capture_above")),
        "capture_surface": count_statuses(filter_rows(rows, "capture_surface")),
    }


def count_statuses(rows):
    counts = {STATUS_SUCCESS: 0, STATUS_MARGINAL: 0, STATUS_FAILED: 0}
    for row in rows:
        counts[row["status"]] = counts.get(row["status"], 0) + 1
    return counts


def filter_rows(rows, target_type):
    return [row for row in rows if row["target_type"] == target_type]


def worst_rows(rows, limit=10):
    return sorted(rows, key=lambda row: row["error_m"], reverse=True)[: int(limit)]


def print_status_summary(label, counts):
    print(
        "%s targets: %d success / %d marginal / %d failed"
        % (
            label,
            counts.get(STATUS_SUCCESS, 0),
            counts.get(STATUS_MARGINAL, 0),
            counts.get(STATUS_FAILED, 0),
        )
    )


def print_capture_summary(rows):
    capture_surface = find_row(rows, "capture_surface")
    capture_above = find_row(rows, "capture_above")
    if capture_surface is not None:
        print(
            "Capture surface: %s, %.2f mm"
            % (capture_surface["status"], capture_surface["error_mm"])
        )
    if capture_above is not None:
        print(
            "Capture above: %s, %.2f mm"
            % (capture_above["status"], capture_above["error_mm"])
        )


def find_row(rows, target_name):
    for row in rows:
        if row["target_name"] == target_name:
            return row
    return None


def ensure_output_directory(output_prefix):
    directory = os.path.dirname(output_prefix)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)


def write_csv(path, rows):
    with open(path, "w") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path, payload):
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)


def plot_xy(rows, scene_geometry, output_path):
    overlays = get_scene_overlays(scene_geometry, include_board=True, include_capture_zone=True)
    figure, axes = plt.subplots(figsize=(9.0, 8.0))
    for name, overlay in sorted(overlays.items()):
        axes.plot(overlay["xy"][:, 0], overlay["xy"][:, 1], linewidth=2.0, label=name.replace("_", " "))

    style_map = {
        ("square_above", STATUS_SUCCESS): {"marker": "^", "color": "#1f77b4", "label": "Above success"},
        ("square_above", STATUS_MARGINAL): {"marker": "^", "color": "#ff7f0e", "label": "Above marginal"},
        ("square_above", STATUS_FAILED): {"marker": "x", "color": "#d62728", "label": "Above failed"},
        ("square_surface", STATUS_SUCCESS): {"marker": "o", "color": "#2ca02c", "label": "Surface success"},
        ("square_surface", STATUS_MARGINAL): {"marker": "o", "color": "#bcbd22", "label": "Surface marginal"},
        ("square_surface", STATUS_FAILED): {"marker": "x", "color": "#8c564b", "label": "Surface failed"},
        ("capture_above", STATUS_SUCCESS): {"marker": "s", "color": "#17becf", "label": "Capture above success"},
        ("capture_above", STATUS_MARGINAL): {"marker": "s", "color": "#ff9896", "label": "Capture above marginal"},
        ("capture_above", STATUS_FAILED): {"marker": "x", "color": "#9467bd", "label": "Capture above failed"},
        ("capture_surface", STATUS_SUCCESS): {"marker": "D", "color": "#7f7f7f", "label": "Capture surface success"},
        ("capture_surface", STATUS_MARGINAL): {"marker": "D", "color": "#c49c94", "label": "Capture surface marginal"},
        ("capture_surface", STATUS_FAILED): {"marker": "x", "color": "#e377c2", "label": "Capture surface failed"},
    }
    legend_entries = {}
    for row in rows:
        style = style_map[(row["target_type"], row["status"])]
        axes.scatter(
            [row["target_x_world_m"]],
            [row["target_y_world_m"]],
            color=style["color"],
            marker=style["marker"],
            s=48,
            alpha=0.95,
        )
        if row["status"] != STATUS_SUCCESS:
            axes.annotate(
                "",
                xy=(row["final_x_world_m"], row["final_y_world_m"]),
                xytext=(row["target_x_world_m"], row["target_y_world_m"]),
                arrowprops={"arrowstyle": "->", "linewidth": 0.8, "color": style["color"], "alpha": 0.55},
            )
        legend_entries[style["label"]] = Line2D(
            [0],
            [0],
            marker=style["marker"],
            color="w",
            markerfacecolor=style["color"],
            markeredgecolor=style["color"],
            markersize=7,
            linestyle="None",
            label=style["label"],
        )

    axes.set_aspect("equal", adjustable="box")
    axes.set_xlabel("World X (m)")
    axes.set_ylabel("World Y (m)")
    axes.set_title("SO101 Square IK in World XY")
    legend_handles = [legend_entries[key] for key in sorted(legend_entries)]
    axes.legend(handles=legend_handles, loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def plot_error_heatmap(rows, output_path):
    heatmap = np.full((8, 8), np.nan, dtype=float)
    for row in rows:
        if not row["square"]:
            continue
        square = row["square"]
        col = "hgfedcba".index(square[0])
        row_index = int(square[1:]) - 1
        existing = heatmap[row_index, col]
        if np.isnan(existing) or row["error_mm"] > existing:
            heatmap[row_index, col] = row["error_mm"]

    figure, axes = plt.subplots(figsize=(8.0, 7.0))
    image = axes.imshow(heatmap, origin="upper", cmap="magma")
    axes.set_title("Worst-Case Square IK Error (mm)")
    axes.set_xticks(range(8))
    axes.set_yticks(range(8))
    axes.set_xticklabels(list("hgfedcba"))
    axes.set_yticklabels([str(index + 1) for index in range(8)])
    for row_index in range(8):
        for col_index in range(8):
            square = grid_to_square(row_index, col_index)
            value = heatmap[row_index, col_index]
            label = square
            if not np.isnan(value):
                label = "%s\n%.1f" % (square, value)
            axes.text(col_index, row_index, label, ha="center", va="center", color="white", fontsize=8)
    color_bar = figure.colorbar(image, ax=axes)
    color_bar.set_label("Error (mm)")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


if __name__ == "__main__":
    main()
