from __future__ import absolute_import

import argparse
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
from chess_robot.robot.reachability import LIMIT_SOURCE_INTERSECTION
from chess_robot.robot.reachability import LIMIT_SOURCE_SOFTWARE
from chess_robot.robot.reachability import LIMIT_SOURCE_URDF
from chess_robot.robot.reachability import resolve_joint_limit_bounds
from chess_robot.robot.tool_frames import describe_tool_frame
from chess_robot.robot.tool_frames import get_tool_frame
from chess_robot.robot.tool_frames import load_tool_frames
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import load_urdf_model
from chess_robot.robot.workspace import get_scene_overlays
from chess_robot.robot.workspace import load_scene_geometry

DEFAULT_URDF_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
DEFAULT_SCENE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
DEFAULT_JOINT_CALIBRATION_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
DEFAULT_JOINT_LIMITS_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "joint_limits.yaml")
DEFAULT_HOME_POSE_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "home_pose.yaml")
POSITION_ONLY_WARNING = "WARNING: position-only IK does not prove orientation, collision, or vertical descent feasibility."
ASYMMETRIC_GRIPPER_WARNING = "WARNING: zero-offset gripper_frame is provisional for asymmetric gripper handling."
NO_TOOL_FRAME_WARNING = "No tool frame file provided; solving IK for gripper_frame_link origin."


def build_parser():
    parser = argparse.ArgumentParser(description="Solve position-only IK for a selected TCP target.")
    parser.add_argument("--urdf", default=DEFAULT_URDF_PATH, help="URDF model path.")
    parser.add_argument("--scene", default=DEFAULT_SCENE_PATH, help="Scene geometry YAML path.")
    parser.add_argument("--joint-calibration", default=DEFAULT_JOINT_CALIBRATION_PATH, help="Joint calibration YAML path.")
    parser.add_argument("--joint-limits", default=DEFAULT_JOINT_LIMITS_PATH, help="Joint software limits YAML path.")
    parser.add_argument("--home-pose", default=DEFAULT_HOME_POSE_PATH, help="Saved servo home pose YAML path.")
    parser.add_argument("--tool-frames", default=None, help="Tool frame YAML path.")
    parser.add_argument("--tcp-frame", default=None, help="Requested TCP frame name from the tool frame YAML.")
    parser.add_argument("--target-world", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"), help="Target XYZ in world frame, metres.")
    parser.add_argument("--output", required=True, help="Output JSON report path.")
    parser.add_argument("--plot-output", default=None, help="Optional output PNG path for a simple XY plot.")
    parser.add_argument("--end-link", default=DEFAULT_END_LINK, help="URDF end link used before any tool-frame offset.")
    parser.add_argument(
        "--limit-source",
        choices=(LIMIT_SOURCE_URDF, LIMIT_SOURCE_SOFTWARE, LIMIT_SOURCE_INTERSECTION),
        default=LIMIT_SOURCE_INTERSECTION,
        help="Joint limit source selection.",
    )
    parser.add_argument("--random-seeds", type=int, default=20, help="Number of random IK seeds to try.")
    parser.add_argument("--workspace-seed-samples", type=int, default=1000, help="Number of sampled TCP seeds used to find a nearby initial guess.")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for reproducible seed generation.")
    parser.add_argument("--max-iters", type=int, default=200, help="Maximum iterations per IK solve.")
    parser.add_argument("--tolerance-m", type=float, default=0.005, help="Success tolerance in metres.")
    parser.add_argument("--damping", type=float, default=0.05, help="Damped least-squares lambda.")
    parser.add_argument("--step-scale", type=float, default=1.0, help="Scale factor applied to each IK step.")
    return parser


def main():
    args = build_parser().parse_args()
    model = load_urdf_model(args.urdf)
    scene_geometry = load_scene_geometry(args.scene)
    calibration = load_joint_calibration(args.joint_calibration) if args.joint_calibration else None
    joint_limits = load_joint_limits(args.joint_limits) if args.joint_limits else None

    for warning in scene_geometry.get("warnings", []):
        print(warning)
    if calibration is not None:
        for warning in calibration.get("warnings", []):
            print(warning)

    tool_frame, tool_frame_warning = resolve_tool_frame(args.tool_frames, args.tcp_frame, args.end_link)
    if tool_frame_warning:
        print(tool_frame_warning)
    tool_frame_description = describe_tool_frame(tool_frame, fallback_name=args.end_link)

    target_world = np.asarray(args.target_world, dtype=float)
    target_robot = world_point_to_robot_base(target_world, scene_geometry)
    joint_limit_bounds = resolve_joint_limit_bounds(
        model,
        limit_source=args.limit_source,
        joint_limits=joint_limits,
        calibration=calibration,
        end_link=args.end_link,
    )

    home_seed = load_home_seed(args.home_pose, calibration)
    workspace_seed = None
    if int(args.workspace_seed_samples) > 0:
        workspace_samples = sample_position_workspace(
            model,
            joint_limit_bounds,
            sample_count=args.workspace_seed_samples,
            seed=args.seed,
            end_link=args.end_link,
            tool_frame=tool_frame,
        )
        workspace_seed = find_nearest_workspace_seed(target_robot, workspace_samples)["joint_positions_rad"]

    result = solve_position_ik_multi_seed(
        model,
        target_robot,
        joint_limit_bounds,
        end_link=args.end_link,
        tool_frame=tool_frame,
        home_joint_positions_rad=home_seed,
        workspace_seed_joint_positions_rad=workspace_seed,
        random_seeds=args.random_seeds,
        seed=args.seed,
        max_iters=args.max_iters,
        tolerance_m=args.tolerance_m,
        damping=args.damping,
        step_scale=args.step_scale,
    )
    final_world = robot_base_point_to_world(result.final_xyz_robot, scene_geometry)
    joint_ticks = build_joint_ticks(result.joint_positions_rad, calibration)

    print("Selected TCP frame: %s" % result.tcp_frame)
    print(
        "Selected tool offset (m): x=%.6f y=%.6f z=%.6f"
        % tuple(result.tool_offset_xyz_m)
    )
    print(
        "Target XYZ world (m): x=%.6f y=%.6f z=%.6f"
        % (target_world[0], target_world[1], target_world[2])
    )
    print(
        "Target XYZ robot (m): x=%.6f y=%.6f z=%.6f"
        % (target_robot[0], target_robot[1], target_robot[2])
    )
    print(
        "Final TCP XYZ robot (m): x=%.6f y=%.6f z=%.6f"
        % (result.final_xyz_robot[0], result.final_xyz_robot[1], result.final_xyz_robot[2])
    )
    print(
        "Final TCP XYZ world (m): x=%.6f y=%.6f z=%.6f"
        % (final_world[0], final_world[1], final_world[2])
    )
    print("Error (m): %.6f" % result.error_m)
    print("Iterations: %d" % result.iterations)
    print("Status: %s" % result.status)
    print("Seed source: %s" % result.seed_source)
    print("Joint angles (rad / deg):")
    for joint_name in result.joint_names:
        print(
            "  %s = %.6f rad / %.3f deg"
            % (
                joint_name,
                result.joint_positions_rad[joint_name],
                result.joint_positions_deg[joint_name],
            )
        )
    if joint_ticks:
        print("Joint ticks:")
        for joint_name in result.joint_names:
            print("  %s = %d" % (joint_name, joint_ticks[joint_name]))
    print(POSITION_ONLY_WARNING)
    print(ASYMMETRIC_GRIPPER_WARNING)

    payload = {
        "inputs": {
            "urdf_path": args.urdf,
            "scene_path": args.scene,
            "joint_calibration_path": args.joint_calibration,
            "joint_limits_path": args.joint_limits,
            "home_pose_path": args.home_pose,
            "tool_frames_path": args.tool_frames,
            "requested_tcp_frame": args.tcp_frame,
            "selected_tcp_frame": result.tcp_frame,
            "selected_tool_offset_xyz_m": tool_frame_description["tool_offset_xyz_m"],
            "selected_tool_offset_rpy_deg": tool_frame_description["tool_offset_rpy_deg"],
            "target_world_m": [float(value) for value in target_world],
            "target_robot_m": [float(value) for value in target_robot],
            "limit_source": joint_limit_bounds["source"],
            "random_seeds": int(args.random_seeds),
            "workspace_seed_samples": int(args.workspace_seed_samples),
            "seed": args.seed,
            "max_iters": int(args.max_iters),
            "tolerance_m": float(args.tolerance_m),
            "damping": float(args.damping),
            "step_scale": float(args.step_scale),
        },
        "result": result.to_dict(),
        "final_world_m": [float(value) for value in final_world],
        "joint_ticks": joint_ticks,
        "position_only_warning": POSITION_ONLY_WARNING,
        "asymmetric_gripper_warning": ASYMMETRIC_GRIPPER_WARNING,
    }
    ensure_parent_directory(args.output)
    with open(args.output, "w") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
    print("Saved JSON report: %s" % args.output)

    if args.plot_output:
        ensure_parent_directory(args.plot_output)
        plot_target_solution(scene_geometry, target_world, final_world, args.plot_output)
        print("Saved plot: %s" % args.plot_output)


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


def build_joint_ticks(joint_positions_rad, calibration):
    if calibration is None:
        return {}
    return dict(
        (joint_name, int(angle_rad_to_tick(joint_name, joint_positions_rad[joint_name], calibration)))
        for joint_name in joint_positions_rad
    )


def ensure_parent_directory(path):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)


def plot_target_solution(scene_geometry, target_world, final_world, output_path):
    overlays = get_scene_overlays(scene_geometry, include_board=True, include_capture_zone=True)
    figure, axes = plt.subplots(figsize=(8.0, 7.0))
    for name, overlay in sorted(overlays.items()):
        axes.plot(overlay["xy"][:, 0], overlay["xy"][:, 1], linewidth=2.0, label=name.replace("_", " "))
    axes.scatter([target_world[0]], [target_world[1]], color="#1f77b4", marker="o", s=80, label="Target")
    axes.scatter([final_world[0]], [final_world[1]], color="#d62728", marker="x", s=90, label="Final TCP")
    axes.annotate(
        "",
        xy=(final_world[0], final_world[1]),
        xytext=(target_world[0], target_world[1]),
        arrowprops={"arrowstyle": "->", "linewidth": 1.5, "color": "#444444"},
    )
    axes.set_aspect("equal", adjustable="box")
    axes.set_xlabel("World X (m)")
    axes.set_ylabel("World Y (m)")
    axes.set_title("Selected TCP IK Target")
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


if __name__ == "__main__":
    main()
