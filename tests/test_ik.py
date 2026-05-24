from __future__ import absolute_import

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
TOOLS_ROOT = os.path.join(ROOT, "tools")
if TOOLS_ROOT not in sys.path:
    sys.path.insert(0, TOOLS_ROOT)

import analyse_square_ik

from chess_robot.robot.ik import robot_base_point_to_world
from chess_robot.robot.ik import solve_position_ik
from chess_robot.robot.ik import solve_position_ik_multi_seed
from chess_robot.robot.ik import world_point_to_robot_base
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_joint_limits
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.joint_limits import load_joint_safety_limits
from chess_robot.robot.reachability import generate_targets
from chess_robot.robot.reachability import resolve_joint_limit_bounds
from chess_robot.robot.tool_frames import compute_tcp_transform
from chess_robot.robot.tool_frames import get_tool_frame
from chess_robot.robot.tool_frames import load_tool_frames
from chess_robot.robot.urdf_model import EXPECTED_ARM_JOINT_NAMES
from chess_robot.robot.urdf_model import load_urdf_model
from chess_robot.robot.workspace import load_scene_geometry

URDF_PATH = os.path.join(ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
SCENE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "scene_geometry.yaml")
TOOL_FRAMES_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "tool_frames.yaml")
JOINT_CALIBRATION_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
JOINT_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_limits.yaml")
JOINT_SAFETY_LIMITS_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_safety_limits.yaml")
HOME_POSE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "home_pose.yaml")


def load_home_joint_positions():
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    pose_ticks = load_pose_ticks(HOME_POSE_PATH)
    return convert_pose_ticks_to_urdf_radians(pose_ticks, calibration)


def load_common_context():
    model = load_urdf_model(URDF_PATH)
    scene_geometry = load_scene_geometry(SCENE_PATH)
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    joint_limits = load_joint_limits(JOINT_LIMITS_PATH)
    joint_safety_limits = load_joint_safety_limits(JOINT_SAFETY_LIMITS_PATH)
    home_joint_positions = load_home_joint_positions()
    tool_frames = load_tool_frames(TOOL_FRAMES_PATH)
    tool_frame = get_tool_frame(tool_frames, "gripper_frame")
    joint_limit_bounds = resolve_joint_limit_bounds(
        model,
        joint_limits=joint_limits,
        joint_safety_limits=joint_safety_limits,
        calibration=calibration,
    )
    return model, scene_geometry, calibration, home_joint_positions, tool_frame, joint_limit_bounds


def test_ik_succeeds_for_saved_home_selected_tcp_target_when_seeded_from_saved_home():
    model, scene_geometry, calibration, home_joint_positions, tool_frame, joint_limit_bounds = load_common_context()
    del scene_geometry, calibration
    target_robot = compute_tcp_transform(model, home_joint_positions, tool_frame=tool_frame)[:3, 3]
    result = solve_position_ik(
        model,
        target_robot,
        home_joint_positions,
        joint_limit_bounds,
        tool_frame=tool_frame,
    )
    assert result.success
    assert result.status == "success"
    assert result.error_m <= 0.005


def test_ik_result_respects_joint_limits():
    model, scene_geometry, calibration, home_joint_positions, tool_frame, joint_limit_bounds = load_common_context()
    del scene_geometry, calibration
    target_robot = compute_tcp_transform(model, home_joint_positions, tool_frame=tool_frame)[:3, 3]
    result = solve_position_ik_multi_seed(
        model,
        target_robot,
        joint_limit_bounds,
        tool_frame=tool_frame,
        home_joint_positions_rad=home_joint_positions,
        random_seeds=3,
        seed=7,
    )
    lower_limits = joint_limit_bounds["lower_limits"]
    upper_limits = joint_limit_bounds["upper_limits"]
    joint_names = joint_limit_bounds["joint_names"]
    for joint_index, joint_name in enumerate(joint_names):
        value = result.joint_positions_rad[joint_name]
        assert value >= lower_limits[joint_index] - 1.0e-9
        assert value <= upper_limits[joint_index] + 1.0e-9


def test_world_to_robot_target_transform_round_trip_works():
    model, scene_geometry, calibration, home_joint_positions, tool_frame, joint_limit_bounds = load_common_context()
    del model, calibration, home_joint_positions, tool_frame, joint_limit_bounds
    point_world = np.asarray((0.003, 0.220, 0.106), dtype=float)
    point_robot = world_point_to_robot_base(point_world, scene_geometry)
    round_trip_world = robot_base_point_to_world(point_robot, scene_geometry)
    assert np.allclose(round_trip_world, point_world)


def test_analyse_square_ik_target_generation_uses_same_black_side_mapping_as_reachability():
    scene_geometry = load_scene_geometry(SCENE_PATH)
    reachability_targets = generate_targets(scene_geometry, above_board_offset_m=0.08, pick_offset_m=0.03)
    ik_targets = analyse_square_ik.generate_ik_targets(scene_geometry, above_board_offset_m=0.08, pick_offset_m=0.03)
    reachability_squares = [target["square"] for target in reachability_targets if target["square"]]
    ik_squares = [target["square"] for target in ik_targets if target["square"]]
    assert reachability_squares == ik_squares
    assert ik_squares[0] == "h1"
    assert ik_squares[-1] == "a8"


def test_gripper_is_excluded_from_ik_outputs():
    model, scene_geometry, calibration, home_joint_positions, tool_frame, joint_limit_bounds = load_common_context()
    del scene_geometry, calibration
    target_robot = compute_tcp_transform(model, home_joint_positions, tool_frame=tool_frame)[:3, 3]
    result = solve_position_ik_multi_seed(
        model,
        target_robot,
        joint_limit_bounds,
        tool_frame=tool_frame,
        home_joint_positions_rad=home_joint_positions,
        random_seeds=1,
        seed=3,
    )
    assert "gripper" not in result.joint_positions_rad
    assert sorted(result.joint_positions_rad.keys()) == sorted(EXPECTED_ARM_JOINT_NAMES)


def test_legacy_and_safety_joint_limit_bounds_currently_match():
    model = load_urdf_model(URDF_PATH)
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    legacy_joint_limits = load_joint_limits(JOINT_LIMITS_PATH)
    joint_safety_limits = load_joint_safety_limits(JOINT_SAFETY_LIMITS_PATH)

    legacy_bounds = resolve_joint_limit_bounds(
        model,
        joint_limits=legacy_joint_limits,
        calibration=calibration,
    )
    safety_bounds = resolve_joint_limit_bounds(
        model,
        joint_safety_limits=joint_safety_limits,
        calibration=calibration,
    )

    assert legacy_bounds["software_profile_kind"] == "legacy"
    assert safety_bounds["software_profile_kind"] == "safety"
    assert np.allclose(legacy_bounds["lower_limits"], safety_bounds["lower_limits"])
    assert np.allclose(legacy_bounds["upper_limits"], safety_bounds["upper_limits"])
