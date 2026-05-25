from __future__ import absolute_import

import math
import os
import sys

import numpy as np

from chess_robot.robot import ik as ik_module
from chess_robot.robot.approach_orientation import transform_world_axis_to_robot_base

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
from chess_robot.robot.joint_calibration import angle_rad_to_tick
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


def test_legacy_and_safety_joint_limit_bounds_are_valid_profiles():
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
    assert legacy_bounds["lower_limits"].shape == safety_bounds["lower_limits"].shape
    assert legacy_bounds["upper_limits"].shape == safety_bounds["upper_limits"].shape
    assert np.all(legacy_bounds["lower_limits"] <= legacy_bounds["upper_limits"])
    assert np.all(safety_bounds["lower_limits"] <= safety_bounds["upper_limits"])


def test_locked_wrist_roll_stays_at_saved_home_and_is_removed_from_optimisation_variables():
    model, scene_geometry, calibration, home_joint_positions, tool_frame, joint_limit_bounds = load_common_context()
    targets = generate_targets(scene_geometry, above_board_offset_m=0.080, pick_offset_m=0.030)
    d4_above = [target for target in targets if target["target_name"] == "d4_above"][0]
    target_robot = world_point_to_robot_base(
        np.asarray((d4_above["x_m"], d4_above["y_m"], d4_above["z_m"]), dtype=float),
        scene_geometry,
    )
    locked = {"wrist_roll": home_joint_positions["wrist_roll"]}
    result = solve_position_ik_multi_seed(
        model,
        target_robot,
        joint_limit_bounds,
        tool_frame=tool_frame,
        home_joint_positions_rad=home_joint_positions,
        random_seeds=5,
        seed=7,
        locked_joint_positions_rad=locked,
    )
    assert "wrist_roll" not in result.optimized_joint_names
    assert abs(result.joint_positions_rad["wrist_roll"] - home_joint_positions["wrist_roll"]) <= 1.0e-12
    assert result.locked_joints_rad["wrist_roll"] == locked["wrist_roll"]
    assert angle_rad_to_tick("wrist_roll", result.joint_positions_rad["wrist_roll"], calibration) == 1091


def _fake_orientation_solver_transform(model, joint_positions_rad, end_link=None, tool_frame=None):
    del model, end_link, tool_frame
    slide = float(joint_positions_rad["slide"])
    tilt = float(joint_positions_rad["tilt"])
    cosine = math.cos(tilt)
    sine = math.sin(tilt)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.asarray([
        [cosine, 0.0, sine],
        [0.0, 1.0, 0.0],
        [-sine, 0.0, cosine],
    ], dtype=float)
    transform[:3, 3] = np.asarray([slide, 0.0, 0.0], dtype=float)
    return transform


def _fake_orientation_position_jacobian(model, joint_map, joint_names=None, end_link=None, tool_frame=None):
    del model, joint_map, joint_names, end_link, tool_frame
    return np.asarray([
        [1.0, 0.0],
        [0.0, 0.0],
        [0.0, 0.0],
    ], dtype=float)


def test_locked_wrist_roll_stays_fixed_when_approach_preference_enabled():
    model, scene_geometry, calibration, home_joint_positions, tool_frame, joint_limit_bounds = load_common_context()
    targets = generate_targets(scene_geometry, above_board_offset_m=0.080, pick_offset_m=0.030)
    d4_above = [target for target in targets if target["target_name"] == "d4_above"][0]
    target_robot = world_point_to_robot_base(
        np.asarray((d4_above["x_m"], d4_above["y_m"], d4_above["z_m"]), dtype=float),
        scene_geometry,
    )
    locked = {"wrist_roll": home_joint_positions["wrist_roll"]}
    result = solve_position_ik_multi_seed(
        model,
        target_robot,
        joint_limit_bounds,
        tool_frame=tool_frame,
        home_joint_positions_rad=home_joint_positions,
        random_seeds=5,
        seed=7,
        locked_joint_positions_rad=locked,
        approach_axis_local=np.asarray([0.0, 0.0, -1.0], dtype=float),
        approach_target_axis=transform_world_axis_to_robot_base(scene_geometry, np.asarray([0.0, 0.0, -1.0], dtype=float)),
        approach_weight=0.05,
        prefer_vertical_approach=True,
        selected_approach_tilt_limit_deg=20.0,
    )
    assert "wrist_roll" not in result.optimized_joint_names
    assert abs(result.joint_positions_rad["wrist_roll"] - home_joint_positions["wrist_roll"]) <= 1.0e-12
    assert result.locked_joints_rad["wrist_roll"] == locked["wrist_roll"]
    assert angle_rad_to_tick("wrist_roll", result.joint_positions_rad["wrist_roll"], calibration) == 1091


def test_approach_preference_reduces_tilt_in_synthetic_case(monkeypatch):
    monkeypatch.setattr(ik_module, "compute_tcp_transform", _fake_orientation_solver_transform)
    monkeypatch.setattr(ik_module, "compute_position_jacobian", _fake_orientation_position_jacobian)
    joint_limits = {
        "joint_names": ["slide", "tilt"],
        "lower_limits": np.asarray([-2.0, -1.5], dtype=float),
        "upper_limits": np.asarray([2.0, 1.5], dtype=float),
    }
    seed = {"slide": 0.0, "tilt": 0.8}
    target_robot = np.asarray([1.0, 0.0, 0.0], dtype=float)
    unconstrained = ik_module.solve_position_ik(
        None,
        target_robot,
        seed,
        joint_limits,
        tolerance_m=1.0e-6,
        max_iters=50,
    )
    preferred = ik_module.solve_position_ik_multi_seed(
        None,
        target_robot,
        joint_limits,
        seeds=[{"source": "provided", "joint_positions_rad": seed}],
        random_seeds=0,
        tolerance_m=1.0e-6,
        max_iters=200,
        approach_axis_local=np.asarray([0.0, 0.0, -1.0], dtype=float),
        approach_target_axis=np.asarray([0.0, 0.0, -1.0], dtype=float),
        approach_weight=0.2,
        prefer_vertical_approach=True,
        selected_approach_tilt_limit_deg=10.0,
    )
    unconstrained_tilt_deg = abs(math.degrees(seed["tilt"]))
    assert unconstrained.success is True
    assert preferred.success is True
    assert preferred.error_m <= 1.0e-4
    assert abs(preferred.joint_positions_rad["tilt"]) < abs(seed["tilt"])
    assert float(preferred.approach_tilt_deg) < unconstrained_tilt_deg
