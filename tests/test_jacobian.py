from __future__ import absolute_import

import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.jacobian import compute_position_jacobian
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.tool_frames import compute_tcp_transform
from chess_robot.robot.tool_frames import get_tool_frame
from chess_robot.robot.tool_frames import load_tool_frames
from chess_robot.robot.urdf_model import EXPECTED_ARM_JOINT_NAMES
from chess_robot.robot.urdf_model import load_urdf_model

URDF_PATH = os.path.join(ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
TOOL_FRAMES_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "tool_frames.yaml")
JOINT_CALIBRATION_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
HOME_POSE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "home_pose.yaml")


def load_home_joint_positions():
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    pose_ticks = load_pose_ticks(HOME_POSE_PATH)
    return convert_pose_ticks_to_urdf_radians(pose_ticks, calibration)


def synthetic_tool_frame(offset_xyz):
    return {
        "name": "synthetic_tcp",
        "parent_link": "gripper_frame_link",
        "xyz_m": np.asarray(offset_xyz, dtype=float),
        "rpy_deg": np.asarray((0.0, 0.0, 0.0), dtype=float),
    }


def test_jacobian_shape_is_three_by_five():
    model = load_urdf_model(URDF_PATH)
    home_joint_positions = load_home_joint_positions()
    tool_frames = load_tool_frames(TOOL_FRAMES_PATH)
    tool_frame = get_tool_frame(tool_frames, "gripper_frame")
    jacobian = compute_position_jacobian(model, home_joint_positions, tool_frame=tool_frame)
    assert jacobian.shape == (3, 5)


def test_jacobian_values_are_finite():
    model = load_urdf_model(URDF_PATH)
    home_joint_positions = load_home_joint_positions()
    jacobian = compute_position_jacobian(model, home_joint_positions)
    assert np.isfinite(jacobian).all()


def test_jacobian_reflects_selected_tcp_offset():
    model = load_urdf_model(URDF_PATH)
    home_joint_positions = load_home_joint_positions()
    jacobian_zero = compute_position_jacobian(model, home_joint_positions)
    jacobian_offset = compute_position_jacobian(
        model,
        home_joint_positions,
        tool_frame=synthetic_tool_frame((0.03, 0.0, 0.0)),
    )
    assert not np.allclose(jacobian_zero, jacobian_offset)


def test_small_jacobian_step_predicts_tcp_displacement():
    model = load_urdf_model(URDF_PATH)
    home_joint_positions = load_home_joint_positions()
    joint_names = list(EXPECTED_ARM_JOINT_NAMES)
    base_vector = np.asarray([home_joint_positions.get(joint_name, 0.0) for joint_name in joint_names], dtype=float)
    delta_q = np.asarray((1.0e-4, -1.2e-4, 9.0e-5, -8.0e-5, 7.0e-5), dtype=float)
    tool_frame = synthetic_tool_frame((0.02, -0.01, 0.015))
    jacobian = compute_position_jacobian(
        model,
        home_joint_positions,
        joint_names=joint_names,
        tool_frame=tool_frame,
    )
    predicted = np.dot(jacobian, delta_q)
    displaced = dict((joint_names[index], float(base_vector[index] + delta_q[index])) for index in range(len(joint_names)))
    base_position = compute_tcp_transform(model, home_joint_positions, tool_frame=tool_frame)[:3, 3]
    displaced_position = compute_tcp_transform(model, displaced, tool_frame=tool_frame)[:3, 3]
    actual = displaced_position - base_position
    assert np.allclose(actual, predicted, atol=2.0e-5)
