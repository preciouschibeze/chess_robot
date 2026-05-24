from __future__ import absolute_import

import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.fk import compute_fk
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import load_joint_calibration
from chess_robot.robot.joint_calibration import load_pose_ticks
from chess_robot.robot.tool_frames import compute_tcp_transform
from chess_robot.robot.tool_frames import get_tool_frame
from chess_robot.robot.tool_frames import load_tool_frames
from chess_robot.robot.urdf_model import load_urdf_model

URDF_PATH = os.path.join(ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
TOOL_FRAMES_PATH = os.path.join(ROOT, "data", "calibration", "gripper", "tool_frames.yaml")
JOINT_CALIBRATION_PATH = os.path.join(ROOT, "data", "calibration", "robot", "joint_calibration.yaml")
HOME_POSE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "home_pose.yaml")


def load_home_joint_positions():
    calibration = load_joint_calibration(JOINT_CALIBRATION_PATH)
    pose_ticks = load_pose_ticks(HOME_POSE_PATH)
    return convert_pose_ticks_to_urdf_radians(pose_ticks, calibration)


def test_tool_frame_config_loads_successfully():
    tool_frames = load_tool_frames(TOOL_FRAMES_PATH)
    assert tool_frames["default_tcp"] == "gripper_frame"
    assert sorted(tool_frames["frames"].keys()) == [
        "fixed_jaw_contact",
        "gripper_frame",
        "held_piece_center",
    ]


def test_default_gripper_frame_offset_is_zero():
    tool_frames = load_tool_frames(TOOL_FRAMES_PATH)
    frame = get_tool_frame(tool_frames, "gripper_frame")
    assert np.allclose(frame["xyz_m"], np.zeros(3, dtype=float))
    assert np.allclose(frame["rpy_deg"], np.zeros(3, dtype=float))


def test_missing_requested_tcp_frame_fails_clearly():
    tool_frames = load_tool_frames(TOOL_FRAMES_PATH)
    with pytest.raises(KeyError):
        get_tool_frame(tool_frames, "missing_tcp")


def test_compute_tcp_transform_zero_tool_offset_matches_fk():
    model = load_urdf_model(URDF_PATH)
    home_joint_positions = load_home_joint_positions()
    tool_frames = load_tool_frames(TOOL_FRAMES_PATH)
    tool_frame = get_tool_frame(tool_frames, "gripper_frame")
    fk_transform = compute_fk(model, home_joint_positions)
    tcp_transform = compute_tcp_transform(model, home_joint_positions, tool_frame=tool_frame)
    assert np.allclose(tcp_transform, fk_transform)


def test_compute_tcp_transform_non_zero_offset_changes_position_as_expected():
    model = load_urdf_model(URDF_PATH)
    home_joint_positions = load_home_joint_positions()
    base_transform = compute_fk(model, home_joint_positions)
    offset = np.asarray((0.01, -0.02, 0.03), dtype=float)
    tool_frame = {
        "name": "synthetic_offset",
        "parent_link": "gripper_frame_link",
        "xyz_m": offset,
        "rpy_deg": np.asarray((0.0, 0.0, 0.0), dtype=float),
    }
    tcp_transform = compute_tcp_transform(model, home_joint_positions, tool_frame=tool_frame)
    expected_position = base_transform[:3, 3] + np.dot(base_transform[:3, :3], offset)
    assert np.allclose(tcp_transform[:3, 3], expected_position)
