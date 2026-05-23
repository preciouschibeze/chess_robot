from __future__ import absolute_import

import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.joint_calibration import angle_rad_to_tick
from chess_robot.robot.joint_calibration import convert_pose_ticks_to_urdf_radians
from chess_robot.robot.joint_calibration import tick_to_angle_deg
from chess_robot.robot.joint_calibration import tick_to_angle_rad


def _calibration():
    return {
        "ticks_per_rev": 4096,
        "provisional": True,
        "joint_order": ["base_yaw", "wrist_roll"],
        "warnings": ["WARNING: joint calibration is marked provisional."],
        "urdf_to_user": {
            "shoulder_pan": "base_yaw",
            "wrist_roll": "wrist_roll",
        },
        "joints": {
            "base_yaw": {
                "user_joint": "base_yaw",
                "urdf_joint": "shoulder_pan",
                "direction_sign": 1,
                "zero_tick": 2048,
            },
            "wrist_roll": {
                "user_joint": "wrist_roll",
                "urdf_joint": "wrist_roll",
                "direction_sign": -1,
                "zero_tick": 1024,
            },
        },
    }


def test_tick_to_angle_conversion_known_values():
    calibration = _calibration()
    assert tick_to_angle_deg("base_yaw", 2048, calibration) == 0.0
    assert abs(tick_to_angle_deg("base_yaw", 3072, calibration) - 90.0) < 1e-6
    assert abs(tick_to_angle_rad("base_yaw", 3072, calibration) - (math.pi / 2.0)) < 1e-6
    assert abs(tick_to_angle_deg("wrist_roll", 0, calibration) - 90.0) < 1e-6


def test_angle_rad_to_tick_inverts_tick_to_angle_rad():
    calibration = _calibration()
    source_tick = 2633
    angle_rad = tick_to_angle_rad("base_yaw", source_tick, calibration)
    recovered_tick = angle_rad_to_tick("base_yaw", angle_rad, calibration)
    assert abs(recovered_tick - source_tick) <= 1


def test_convert_pose_ticks_returns_urdf_joint_names():
    calibration = _calibration()
    pose_ticks = {
        "base_yaw": 3072,
        "wrist_roll": 1024,
        "gripper": 1600,
    }
    converted = convert_pose_ticks_to_urdf_radians(pose_ticks, calibration)
    assert sorted(converted.keys()) == ["shoulder_pan", "wrist_roll"]
    assert abs(converted["shoulder_pan"] - (math.pi / 2.0)) < 1e-6
    assert abs(converted["wrist_roll"] - 0.0) < 1e-6
