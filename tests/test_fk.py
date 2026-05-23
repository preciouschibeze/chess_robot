from __future__ import absolute_import

import os

import numpy as np

from chess_robot.robot.fk import compute_fk
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import EXPECTED_ARM_JOINT_NAMES
from chess_robot.robot.urdf_model import GRIPPER_JOINT_NAME
from chess_robot.robot.urdf_model import load_urdf_model


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(REPO_ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")


def test_urdf_loads_expected_arm_chain():
    model = load_urdf_model(URDF_PATH)

    arm_joint_names = [joint.name for joint in model.get_arm_chain()]
    serial_chain_names = [joint.name for joint in model.get_chain(DEFAULT_END_LINK)]

    assert model.root_link == "base_link"
    assert DEFAULT_END_LINK in model.links
    assert arm_joint_names == list(EXPECTED_ARM_JOINT_NAMES)
    assert GRIPPER_JOINT_NAME not in arm_joint_names
    assert serial_chain_names == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper_frame_joint",
    ]


def test_movable_joints_report_gripper_but_not_in_arm_chain():
    model = load_urdf_model(URDF_PATH)

    movable_joint_names = [joint.name for joint in model.get_movable_joints()]
    arm_joint_names = [joint.name for joint in model.get_arm_chain()]

    assert GRIPPER_JOINT_NAME in movable_joint_names
    assert GRIPPER_JOINT_NAME not in arm_joint_names


def test_compute_fk_returns_finite_homogeneous_transform():
    model = load_urdf_model(URDF_PATH)
    joint_positions = dict((joint_name, 0.0) for joint_name in EXPECTED_ARM_JOINT_NAMES)

    end_transform, details = compute_fk(model, joint_positions, return_details=True)

    assert end_transform.shape == (4, 4)
    assert np.isfinite(end_transform).all()
    assert np.allclose(end_transform[3], np.array([0.0, 0.0, 0.0, 1.0]))
    assert DEFAULT_END_LINK in details["link_transforms"]
    assert details["chain"][-1]["child_link"] == DEFAULT_END_LINK
