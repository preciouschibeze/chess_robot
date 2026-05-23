from __future__ import absolute_import

import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from chess_robot.robot.urdf_model import EXPECTED_ARM_JOINT_NAMES
from chess_robot.robot.urdf_model import GRIPPER_JOINT_NAME
from chess_robot.robot.urdf_model import load_urdf_model
from chess_robot.robot.workspace import compute_workspace_points
from chess_robot.robot.workspace import load_scene_geometry
from chess_robot.robot.workspace import sample_joint_positions
from chess_robot.robot.workspace import transform_points_to_world
from tools.plot_workspace import select_home_joint_positions


URDF_PATH = os.path.join(ROOT, "data", "calibration", "robot", "so101_new_calib.urdf")
SCENE_PATH = os.path.join(ROOT, "data", "calibration", "robot", "scene_geometry.yaml")


class _Args(object):
    def __init__(self, **kwargs):
        self.home_joints_rad = kwargs.get("home_joints_rad")
        self.home_joints_deg = kwargs.get("home_joints_deg")
        self.home_pose = kwargs.get("home_pose")
        self.joint_calibration = kwargs.get("joint_calibration")


def test_sample_joint_positions_respects_scaled_limits_and_excludes_gripper():
    model = load_urdf_model(URDF_PATH)
    joint_samples = sample_joint_positions(model, samples=64, seed=13, limit_scale=0.5)

    assert joint_samples["positions"].shape == (64, len(EXPECTED_ARM_JOINT_NAMES))
    assert joint_samples["joint_names"] == list(EXPECTED_ARM_JOINT_NAMES)
    assert GRIPPER_JOINT_NAME not in joint_samples["joint_names"]

    lower_limits = joint_samples["lower_limits"]
    upper_limits = joint_samples["upper_limits"]
    assert np.all(joint_samples["positions"] >= lower_limits[np.newaxis, :])
    assert np.all(joint_samples["positions"] <= upper_limits[np.newaxis, :])


def test_compute_workspace_points_returns_finite_xyz_points():
    model = load_urdf_model(URDF_PATH)
    joint_samples = sample_joint_positions(model, samples=25, seed=5, limit_scale=0.4)
    points = compute_workspace_points(model, joint_samples)

    assert points.shape == (25, 3)
    assert np.isfinite(points).all()


def test_scene_transform_rotates_urdf_x_into_world_y_when_yaw_minus_90(tmpdir):
    scene_path = str(tmpdir.join("scene.yaml"))
    document = {
        "scene": {
            "robot_base": {
                "xyz_m": [0.0, 0.0, 0.0],
                "rpy_deg": [0.0, 0.0, -90.0],
                "size_m": [0.1, 0.1, 0.1],
            },
            "overhead_camera": {
                "xyz_m": [0.0, 0.0, 0.5],
                "size_m": [0.1, 0.1, 0.1],
            },
            "chessboard": {
                "xyz_m": [0.0, 0.2, 0.02],
                "size_m": 0.252,
                "height_m": 0.02,
            },
            "capture_zone": {
                "xyz_m": [0.2, 0.2, 0.0],
                "size_m": [0.04, 0.14, 0.05],
            },
        }
    }
    with open(scene_path, "w") as handle:
        yaml.safe_dump(document, handle, default_flow_style=False)

    scene_geometry = load_scene_geometry(scene_path)
    transformed = transform_points_to_world(np.asarray(((1.0, 0.0, 0.0),), dtype=float), scene_geometry)
    assert np.allclose(transformed[0], np.asarray((0.0, 1.0, 0.0), dtype=float), atol=1e-6)


def test_explicit_home_joints_rad_override_zero_fallback():
    args = _Args(home_joints_rad=["shoulder_pan=-1.57", "elbow_flex=1.68"])
    joint_positions, source, warnings = select_home_joint_positions(args, list(EXPECTED_ARM_JOINT_NAMES))

    assert source == "explicit-rad"
    assert warnings == []
    assert abs(joint_positions["shoulder_pan"] + 1.57) < 1e-6
    assert abs(joint_positions["elbow_flex"] - 1.68) < 1e-6
    assert joint_positions["shoulder_lift"] == 0.0


def test_scene_file_loads_current_robot_base_rotation():
    scene_geometry = load_scene_geometry(SCENE_PATH)
    transformed = transform_points_to_world(np.asarray(((1.0, 0.0, 0.0),), dtype=float), scene_geometry)
    expected = scene_geometry["robot_base"]["xyz_m"] + np.asarray((0.0, 1.0, 0.0), dtype=float)
    assert np.allclose(transformed[0], expected, atol=1e-6)
