"""Forward kinematics helpers for the SO101 URDF chain."""

from __future__ import absolute_import

import math

import numpy as np

from chess_robot.robot.urdf_model import DEFAULT_END_LINK


def compute_fk(model, joint_positions, end_link=DEFAULT_END_LINK, root_link=None, return_details=False):
    """Compute the homogeneous transform to the requested end link."""

    chain = model.get_chain(end_link=end_link, root_link=root_link)
    root_link = root_link or model.root_link
    current_transform = np.eye(4, dtype=float)
    link_transforms = {root_link: current_transform.copy()}
    chain_details = []

    for joint in chain:
        before_motion = np.dot(current_transform, origin_transform(joint.origin_xyz, joint.origin_rpy))
        joint_position = float(joint_positions.get(joint.name, 0.0))
        after_motion = np.dot(before_motion, joint_motion_transform(joint, joint_position))

        current_transform = after_motion
        link_transforms[joint.child] = current_transform.copy()
        chain_details.append(
            {
                "joint_name": joint.name,
                "joint_type": joint.joint_type,
                "parent_link": joint.parent,
                "child_link": joint.child,
                "joint_position": joint_position,
                "joint_transform": before_motion.copy(),
                "child_transform": current_transform.copy(),
            }
        )

    if not return_details:
        return current_transform

    details = {
        "root_link": root_link,
        "end_link": end_link,
        "chain": chain_details,
        "link_transforms": link_transforms,
    }
    return current_transform, details


def origin_transform(origin_xyz, origin_rpy):
    return make_transform(rpy_matrix(origin_rpy), origin_xyz)


def joint_motion_transform(joint, joint_position):
    if joint.joint_type == "fixed":
        return np.eye(4, dtype=float)
    if joint.joint_type in ("revolute", "continuous"):
        return make_transform(axis_angle_matrix(joint.axis, joint_position), (0.0, 0.0, 0.0))
    if joint.joint_type == "prismatic":
        axis = np.asarray(joint.axis, dtype=float)
        return make_transform(np.eye(3, dtype=float), axis * joint_position)
    raise ValueError("Unsupported joint type for FK: %s" % joint.joint_type)


def make_transform(rotation, translation):
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = np.asarray(rotation, dtype=float)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    return transform


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    return np.dot(np.dot(rotation_matrix_z(yaw), rotation_matrix_y(pitch)), rotation_matrix_x(roll))


def rotation_matrix_x(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        ((1.0, 0.0, 0.0), (0.0, cosine, -sine), (0.0, sine, cosine)),
        dtype=float,
    )


def rotation_matrix_y(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        ((cosine, 0.0, sine), (0.0, 1.0, 0.0), (-sine, 0.0, cosine)),
        dtype=float,
    )


def rotation_matrix_z(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)),
        dtype=float,
    )


def axis_angle_matrix(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0.0:
        return np.eye(3, dtype=float)

    x_axis, y_axis, z_axis = axis / norm
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1.0 - cosine

    return np.array(
        (
            (
                cosine + (x_axis * x_axis * one_minus_cosine),
                (x_axis * y_axis * one_minus_cosine) - (z_axis * sine),
                (x_axis * z_axis * one_minus_cosine) + (y_axis * sine),
            ),
            (
                (y_axis * x_axis * one_minus_cosine) + (z_axis * sine),
                cosine + (y_axis * y_axis * one_minus_cosine),
                (y_axis * z_axis * one_minus_cosine) - (x_axis * sine),
            ),
            (
                (z_axis * x_axis * one_minus_cosine) - (y_axis * sine),
                (z_axis * y_axis * one_minus_cosine) + (x_axis * sine),
                cosine + (z_axis * z_axis * one_minus_cosine),
            ),
        ),
        dtype=float,
    )
