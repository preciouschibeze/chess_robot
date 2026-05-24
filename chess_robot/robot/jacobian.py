from __future__ import absolute_import

import numpy as np

from chess_robot.robot.tool_frames import compute_tcp_transform
from chess_robot.robot.urdf_model import DEFAULT_END_LINK


def compute_position_jacobian(
    model,
    joint_positions_rad,
    joint_names=None,
    end_link=DEFAULT_END_LINK,
    tool_frame=None,
    eps=1e-5,
):
    eps = float(eps)
    if eps <= 0.0:
        raise ValueError("eps must be greater than zero.")

    joint_names = _normalise_joint_names(model, joint_names, end_link=end_link)
    base_vector = _joint_vector_from_input(joint_positions_rad, joint_names)
    jacobian = np.zeros((3, len(joint_names)), dtype=float)

    for joint_index in range(len(joint_names)):
        plus_vector = base_vector.copy()
        minus_vector = base_vector.copy()
        plus_vector[joint_index] += eps
        minus_vector[joint_index] -= eps
        plus_point = compute_tcp_transform(
            model,
            _joint_map_from_vector(joint_names, plus_vector),
            end_link=end_link,
            tool_frame=tool_frame,
        )[:3, 3]
        minus_point = compute_tcp_transform(
            model,
            _joint_map_from_vector(joint_names, minus_vector),
            end_link=end_link,
            tool_frame=tool_frame,
        )[:3, 3]
        jacobian[:, joint_index] = (plus_point - minus_point) / (2.0 * eps)

    return jacobian


def _normalise_joint_names(model, joint_names, end_link):
    if joint_names is None:
        return [joint.name for joint in model.get_arm_chain(end_link=end_link)]
    return [str(joint_name) for joint_name in joint_names]


def _joint_vector_from_input(joint_positions_rad, joint_names):
    if isinstance(joint_positions_rad, dict):
        return np.asarray(
            [float(joint_positions_rad.get(joint_name, 0.0)) for joint_name in joint_names],
            dtype=float,
        )

    vector = np.asarray(joint_positions_rad, dtype=float)
    if vector.shape != (len(joint_names),):
        raise ValueError(
            "Joint position vector must have shape (%d,), got %s."
            % (len(joint_names), vector.shape)
        )
    return vector


def _joint_map_from_vector(joint_names, joint_vector):
    return dict(
        (joint_names[joint_index], float(joint_vector[joint_index]))
        for joint_index in range(len(joint_names))
    )
