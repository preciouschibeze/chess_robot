from __future__ import absolute_import

import numpy as np

from chess_robot.robot.approach_orientation import DEFAULT_APPROACH_AXIS_LOCAL
from chess_robot.robot.approach_orientation import WORLD_DOWN_AXIS
from chess_robot.robot.approach_orientation import approach_tilt_deg
from chess_robot.robot.approach_orientation import make_approach_angle_check
from chess_robot.robot.approach_orientation import normalize_vector
from chess_robot.robot.approach_orientation import resolve_approach_axis_local as resolve_selected_approach_axis_local
from chess_robot.robot.approach_orientation import transform_local_axis_to_world
from chess_robot.robot.tool_frames import compute_tcp_transform


def board_top_z_m(scene_geometry):
    return float(scene_geometry["chessboard"]["top_z_m"])


def low_zone_z_m(scene_geometry, board_clearance_m):
    return board_top_z_m(scene_geometry) + float(board_clearance_m)


def resolve_approach_axis_local(tool_frame):
    resolved = resolve_selected_approach_axis_local(tool_frame=tool_frame)
    return resolved["axis_local"], resolved["defaulted"]


def approach_axis_world(tcp_world_transform, approach_axis_local):
    return transform_local_axis_to_world(tcp_world_transform, approach_axis_local)


def interpolate_joint_positions(start_joint_positions_rad, target_joint_positions_rad, joint_names, samples_count):
    count = max(2, int(samples_count))
    waypoints = []
    for sample_index in range(count):
        fraction = float(sample_index) / float(count - 1)
        positions = {}
        for joint_name in joint_names:
            start_value = float(start_joint_positions_rad[joint_name])
            target_value = float(target_joint_positions_rad[joint_name])
            positions[joint_name] = start_value + ((target_value - start_value) * fraction)
        waypoints.append(positions)
    return waypoints


def validate_sampled_tcp_path(samples_world_xyz, low_zone_z, xy_motion_epsilon_m):
    samples = np.asarray(samples_world_xyz, dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise ValueError("samples_world_xyz must be an Nx3 array.")
    if samples.shape[0] < 2:
        raise ValueError("At least two path samples are required.")

    start_xy = samples[0, :2]
    end_xy = samples[-1, :2]
    xy_delta = float(np.linalg.norm(end_xy - start_xy))
    min_z = float(np.min(samples[:, 2]))
    low_zone_z = float(low_zone_z)
    xy_changing = xy_delta > float(xy_motion_epsilon_m)
    passed = (not xy_changing) or min_z >= low_zone_z
    failure_reason = None
    if not passed:
        failure_reason = (
            "XY-changing board motion predicted min z %.6f m below low zone %.6f m."
            % (min_z, low_zone_z)
        )
    return {
        "xy_delta_m": xy_delta,
        "min_z_m": min_z,
        "low_zone_z_m": low_zone_z,
        "passed": bool(passed),
        "failure_reason": failure_reason,
        "samples_count": int(samples.shape[0]),
        "xy_changing": bool(xy_changing),
        "current_tcp_world_xyz_m": [float(value) for value in samples[0]],
        "target_tcp_world_xyz_m": [float(value) for value in samples[-1]],
    }


def validate_joint_interpolated_tcp_path(
        model,
        scene_geometry,
        current_joint_positions_rad,
        target_joint_positions_rad,
        joint_names,
        end_link,
        tool_frame,
        low_zone_z,
        xy_motion_epsilon_m,
        samples_count):
    world_T_robot = np.asarray(scene_geometry["world_T_robot_base"], dtype=float)
    samples = []
    for joint_positions in interpolate_joint_positions(
            current_joint_positions_rad,
            target_joint_positions_rad,
            joint_names,
            samples_count):
        robot_T_tcp = compute_tcp_transform(
            model,
            joint_positions,
            end_link=end_link,
            tool_frame=tool_frame,
        )
        world_T_tcp = np.dot(world_T_robot, robot_T_tcp)
        samples.append(world_T_tcp[:3, 3].copy())
    return validate_sampled_tcp_path(samples, low_zone_z, xy_motion_epsilon_m)
