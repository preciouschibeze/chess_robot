from __future__ import absolute_import

import math

import numpy as np

from chess_robot.robot.tool_frames import compute_tcp_transform


DEFAULT_APPROACH_AXIS_LOCAL = np.asarray((0.0, 0.0, -1.0), dtype=float)
WORLD_DOWN_AXIS = np.asarray((0.0, 0.0, -1.0), dtype=float)


def board_top_z_m(scene_geometry):
    return float(scene_geometry["chessboard"]["top_z_m"])


def low_zone_z_m(scene_geometry, board_clearance_m):
    return board_top_z_m(scene_geometry) + float(board_clearance_m)


def normalize_vector(values, name):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,):
        raise ValueError("%s must have shape (3,)." % name)
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        raise ValueError("%s must be non-zero." % name)
    return vector / norm


def resolve_approach_axis_local(tool_frame):
    raw_axis = None
    if tool_frame is not None:
        raw_axis = tool_frame.get("approach_axis_local")
    defaulted = raw_axis is None
    axis = DEFAULT_APPROACH_AXIS_LOCAL if defaulted else raw_axis
    return normalize_vector(axis, "approach_axis_local"), defaulted


def approach_axis_world(tcp_world_transform, approach_axis_local):
    transform = np.asarray(tcp_world_transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("tcp_world_transform must have shape (4, 4).")
    local_axis = normalize_vector(approach_axis_local, "approach_axis_local")
    world_axis = np.dot(transform[:3, :3], local_axis)
    return normalize_vector(world_axis, "approach_axis_world")


def approach_tilt_deg(axis_world, reference_down_axis=None):
    axis = normalize_vector(axis_world, "approach_axis_world")
    down = normalize_vector(reference_down_axis if reference_down_axis is not None else WORLD_DOWN_AXIS, "reference_down_axis")
    dot = float(np.dot(axis, down))
    dot = max(-1.0, min(1.0, dot))
    return float(math.degrees(math.acos(dot)))


def make_approach_angle_check(tilt_deg, max_tilt_deg):
    ok = float(tilt_deg) <= float(max_tilt_deg)
    return {
        "passed": bool(ok),
        "tilt_deg": float(tilt_deg),
        "max_tilt_deg": float(max_tilt_deg),
        "failure_reason": None if ok else "Approach tilt %.3f deg exceeds limit %.3f deg." % (
            float(tilt_deg),
            float(max_tilt_deg),
        ),
    }


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
