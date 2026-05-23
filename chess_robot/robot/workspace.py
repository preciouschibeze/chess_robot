from __future__ import absolute_import

import numpy as np
import yaml

from chess_robot.robot.fk import compute_fk
from chess_robot.robot.fk import make_transform
from chess_robot.robot.fk import rpy_matrix
from chess_robot.robot.urdf_model import DEFAULT_END_LINK


def load_scene_geometry(scene_path):
    with open(scene_path, "r") as handle:
        document = yaml.safe_load(handle) or {}
    return normalise_scene_geometry(document)


def normalise_scene_geometry(document):
    scene = document.get("scene", document)
    if not isinstance(scene, dict):
        raise ValueError("Scene geometry file must contain a top-level 'scene' mapping.")

    warnings = []
    robot_base = _parse_robot_base_geometry(scene, warnings)
    overhead_camera = _parse_origin_geometry(scene, "overhead_camera")
    chessboard = _parse_board_geometry(scene)
    capture_zone = _parse_capture_zone_geometry(scene)
    world_T_robot_base = make_transform(robot_base["rotation_matrix"], robot_base["xyz_m"])

    return {
        "robot_base": robot_base,
        "overhead_camera": overhead_camera,
        "chessboard": chessboard,
        "capture_zone": capture_zone,
        "world_T_robot_base": world_T_robot_base,
        "warnings": warnings,
    }


def scene_transform_matrix(scene_geometry):
    return np.asarray(scene_geometry["world_T_robot_base"], dtype=float)


def sample_joint_positions(model, samples, seed=None, limit_scale=1.0):
    sample_count = int(samples)
    if sample_count < 0:
        raise ValueError("Sample count must be non-negative.")
    if float(limit_scale) <= 0.0:
        raise ValueError("Joint limit scale must be greater than zero.")

    arm_chain = model.get_arm_chain(end_link=DEFAULT_END_LINK)
    joint_names = []
    lower_limits = []
    upper_limits = []
    for joint in arm_chain:
        if joint.limit is None or joint.limit.lower is None or joint.limit.upper is None:
            raise ValueError("Joint %s is missing numeric URDF limits." % joint.name)
        scaled_lower, scaled_upper = scale_joint_limits(
            joint.limit.lower,
            joint.limit.upper,
            limit_scale=limit_scale,
        )
        joint_names.append(joint.name)
        lower_limits.append(scaled_lower)
        upper_limits.append(scaled_upper)

    lower_limits = np.asarray(lower_limits, dtype=float)
    upper_limits = np.asarray(upper_limits, dtype=float)
    random_state = np.random.RandomState(seed)
    positions = random_state.uniform(
        low=lower_limits,
        high=upper_limits,
        size=(sample_count, len(joint_names)),
    )

    return {
        "joint_names": joint_names,
        "positions": positions,
        "lower_limits": lower_limits,
        "upper_limits": upper_limits,
        "seed": seed,
        "limit_scale": float(limit_scale),
    }


def compute_workspace_points(model, joint_samples, end_link=DEFAULT_END_LINK):
    joint_names, positions = _normalise_joint_samples(joint_samples)
    points = np.empty((positions.shape[0], 3), dtype=float)
    for index, sample in enumerate(positions):
        joint_positions = dict(
            (joint_names[joint_index], float(sample[joint_index]))
            for joint_index in range(len(joint_names))
        )
        transform = compute_fk(model, joint_positions, end_link=end_link)
        points[index] = transform[:3, 3]
    return points


def compute_home_tcp_point(model, joint_positions=None, end_link=DEFAULT_END_LINK):
    joint_names = [joint.name for joint in model.get_arm_chain(end_link=end_link)]
    positions = np.zeros((1, len(joint_names)), dtype=float)
    if joint_positions:
        for joint_index, joint_name in enumerate(joint_names):
            positions[0, joint_index] = float(joint_positions.get(joint_name, 0.0))
    points = compute_workspace_points(
        model,
        {"joint_names": joint_names, "positions": positions},
        end_link=end_link,
    )
    return points[0]


def transform_pose_to_world(robot_base_T_pose, scene_geometry):
    return np.dot(scene_transform_matrix(scene_geometry), np.asarray(robot_base_T_pose, dtype=float))


def transform_point_to_world(point, scene_geometry):
    return transform_points_to_world(np.asarray((point,), dtype=float), scene_geometry)[0]


def transform_points_to_world(points, scene_geometry):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Points must be an Nx3 array.")
    transform = scene_transform_matrix(scene_geometry)
    homogeneous = np.ones((points.shape[0], 4), dtype=float)
    homogeneous[:, :3] = points
    transformed = np.dot(transform, homogeneous.T).T
    return transformed[:, :3]


def get_scene_overlays(scene_geometry, include_board=True, include_capture_zone=True):
    overlays = {}
    if include_board:
        board = scene_geometry["chessboard"]
        overlays["chessboard"] = {
            "xy": _make_rectangle_xyz(
                center_xy=board["xy_center_m"],
                size_xy=board["size_xy_m"],
                z_value=board["top_z_m"],
            ),
            "xz": _make_rectangle_xz(
                center_x=board["xy_center_m"][0],
                size_x=board["size_xy_m"][0],
                z_min=board["base_z_m"],
                z_max=board["top_z_m"],
            ),
            "center_m": board["xyz_m"].copy(),
            "base_z_m": board["base_z_m"],
            "top_z_m": board["top_z_m"],
        }
    if include_capture_zone:
        capture_zone = scene_geometry["capture_zone"]
        overlays["capture_zone"] = {
            "xy": _make_rectangle_xyz(
                center_xy=capture_zone["xy_center_m"],
                size_xy=capture_zone["size_xy_m"],
                z_value=capture_zone["top_z_m"],
            ),
            "xz": _make_rectangle_xz(
                center_x=capture_zone["xy_center_m"][0],
                size_x=capture_zone["size_xy_m"][0],
                z_min=capture_zone["base_z_m"],
                z_max=capture_zone["top_z_m"],
            ),
            "center_m": capture_zone["xyz_m"].copy(),
            "base_z_m": capture_zone["base_z_m"],
            "top_z_m": capture_zone["top_z_m"],
        }
    return overlays


def get_scene_markers(scene_geometry):
    return {
        "robot_base": scene_geometry["robot_base"]["xyz_m"].copy(),
        "overhead_camera": scene_geometry["overhead_camera"]["xyz_m"].copy(),
    }


def scale_joint_limits(lower, upper, limit_scale=1.0):
    lower = float(lower)
    upper = float(upper)
    limit_scale = float(limit_scale)
    midpoint = 0.5 * (lower + upper)
    half_span = 0.5 * (upper - lower) * limit_scale
    scaled_lower = max(lower, midpoint - half_span)
    scaled_upper = min(upper, midpoint + half_span)
    if scaled_lower > scaled_upper:
        scaled_lower = midpoint
        scaled_upper = midpoint
    return scaled_lower, scaled_upper


def workspace_bounds(points):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Points must be an Nx3 array.")
    if points.shape[0] == 0:
        raise ValueError("At least one point is required to compute bounds.")
    return {
        "min": points.min(axis=0),
        "max": points.max(axis=0),
    }


def _normalise_joint_samples(joint_samples):
    if not isinstance(joint_samples, dict):
        raise TypeError("Joint samples must be provided as a dictionary.")
    if "joint_names" not in joint_samples or "positions" not in joint_samples:
        raise ValueError("Joint samples must contain 'joint_names' and 'positions'.")
    joint_names = list(joint_samples["joint_names"])
    positions = np.asarray(joint_samples["positions"], dtype=float)
    if positions.ndim != 2:
        raise ValueError("Joint sample positions must be a 2D array.")
    if positions.shape[1] != len(joint_names):
        raise ValueError("Joint sample width does not match joint names.")
    return joint_names, positions


def _parse_robot_base_geometry(scene, warnings):
    geometry = _require_mapping(scene, "robot_base")
    xyz = _as_float_array(geometry.get("xyz_m"), 3, "robot_base.xyz_m")
    size = _as_float_array(geometry.get("size_m"), 3, "robot_base.size_m")
    rpy_deg = geometry.get("rpy_deg")
    if rpy_deg is None:
        warnings.append("WARNING: scene.robot_base.rpy_deg missing; defaulting to [0.0, 0.0, 0.0].")
        rpy_deg = (0.0, 0.0, 0.0)
    rpy_deg = _as_float_array(rpy_deg, 3, "robot_base.rpy_deg")
    rotation_matrix = _scene_rpy_deg_to_rotation_matrix(rpy_deg)
    return {
        "xyz_m": xyz,
        "size_m": size,
        "rpy_deg": rpy_deg,
        "rotation_matrix": rotation_matrix,
    }


def _parse_origin_geometry(scene, key):
    geometry = _require_mapping(scene, key)
    xyz = _as_float_array(geometry.get("xyz_m"), 3, "%s.xyz_m" % key)
    size = _as_float_array(geometry.get("size_m"), 3, "%s.size_m" % key)
    return {
        "xyz_m": xyz,
        "size_m": size,
    }


def _parse_board_geometry(scene):
    geometry = _require_mapping(scene, "chessboard")
    xyz = _as_float_array(geometry.get("xyz_m"), 3, "chessboard.xyz_m")
    size_xy = _normalise_xy_size(geometry.get("size_m"), "chessboard.size_m")
    height = float(geometry.get("height_m", xyz[2]))
    top_z = float(xyz[2])
    base_z = max(0.0, top_z - height)
    return {
        "xyz_m": xyz,
        "xy_center_m": xyz[:2].copy(),
        "size_xy_m": size_xy,
        "height_m": height,
        "base_z_m": base_z,
        "top_z_m": top_z,
    }


def _parse_capture_zone_geometry(scene):
    geometry = _require_mapping(scene, "capture_zone")
    xyz = _as_float_array(geometry.get("xyz_m"), 3, "capture_zone.xyz_m")
    size = _as_float_array(geometry.get("size_m"), 3, "capture_zone.size_m")
    return {
        "xyz_m": xyz,
        "xy_center_m": xyz[:2].copy(),
        "size_xy_m": size[:2].copy(),
        "height_m": float(size[2]),
        "base_z_m": float(xyz[2]),
        "top_z_m": float(xyz[2] + size[2]),
    }


def _scene_rpy_deg_to_rotation_matrix(rpy_deg):
    rpy_rad = np.radians(np.asarray(rpy_deg, dtype=float))
    # Match the requested board convention: yaw=-90 maps URDF +X into world +Y.
    return rpy_matrix(tuple(-value for value in rpy_rad))


def _require_mapping(mapping, key):
    value = mapping.get(key)
    if not isinstance(value, dict):
        raise ValueError("Expected '%s' to be a mapping." % key)
    return value


def _as_float_array(values, expected_length, name):
    if values is None:
        raise ValueError("Missing %s." % name)
    array = np.asarray(values, dtype=float)
    if array.shape != (expected_length,):
        raise ValueError("Expected %s to have shape (%d,), got %s." % (name, expected_length, array.shape))
    return array


def _normalise_xy_size(value, name):
    if value is None:
        raise ValueError("Missing %s." % name)
    if isinstance(value, (int, float)):
        size = float(value)
        return np.asarray((size, size), dtype=float)
    array = np.asarray(value, dtype=float)
    if array.shape == (2,):
        return array
    raise ValueError("Expected %s to be a scalar or length-2 sequence." % name)


def _make_rectangle_xyz(center_xy, size_xy, z_value):
    half_size = np.asarray(size_xy, dtype=float) / 2.0
    center_xy = np.asarray(center_xy, dtype=float)
    x_min = center_xy[0] - half_size[0]
    x_max = center_xy[0] + half_size[0]
    y_min = center_xy[1] - half_size[1]
    y_max = center_xy[1] + half_size[1]
    return np.asarray(
        [
            (x_min, y_min, z_value),
            (x_max, y_min, z_value),
            (x_max, y_max, z_value),
            (x_min, y_max, z_value),
            (x_min, y_min, z_value),
        ],
        dtype=float,
    )


def _make_rectangle_xz(center_x, size_x, z_min, z_max):
    half_size_x = float(size_x) / 2.0
    x_min = float(center_x) - half_size_x
    x_max = float(center_x) + half_size_x
    return np.asarray(
        [
            (x_min, z_min),
            (x_max, z_min),
            (x_max, z_max),
            (x_min, z_max),
            (x_min, z_min),
        ],
        dtype=float,
    )
