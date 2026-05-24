from __future__ import absolute_import

import numpy as np

from chess_robot.robot.fk import compute_fk
from chess_robot.robot.joint_calibration import convert_limits_ticks_to_angle_limits
from chess_robot.robot.urdf_model import DEFAULT_END_LINK
from chess_robot.robot.urdf_model import EXPECTED_ARM_JOINT_NAMES
from chess_robot.robot.workspace import transform_points_to_world
from chess_robot.robot.workspace import workspace_bounds

BLACK_FILES = "hgfedcba"
STATUS_REACHABLE = "reachable"
STATUS_MARGINAL = "marginal"
STATUS_UNREACHABLE = "unreachable"
STATUS_ORDER = (STATUS_REACHABLE, STATUS_MARGINAL, STATUS_UNREACHABLE)
LIMIT_SOURCE_URDF = "urdf"
LIMIT_SOURCE_SOFTWARE = "software"
LIMIT_SOURCE_INTERSECTION = "intersection"
REPORT_FIELDNAMES = [
    "target_name",
    "target_type",
    "square",
    "x_m",
    "y_m",
    "z_m",
    "nearest_x_m",
    "nearest_y_m",
    "nearest_z_m",
    "nearest_distance_m",
    "nearest_distance_mm",
    "status",
    "nearest_shoulder_pan_rad",
    "nearest_shoulder_lift_rad",
    "nearest_elbow_flex_rad",
    "nearest_wrist_flex_rad",
    "nearest_wrist_roll_rad",
]


def grid_to_square(row, col):
    row = int(row)
    col = int(col)
    if row < 0 or row >= 8:
        raise ValueError("row must be in [0, 7].")
    if col < 0 or col >= 8:
        raise ValueError("col must be in [0, 7].")
    return "%s%d" % (BLACK_FILES[col], row + 1)


def board_square_size_xy_m(board_geometry):
    size_xy = np.asarray(board_geometry["size_xy_m"], dtype=float)
    if size_xy.shape != (2,):
        raise ValueError("Board size must be a length-2 XY array.")
    return size_xy / 8.0


def board_square_size_m(board_geometry):
    square_size_xy = board_square_size_xy_m(board_geometry)
    return float(square_size_xy.mean())


def generate_square_centers(scene_geometry):
    board = scene_geometry["chessboard"]
    center_xy = np.asarray(board["xy_center_m"], dtype=float)
    size_xy = np.asarray(board["size_xy_m"], dtype=float)
    square_size_xy = board_square_size_xy_m(board)
    x_min = float(center_xy[0] - (size_xy[0] / 2.0))
    y_max = float(center_xy[1] + (size_xy[1] / 2.0))
    board_top_z = float(board["top_z_m"])
    centers = []
    for row in range(8):
        for col in range(8):
            centers.append(
                {
                    "row": row,
                    "col": col,
                    "square": grid_to_square(row, col),
                    "x_m": x_min + ((col + 0.5) * square_size_xy[0]),
                    "y_m": y_max - ((row + 0.5) * square_size_xy[1]),
                    "z_m": board_top_z,
                }
            )
    return centers


def generate_targets(
    scene_geometry,
    above_board_offset_m=0.080,
    pick_offset_m=0.030,
    capture_above_offset_m=0.080,
):
    board = scene_geometry["chessboard"]
    capture_zone = scene_geometry["capture_zone"]
    board_top_z = float(board["top_z_m"])
    targets = []
    for center in generate_square_centers(scene_geometry):
        square = center["square"]
        common = {
            "square": square,
            "row": center["row"],
            "col": center["col"],
            "x_m": float(center["x_m"]),
            "y_m": float(center["y_m"]),
        }
        targets.append(
            _make_target(
                "%s_surface" % square,
                "square_surface",
                z_m=board_top_z + float(pick_offset_m),
                **common
            )
        )
        targets.append(
            _make_target(
                "%s_above" % square,
                "square_above",
                z_m=board_top_z + float(above_board_offset_m),
                **common
            )
        )

    capture_xy = np.asarray(capture_zone["xy_center_m"], dtype=float)
    capture_base_z = float(capture_zone["base_z_m"])
    targets.append(
        _make_target(
            "capture_surface",
            "capture_surface",
            square="",
            row=None,
            col=None,
            x_m=float(capture_xy[0]),
            y_m=float(capture_xy[1]),
            z_m=capture_base_z + float(pick_offset_m),
        )
    )
    targets.append(
        _make_target(
            "capture_above",
            "capture_above",
            square="",
            row=None,
            col=None,
            x_m=float(capture_xy[0]),
            y_m=float(capture_xy[1]),
            z_m=capture_base_z + float(capture_above_offset_m),
        )
    )
    return targets


def resolve_joint_limit_bounds(
    model,
    limit_source=LIMIT_SOURCE_INTERSECTION,
    joint_limits=None,
    calibration=None,
    end_link=DEFAULT_END_LINK,
):
    arm_chain = model.get_arm_chain(end_link=end_link)
    software_limits = None
    if limit_source in (LIMIT_SOURCE_SOFTWARE, LIMIT_SOURCE_INTERSECTION):
        if joint_limits is None or calibration is None:
            raise ValueError("Software or intersection limit source requires joint limits and calibration.")
        software_limits = convert_limits_ticks_to_angle_limits(joint_limits, calibration)

    joint_names = []
    lower_limits = []
    upper_limits = []
    joint_entries = []
    for joint in arm_chain:
        if joint.limit is None or joint.limit.lower is None or joint.limit.upper is None:
            raise ValueError("Joint %s is missing numeric URDF limits." % joint.name)
        urdf_lower = float(joint.limit.lower)
        urdf_upper = float(joint.limit.upper)
        selected_lower = urdf_lower
        selected_upper = urdf_upper
        software_lower = None
        software_upper = None
        if software_limits is not None:
            software_entry = software_limits.get(joint.name)
            if not isinstance(software_entry, dict):
                raise ValueError("Software joint limits missing for joint %s." % joint.name)
            if software_entry.get("lower_rad") is None or software_entry.get("upper_rad") is None:
                raise ValueError("Software joint limits missing radian bounds for joint %s." % joint.name)
            software_lower = float(software_entry["lower_rad"])
            software_upper = float(software_entry["upper_rad"])
            if limit_source == LIMIT_SOURCE_SOFTWARE:
                selected_lower = software_lower
                selected_upper = software_upper
            elif limit_source == LIMIT_SOURCE_INTERSECTION:
                selected_lower = max(urdf_lower, software_lower)
                selected_upper = min(urdf_upper, software_upper)
        if selected_upper < selected_lower:
            raise ValueError(
                "Resolved joint limits are empty for %s: lower=%.6f upper=%.6f"
                % (joint.name, selected_lower, selected_upper)
            )
        joint_names.append(joint.name)
        lower_limits.append(selected_lower)
        upper_limits.append(selected_upper)
        joint_entries.append(
            {
                "joint_name": joint.name,
                "urdf_lower_rad": urdf_lower,
                "urdf_upper_rad": urdf_upper,
                "software_lower_rad": software_lower,
                "software_upper_rad": software_upper,
                "selected_lower_rad": selected_lower,
                "selected_upper_rad": selected_upper,
            }
        )
    return {
        "source": str(limit_source),
        "joint_names": joint_names,
        "lower_limits": np.asarray(lower_limits, dtype=float),
        "upper_limits": np.asarray(upper_limits, dtype=float),
        "joint_entries": joint_entries,
    }


def sample_workspace(
    model,
    scene_geometry,
    joint_limit_bounds,
    samples,
    seed=None,
    end_link=DEFAULT_END_LINK,
):
    sample_count = int(samples)
    if sample_count <= 0:
        raise ValueError("Sample count must be positive.")
    joint_names = list(joint_limit_bounds["joint_names"])
    lower_limits = np.asarray(joint_limit_bounds["lower_limits"], dtype=float)
    upper_limits = np.asarray(joint_limit_bounds["upper_limits"], dtype=float)
    if lower_limits.shape != upper_limits.shape:
        raise ValueError("Joint limit arrays must have matching shapes.")
    random_state = np.random.RandomState(seed)
    positions = random_state.uniform(
        low=lower_limits,
        high=upper_limits,
        size=(sample_count, len(joint_names)),
    )
    tcp_points_base = np.empty((sample_count, 3), dtype=float)
    for index, sample in enumerate(positions):
        tcp_points_base[index] = _sample_tcp_point(model, joint_names, sample, end_link=end_link)
    tcp_points_world = transform_points_to_world(tcp_points_base, scene_geometry)
    return {
        "joint_names": joint_names,
        "positions": positions,
        "tcp_points_base_m": tcp_points_base,
        "tcp_points_world_m": tcp_points_world,
        "lower_limits": lower_limits,
        "upper_limits": upper_limits,
        "seed": seed,
        "sample_count": sample_count,
        "end_link": end_link,
        "limit_source": joint_limit_bounds["source"],
        "bounds_world": workspace_bounds(tcp_points_world),
    }


def classify_distance(distance_m, reachable_threshold_m, marginal_threshold_m):
    distance_m = float(distance_m)
    reachable_threshold_m = float(reachable_threshold_m)
    marginal_threshold_m = float(marginal_threshold_m)
    if reachable_threshold_m < 0.0:
        raise ValueError("Reachable threshold must be non-negative.")
    if marginal_threshold_m < reachable_threshold_m:
        raise ValueError("Marginal threshold must be greater than or equal to reachable threshold.")
    if distance_m <= reachable_threshold_m:
        return STATUS_REACHABLE
    if distance_m <= marginal_threshold_m:
        return STATUS_MARGINAL
    return STATUS_UNREACHABLE


def find_nearest_workspace_sample(target_xyz_m, workspace_points_world_m):
    target_xyz_m = np.asarray(target_xyz_m, dtype=float)
    workspace_points_world_m = np.asarray(workspace_points_world_m, dtype=float)
    if workspace_points_world_m.ndim != 2 or workspace_points_world_m.shape[1] != 3:
        raise ValueError("Workspace points must be an Nx3 array.")
    if workspace_points_world_m.shape[0] == 0:
        raise ValueError("Workspace points must not be empty.")
    delta = workspace_points_world_m - target_xyz_m[np.newaxis, :]
    squared_distance = np.sum(delta * delta, axis=1)
    nearest_index = int(np.argmin(squared_distance))
    nearest_distance_m = float(np.sqrt(squared_distance[nearest_index]))
    return nearest_index, nearest_distance_m


def analyse_target_reachability(
    targets,
    workspace_samples,
    reachable_threshold_m=0.020,
    marginal_threshold_m=0.050,
):
    joint_names = list(workspace_samples["joint_names"])
    positions = np.asarray(workspace_samples["positions"], dtype=float)
    workspace_points_world_m = np.asarray(workspace_samples["tcp_points_world_m"], dtype=float)
    rows = []
    for target in targets:
        target_xyz_m = np.asarray(
            (target["x_m"], target["y_m"], target["z_m"]),
            dtype=float,
        )
        nearest_index, nearest_distance_m = find_nearest_workspace_sample(
            target_xyz_m,
            workspace_points_world_m,
        )
        nearest_xyz_m = workspace_points_world_m[nearest_index]
        nearest_joint_map = dict(
            (joint_names[joint_index], float(positions[nearest_index, joint_index]))
            for joint_index in range(len(joint_names))
        )
        rows.append(
            build_report_row(
                target,
                nearest_xyz_m,
                nearest_distance_m,
                classify_distance(
                    nearest_distance_m,
                    reachable_threshold_m,
                    marginal_threshold_m,
                ),
                nearest_joint_map,
            )
        )
    return rows


def build_report_row(target, nearest_xyz_m, nearest_distance_m, status, nearest_joint_map):
    nearest_xyz_m = np.asarray(nearest_xyz_m, dtype=float)
    row = {
        "target_name": target["target_name"],
        "target_type": target["target_type"],
        "square": target.get("square", "") or "",
        "x_m": float(target["x_m"]),
        "y_m": float(target["y_m"]),
        "z_m": float(target["z_m"]),
        "nearest_x_m": float(nearest_xyz_m[0]),
        "nearest_y_m": float(nearest_xyz_m[1]),
        "nearest_z_m": float(nearest_xyz_m[2]),
        "nearest_distance_m": float(nearest_distance_m),
        "nearest_distance_mm": float(nearest_distance_m) * 1000.0,
        "status": str(status),
    }
    for joint_name in EXPECTED_ARM_JOINT_NAMES:
        row["nearest_%s_rad" % joint_name] = _joint_value_or_none(nearest_joint_map, joint_name)
    return row


def count_statuses(rows):
    counts = dict((status, 0) for status in STATUS_ORDER)
    for row in rows:
        counts[row["status"]] = counts.get(row["status"], 0) + 1
    return counts


def filter_rows_by_target_type(rows, target_type):
    return [row for row in rows if row.get("target_type") == target_type]


def worst_rows(rows, limit=10):
    limit = int(limit)
    return sorted(rows, key=lambda row: row["nearest_distance_m"], reverse=True)[:limit]


def _make_target(target_name, target_type, square, row, col, x_m, y_m, z_m):
    return {
        "target_name": str(target_name),
        "target_type": str(target_type),
        "square": str(square),
        "row": row,
        "col": col,
        "x_m": float(x_m),
        "y_m": float(y_m),
        "z_m": float(z_m),
    }


def _sample_tcp_point(model, joint_names, sample, end_link):
    joint_positions = dict(
        (joint_names[joint_index], float(sample[joint_index]))
        for joint_index in range(len(joint_names))
    )
    transform = compute_fk(model, joint_positions, end_link=end_link)
    return transform[:3, 3]


def _joint_value_or_none(nearest_joint_map, joint_name):
    if joint_name not in nearest_joint_map:
        return None
    return float(nearest_joint_map[joint_name])
