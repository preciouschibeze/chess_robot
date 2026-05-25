from __future__ import absolute_import

import math

import numpy as np

from chess_robot.robot.approach_orientation import approach_tilt_deg
from chess_robot.robot.approach_orientation import axis_name_from_vector
from chess_robot.robot.approach_orientation import make_approach_angle_check
from chess_robot.robot.approach_orientation import normalize_vector
from chess_robot.robot.jacobian import compute_position_jacobian
from chess_robot.robot.tool_frames import compute_tcp_transform
from chess_robot.robot.tool_frames import describe_tool_frame
from chess_robot.robot.urdf_model import DEFAULT_END_LINK

LIMIT_HIT_TOLERANCE_RAD = 1e-6
FINITE_DIFFERENCE_STEP_RAD = 1e-5


class IKResult(object):
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

    def to_dict(self):
        return {
            "success": bool(self.success),
            "status": str(self.status),
            "target_xyz_robot": _array_to_list(self.target_xyz_robot),
            "final_xyz_robot": _array_to_list(self.final_xyz_robot),
            "error_xyz_robot": _array_to_list(self.error_xyz_robot),
            "error_m": float(self.error_m),
            "iterations": int(self.iterations),
            "joint_positions_rad": _mapping_to_float_dict(self.joint_positions_rad),
            "joint_positions_deg": _mapping_to_float_dict(self.joint_positions_deg),
            "limit_margin_rad": _mapping_to_float_dict(self.limit_margin_rad),
            "hit_limit_joints": list(self.hit_limit_joints),
            "end_link": str(self.end_link),
            "tcp_frame": str(self.tcp_frame),
            "tool_offset_xyz_m": _array_to_list(self.tool_offset_xyz_m),
            "tool_offset_rpy_deg": _array_to_list(self.tool_offset_rpy_deg),
            "seed_source": str(self.seed_source),
            "candidate_count": int(self.candidate_count),
            "joint_names": list(self.joint_names),
            "optimized_joint_names": list(self.optimized_joint_names),
            "locked_joints_rad": _mapping_to_float_dict(self.locked_joints_rad),
            "approach_axis_local": _optional_array_to_list(getattr(self, "approach_axis_local", None)),
            "approach_axis_name": getattr(self, "approach_axis_name", None),
            "approach_target_axis": _optional_array_to_list(getattr(self, "approach_target_axis", None)),
            "approach_tilt_deg": _optional_float(getattr(self, "approach_tilt_deg", None)),
            "approach_weight": _optional_float(getattr(self, "approach_weight", None)),
            "approach_preferred": bool(getattr(self, "approach_preferred", False)),
            "approach_enforced": bool(getattr(self, "approach_enforced", False)),
            "selected_approach_tilt_limit_deg": _optional_float(getattr(self, "selected_approach_tilt_limit_deg", None)),
            "approach_angle_check": getattr(self, "approach_angle_check", None),
        }


def solve_position_ik(
    model,
    target_xyz_robot,
    seed_joint_positions_rad,
    joint_limits_rad,
    end_link=DEFAULT_END_LINK,
    tool_frame=None,
    max_iters=200,
    tolerance_m=0.005,
    damping=0.05,
    step_scale=1.0,
    locked_joint_positions_rad=None,
):
    target_xyz_robot = _as_xyz(target_xyz_robot, "target_xyz_robot")
    max_iters = int(max_iters)
    tolerance_m = float(tolerance_m)
    damping = float(damping)
    step_scale = float(step_scale)
    if max_iters <= 0:
        raise ValueError("max_iters must be positive.")
    if tolerance_m < 0.0:
        raise ValueError("tolerance_m must be non-negative.")
    if damping < 0.0:
        raise ValueError("damping must be non-negative.")
    if step_scale <= 0.0:
        raise ValueError("step_scale must be greater than zero.")

    joint_names, lower_limits, upper_limits = _normalise_joint_limits(
        model,
        joint_limits_rad,
        end_link=end_link,
    )
    joint_vector = _joint_vector_from_input(seed_joint_positions_rad, joint_names)
    joint_vector = np.clip(joint_vector, lower_limits, upper_limits)
    locked_joint_positions_rad = _normalise_locked_joint_positions(
        locked_joint_positions_rad,
        joint_names,
    )
    active_joint_indices = [
        joint_index
        for joint_index in range(len(joint_names))
        if joint_names[joint_index] not in locked_joint_positions_rad
    ]
    joint_vector = _apply_locked_joint_positions(joint_vector, joint_names, locked_joint_positions_rad)

    status = "max_iters"
    success = False
    final_xyz_robot = None
    error_xyz_robot = None
    iterations = 0

    for iteration in range(max_iters + 1):
        joint_map = _joint_map_from_vector(joint_names, joint_vector)
        final_xyz_robot = compute_tcp_transform(
            model,
            joint_map,
            end_link=end_link,
            tool_frame=tool_frame,
        )[:3, 3].copy()
        error_xyz_robot = target_xyz_robot - final_xyz_robot
        error_m = float(np.linalg.norm(error_xyz_robot))
        iterations = iteration

        if error_m <= tolerance_m:
            success = True
            status = "success"
            break
        if iteration >= max_iters:
            break

        if not active_joint_indices:
            status = "locked_joints_fixed"
            break

        jacobian = compute_position_jacobian(
            model,
            joint_map,
            joint_names=joint_names,
            end_link=end_link,
            tool_frame=tool_frame,
        )
        active_jacobian = jacobian[:, active_joint_indices]
        damp_matrix = np.dot(active_jacobian, active_jacobian.T) + ((damping ** 2) * np.eye(3, dtype=float))
        delta_q = np.dot(active_jacobian.T, np.linalg.solve(damp_matrix, error_xyz_robot))
        if not np.isfinite(delta_q).all():
            status = "non_finite_step"
            break
        updated_joint_vector = np.asarray(joint_vector, dtype=float).copy()
        for delta_index, joint_index in enumerate(active_joint_indices):
            updated_joint_vector[joint_index] = updated_joint_vector[joint_index] + (step_scale * delta_q[delta_index])
        joint_vector = np.clip(updated_joint_vector, lower_limits, upper_limits)
        joint_vector = _apply_locked_joint_positions(joint_vector, joint_names, locked_joint_positions_rad)

    limit_margin_rad = _compute_limit_margins(joint_names, joint_vector, lower_limits, upper_limits)
    hit_limit_joints = [
        joint_name
        for joint_name, margin in limit_margin_rad.items()
        if margin <= LIMIT_HIT_TOLERANCE_RAD
    ]
    joint_positions_rad = _joint_map_from_vector(joint_names, joint_vector)
    joint_positions_deg = dict(
        (joint_name, math.degrees(float(joint_positions_rad[joint_name])))
        for joint_name in joint_names
    )
    tool_frame_description = describe_tool_frame(tool_frame, fallback_name=end_link)

    return IKResult(
        success=success,
        status=status,
        target_xyz_robot=target_xyz_robot,
        final_xyz_robot=final_xyz_robot,
        error_xyz_robot=error_xyz_robot,
        error_m=float(np.linalg.norm(error_xyz_robot)),
        iterations=iterations,
        joint_positions_rad=joint_positions_rad,
        joint_positions_deg=joint_positions_deg,
        limit_margin_rad=limit_margin_rad,
        hit_limit_joints=hit_limit_joints,
        end_link=end_link,
        tcp_frame=tool_frame_description["tcp_frame"],
        tool_offset_xyz_m=np.asarray(tool_frame_description["tool_offset_xyz_m"], dtype=float),
        tool_offset_rpy_deg=np.asarray(tool_frame_description["tool_offset_rpy_deg"], dtype=float),
        seed_source="single_seed",
        candidate_count=1,
        joint_names=joint_names,
        optimized_joint_names=[joint_names[joint_index] for joint_index in active_joint_indices],
        locked_joints_rad=locked_joint_positions_rad,
    )


def solve_position_ik_with_approach(
    model,
    target_xyz_robot,
    seed_joint_positions_rad,
    joint_limits_rad,
    end_link=DEFAULT_END_LINK,
    tool_frame=None,
    max_iters=200,
    tolerance_m=0.005,
    damping=0.05,
    step_scale=1.0,
    locked_joint_positions_rad=None,
    approach_axis_local=None,
    approach_target_axis=None,
    approach_weight=0.05,
    enforce_approach_angle=False,
    selected_approach_tilt_limit_deg=None,
    approach_axis_name=None,
):
    target_xyz_robot = _as_xyz(target_xyz_robot, "target_xyz_robot")
    max_iters = int(max_iters)
    tolerance_m = float(tolerance_m)
    damping = float(damping)
    step_scale = float(step_scale)
    approach_weight = float(approach_weight)
    if max_iters <= 0:
        raise ValueError("max_iters must be positive.")
    if tolerance_m < 0.0:
        raise ValueError("tolerance_m must be non-negative.")
    if damping < 0.0:
        raise ValueError("damping must be non-negative.")
    if step_scale <= 0.0:
        raise ValueError("step_scale must be greater than zero.")
    if approach_weight < 0.0:
        raise ValueError("approach_weight must be non-negative.")
    if approach_axis_local is None:
        raise ValueError("approach_axis_local is required for orientation-constrained IK.")
    if approach_target_axis is None:
        raise ValueError("approach_target_axis is required for orientation-constrained IK.")

    approach_axis_local = normalize_vector(approach_axis_local, "approach_axis_local")
    approach_target_axis = normalize_vector(approach_target_axis, "approach_target_axis")
    if selected_approach_tilt_limit_deg is None:
        selected_approach_tilt_limit_deg = 180.0
    selected_approach_tilt_limit_deg = float(selected_approach_tilt_limit_deg)

    joint_names, lower_limits, upper_limits = _normalise_joint_limits(
        model,
        joint_limits_rad,
        end_link=end_link,
    )
    joint_vector = _joint_vector_from_input(seed_joint_positions_rad, joint_names)
    joint_vector = np.clip(joint_vector, lower_limits, upper_limits)
    locked_joint_positions_rad = _normalise_locked_joint_positions(
        locked_joint_positions_rad,
        joint_names,
    )
    active_joint_indices = [
        joint_index
        for joint_index in range(len(joint_names))
        if joint_names[joint_index] not in locked_joint_positions_rad
    ]
    joint_vector = _apply_locked_joint_positions(joint_vector, joint_names, locked_joint_positions_rad)

    status = "max_iters"
    success = False
    final_xyz_robot = None
    error_xyz_robot = None
    approach_axis_current = None
    approach_tilt_current_deg = None
    approach_angle_check = None
    iterations = 0

    for iteration in range(max_iters + 1):
        joint_map = _joint_map_from_vector(joint_names, joint_vector)
        tcp_transform = compute_tcp_transform(
            model,
            joint_map,
            end_link=end_link,
            tool_frame=tool_frame,
        )
        final_xyz_robot = tcp_transform[:3, 3].copy()
        approach_axis_current = normalize_vector(
            np.dot(tcp_transform[:3, :3], approach_axis_local),
            "approach_axis_robot",
        )
        error_xyz_robot = target_xyz_robot - final_xyz_robot
        axis_error = approach_target_axis - approach_axis_current
        error_m = float(np.linalg.norm(error_xyz_robot))
        approach_tilt_current_deg = approach_tilt_deg(
            approach_axis_current,
            reference_down_axis=approach_target_axis,
        )
        approach_angle_check = make_approach_angle_check(
            approach_tilt_current_deg,
            selected_approach_tilt_limit_deg,
        )
        iterations = iteration

        if error_m <= tolerance_m and (not bool(enforce_approach_angle) or bool(approach_angle_check["passed"])):
            success = True
            status = "success"
            break
        if iteration >= max_iters:
            break
        if not active_joint_indices:
            status = "locked_joints_fixed"
            break

        residual = np.concatenate((error_xyz_robot, approach_weight * axis_error))
        active_jacobian = _finite_difference_combined_jacobian(
            model,
            joint_names,
            joint_vector,
            active_joint_indices,
            end_link,
            tool_frame,
            approach_axis_local,
            final_xyz_robot,
            approach_axis_current,
        )
        active_jacobian[3:, :] = approach_weight * active_jacobian[3:, :]
        damp_matrix = np.dot(active_jacobian, active_jacobian.T) + ((damping ** 2) * np.eye(active_jacobian.shape[0], dtype=float))
        delta_q = np.dot(active_jacobian.T, np.linalg.solve(damp_matrix, residual))
        if not np.isfinite(delta_q).all():
            status = "non_finite_step"
            break
        updated_joint_vector = np.asarray(joint_vector, dtype=float).copy()
        for delta_index, joint_index in enumerate(active_joint_indices):
            updated_joint_vector[joint_index] = updated_joint_vector[joint_index] + (step_scale * delta_q[delta_index])
        joint_vector = np.clip(updated_joint_vector, lower_limits, upper_limits)
        joint_vector = _apply_locked_joint_positions(joint_vector, joint_names, locked_joint_positions_rad)

    limit_margin_rad = _compute_limit_margins(joint_names, joint_vector, lower_limits, upper_limits)
    hit_limit_joints = [
        joint_name
        for joint_name, margin in limit_margin_rad.items()
        if margin <= LIMIT_HIT_TOLERANCE_RAD
    ]
    joint_positions_rad = _joint_map_from_vector(joint_names, joint_vector)
    joint_positions_deg = dict(
        (joint_name, math.degrees(float(joint_positions_rad[joint_name])))
        for joint_name in joint_names
    )
    tool_frame_description = describe_tool_frame(tool_frame, fallback_name=end_link)

    return IKResult(
        success=success,
        status=status,
        target_xyz_robot=target_xyz_robot,
        final_xyz_robot=final_xyz_robot,
        error_xyz_robot=error_xyz_robot,
        error_m=float(np.linalg.norm(error_xyz_robot)),
        iterations=iterations,
        joint_positions_rad=joint_positions_rad,
        joint_positions_deg=joint_positions_deg,
        limit_margin_rad=limit_margin_rad,
        hit_limit_joints=hit_limit_joints,
        end_link=end_link,
        tcp_frame=tool_frame_description["tcp_frame"],
        tool_offset_xyz_m=np.asarray(tool_frame_description["tool_offset_xyz_m"], dtype=float),
        tool_offset_rpy_deg=np.asarray(tool_frame_description["tool_offset_rpy_deg"], dtype=float),
        seed_source="single_seed",
        candidate_count=1,
        joint_names=joint_names,
        optimized_joint_names=[joint_names[joint_index] for joint_index in active_joint_indices],
        locked_joints_rad=locked_joint_positions_rad,
        approach_axis_local=approach_axis_local,
        approach_axis_name=approach_axis_name or axis_name_from_vector(approach_axis_local),
        approach_target_axis=approach_target_axis,
        approach_tilt_deg=approach_tilt_current_deg,
        approach_weight=approach_weight,
        approach_preferred=True,
        approach_enforced=bool(enforce_approach_angle),
        selected_approach_tilt_limit_deg=selected_approach_tilt_limit_deg,
        approach_angle_check=approach_angle_check,
    )


def solve_position_ik_multi_seed(
    model,
    target_xyz_robot,
    joint_limits_rad,
    end_link=DEFAULT_END_LINK,
    tool_frame=None,
    seeds=None,
    home_joint_positions_rad=None,
    workspace_seed_joint_positions_rad=None,
    random_seeds=0,
    seed=None,
    max_iters=200,
    tolerance_m=0.005,
    damping=0.05,
    step_scale=1.0,
    locked_joint_positions_rad=None,
    approach_axis_local=None,
    approach_target_axis=None,
    approach_weight=0.05,
    prefer_vertical_approach=False,
    enforce_approach_angle=False,
    selected_approach_tilt_limit_deg=None,
    approach_axis_name=None,
):
    joint_names, lower_limits, upper_limits = _normalise_joint_limits(
        model,
        joint_limits_rad,
        end_link=end_link,
    )
    deterministic_seed_entries = []
    if seeds:
        for seed_entry in seeds:
            if isinstance(seed_entry, dict) and "joint_positions_rad" in seed_entry:
                deterministic_seed_entries.append(
                    {
                        "source": str(seed_entry.get("source") or "provided"),
                        "joint_positions_rad": seed_entry["joint_positions_rad"],
                    }
                )
            else:
                deterministic_seed_entries.append({"source": "provided", "joint_positions_rad": seed_entry})

    deterministic_seed_entries.extend(
        build_seed_candidates(
            joint_names,
            home_joint_positions_rad=home_joint_positions_rad,
            workspace_seed_joint_positions_rad=workspace_seed_joint_positions_rad,
        )
    )
    deterministic_seed_entries = _deduplicate_seed_entries(deterministic_seed_entries, joint_names)
    if not deterministic_seed_entries:
        raise ValueError("At least one IK seed is required.")

    best_success, best_failure = _evaluate_seed_entries(
        deterministic_seed_entries,
        model,
        target_xyz_robot,
        joint_limits_rad,
        end_link=end_link,
        tool_frame=tool_frame,
        max_iters=max_iters,
        tolerance_m=tolerance_m,
        damping=damping,
        step_scale=step_scale,
        locked_joint_positions_rad=locked_joint_positions_rad,
        approach_axis_local=approach_axis_local,
        approach_target_axis=approach_target_axis,
        approach_weight=approach_weight,
        prefer_vertical_approach=prefer_vertical_approach,
        enforce_approach_angle=enforce_approach_angle,
        selected_approach_tilt_limit_deg=selected_approach_tilt_limit_deg,
        approach_axis_name=approach_axis_name,
    )
    if best_success is not None:
        return best_success

    if int(random_seeds) <= 0:
        return best_failure

    random_seed_entries = build_random_seed_candidates(
        joint_names,
        lower_limits,
        upper_limits,
        random_seeds=random_seeds,
        seed=seed,
    )
    random_seed_entries = _deduplicate_seed_entries(random_seed_entries, joint_names)
    random_success, random_failure = _evaluate_seed_entries(
        random_seed_entries,
        model,
        target_xyz_robot,
        joint_limits_rad,
        end_link=end_link,
        tool_frame=tool_frame,
        max_iters=max_iters,
        tolerance_m=tolerance_m,
        damping=damping,
        step_scale=step_scale,
        locked_joint_positions_rad=locked_joint_positions_rad,
        approach_axis_local=approach_axis_local,
        approach_target_axis=approach_target_axis,
        approach_weight=approach_weight,
        prefer_vertical_approach=prefer_vertical_approach,
        enforce_approach_angle=enforce_approach_angle,
        selected_approach_tilt_limit_deg=selected_approach_tilt_limit_deg,
        approach_axis_name=approach_axis_name,
    )
    if random_success is not None:
        return random_success
    if best_failure is None:
        return random_failure
    if random_failure is None:
        return best_failure
    if _is_better_failure(random_failure, best_failure):
        return random_failure
    return best_failure


def build_seed_candidates(
    joint_names,
    home_joint_positions_rad=None,
    workspace_seed_joint_positions_rad=None,
):
    candidates = []
    if home_joint_positions_rad is not None:
        candidates.append({"source": "saved_home_pose", "joint_positions_rad": home_joint_positions_rad})
    candidates.append(
        {
            "source": "all_zero",
            "joint_positions_rad": dict((joint_name, 0.0) for joint_name in joint_names),
        }
    )
    if workspace_seed_joint_positions_rad is not None:
        candidates.append(
            {
                "source": "nearest_workspace_sample",
                "joint_positions_rad": workspace_seed_joint_positions_rad,
            }
        )
    return candidates


def build_random_seed_candidates(joint_names, lower_limits, upper_limits, random_seeds=0, seed=None):
    candidates = []
    random_state = np.random.RandomState(seed)
    for random_index in range(int(random_seeds)):
        sample = random_state.uniform(low=lower_limits, high=upper_limits)
        candidates.append(
            {
                "source": "random_%02d" % random_index,
                "joint_positions_rad": _joint_map_from_vector(joint_names, sample),
            }
        )
    return candidates


def sample_position_workspace(
    model,
    joint_limits_rad,
    sample_count,
    seed=None,
    end_link=DEFAULT_END_LINK,
    tool_frame=None,
):
    joint_names, lower_limits, upper_limits = _normalise_joint_limits(
        model,
        joint_limits_rad,
        end_link=end_link,
    )
    sample_count = int(sample_count)
    if sample_count <= 0:
        raise ValueError("sample_count must be positive.")
    random_state = np.random.RandomState(seed)
    positions = random_state.uniform(
        low=lower_limits,
        high=upper_limits,
        size=(sample_count, len(joint_names)),
    )
    tcp_points_robot_m = np.empty((sample_count, 3), dtype=float)
    for sample_index in range(sample_count):
        joint_map = _joint_map_from_vector(joint_names, positions[sample_index])
        tcp_points_robot_m[sample_index] = compute_tcp_transform(
            model,
            joint_map,
            end_link=end_link,
            tool_frame=tool_frame,
        )[:3, 3]
    return {
        "joint_names": joint_names,
        "positions": positions,
        "tcp_points_robot_m": tcp_points_robot_m,
        "lower_limits": lower_limits,
        "upper_limits": upper_limits,
        "sample_count": sample_count,
        "seed": seed,
    }


def find_nearest_workspace_seed(target_xyz_robot, workspace_samples):
    target_xyz_robot = _as_xyz(target_xyz_robot, "target_xyz_robot")
    points = np.asarray(workspace_samples["tcp_points_robot_m"], dtype=float)
    delta = points - target_xyz_robot[np.newaxis, :]
    squared_distance = np.sum(delta * delta, axis=1)
    nearest_index = int(np.argmin(squared_distance))
    joint_names = list(workspace_samples["joint_names"])
    sample = np.asarray(workspace_samples["positions"], dtype=float)[nearest_index]
    return {
        "joint_positions_rad": _joint_map_from_vector(joint_names, sample),
        "index": nearest_index,
        "distance_m": float(np.sqrt(squared_distance[nearest_index])),
    }


def world_point_to_robot_base(point_world, scene_geometry):
    robot_T_world = np.linalg.inv(np.asarray(scene_geometry["world_T_robot_base"], dtype=float))
    return _transform_point(robot_T_world, point_world)


def robot_base_point_to_world(point_robot, scene_geometry):
    world_T_robot = np.asarray(scene_geometry["world_T_robot_base"], dtype=float)
    return _transform_point(world_T_robot, point_robot)


def _transform_point(transform, point_xyz):
    point_xyz = _as_xyz(point_xyz, "point_xyz")
    homogeneous = np.ones(4, dtype=float)
    homogeneous[:3] = point_xyz
    return np.dot(np.asarray(transform, dtype=float), homogeneous)[:3]


def _normalise_joint_limits(model, joint_limits_rad, end_link):
    if joint_limits_rad is None:
        raise ValueError("joint_limits_rad is required.")
    joint_names = [str(name) for name in joint_limits_rad.get("joint_names") or []]
    lower_limits = np.asarray(joint_limits_rad.get("lower_limits"), dtype=float)
    upper_limits = np.asarray(joint_limits_rad.get("upper_limits"), dtype=float)
    if not joint_names:
        joint_names = [joint.name for joint in model.get_arm_chain(end_link=end_link)]
    if lower_limits.shape != (len(joint_names),):
        raise ValueError("lower_limits must have shape (%d,), got %s." % (len(joint_names), lower_limits.shape))
    if upper_limits.shape != (len(joint_names),):
        raise ValueError("upper_limits must have shape (%d,), got %s." % (len(joint_names), upper_limits.shape))
    return joint_names, lower_limits, upper_limits


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


def _compute_limit_margins(joint_names, joint_vector, lower_limits, upper_limits):
    margins = {}
    for joint_index, joint_name in enumerate(joint_names):
        lower_margin = float(joint_vector[joint_index] - lower_limits[joint_index])
        upper_margin = float(upper_limits[joint_index] - joint_vector[joint_index])
        margins[joint_name] = min(lower_margin, upper_margin)
    return margins


def _normalise_locked_joint_positions(locked_joint_positions_rad, joint_names):
    if locked_joint_positions_rad is None:
        return {}
    normalised = {}
    valid_joint_names = set(joint_names)
    for joint_name, value in locked_joint_positions_rad.items():
        joint_name = str(joint_name)
        if joint_name not in valid_joint_names:
            raise ValueError("Locked joint %s is not part of this IK chain." % joint_name)
        normalised[joint_name] = float(value)
    return normalised


def _apply_locked_joint_positions(joint_vector, joint_names, locked_joint_positions_rad):
    if not locked_joint_positions_rad:
        return np.asarray(joint_vector, dtype=float)
    updated = np.asarray(joint_vector, dtype=float).copy()
    for joint_index, joint_name in enumerate(joint_names):
        if joint_name in locked_joint_positions_rad:
            updated[joint_index] = float(locked_joint_positions_rad[joint_name])
    return updated


def _evaluate_seed_entries(
    seed_entries,
    model,
    target_xyz_robot,
    joint_limits_rad,
    end_link,
    tool_frame,
    max_iters,
    tolerance_m,
    damping,
    step_scale,
    locked_joint_positions_rad,
    approach_axis_local=None,
    approach_target_axis=None,
    approach_weight=0.05,
    prefer_vertical_approach=False,
    enforce_approach_angle=False,
    selected_approach_tilt_limit_deg=None,
    approach_axis_name=None,
):
    best_success = None
    best_failure = None
    approach_enabled = bool(prefer_vertical_approach or enforce_approach_angle)
    for seed_entry in seed_entries:
        if approach_enabled:
            result = solve_position_ik_with_approach(
                model,
                target_xyz_robot,
                seed_entry["joint_positions_rad"],
                joint_limits_rad,
                end_link=end_link,
                tool_frame=tool_frame,
                max_iters=max_iters,
                tolerance_m=tolerance_m,
                damping=damping,
                step_scale=step_scale,
                locked_joint_positions_rad=locked_joint_positions_rad,
                approach_axis_local=approach_axis_local,
                approach_target_axis=approach_target_axis,
                approach_weight=approach_weight,
                enforce_approach_angle=enforce_approach_angle,
                selected_approach_tilt_limit_deg=selected_approach_tilt_limit_deg,
                approach_axis_name=approach_axis_name,
            )
        else:
            result = solve_position_ik(
                model,
                target_xyz_robot,
                seed_entry["joint_positions_rad"],
                joint_limits_rad,
                end_link=end_link,
                tool_frame=tool_frame,
                max_iters=max_iters,
                tolerance_m=tolerance_m,
                damping=damping,
                step_scale=step_scale,
                locked_joint_positions_rad=locked_joint_positions_rad,
            )
            result.approach_axis_local = None
            result.approach_axis_name = None
            result.approach_target_axis = None
            result.approach_tilt_deg = None
            result.approach_weight = float(approach_weight)
            result.approach_preferred = False
            result.approach_enforced = False
            result.selected_approach_tilt_limit_deg = selected_approach_tilt_limit_deg
            result.approach_angle_check = None
        result.seed_source = str(seed_entry["source"])
        result.candidate_count = len(seed_entries)
        if result.success:
            if best_success is None or _is_better_success(result, best_success):
                best_success = result
        else:
            if best_failure is None or _is_better_failure(result, best_failure):
                best_failure = result
    return best_success, best_failure


def _finite_difference_combined_jacobian(
    model,
    joint_names,
    joint_vector,
    active_joint_indices,
    end_link,
    tool_frame,
    approach_axis_local,
    base_position,
    base_axis,
    epsilon=FINITE_DIFFERENCE_STEP_RAD,
):
    active_count = len(active_joint_indices)
    jacobian = np.zeros((6, active_count), dtype=float)
    for active_index, joint_index in enumerate(active_joint_indices):
        perturbed = np.asarray(joint_vector, dtype=float).copy()
        perturbed[joint_index] = perturbed[joint_index] + float(epsilon)
        perturbed_map = _joint_map_from_vector(joint_names, perturbed)
        perturbed_transform = compute_tcp_transform(
            model,
            perturbed_map,
            end_link=end_link,
            tool_frame=tool_frame,
        )
        perturbed_position = perturbed_transform[:3, 3].copy()
        perturbed_axis = normalize_vector(
            np.dot(perturbed_transform[:3, :3], approach_axis_local),
            "approach_axis_robot",
        )
        jacobian[:3, active_index] = (perturbed_position - base_position) / float(epsilon)
        jacobian[3:, active_index] = (perturbed_axis - base_axis) / float(epsilon)
    return jacobian


def _deduplicate_seed_entries(seed_entries, joint_names):
    deduplicated = []
    seen_keys = set()
    for seed_entry in seed_entries:
        seed_vector = _joint_vector_from_input(seed_entry["joint_positions_rad"], joint_names)
        rounded_key = tuple(round(float(value), 10) for value in seed_vector)
        if rounded_key in seen_keys:
            continue
        seen_keys.add(rounded_key)
        deduplicated.append(
            {
                "source": str(seed_entry["source"]),
                "joint_positions_rad": _joint_map_from_vector(joint_names, seed_vector),
            }
        )
    return deduplicated


def _is_better_success(candidate, incumbent):
    candidate_margin = min(candidate.limit_margin_rad.values())
    incumbent_margin = min(incumbent.limit_margin_rad.values())
    if bool(getattr(candidate, "approach_preferred", False)) or bool(getattr(candidate, "approach_enforced", False)):
        candidate_tilt = float(getattr(candidate, "approach_tilt_deg", float("inf")))
        incumbent_tilt = float(getattr(incumbent, "approach_tilt_deg", float("inf")))
        return (candidate_tilt, candidate.error_m, -candidate_margin, candidate.iterations) < (
            incumbent_tilt,
            incumbent.error_m,
            -incumbent_margin,
            incumbent.iterations,
        )
    return (candidate.error_m, -candidate_margin, candidate.iterations) < (
        incumbent.error_m,
        -incumbent_margin,
        incumbent.iterations,
    )


def _is_better_failure(candidate, incumbent):
    candidate_tilt = float(getattr(candidate, "approach_tilt_deg", float("inf")))
    incumbent_tilt = float(getattr(incumbent, "approach_tilt_deg", float("inf")))
    return (candidate.error_m, candidate_tilt, candidate.iterations) < (
        incumbent.error_m,
        incumbent_tilt,
        incumbent.iterations,
    )


def _as_xyz(values, name):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,):
        raise ValueError("Expected %s to have shape (3,), got %s." % (name, vector.shape))
    return vector


def _optional_array_to_list(values):
    if values is None:
        return None
    return _array_to_list(values)


def _optional_float(value):
    if value is None:
        return None
    return float(value)


def _array_to_list(values):
    return [float(value) for value in np.asarray(values, dtype=float)]


def _mapping_to_float_dict(mapping):
    return dict((str(key), float(value)) for key, value in mapping.items())
