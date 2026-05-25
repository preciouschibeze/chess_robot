from __future__ import absolute_import

import math

import numpy as np


WORLD_DOWN_AXIS = np.asarray((0.0, 0.0, -1.0), dtype=float)
DEFAULT_APPROACH_AXIS_LOCAL = np.asarray((0.0, 0.0, -1.0), dtype=float)

AXIS_NAME_MAP = {
    "plus_x": np.asarray((1.0, 0.0, 0.0), dtype=float),
    "minus_x": np.asarray((-1.0, 0.0, 0.0), dtype=float),
    "plus_y": np.asarray((0.0, 1.0, 0.0), dtype=float),
    "minus_y": np.asarray((0.0, -1.0, 0.0), dtype=float),
    "plus_z": np.asarray((0.0, 0.0, 1.0), dtype=float),
    "minus_z": np.asarray((0.0, 0.0, -1.0), dtype=float),
}

AXIS_CANDIDATE_ORDER = (
    "plus_x",
    "minus_x",
    "plus_y",
    "minus_y",
    "plus_z",
    "minus_z",
)


def normalize_vector(values, name):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,):
        raise ValueError("%s must have shape (3,)." % name)
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        raise ValueError("%s must be non-zero." % name)
    return vector / norm


def axis_vector_from_name(name):
    key = str(name).strip().lower()
    if key not in AXIS_NAME_MAP:
        raise ValueError("Unknown approach axis name: %s" % name)
    return normalize_vector(AXIS_NAME_MAP[key], "approach_axis_local")


def axis_name_from_vector(values, tolerance=1.0e-9):
    axis = normalize_vector(values, "approach_axis_local")
    for name in AXIS_CANDIDATE_ORDER:
        if np.linalg.norm(axis - AXIS_NAME_MAP[name]) <= float(tolerance):
            return name
    return None


def resolve_approach_axis_local(tool_frame=None, approach_axis_name=None, approach_axis_local=None):
    if approach_axis_name is not None and approach_axis_local is not None:
        raise ValueError("Use either approach_axis_name or approach_axis_local, not both.")

    if approach_axis_name is not None:
        axis = axis_vector_from_name(approach_axis_name)
        return {
            "axis_local": axis,
            "axis_name": str(approach_axis_name).strip().lower(),
            "source": "cli_name",
            "defaulted": False,
        }

    if approach_axis_local is not None:
        axis = normalize_vector(approach_axis_local, "approach_axis_local")
        return {
            "axis_local": axis,
            "axis_name": axis_name_from_vector(axis),
            "source": "cli_local",
            "defaulted": False,
        }

    raw_axis = None
    if tool_frame is not None:
        raw_axis = tool_frame.get("approach_axis_local")
    if raw_axis is None:
        axis = DEFAULT_APPROACH_AXIS_LOCAL.copy()
        return {
            "axis_local": axis,
            "axis_name": axis_name_from_vector(axis),
            "source": "default",
            "defaulted": True,
        }

    axis = normalize_vector(raw_axis, "approach_axis_local")
    return {
        "axis_local": axis,
        "axis_name": axis_name_from_vector(axis),
        "source": "tool_frame",
        "defaulted": False,
    }


def transform_local_axis_to_world(tcp_world_transform, approach_axis_local):
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


def inspect_candidate_axes(tcp_world_transform, reference_down_axis=None):
    down = normalize_vector(reference_down_axis if reference_down_axis is not None else WORLD_DOWN_AXIS, "reference_down_axis")
    ranked = []
    for name in AXIS_CANDIDATE_ORDER:
        local_axis = axis_vector_from_name(name)
        world_axis = transform_local_axis_to_world(tcp_world_transform, local_axis)
        ranked.append(
            {
                "axis_name": name,
                "axis_local": [float(value) for value in local_axis],
                "axis_world": [float(value) for value in world_axis],
                "tilt_deg": float(approach_tilt_deg(world_axis, down)),
            }
        )
    ranked.sort(key=lambda item: (float(item["tilt_deg"]), item["axis_name"]))
    return ranked


def best_candidate_axis(tcp_world_transform, reference_down_axis=None):
    ranked = inspect_candidate_axes(tcp_world_transform, reference_down_axis=reference_down_axis)
    return ranked[0] if ranked else None


def transform_world_axis_to_robot_base(scene_geometry, axis_world):
    world_t_robot = np.asarray(scene_geometry["world_T_robot_base"], dtype=float)
    robot_axis = np.dot(np.linalg.inv(world_t_robot)[:3, :3], normalize_vector(axis_world, "axis_world"))
    return normalize_vector(robot_axis, "robot_axis")


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
