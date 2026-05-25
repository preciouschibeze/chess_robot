from __future__ import absolute_import

import numpy as np
import yaml

from chess_robot.robot.fk import compute_fk
from chess_robot.robot.fk import make_transform
from chess_robot.robot.fk import rpy_matrix
from chess_robot.robot.urdf_model import DEFAULT_END_LINK

DEFAULT_TCP_FRAME = "gripper_frame"


def load_tool_frames(path):
    with open(path, "r") as handle:
        document = yaml.safe_load(handle) or {}
    root = document.get("tool_frames", document)
    if not isinstance(root, dict):
        raise ValueError("Tool frame file must contain a 'tool_frames' mapping.")

    frames_root = root.get("frames") or {}
    if not isinstance(frames_root, dict):
        raise ValueError("Tool frame file must contain a 'frames' mapping.")

    default_tcp = str(root.get("default_tcp") or DEFAULT_TCP_FRAME)
    frames = {}
    for frame_name, raw_frame in frames_root.items():
        if not isinstance(raw_frame, dict):
            raise ValueError("Tool frame %s must be a mapping." % frame_name)
        frames[str(frame_name)] = {
            "name": str(frame_name),
            "parent_link": str(raw_frame.get("parent_link") or DEFAULT_END_LINK),
            "xyz_m": _as_float_vector(raw_frame.get("xyz_m", (0.0, 0.0, 0.0)), "xyz_m"),
            "rpy_deg": _as_float_vector(raw_frame.get("rpy_deg", (0.0, 0.0, 0.0)), "rpy_deg"),
            "approach_axis_local": _as_float_vector(raw_frame.get("approach_axis_local", (0.0, 0.0, -1.0)), "approach_axis_local"),
            "approach_axis_local_defaulted": raw_frame.get("approach_axis_local") is None,
            "notes": str(raw_frame.get("notes") or ""),
        }

    if default_tcp not in frames:
        raise ValueError("Default TCP frame %s was not found in %s." % (default_tcp, path))

    return {
        "path": path,
        "default_tcp": default_tcp,
        "frames": frames,
    }


def get_tool_frame(tool_frames, tcp_frame_name=None):
    if tool_frames is None:
        return None
    requested_name = tcp_frame_name or tool_frames.get("default_tcp")
    if not requested_name:
        raise ValueError("No TCP frame name was requested and no default TCP is configured.")
    frame = (tool_frames.get("frames") or {}).get(str(requested_name))
    if frame is None:
        raise KeyError(
            "Requested TCP frame %s was not found in %s."
            % (requested_name, tool_frames.get("path", "<unknown>"))
        )
    return frame


def tool_transform_from_config(tool_frame):
    if tool_frame is None:
        return np.eye(4, dtype=float)
    xyz_m = np.asarray(tool_frame["xyz_m"], dtype=float)
    rpy_deg = np.asarray(tool_frame["rpy_deg"], dtype=float)
    if xyz_m.shape != (3,):
        raise ValueError("Tool frame xyz_m must have shape (3,).")
    if rpy_deg.shape != (3,):
        raise ValueError("Tool frame rpy_deg must have shape (3,).")
    rpy_rad = np.radians(rpy_deg)
    return make_transform(rpy_matrix(rpy_rad), xyz_m)


def compute_tcp_transform(model, joint_positions_rad, end_link=DEFAULT_END_LINK, tool_frame=None):
    parent_link = end_link
    if tool_frame is not None:
        parent_link = str(tool_frame.get("parent_link") or end_link)
    base_T_parent = compute_fk(model, joint_positions_rad, end_link=parent_link)
    if tool_frame is None:
        return base_T_parent
    return np.dot(base_T_parent, tool_transform_from_config(tool_frame))


def compute_tcp_position(model, joint_positions_rad, end_link=DEFAULT_END_LINK, tool_frame=None):
    return compute_tcp_transform(
        model,
        joint_positions_rad,
        end_link=end_link,
        tool_frame=tool_frame,
    )[:3, 3].copy()


def describe_tool_frame(tool_frame, fallback_name=DEFAULT_END_LINK):
    if tool_frame is None:
        return {
            "tcp_frame": str(fallback_name),
            "tool_offset_xyz_m": [0.0, 0.0, 0.0],
            "tool_offset_rpy_deg": [0.0, 0.0, 0.0],
        }
    return {
        "tcp_frame": str(tool_frame.get("name") or fallback_name),
        "tool_offset_xyz_m": [float(value) for value in np.asarray(tool_frame["xyz_m"], dtype=float)],
        "tool_offset_rpy_deg": [float(value) for value in np.asarray(tool_frame["rpy_deg"], dtype=float)],
        "approach_axis_local": [float(value) for value in np.asarray(tool_frame["approach_axis_local"], dtype=float)],
        "approach_axis_local_defaulted": bool(tool_frame.get("approach_axis_local_defaulted", False)),
    }


def _as_float_vector(values, name):
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,):
        raise ValueError("Expected %s to have shape (3,), got %s." % (name, vector.shape))
    return vector
